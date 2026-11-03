# 云台固件修改建议（给电控）

> **状态更新（2026-11-02）：本文是修改前的历史建议。电控源码现已修改，等待电控烧录；40 项主机隔离检查通过，但目标编译结果及板上验证待提供。下文示例没有覆盖完整停止链路，不要用它覆盖当前实现。当前状态及资料见[开发交接第 5 节](DEVELOPMENT.md#5-云台串口协议)。**

对象：云台固件 sentry_gimbol_new（STM32F4，FreeRTOS，USB CDC 与视觉通信）。依据：Task/Src/Auto_aim.c、Src/usbd_cdc_if.c、Task/Src/GimbolControl.c、Task/Src/ModeControl.c。2026-11-02。

视觉端已经按现有固件的帧格式和角度约定实现（见[开发交接](DEVELOPMENT.md)第 5 节），**帧格式不需要改**。下面是接收逻辑和上报字段的修改。

| # | 问题 | 影响 | 优先级 |
|---|---|---|---|
| 1 | 没有视觉指令超时 | 拔 USB 或视觉崩溃后，云台一直执行最后一帧；最后一帧是 mode 2 时持续开火 | **必须，装弹测试前** |
| 2 | CRC 算了但没用 | 校验失败的帧照样控制云台 | 必须 |
| 3 | 只检查首字节 0x50，不按帧切分 | 数据错位时角度跳变 | 必须 |
| 4 | bullet_speed 固定 22 | 视觉只能用配置里的弹速，换档、衰减时弹道不准 | 建议 |
| 5 | bullet_count 固定 1 | 测不出"开火 → 出弹"延迟 | 建议 |

## 1～3：接收逻辑

**现状**：`Auto_aim_Task` 每个循环调用 `usb_recieve`，后者手动调用 `CDC_Receive_FS(usb_rx_buf, …)`。这个函数本是 USB 收包回调，手动调用只会重新设置接收缓冲区，并不表示有新数据到来。随后 `if (usb_rx_buf[0] == 0x50)` 就把缓冲区拷进 `Gimbol_Shoot`。缓冲区里一直留着上一包，所以固件分不出"新命令"和"旧数据"。`crc_ok` 只被赋值，控制代码（GimbolControl.c、ModeControl.c）直接用 `Gimbol_Shoot.mode/yaw/pitch`。

**改法**：在 USB 收包回调里处理数据——按帧头查找、CRC 通过才更新、记录时间；任务里检查超时。

```c
/* Src/usbd_cdc_if.c，USER CODE 区内 */
extern void vision_on_receive(uint8_t *buf, uint32_t len);
int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  vision_on_receive(Buf, *Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
```

```c
/* Task/Src/Auto_aim.c */
#define VISION_TIMEOUT_MS 50U       /* 视觉每 2 ms 发一帧，50 ms 足够宽 */
static volatile uint32_t last_valid_tick;
static AUTO_RX rx_latest;           /* 中断里写，任务里读 */
static volatile uint8_t rx_fresh;

/* USB 中断上下文：只做查找、校验和拷贝 */
void vision_on_receive(uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i + sizeof(AUTO_RX) <= len; ++i) {
    if (buf[i] == 0x50 && CRC16_Verify(&buf[i], sizeof(AUTO_RX))) {
      memcpy(&rx_latest, &buf[i], sizeof(AUTO_RX));
      rx_fresh = 1;
      last_valid_tick = HAL_GetTick();
      i += sizeof(AUTO_RX) - 1;
    }
  }
}

void Auto_aim_Task(void)
{
  if (rx_fresh) {                    /* 关中断拷贝，避免读到一半被改写 */
    __disable_irq(); Gimbol_Shoot = rx_latest; rx_fresh = 0; __enable_irq();
  }
  if (HAL_GetTick() - last_valid_tick > VISION_TIMEOUT_MS)
    Gimbol_Shoot.mode = 0;           /* 视觉断线或停发：不控云台、不开火 */
  usb_send_AutoAim();
}
```

删除原来的 `usb_recieve()` 调用和 `crc_ok`。`Auto_aim_RX()` 里取反 pitch 的 `AUTO_RX_` 没有被控制代码使用，可以一起删掉，避免误解。

注意：
- 视觉一帧 28 字节，USB 全速一包最多 64 字节，一包里可能有两帧，也可能半帧跨两包。上面的写法会丢掉跨包的半帧，因为视觉 2 ms 发一次，丢一帧无影响。要更严谨，可以改成环形缓冲区。
- 超时后 `mode = 0`：在 AUTO 模式下，`Gimbol_AUTO` 会在 1 秒后进入扫描（原逻辑 `AUTO_AIM_LOST_TIMEOUT_MS`）；在 RC 模式下回到遥控。发射逻辑看 `Gimbol_Shoot.mode == 2`，超时后不会再开火。

## 4～5：上报弹速和发弹计数

`Auto_aim_TX` 里：

```c
AUTO_TX_->bullet_speed = 22;   /* 改成裁判系统 0x0208 射击数据的初速（m/s，float） */
AUTO_TX_->bullet_count = 1;    /* 改成累计发弹数：每收到一次 0x0208 就加 1，或用拨盘计数 */
```

请说明 bullet_count 在什么时刻加 1（裁判系统检测到出弹 / 拨盘转过一格），视觉据此解读测出的延迟。改好后告诉视觉，视觉打开配置 `aim.gimbal_link.use_bullet_speed: true`。

## 修改后的验证

1. 视觉正常运行，拔掉 USB：云台在 50 ms 内停止跟随视觉，不开火。
2. 视觉 `kill -9` aim_node：同上；aim_node 1 秒后重启，云台重新跟随。
3. 用串口调试工具发一帧故意改错 CRC 的数据：云台不动。
4. 视觉 `/diagnostics` 里 gimbal_crc_errors 不增长；`/gimbal/state` 的 bullet_speed 随档位变化，bullet_count 随开火增加。

这些对应[实车测试方案](TEST_PLAN.md)第 10.3、10.4 节。改好后，第 0 节第 7 条的临时限制可以取消。
