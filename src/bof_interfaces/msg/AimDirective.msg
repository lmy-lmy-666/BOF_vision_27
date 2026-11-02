# Decision -> auto aim, topic /sentry/aim_directive. Labels follow ArmorCandidate:
# 0 sentry, 1 hero, 2 engineer, 3/4/5 infantry, 6 outpost, 7 base.
# Note: rm_interfaces SentryLockEnemy uses 7=sentry, 6=outpost, 8=base; convert at the decision side.
std_msgs/Header header
# false: vision does not command the gimbal (decision/search owns it).
bool aim_enable true
# Fire permission (heat, game stage, friendly fire). Fire also needs auto_fire and a steady solution.
bool fire_enable false
# -1 none; otherwise only this label is tracked.
int32 lock_label -1
# -1 none; otherwise this label ranks first.
int32 priority_label -1
# Labels that must not be targeted (invincible, respawn protection).
int32[] excluded_labels
# Referee knowledge: true after the outpost stopped spinning (3 min, first destroyed, base armour open).
bool outpost_stopped false
