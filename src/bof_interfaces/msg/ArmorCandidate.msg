# Image pixels, ordered TL, TR, BR, BL. z is always zero.
# Pixel observation only; classification does not imply a valid 3D pose.
geometry_msgs/Point32[4] corners
# -1 for neural detections.
float32 geometric_score -1.0
# -1 for geometric detections; otherwise neural objectness, not calibrated accuracy.
float32 model_confidence -1.0
# -1 unknown; 0 sentry, 1 hero, 2 engineer, 3/4/5 infantry, 6 outpost, 7 base.
int32 label -1
int32 raw_class -1
# 0 unknown, 1 small, 2 large; only explicit model class semantics.
uint8 size_hint 0
