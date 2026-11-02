std_msgs/Header header
bool valid
uint64 track_id
# Pixels, not meters or radians. Zero when invalid.
float64 u
float64 v
# True means a unique current candidate was associated, including tentative observations.
# Only valid=true permits consumption of u/v; false always has zero u/v.
bool observed
# Same semantic labels as ArmorCandidate; track_id remains an association id.
int32 label -1
# Index into same-frame ArmorCandidates, -1 when no unique association.
int32 candidate_index -1
# Raw current candidate label; may differ from stable label during label_pending.
int32 observed_label -1
string status "lost"
float64 association_error_px -1.0
