# Host PC GPS 카메라가 발행하는 메시지
# Field frame: origin (0,0) at one corner
# x: 0..1880 mm, y: 0..1410 mm
# Topic: /pinky_{ROS_DOMAIN_ID}/gps_pos
std_msgs/Header header
uint8  pinky_id
float64 x_mm
float64 y_mm
float64 yaw_deg
# optional fields (future-proof, default 0/false if not published)
uint8  confidence    # 0=unknown, 1~100=percent
bool   is_valid      # true if marker detected
