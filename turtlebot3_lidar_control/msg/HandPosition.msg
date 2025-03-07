float32 angle
float32 distance

std_msgs/Header header    # header (timestamp)
float32 angle_min        # องศาเริ่มต้นของ Lidar
float32 angle_max        # องศาสูงสุดของ Lidar
float32 angle_increment  # องศาที่เปลี่ยนแปลงในแต่ละจุด
float32 range_min        # ระยะต่ำสุดที่วัดได้
float32 range_max        # ระยะสูงสุดที่วัดได้
float32[] ranges         # ข้อมูลระยะทางแต่ละจุด (หน่วยเป็นเมตร)

geometry_msgs/Vector3 linear  # ความเร็วเชิงเส้น (เดินหน้า/ถอยหลัง)
geometry_msgs/Vector3 angular # ความเร็วเชิงมุม (หมุนซ้าย/หมุนขวา)
