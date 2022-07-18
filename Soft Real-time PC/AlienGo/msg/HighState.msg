uint8 levelFlag
uint16 commVersion
uint16 robotID
uint32 SN
uint8 bandWidth
uint8 mode
IMU imu
uint8 gaitType                      # 0.trot | 1.trot running | 2.climb stair
float32 footRaiseHeight             # (unit: m, default: 0.08m), foot swing height while walking
float32 bodyHeight                  # (unit: m)
float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 yawSpeed                    # (unit: rad/s), rotateSpeed in body frame
Cartesian[4] footPosition2Body      # foot position relative to body
Cartesian[4] footSpeed2Body         # foot speed relative to body
int16[4] footForce
uint8[40] wirelessRemote
uint32 reserve
uint32 crc