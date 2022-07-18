uint8 levelFlag
uint16 commVersion
uint16 robotID
uint32 SN
uint8 bandWidth
uint8 mode                      # 0. idle, default stand
                                # 1. force stand (controlled by bodyHeight + rpy)
                                # 2. target velocity walking (controlled by velocity + yawSpeed)
                                # 3. target position walking (controlled by position + rpy[2])
                                # 4. path mode walking (reserve for future release)
                                # 5. position stand down.
                                # 6. position stand up
                                # 7. damping mode
                                # 8. recovery mode
uint8 gaitType                  # 0.trot | 1.trot running | 2.climb stair
uint8 speedLevel                # 0. default low speed. 1. medium speed 2. high speed.
float32 footRaiseHeight         # (unit: m, default: 0.08m), foot up height while walking
float32 bodyHeight              # (unit: m, default: 0.28m),
float32[2] position              # (unit: m), desired position in inertial frame
float32[3] rpy                  # (unit: rad), desired yaw-pitch-roll euler angle, expressed in roll(rpy[0]) pitch(rpy[1]) yaw(rpy[2])
float32[2] velocity             # (unit: m/s), forwardSpeed, sideSpeed in body frame
float32 yawSpeed                # (unit: rad/s), rotateSpeed in body frame
LED[4] led
uint8[40] wirelessRemote
uint32 reserve
int32 crc