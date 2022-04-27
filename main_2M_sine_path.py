from motorLLC_sync import *
#from pathGen import *
import time

MIN_POS_V  = 100               # Dynamixel will rotate between this value
MAX_POS_V  = 2000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

index = 0

motorNo = 2
motorIDs = [1, 2]
motorType = [2.0, 2.0]

mc = motorLLC()
mc.set_motor_IDs(motorNo, motorIDs, motorType)
mc.open()
mc.torque_enable()

ST = 0.01     #sampling time
for tt in range(1,1000):

    eTime = tt * 0.01
    path_omega = pi
    path_radius = 1000           # the unit is pulse
    pos, vel = TestPath(path_omega, path_radius, eTime)
    mc.moveTo(pos)

    time.sleep(ST)
    print(pos)

mc.close()
