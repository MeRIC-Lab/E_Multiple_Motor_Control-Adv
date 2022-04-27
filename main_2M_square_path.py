from motorLLC_sync import *
import time

MIN_POS_V  = 100               # Dynamixel will rotate between this value
MAX_POS_V  = 2000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

index = 0

motorNo = 2
motorIDs = [1, 2]
motorType = [2.0, 2.0]
dxl_goal_pos = [[MIN_POS_V, MIN_POS_V], [MAX_POS_V, MAX_POS_V]]         # Goal position
dxl_goal_vel = [[30, 30], [150, 150]]         # Goal velocity

mc = motorLLC()
mc.set_motor_IDs(motorNo, motorIDs, motorType)
mc.open()
mc.torque_enable()

for i in range(1,6):

    #mc.moveTo(dxl_goal_pos[index])
    mc.moveTo_wVel_rpm(dxl_goal_pos[index], dxl_goal_vel[index])

    #time.sleep(4)
    while 1:
        dxl_present_pos = mc.readPos()

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t" %
              (motorIDs[0], dxl_goal_pos[index][0], dxl_present_pos[0],
               motorIDs[1], dxl_goal_pos[index][1], dxl_present_pos[1]))

        if abs(dxl_goal_pos[index][0] - dxl_present_pos[0]) < MOVING_STATUS_THRESHOLD:
            if abs(dxl_goal_pos[index][1] - dxl_present_pos[1]) < MOVING_STATUS_THRESHOLD:
                print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (motorIDs[0], dxl_goal_pos[index][0], dxl_present_pos[0], motorIDs[1], dxl_goal_pos[index][1], dxl_present_pos[1]))
                break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

mc.close()
