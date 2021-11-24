#!/usr/bin/env python
#-*- coding: utf-8 -*-

from enum import Enum

import rospy
from gantry_robot.msg import *
from gantry_robot.srv import *

SRV_SUCCESS	= 1
NOT_DONE = -1

# MAIN_HZ = 1
MAIN_HZ = 1000

g_info = Info()
def infoCallback(info):
    g_info = info
    # rospy.loginfo("info msg: " + str(info))

g_done_srv = NOT_DONE
def serviceDoneCallback(req):
    global g_done_srv
    g_done_srv = req.command
    # rospy.loginfo('req.command: ' + str(req.command))
    return CommandResponse(SRV_SUCCESS)

class ActionState(Enum):
    INIT = 0
    LOCATION1 = 1
    LOCATION2 = 2
    LOCATION3 = 3
    LOCATION4 = 4
    HOME = 5
    STOP = 6
    STOP_INIT = 7
    ERROR = 8
    ERROR_INIT = 9
    IDLE = 10

class CommandState(Enum):
    INIT = 0
    HOME = 1
    LOCATION = 2
    POSITION = 3
    JOG = 4
    STOP = 5
    IDLE = 6
    ERROR = 7
    NOT_SET = -1

def main():
    global g_done_srv
    rospy.init_node('test_gantry_robot')
    
    rospy.wait_for_service('/gantry_robot/gantry_robot_command')
    client_command = rospy.ServiceProxy('/gantry_robot/gantry_robot_command', Command)
    
    rospy.wait_for_service('/gantry_robot/gantry_robot_location')
    client_location = rospy.ServiceProxy('/gantry_robot/gantry_robot_location', Location)

    rospy.Service('/gantry_robot/gantry_robot_done', Command, serviceDoneCallback)

    rospy.Subscriber('/gantry_robot/gantry_robot_info', Info, infoCallback)


    rate = rospy.Rate(MAIN_HZ)

    actionState = ActionState.INIT

    client_command(CommandState.INIT.value)

    while not rospy.is_shutdown():
        if actionState == ActionState.INIT:
            if g_done_srv == CommandState.INIT.value:
                actionState = ActionState.LOCATION1
                rospy.loginfo('INIT done')

                g_done_srv = NOT_DONE
                x = 0.0
                y = 0.0
                z = 0.0
                client_location(x, y, z)
        elif actionState == ActionState.LOCATION1:
            if g_done_srv == CommandState.LOCATION.value:
                actionState = ActionState.LOCATION2
                rospy.loginfo('LOCATION1 done')

                g_done_srv = NOT_DONE
                x = 0.0
                y = 0.0
                z = 0.0
                client_location(x, y, z)
        elif actionState == ActionState.LOCATION2:
            if g_done_srv == CommandState.LOCATION.value:
                actionState = ActionState.LOCATION3
                rospy.loginfo('LOCATION2 done')

                g_done_srv = NOT_DONE
                x = 0.0
                y = 0.0
                z = 0.0
                client_location(x, y, z)
        elif actionState == ActionState.LOCATION3:
            if g_done_srv == CommandState.LOCATION.value:
                actionState = ActionState.LOCATION4
                rospy.loginfo('LOCATION3 done')

                g_done_srv = NOT_DONE
                x = 0.0
                y = 0.0
                z = 0.0
                client_location(x, y, z)
        elif actionState == ActionState.LOCATION4:
            if g_done_srv == CommandState.LOCATION.value:
                actionState = ActionState.HOME
                rospy.loginfo('LOCATION4 done')

                g_done_srv = NOT_DONE
                client_command(CommandState.HOME.value)
        elif actionState == ActionState.HOME:
            if g_done_srv == CommandState.HOME.value:
                actionState = ActionState.STOP
                rospy.loginfo('HOME done')

                g_done_srv = NOT_DONE
                client_command(CommandState.STOP.value)
        elif actionState == ActionState.STOP:
            if g_done_srv == CommandState.STOP.value:
                actionState = ActionState.STOP_INIT
                rospy.loginfo('STOP done')

                g_done_srv = NOT_DONE
                client_command(CommandState.INIT.value)
        elif actionState == ActionState.STOP_INIT:
            if g_done_srv == CommandState.INIT.value:
                actionState = ActionState.ERROR
                rospy.loginfo('STOP_INIT done')

                g_done_srv = NOT_DONE
                client_command(CommandState.ERROR.value)
        elif actionState == ActionState.ERROR:
            if g_done_srv == CommandState.ERROR.value:
                actionState = ActionState.ERROR_INIT
                rospy.loginfo('ERROR done')

                g_done_srv = NOT_DONE
                client_command(CommandState.INIT.value)
        elif actionState == ActionState.ERROR_INIT:
            if g_done_srv == CommandState.INIT.value:
                actionState = ActionState.IDLE
                rospy.loginfo('ERROR_INIT done')
        elif actionState == ActionState.IDLE:
            rospy.loginfo('finished')
            return
        else:
            rospy.loginfo('Unknown else')
            return

        rate.sleep()

    print('end')

if __name__ == "__main__":
    main()