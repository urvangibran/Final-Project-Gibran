#!/usr/bin/env python

import sys
import rospy
from control.srv import pidFeedback

def initiate_pid(feedback):
    rospy.wait_for_service('Set_Feedback')
    try:
        pid_req = rospy.ServiceProxy('Set_Feedback', pidFeedback)
        response = pid_req(feedback)
        print(f'PID result = {response.newResult}')
        print(f'Cur = {response.cur}')
        print(f'Integral = {response.itg}')
        print(f'Derivative = {response.drv}')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [Feedback]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:    
        x = float(sys.argv[1])
        initiate_pid(x)
        
    else:
        print(usage())
        sys.exit(1)

    # print(f'pid result = {initiate_pid(x)}')