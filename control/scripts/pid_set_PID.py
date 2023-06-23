#!/usr/bin/env python

import sys
import rospy
from control.srv import pidSet

def initiate_pid(p, i, d):
    rospy.wait_for_service('Set_PID_Value')
    try:
        pid_req = rospy.ServiceProxy('Set_PID_Value', pidSet)
        response = pid_req(p, i, d)
        return response.nResult
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [P I D]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:    
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        
    else:
        print(usage())
        sys.exit(1)
    print('pid result = %.2f'%initiate_pid(x, y, z))