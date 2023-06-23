#!/usr/bin/env python

import sys
import rospy
from control.srv import pidInitiate 
from control.msg import truster

def initiate_pid(p, i, d, set):
    rospy.wait_for_service('Initiate_PID')
    try:
        pid_req = rospy.ServiceProxy('Initiate_PID', pidInitiate)
        resp1 = pid_req(p, i, d, set)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [P I D Target]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:    
        w = float(sys.argv[1])
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        z = float(sys.argv[4])
        
    else:
        print(usage())
        sys.exit(1)
    # print("Requesting %s+%s"%(x, y))
    # print("%s + %s = %s"%(x, y, initiate_pid(x, y)))
    print('pid result = %.2f'%initiate_pid(w, x, y, z))
    # print(f'pid result = {initiate_pid(w, x, y, z)}')