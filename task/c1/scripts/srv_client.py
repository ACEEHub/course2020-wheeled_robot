#!/usr/bin/env python
import sys
import rospy
from c1.srv import *

def usage():
    return "%s [x,y]"%sys.argv[0]

def add_two_ints_request(x,y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints',AddTwoInts)
        response = add_two_ints(x,y)
        return response.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print("")
    print("%s + %s = %s"%(x, y, add_two_ints_request(x,y)))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass