#!/usr/bin/env python

import sys
import rospy
from ros_service_assignment.srv import RectangleAeraService
from ros_service_assignment.srv import RectangleAeraServiceRequest
from ros_service_assignment.srv import RectangleAeraServiceResponse

def rectangle_aera_client(width, height):
    rospy.wait_for_service('rectangle_aera')
    try:
        rectangle_aera = rospy.ServiceProxy('rectangle_aera', RectangleAeraService)
        resp1 = rectangle_aera(width, height)
        return resp1.area
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [width height]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = float(sys.argv[1])
        height = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s*%s"%(width, height)
    area = rectangle_aera_client(width, height)
    print "%s * %s = %s"%(width, height, area)
