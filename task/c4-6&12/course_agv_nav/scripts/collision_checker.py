#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import numpy
import math

MAX_LASER_RANGE = 30

class Checker():
    def __init__(self):
        self.threshold = 0.1 # use to check collision

        self.path = numpy.zeros([3,0])
        # x y yaw
        self.robot_x = numpy.zeros([3,1])
        # ros topic
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.collision_pub = rospy.Publisher('/collision_checker_result',Bool,queue_size=5)
        self.marker_pub = rospy.Publisher('/collision_marker',Marker,queue_size=1)
	## global real pose TODO replace with slam pose
	self.pose_sub = rospy.Subscriber('/gazebo/course_agv__robot_base',Pose,self.poseCallback)

    def poseCallback(self,msg):
        p = msg.position
        o = msg.orientation
        e = tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])
        self.robot_x = numpy.array([[p.x,p.y,e[2]]]).T

    def pathCallback(self,msg):
        self.path = self.pathToNumpy(msg) # shape (3,pose_num)
        print("get path! shape: ",self.path.shape)

    def laserCallback(self,msg):
        np_msg = self.laserToNumpy(msg)
        obs = self.u2T(self.robot_x).dot(np_msg)
	## TODO update robot global pose
        if self.collision_check(obs):
            self.publish_collision()

    def collision_check(self,obs):
        if self.path.shape[1] == 0:
            return False
	res = False
	## TODO
        return res

    def publish_collision(self):
        res = Bool()
        res.data = True
        self.collision_pub.publish(res)

    def pathToNumpy(self,msg):
        pos = numpy.ones([0,3])
        for i in range(len(msg.poses)):
            p = msg.poses[i].pose.position
            pos = numpy.append(pos,[[p.x,p.y,1]],axis=0)
        return pos.T

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = numpy.ones([3,total_num])
        range_l = numpy.array(msg.ranges)
        range_l[range_l == numpy.inf] = MAX_LASER_RANGE
        angle_l = numpy.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = numpy.vstack((numpy.multiply(numpy.cos(angle_l),range_l),numpy.multiply(numpy.sin(angle_l),range_l)))
        return pc

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = numpy.array([[t[0,2],t[1,2],dw]])
        return u.T
    
    def u2T(self,u):
        w = u[2]
        dx = u[0]
        dy = u[1]

        return numpy.array([
            [ math.cos(w),-math.sin(w), dx],
            [ math.sin(w), math.cos(w), dy],
            [0,0,1]
        ])

def main():
    rospy.init_node('collision_checker_node')
    n = Checker()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
