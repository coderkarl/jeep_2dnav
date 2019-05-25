#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import time

class MowPath():
    def __init__(self):
        rospy.init_node('mow_path')
        time.sleep(0.5)
        #rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=2)
        
        self.nav_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        #/move_base_simple/goal geometry_msgs/PoseStamped
        #'{ header: {stamp: now, frame_id: "map"}, pose: { position: {x: 10.0, y: 30.0, z: 0.0}, orientation: {w: 1.0}}}'

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(1.0)
        rospy.loginfo("Mow Path Initialized")
        
        self.init_x = 6.0
        self.init_y = 1.0
        
        self.mx1 = 4.0 #2.0
        self.my1 = -16.0 #-8.0
        
        self.mx2 = -12.0 #-13.0
        self.my2 = -22.0 #-16.0
        
        self.wpx = self.init_x
        self.wpy = self.init_y
        
        self.state = 'init'
        self.near_thresh = 1.5
        
        next_wp = PoseStamped()
        next_wp.header.stamp = rospy.Time.now()
        next_wp.header.frame_id = "map"
        next_wp.pose.position.x = self.wpx
        next_wp.pose.position.y = self.wpy
        next_wp.pose.position.z = 0.0
        next_wp.pose.orientation.w = 1.0
        self.next_wp = next_wp
        
        self.nav_goal_pub.publish(next_wp)
        rospy.loginfo("Initial waypoint published")
        
    
    def near_wp(self, x, y, thresh = 0.3):
        dist_sqd = (x-self.wpx)**2 + (y-self.wpy)**2
        return dist_sqd < thresh**2
        
    def near_end(self, x, y, thresh = 1.0):
        dist_sqd = (x-self.mx2)**2 + (y-self.my2)**2
        return dist_sqd < thresh**2
        
    def x_near1(self,x,thresh = 0.3):
        return abs(x-self.mx1) < thresh
        
    def x_near2(self,x,thresh = 0.3):
        return abs(x-self.mx2) < thresh
        
    def pub_waypoint(self):
        self.next_wp.pose.position.x = self.wpx
        self.next_wp.pose.position.y = self.wpy
        self.next_wp.header.stamp = rospy.Time.now()
        self.nav_goal_pub.publish(self.next_wp)
        rospy.loginfo("Next waypoint published")
        
    def update_waypoint(self):
        try:
            self.transform = self.tf_buffer.lookup_transform("map","laser", rospy.Time())
            #self.transform = self.tf_buffer.waitForTransform("map","laser", msg.header.stamp)
            
            tx = self.transform.transform.translation.x
            ty = self.transform.transform.translation.y
            quat = self.transform.transform.rotation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            (roll, pitch, yaw) = euler_from_quaternion(quat_list)
            #print "bot x,y,theta:", tx, ty, yaw
            
            if self.near_wp(tx,ty, self.near_thresh) and (not self.state == 'done'):
                if(self.state == 'init'):
                    self.wpx = self.mx1
                    self.wpy = self.my1
                    self.state = 'mowing_rev'
                elif(self.state == 'mowing_rev'):
                    self.wpx = self.mx2
                    self.wpy = ty - 0.5
                    self.state = 'mowing_fwd'
                elif(self.state == 'mowing_fwd'):
                    if self.near_end(tx,ty,2.0):
                        self.wpx = self.init_x
                        self.wpy = self.init_y
                        self.state = 'done'
                    else:
                        self.wpx = self.mx1
                        self.wpy = ty - 0.5
                        self.state = 'mowing_rev'
                        
                
                self.pub_waypoint()
                rospy.loginfo("Mow State: %s",self.state)
                    
        except Exception as e:
            print "tf issue"
            print repr(e)
            pass

if __name__ == '__main__':
    try:
        mow_path = MowPath()
        rospy.loginfo("Starting Mow Path")
        #rospy.spin()
        r = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            mow_path.update_waypoint()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
