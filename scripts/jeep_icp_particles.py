#!/usr/bin/env python

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist, TransformStamped
#from wheele_msgs.msg import SpeedCurve
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from sensor_msgs.msg import PointCloud2 #LaserScan
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import sensor_msgs.point_cloud2 as pcl2

from slam_04_d_apply_transform_question import\
     estimate_transform, apply_transform#, correct_pose

class MapRef():
    def __init__(self, ref_type, x1, y1, x2=0., y2=0.):
        self.ref_type = ref_type
        if(self.ref_type == 'vert'):
            self.y1 = min(y1,y2)
            self.y2 = max(y1,y2)
            self.x1 = x1
            self.x2 = x2
        elif(self.ref_type == 'horz'):
            self.x1 = min(x1,x2)
            self.x2 = max(x1,x2)
            self.y1 = y1
            self.y2 = y2
        else:
            self.x1 = x1
            self.y1 = y1
            self.x2 = None
            self.y2 = None

class JeepICP():
    def __init__(self):
        self.landmarks = []
        #~ self.landmarks.append(MapRef('horz',0.,0.,-12.7,0.))
        #~ self.landmarks.append(MapRef('vert',-12.7,0.,-12.7,9.05))
        #~ self.landmarks.append(MapRef('horz',-12.7,9.05,0.0,9.05))
        #~ self.landmarks.append(MapRef('vert',0.0,9.05,0.0,0.0))
        
        #~ self.landmarks.append(MapRef('horz',13.55,-21.,10.2,-21.))
        #~ self.landmarks.append(MapRef('vert',10.2,-21.,10.2,-17.))
        #~ self.landmarks.append(MapRef('horz',10.2,-17.,13.55,-17.))
        #~ self.landmarks.append(MapRef('vert',13.55,-17.,13.55,-21.))
        
        self.landmarks.append(MapRef('horz',0.,-3.,-12.7,-3.))
        self.landmarks.append(MapRef('vert',-12.7,-3.,-12.7,6.05))
        self.landmarks.append(MapRef('horz',-12.7,6.05,0.0,6.05))
        self.landmarks.append(MapRef('vert',0.0,6.05,0.0,-3.0))
        
        self.landmarks.append(MapRef('horz',13.55,-24.,10.2,-24.))
        self.landmarks.append(MapRef('vert',10.2,-24.,10.2,-20.))
        self.landmarks.append(MapRef('horz',10.2,-20.,13.55,-20.))
        self.landmarks.append(MapRef('vert',13.55,-20.,13.55,-24.))
        
        self.landmarks.append(MapRef('point',2.5, -3.))
        self.landmarks.append(MapRef('point',10.8, -5.5))
        self.landmarks.append(MapRef('point',19.1, -5.3))
        self.landmarks.append(MapRef('point',19.2, -11.7))
        self.landmarks.append(MapRef('point',19.3, 3.8))
        
        self.nLandmarks = len(self.landmarks)
        
        rospy.init_node('jeep_icp')
        #self.odom_pub = rospy.Publisher('odom_icp', Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('/pc2', PointCloud2, self.cloud_callback, queue_size = 1)
        #self.cloud_pub = rospy.Publisher('cloud_map', PointCloud2, queue_size=1)
        
        self.odom_x = 5.0 #6.0
        self.odom_y = 0. #-8.0
        self.odom_theta = -0.0 #Try -0.03 w.r.t. map alignment (landmarks made square above)
        
        self.min_point_pairs = 4
        self.max_correlation_dist = 1.0 #meters, max correlation dist for pairing lidar points with known map points
        self.FOUND_DIST_THRESH = 0.1 #meters, stop searching for a better landmark match if distance is less than this
        self.min_scan_elev = -1.0 #meters, points below this height will be ignored (e.g. ground readings maybe)
        self.MAX_DELTA_THETA_RAD = 0.1 #radians, ignore icp transformations with abs(sin(theta)) > this
        self.MAX_DELTA_X = 2.0 #meters, ignore icp transformations with abs(delta x) > this
        self.MAX_DELTA_Y = 2.0 #meters
        self.alpha = 0.2 # complementary filter: odom x,y,theta = odom*(1-alpha) + icp_odom*(alpha)
        
        #self.tf_listener = tf.TransformListener()
        self.transform = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.cloud_count = 0
        
        self.prev_eval_dist_sum = -1.
        self.first_scan = True
        
    def get_dist(self, ref, pt, eps = 2.0):
        ref_pt = None
        dist = 100.
        if(ref.ref_type == 'vert'):
            dist = abs(pt[0] - ref.x1)
            if(dist < eps):
                if(ref.y1 - eps < pt[1] and pt[1] < ref.y2 + eps):
                    if(pt[1] < ref.y1):
                        ref_pt = (ref.x1, ref.y1)
                    elif(pt[1] > ref.y2):
                        ref_pt = (ref.x1, ref.y2)
                    else:
                        ref_pt = (ref.x1, pt[1])
        elif(ref.ref_type == 'horz'):
            dist = abs(pt[1] - ref.y1)
            if(dist < eps):
                if(ref.x1 - eps < pt[0] and pt[0] < ref.x2 + eps):
                    if(pt[0] < ref.x1):
                        ref_pt = (ref.x1, ref.y1)
                    elif(pt[0] > ref.x2):
                        ref_pt = (ref.x2, ref.y1)
                    else:
                        ref_pt = (pt[0], ref.y1)
        elif(ref.ref_type == 'point'):
            dist = math.sqrt((pt[0]-ref.x1)**2 + (pt[1]-ref.y1)**2)
            if(dist < eps):
                ref_pt = (ref.x1, ref.y1)
        return dist, ref_pt
    
    def trafo_eval(self, trafo, left_list, right_list):
        cur_dist_sum = 0
        new_dist_sum = 0
        for l,r in zip(left_list,right_list):
            cur_dist = math.sqrt((l[0]-r[0])**2 + (l[1]-r[1])**2)
            cur_dist_sum += cur_dist
            new_l = apply_transform(trafo,(l[0],l[1]))
            new_dist = math.sqrt((new_l[0]-r[0])**2 + (new_l[1]-r[1])**2)
            new_dist_sum += new_dist
        print "cur_sum, new_sum: ", cur_dist_sum, ", ", new_dist_sum
        return cur_dist_sum, new_dist_sum
        
    def cloud_callback(self, msg):
        #self.cloud_count += 1
        #print "Cloud has ", len(msg.data), " points" //len(msg.data) does not give num points in cloud. It is encoded differently.
        if(self.cloud_count <=1): #TEMPORARY JUST TO TESTS MULTIPLE ITERATIONS WITH ONE CLOUD
            for k in range(1): #PROBABLY REMOVE THE FOR LOOP, ITERATIONS FOR ICP
                print "iter ", k
                dx = 0.0
                dy = 0.0
                s = 0.
                c = 1.
                try:
                    # TRANSFORM THE LASER POINTCLOUD FROM THE LASER FRAME TO MAP FRAME...
                    #    LOOKUP THE TRANSFORM AT THE TIME THAT CORRESPONDNS WITH THE POINTCLOUD DATA (msg.header.stamp)
                    #    IF MUTLIPLE ITERATIONS ARE DONE FOR ONE POINTCLOUD MESSAGE, YOU WILL NEED TO USE A DIFFERENT TIME
                    #        IF YOU WANT TO TRY TO USE update_pose() EACH ITERATION
                    #        MAYBE YOU CAN JUST DEFINE self.transform properly using self.odom_x,y,theta
                    #    ANOTHER OPTION MAY BE TO DO THE ITERATIONS AFTER left_list IS FILLED AND JUST TRANSFORM left_list
                    #if(self.transform == None):
                    
                    # not working with bag file. Requires extrapolation into future ( < 50 msec)
                    # tf MessageFilter or waitForTransform?
                    #self.transform = self.tf_buffer.lookup_transform("map","laser", msg.header.stamp) # Works with nav_sim, not bag, not jeep live
                    self.transform = self.tf_buffer.lookup_transform("map","laser", rospy.Time()) # rospy.Time(0) requests latest
                    #self.transform = self.tf_buffer.waitForTransform("map","laser", msg.header.stamp)

                    pc_map = do_transform_cloud(msg, self.transform)
                    #self.cloud_pub.publish(pc_map) #Temporary for validating the transformed point cloud
                    print "NEW POINT CLOUD"
                    r = 0
                    best_r = 0
                    left_list  = []
                    right_list = []
                    ref_type_list = []
                    nc = 0
                    numPoints = 0
                    
                    prev_x, prev_y = 0.,0. # THESE DUPLICATE POINTS MOSTLY COME FROM READINGS AT LOCAL LASER COORDINATE (0,0)
                    delta_x_sum = 0.
                    delta_y_sum = 0.
                    delta_x_count = 0.
                    delta_y_count = 0.
                    #for point_map in pcl2.read_points(pc_map, skip_nans=True):
                    # for points in both (map frame) and (in laser frame directly from scan msg)
                    for point_map, point_laser in zip(pcl2.read_points(pc_map, skip_nans=True), pcl2.read_points(msg, skip_nans=True)):
                        #numPoints += 1
                        corr_found = False
                        range_sqd = point_laser[0]**2 + point_laser[1]**2 # only if you loop thru local laser points
                        
                        if( (point_map[2] > self.min_scan_elev) and (range_sqd > 0.25 and range_sqd < 900.0 ) ):
                            numPoints +=1
                            pt_x = point_map[0]
                            pt_y = point_map[1]
                            if(numPoints > 1):
                                dist_sqd = (pt_x-prev_x)**2 + (pt_y-prev_y)**2
                                if(dist_sqd < 0.3):
                                    print "duplicate point ", pt_x, ", ", pt_y
                                    continue
                            prev_x = pt_x
                            prev_y = pt_y
                            #pt_z = point_map[2]
                            #print pt_x, pt_y
                            dist = 100.
                            r_count = 0
                            
                            # Loop thru landmarks and find closest landmark point to current point
                            ref_type = None
                            while(r_count < self.nLandmarks):
                                ref = self.landmarks[r]
                                pot_dist, ref_pt = self.get_dist(ref,(pt_x, pt_y), self.max_correlation_dist )
                                if(pot_dist < dist and not(ref_pt==None) ):
                                    dist = pot_dist
                                    best_r = r
                                    if(not corr_found):
                                        corr_found = True
                                        left_list.append( (pt_x, pt_y) )
                                        right_list.append(ref_pt)
                                        ref_type = ref.ref_type
                                        ref_type_list.append(ref_type)
                                        nc += 1
                                    else:
                                        right_list[nc-1] = ref_pt
                                        ref_type = ref.ref_type
                                        ref_type_list[nc-1] = ref_type
                                    
                                    #if(dist < self.FOUND_DIST_THRESH): # BAD BECAUSE THE EAST VS. WEST SHED WALL IS CLOSE
                                    #    break
                                r = (r+1)%(self.nLandmarks)
                                r_count += 1
                            r = best_r
                            if(not ref_type == None):
                                if(not ref_type == 'horz'):
                                    delta_x_count += 1
                                    delta_x_sum += right_list[nc-1][0] - pt_x
                                if(not ref_type == 'vert'):
                                    delta_y_count += 1
                                    delta_y_sum += right_list[nc-1][1] - pt_y
                            
                    print "numPoints: ", numPoints
                    print "delta_x_count, delta_x_sum: ", delta_x_count, delta_x_sum
                    print "delta_y_count, delta_y_sum: ", delta_y_count, delta_y_sum
                    print "lx, ly, rx, ry"
                    #~ for pk, ref_type in enumerate(ref_type_list):
                        #~ if(ref_type == 'horz' and delta_x_count > 0):
                            #~ right_list[pk] = (right_list[pk][0]+delta_x_sum/delta_x_count, right_list[pk][1])
                        #~ if(ref_type == 'vert' and delta_y_count > 0):
                            #~ right_list[pk] = (right_list[pk][0], right_list[pk][1]+delta_y_sum/delta_y_count)
                    for l,r in zip(left_list,right_list):
                        print l[0], ',', l[1], ',', r[0], ',', r[1]
                    if(len(left_list) >= self.min_point_pairs):
                        trafo = estimate_transform(left_list, right_list, fix_scale = True)
                        cur_dist_sum = 0.
                        new_dist_sum = 0.
                        if(not trafo == None):
                            cur_dist_sum, new_dist_sum = self.trafo_eval(trafo, left_list, right_list)
                            if(self.first_scan):
                                self.first_scan = False
                                self.prev_eval_dist_sum = cur_dist_sum
                        if( (not trafo == None) and (new_dist_sum < cur_dist_sum+1.0) and (abs(cur_dist_sum - self.prev_eval_dist_sum) < 30.) ):
                            self.prev_eval_dist_sum = cur_dist_sum
                            print "enough points, better trafo"
                            if(abs(trafo[2]) < self.MAX_DELTA_THETA_RAD and abs(trafo[3]) < self.MAX_DELTA_X and abs(trafo[4]) < self.MAX_DELTA_Y ):
                                #print "lx, ly, rx, ry"
                                #for l,r in zip(left_list,right_list):
                                #    print l[0], ',', l[1], ',', r[0], ',', r[1]
                                print "trafo:"
                                print trafo
                                la, c, s, dx, dy = trafo
                        else:
                            print "exit for loop, iter ", k
                            break
                                    
                #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                except Exception as e:
                    print "tf issue"
                    print repr(e)
                    pass
                print c,s,dx,dy
                
                #Complementary, low pass, filter: filt = filt*(1-alpha) + raw*(alpha)
                alpha = self.alpha
                
                x = self.odom_x
                y = self.odom_y
                raw_x = c*x - s*y + dx
                self.odom_x = x*(1.-alpha) + raw_x*(alpha)
                raw_y = s*x + c*y + dy
                self.odom_y = y*(1.-alpha) + raw_y*(alpha)
                self.odom_theta = self.odom_theta*(1.-alpha) + math.atan2(s,c)*(alpha) 
                
                #self.update_pose()   
        
        #~ try:
            #~ points_map = self.tf_listener.transformPointCloud("map",msg.data)
            #~ print points_map
        #~ except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #~ pass
        
    def update_pose(self):
        odom_quat = quaternion_from_euler(0, 0, self.odom_theta)
        self.odom_broadcaster.sendTransform(
        (self.odom_x, self.odom_y, 0.),
        odom_quat,
        rospy.Time.now(),
        "odom",
        "map"
        )
        
        # Update robot pose state from tf listener
        #try:
        #    self.transform = self.tf_buffer.lookup_transform("map","laser", rospy.Time())
            
            
            #self.transform = self.tf_listener.lookupTransform('map', 'laser', rospy.Time(0))
            
            #(trans,quat) = self.tf_listener.lookupTransform('/base_link', '/map', rospy.Time(0))
            #tx = trans[0]
            #ty = trans[1]
            #quat_list = [quat.x, quat.y, quat.z, quat.w]
            #(roll, pitch, yaw) = euler_from_quaternion (quat)
            #theta = yaw
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    pass
        
        #if(self.auto_mode > 1600):
        #self.cmd_pub.publish(spdCrv)

if __name__ == '__main__':
    try:
        jeep_icp = JeepICP()
        rospy.loginfo("Starting Jeep ICP")
        #rospy.spin()
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            jeep_icp.update_pose()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
