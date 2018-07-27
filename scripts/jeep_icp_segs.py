#!/usr/bin/env python

# TODO
# don't pair with ref walls that cannot be seen
# FIGURE OUT WHY THE ODOM GOES TO THE RIGHT NEAR THE SHED! TRY OTHER BAG FILES. TRY LIVE

import rospy, math
from math import sin, cos
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
        self.length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
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
            self.length = 0.
#end class MapRef

def get_angle(pt1, pt2):
    return math.atan2(pt2[1]-pt1[1], pt2[0]-pt1[0])

class LineSeg():
    def __init__(self, nPts, x1, y1, x2, y2):
        xA = x1
        yA = y1
        xB = x2
        yB = y2
        self.angle = get_angle((x1,y1),(x2,y2))
        if(abs(sin(self.angle)) < 0.707):
            self.line_type = 'horz'
            if(x1 > x2):
                xA = x2
                yA = y2
                xB = x1
                yB = y1
            #end swap
            if(abs(yB-yA) < 0.5):
                avg = (yA+yB)/2
                yA = avg
                yB = avg
        else:
            self.line_type = 'vert'
            if(y1 > y2):
                xA = x2
                yA = y2
                xB = x1
                yB = y1
            #end swap
            if(abs(xB-xA) < 0.5):
                avg = (xA+xB)/2
                xA = avg
                xB = avg
        #end horz/vert
        self.x1 = xA
        self.y1 = yA
        self.x2 = xB
        self.y2 = yB
        self.nPts = nPts
        self.length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    def get_ref_dist_sqd(self,ref):
        dist_sqd = 100.**2
        if(ref.ref_type == self.line_type):
            dist_sqd = (self.x2-ref.x2)**2 + (self.y2-ref.y2)**2
        return dist_sqd
    def get_trafo(self,ref):
        left_list = []
        right_list = []
        if(abs(ref.length - self.length)/self.length < 0.1):
            print "SEG AND REF LENGTH MATCH"
            left_list.append( (self.x1, self.y1) )
            left_list.append( (self.x2, self.y2) )
            right_list.append( (ref.x1, ref.y1) )
            right_list.append( (ref.x2, ref.y2) )
        elif(self.length > ref.length):
            print "SEG LENGTH > REF LENGTH"
            distAsqd = (self.x1-ref.x1)**2 + (self.y1-ref.y1)**2
            distBsqd = (self.x2-ref.x2)**2 + (self.y2-ref.y2)**2
            if(distAsqd < distBsqd):
                segX = self.x1 + ref.length*cos(self.angle)
                segY = self.y1 + ref.length*sin(self.angle)
                left_list.append( (self.x1, self.y1) )
                left_list.append( (segX, segY) )
            else:
                segX = self.x2 - ref.length*cos(self.angle)
                segY = self.y2 - ref.length*sin(self.angle)
                left_list.append( (segX, segY) )
                left_list.append( (self.x2, self.y2) )
            right_list.append( (ref.x1, ref.y1) )
            right_list.append( (ref.x2, ref.y2) )
        else:
            print "SEG LENGTH < REF LENGTH"
            left_list.append( (self.x1, self.y1) )
            left_list.append( (self.x2, self.y2) )
            if(self.line_type == 'horz'):
                if(self.x1 < ref.x1):
                    refX = ref.x1 + self.length
                    right_list.append( (ref.x1, ref.y1) )
                    right_list.append( (refX, ref.y2) )
                elif(self.x2 > ref.x2):
                    refX = ref.x2 - self.length #make this more general by using ref.angle
                    right_list.append((refX, ref.y1))
                    right_list.append((ref.x2, ref.y2))
                else:
                    right_list.append( (self.x1, ref.y1) )
                    right_list.append( (self.x2, ref.y2) )
            if(self.line_type == 'vert'):
                if(self.y1 < ref.y1):
                    refY = ref.y1 + self.length
                    right_list.append( (ref.x1, ref.y1) )
                    right_list.append( (ref.x2, refY) )
                elif(self.y2 > ref.y2):
                    refY = ref.y2 - self.length #make this more general by using ref.angle
                    right_list.append((ref.x1, refY))
                    right_list.append((ref.x2, ref.y2))
                else:
                    right_list.append( (ref.x1, self.y1) )
                    right_list.append( (ref.x2, self.y2) )
        print left_list[0], "<>", right_list[0]
        print left_list[1], "<>", right_list[1]
        return estimate_transform(left_list, right_list, fix_scale = True)
            

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
        
        self.landmarks.append(MapRef('horz',2.3,8.7,6.5,8.7)) # deck south
        self.landmarks.append(MapRef('horz',8.8,13.2,13.9,13.2)) # house SE
        
        self.landmarks.append(MapRef('horz',0.,-3.,-12.7,-3.)) # garage
        self.landmarks.append(MapRef('vert',-12.7,-3.,-12.7,6.05))
        self.landmarks.append(MapRef('horz',-12.7,6.05,0.0,6.05))
        self.landmarks.append(MapRef('vert',0.0,6.05,0.0,-3.0))
        
        self.landmarks.append(MapRef('horz',13.55,-24.,10.2,-24.)) # shed
        self.landmarks.append(MapRef('vert',10.2,-24.,10.2,-20.))
        self.landmarks.append(MapRef('horz',10.2,-20.,13.55,-20.))
        self.landmarks.append(MapRef('vert',13.55,-20.,13.55,-24.))
        
        self.landmarks.append(MapRef('point',2.5, -3.)) # garage post
        self.landmarks.append(MapRef('point',10.8, -5.5)) # four trees
        self.landmarks.append(MapRef('point',18.1, -5.3)) #18 or 19?
        self.landmarks.append(MapRef('point',18.2, -11.7))
        self.landmarks.append(MapRef('point',18.3, 3.8))
        
        self.nLandmarks = len(self.landmarks)
        
        rospy.init_node('jeep_icp')
        #self.odom_pub = rospy.Publisher('odom_icp', Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('/pc2', PointCloud2, self.cloud_callback, queue_size = 1)
        #self.cloud_pub = rospy.Publisher('cloud_map', PointCloud2, queue_size=1)
        
        self.odom_x = 6.0 #6.0
        self.odom_y = 1. #-8.0
        self.odom_theta = 0.03 #Try -0.03 w.r.t. map alignment (landmarks made square above)
        
        self.min_point_pairs = 7
        self.max_correlation_dist = 2.0 #meters, max correlation dist for pairing lidar points with known map points
        self.FOUND_DIST_THRESH = 0.1 #meters, stop searching for a better landmark match if distance is less than this
        self.min_scan_elev = -0.65 #meters, points below this height will be ignored (e.g. ground readings maybe)
        self.MAX_DELTA_THETA_RAD = 0.2 #radians, ignore icp transformations with abs(sin(theta)) > this
        self.MAX_DELTA_X = 3.0 #meters, ignore icp transformations with abs(delta x) > this
        self.MAX_DELTA_Y = 3.0 #meters
        self.alpha = 0.2 # complementary filter: odom x,y,theta = odom*(1-alpha) + icp_odom*(alpha)
        
        # line segment find params, NOTE current issue transitioning from point 59 to 0 (need to overlap and merge maybe)
        #    ALSO, you are not re-using the corner point. Shed west wall counted as line to NW corner, but North wall does not start with NW corner
        self.angle_eps_long_rad = 15.0*3.14/180.
        self.angle_eps_short_rad = 35.0*3.14/180. #allowed if consec point dist is < short_gap
        self.short_gap = 3.5
        self.min_nSegPts = 4
        self.max_seg_correlation_dist = 15.
        self.prev_trafo_seg_type = 'horz'
        
        self.segMAX_DELTA_THETA_RAD = 0.4 #radians, ignore icp transformations with abs(sin(theta)) > this
        self.segMAX_DELTA_X = 5.0 #meters, ignore icp transformations with abs(delta x) > this
        self.segMAX_DELTA_Y = 5.0 #meters
        self.seg_alpha = 0.2
        
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
                    if(pt[1] < ref.y1 and 0):
                        ref_pt = (ref.x1, ref.y1)
                    elif(pt[1] > ref.y2 and 0):
                        ref_pt = (ref.x1, ref.y2)
                    else:
                        ref_pt = (ref.x1, pt[1])
        elif(ref.ref_type == 'horz'):
            dist = abs(pt[1] - ref.y1)
            if(dist < eps):
                if(ref.x1 - eps < pt[0] and pt[0] < ref.x2 + eps):
                    if(pt[0] < ref.x1 and 0):
                        ref_pt = (ref.x1, ref.y1)
                    elif(pt[0] > ref.x2 and 0):
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
        
    def get_abs_angle_diff(self, ang1, ang2):
        angle_diff = ang1 - ang2
        if(angle_diff >= math.pi):
            angle_diff = angle_diff - 2*math.pi
        elif(angle_diff < -math.pi):
            angle_diff = angle_diff + 2*math.pi
        return abs(angle_diff)
        
    def find_match(self, rk, pt_x, pt_y):
        dist = 100.
        r_count = 0
        match_found = False
        best_rk = rk
        best_ref_pt = None
        best_ref = None
        
        # Loop thru landmarks and find closest landmark point to current point
        ref_type = None
        while(r_count < self.nLandmarks):
            ref = self.landmarks[rk]
            pot_dist, ref_pt = self.get_dist(ref,(pt_x, pt_y), self.max_correlation_dist )
            if(pot_dist < dist and not(ref_pt==None) ):
                dist = pot_dist
                best_rk = rk
                match_found = True
                best_ref_pt = ref_pt
                best_ref = ref
                
                #if(dist < self.FOUND_DIST_THRESH): # BAD BECAUSE THE EAST VS. WEST SHED WALL IS CLOSE
                #    break
            rk = (rk+1)%(self.nLandmarks)
            r_count += 1
        return best_rk, best_ref_pt, best_ref
        
    def find_seg_match(self, rk, seg):
        dist_sqd = 100.**2
        r_count = 0
        match_found = False
        best_rk = rk
        best_ref_pt = None
        best_ref = None
        
        # Loop thru landmarks and find closest landmark point to seg
        ref_type = None
        while(r_count < self.nLandmarks):
            ref = self.landmarks[rk]
            pot_dist_sqd = seg.get_ref_dist_sqd(ref)
            if(pot_dist_sqd < dist_sqd ):
                dist_sqd = pot_dist_sqd
                best_rk = rk
                match_found = True
                best_ref = ref
                
                #if(dist < self.FOUND_DIST_THRESH): # BAD BECAUSE THE EAST VS. WEST SHED WALL IS CLOSE
                #    break
            rk = (rk+1)%(self.nLandmarks)
            r_count += 1
        return best_rk, best_ref
        
    def cloud_callback(self, msg):
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
            print "\nNEW POINT CLOUD"
            rk = 0
            best_rk = 0 #unused, right?
            left_list  = []
            right_list = []
            ref_type_list = []
            nc = 0
            numPoints = 0
            trafo = None
            
            prev_x, prev_y = 0.,0. # THESE DUPLICATE POINTS MOSTLY COME FROM READINGS AT LOCAL LASER COORDINATE (0,0)
            delta_x_sum = 0.
            delta_y_sum = 0.
            delta_x_count = 0.
            delta_y_count = 0.
            
            line_segs = []
            line_seg_pts = []
            nSegPts = 0
            nLineSegs = 0
            seg_rk = 0
            trafo_seg_horz = None
            trafo_seg_vert = None
            
            #for point_map in pcl2.read_points(pc_map, skip_nans=True):
            # for points in both (map frame) and (in laser frame directly from scan msg)
            for k, (point_map, point_laser) in enumerate(zip(pcl2.read_points(pc_map, skip_nans=True), pcl2.read_points(msg, skip_nans=True))):                    
                corr_found = False
                range_sqd = point_laser[0]**2 + point_laser[1]**2 # only if you loop thru local laser points
                if( (point_map[2] > self.min_scan_elev) and (range_sqd > 0.25 and range_sqd < 625.0 ) ):
                    numPoints +=1
                    pt_x = point_map[0]
                    pt_y = point_map[1]
                    
                    # skip duplicate points, close points
                    if(numPoints > 1):
                        dist_sqd = (pt_x-prev_x)**2 + (pt_y-prev_y)**2
                        if(dist_sqd < 0.01):
                            #print "duplicate point ", pt_x, ", ", pt_y
                            continue
                    prev_x = pt_x
                    prev_y = pt_y
                    
                    # find line segments
                    if(nSegPts==0):
                        segPtA = point_map
                        segPt1 = point_map
                        line_seg_pts.append(segPtA)
                        ang_cur = 0.
                        nSegPts = 1
                    else:
                        segPtB = point_map
                        ang = get_angle(segPt1,segPtB)
                        sub_seg_dist_sqd = (segPtB[0]-segPt1[0])**2 + (segPtB[1]-segPt1[1])**2
                        angA = get_angle(segPtA,segPtB)
                            #or (nSegPts > 2 and abs(angA-ang_cur) < self.angle_eps_rad) # too easy for corner transition to blend with prev seg
                        if(sub_seg_dist_sqd < self.short_gap**2):
                            angle_eps = self.angle_eps_short_rad
                        else:
                            angle_eps = self.angle_eps_long_rad
                        #end if
                        
                        # temp debug
                        #~ print "DEBUG LINE SEGMENTS"
                        #~ print "pt1: ", segPt1
                        #~ print "ptB: ", segPtB
                        #~ print "ang: ", ang, ", sub_dist: ", math.sqrt(sub_seg_dist_sqd)
                        #~ print "ang diff: ", self.get_abs_angle_diff(ang,ang_cur)
                        #~ print "END DEBUG LINE SEGMENTS"
                        
                        if( (nSegPts == 1 or self.get_abs_angle_diff(ang,ang_cur) < angle_eps  )
                           and sub_seg_dist_sqd < 8.**2): # and segPtB[3]-segPt1[3] <= 2 # not available with bag data from scanse
                            line_seg_pts.append(segPtB)
                            nSegPts += 1
                            #~ print "SEG Pt PASSED"
                            if(nSegPts == 2):
                                ang_cur = ang # ang_cur = ang_cur*(1-alpha) + ang*alpha #only if nSegPts > 2
                            #end if
                        else:
                            net_angle = get_angle(segPtA,segPt1)
                            if(nSegPts >= self.min_nSegPts and (abs(cos(net_angle)) < 0.2 or abs(sin(net_angle)) < 0.2) ):
                                nLineSegs += 1
                                line_segs.append(LineSeg(nSegPts, segPtA[0], segPtA[1], segPt1[0],segPt1[1]) )
                                print "FOUND LINE SEGMENT, nSegPts: ", nSegPts, "angle: ", get_angle(segPtA,segPt1)
                                print line_segs[-1].x1, line_segs[-1].y1
                                print line_segs[-1].x2, line_segs[-1].y2 
                                for pk in range(nSegPts):
                                    print line_seg_pts[pk]
                                #end print line seg pts
                                
                                # Find best ref line seg
                                seg_rk, ref = self.find_seg_match(seg_rk,line_segs[-1])
                                print "ref pts (modified based on seg length): ", ref.x1, ref.y1, ref.x2, ref.y2
                                if(line_segs[-1].line_type == 'horz'):
                                    trafo_seg_horz = line_segs[-1].get_trafo(ref)
                                elif(line_segs[-1].line_type == 'vert'):
                                    trafo_seg_vert = line_segs[-1].get_trafo(ref)
                                
                            #end if enough segPts for valid line segment
                            segPtA = segPt1
                            ang_cur = ang
                            line_seg_pts = []
                            if(sub_seg_dist_sqd < 8.**2):
                                nSegPts = 2
                                line_seg_pts.append(segPtA)
                                line_seg_pts.append(segPtB)
                            else:
                                nSegPts = 1
                                segPtA = segPtB
                                line_seg_pts.append(segPtA)
                            #end if
                        #end if
                        segPt1 = segPtB
                    #end if nSegPs == 0, else

                    #pt_z = point_map[2]
                    
                    # point to point matching for icp
                    rk, ref_pt, ref = self.find_match(rk, pt_x, pt_y)
                    if(not ref == None):
                        left_list.append((pt_x,pt_y))
                        right_list.append(ref_pt)
                        ref_type_list.append(ref.ref_type)
                    #
                            
                #end if valid point
            #end loop thru all points
            
            if(nLineSegs == 0):        
                print "numPoints: ", numPoints
                print "lx, ly, rx, ry"
                #for l,r in zip(left_list,right_list):
                #    print l[0], ',', l[1], ',', r[0], ',', r[1]
                if(len(left_list) >= self.min_point_pairs):
                    trafo = estimate_transform(left_list, right_list, fix_scale = True)
                    cur_dist_sum = 0.
                    new_dist_sum = 0.
                    if(not trafo == None):
                        cur_dist_sum, new_dist_sum = self.trafo_eval(trafo, left_list, right_list)
                        if(self.first_scan):
                            self.first_scan = False
                            self.prev_eval_dist_sum = cur_dist_sum
                        #end if
                    #end if
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
                        #end if
                    #end if
                #end if enough point pairs
            else: #end if not line seg trafo
                if(trafo_seg_horz and trafo_seg_vert):
                    if(self.prev_trafo_seg_type == 'vert'):
                        print "horz seg trafo:"
                        trafo = trafo_seg_horz
                        self.prev_trafo_seg_type = 'horz'
                    else:
                        print "vert seg trafo:"
                        trafo = trafo_seg_vert
                        self.prev_trafo_seg_type = 'vert'
                    # end horz vert trafo alternate
                elif(trafo_seg_horz):
                    print "horz seg trafo:"
                    trafo = trafo_seg_horz
                    self.prev_trafo_seg_type = 'horz'
                elif(trafo_seg_vert):
                    print "vert seg trafo:"
                    trafo = trafo_seg_vert
                    self.prev_trafo_seg_type = 'vert'
                #end if both trafo_seg avaialable vs. one
                if(trafo):
                    print trafo
                    if(abs(trafo[2]) < self.segMAX_DELTA_THETA_RAD and abs(trafo[3]) < self.segMAX_DELTA_X and abs(trafo[4]) < self.segMAX_DELTA_Y ):
                        print "VALID TRAFO"
                        la, c, s, dx, dy = trafo
            #end if trafo vs. trafo_seg
                            
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        except Exception as e:
            nLineSegs = 0
            print "tf issue"
            print repr(e)
            pass
        #end try, except
        print c,s,dx,dy
        
        #Complementary, low pass, filter: filt = filt*(1-alpha) + raw*(alpha)
        if(nLineSegs > 0):
            alpha = self.seg_alpha
        else:
            alpha = self.alpha
        
        x = self.odom_x
        y = self.odom_y
        raw_x = c*x - s*y + dx
        self.odom_x = x*(1.-alpha) + raw_x*(alpha)
        raw_y = s*x + c*y + dy
        self.odom_y = y*(1.-alpha) + raw_y*(alpha)
        self.odom_theta = self.odom_theta*(1.-alpha) + math.atan2(s,c)*(alpha) 
        
        self.update_pose()
    #end def cloud_callback
        
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
        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            jeep_icp.update_pose()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
