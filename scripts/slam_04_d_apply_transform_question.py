# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, use this transform to correct the pose.
# 04_d_apply_transform
# Claus Brenner, 14 NOV 2012

from math import sqrt, atan2

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Insert your previous solution here.
    i=0
    for c in cylinders:
        # Find closest cylinder in reference cylinder set.
        best_dist_2 = max_radius * max_radius
        found = False
        j=0
        for ref in reference_cylinders:
            dx, dy = ref[0] - c[0], ref[1] - c[1]
            dist_2 = dx * dx + dy * dy
            if dist_2 < best_dist_2:
                best_dist_2 = dist_2
                best_ind = j
                found = True
            j += 1
        if(found):
            cylinder_pairs.append((i,best_ind))
        i += 1
    return cylinder_pairs

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)

    if(len(left_list) > 1):
        try:
            sum_lp_sqd = 0
            sum_rp_sqd = 0
            sum_rpx_lpx = 0
            sum_rpy_lpy = 0
            sum_rpx_lpy = 0
            sum_rpy_lpx = 0
            
            for k in range(len(left_list)):
                L = left_list[k]
                R = right_list[k]
                sum_lp_sqd += (L[0]-lc[0])**2+(L[1]-lc[1])**2
                sum_rp_sqd += (R[0]-rc[0])**2+(R[1]-rc[1])**2
                sum_rpx_lpx += (R[0]-rc[0])*(L[0]-lc[0])
                sum_rpy_lpy += (R[1]-rc[1])*(L[1]-lc[1])
                sum_rpx_lpy += (R[0]-rc[0])*(L[1]-lc[1])
                sum_rpy_lpx += (R[1]-rc[1])*(L[0]-lc[0])
            if(fix_scale):
                la = 1
            else:
                la = sqrt(sum_rp_sqd/sum_lp_sqd)

            c_term = sum_rpx_lpx + sum_rpy_lpy
            s_term = -sum_rpx_lpy + sum_rpy_lpx
            mag = sqrt(c_term*c_term + s_term*s_term)
            c = c_term/mag
            s = s_term/mag
            
            tx = rc[0]-la*(c*lc[0]-s*lc[1])
            ty = rc[1]-la*(s*lc[0]+c*lc[1])

            #~ if(abs(tx) > 400 or abs(ty) > 400 or abs(s) > 20*3.14/180):
                #~ c = 1
                #~ s = 0
                #~ tx = 0
                #~ ty = 0
            
            return la, c, s, tx, ty
        except:
            print 'error'
            return None 
    else:
        return None

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    
    # --->>> This is what you'll have to implement.
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * pose[0] - las * pose[1] + tx
    y = las * pose[0] + lac * pose[1] + ty
    th = pose[2] + atan2(las,lac)

    return (x, y, th)  # Replace this by the corrected pose.

# Returns a new similarity transform, which is the concatenation of
# transform a and b, "a after b".
# The transform is described in the form of:
# (scale, cos(angle), sin(angle), translate_x, translate_y)
# i.e., the angle is described by a direction vector.
def concatenate_transform(a, b):
    laa, ca, sa, txa, tya = a
    lab, cb, sb, txb, tyb = b

    # New lambda is multiplication.
    la = laa * lab

    # New rotation matrix uses trigonometric angle sum theorems.
    c = ca*cb - sa*sb
    s = sa*cb + ca*sb

    # New translation is a translation plus rotated b translation.
    tx = txa + laa * ca * txb - laa * sa * tyb
    ty = tya + laa * sa * txb + laa * ca * tyb

    return (la, c, s, tx, ty)
