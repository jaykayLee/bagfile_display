import sys
import numpy as np
import datetime
import time

from gcm import CMConst


def print_data_step_hello():
    print(f'--data_step_python')


# -------------------------------------
# clipping by axis min/max
def process_clipping_by_area(points, cliparea, clipopt, idx_doppler = 9):
    if len(list(filter(lambda x:  not np.isnan(x), cliparea ))) > 0  :
        clips = list(cliparea)
        opts = list(clipopt)
        if np.isnan(clips[0]): clips[0] = -sys.float_info.max;  opts[0] = CMConst.CLIP_INCLUDE
        if np.isnan(clips[1]): clips[1] = sys.float_info.max;   opts[0] = CMConst.CLIP_INCLUDE
        if np.isnan(clips[2]): clips[2] = -sys.float_info.max;  opts[1] = CMConst.CLIP_INCLUDE
        if np.isnan(clips[3]): clips[3] = sys.float_info.max;   opts[1] = CMConst.CLIP_INCLUDE
        if np.isnan(clips[4]): clips[4] = -sys.float_info.max;  opts[2] = CMConst.CLIP_INCLUDE
        if np.isnan(clips[5]): clips[5] = sys.float_info.max;   opts[2] = CMConst.CLIP_INCLUDE
        if len(cliparea)==8 : ## doppler option check..
            if np.isnan(clips[6]): clips[6] = 1;    opts[3] = CMConst.CLIP_EXCLUDE
            if np.isnan(clips[7]): clips[7] = -1;   opts[3] = CMConst.CLIP_EXCLUDE
        if opts[0] == CMConst.CLIP_INCLUDE :
            m1 = np.logical_and( (points[:,0]>clips[0]), (points[:,0]<clips[1]) )
        else :
            m1 = np.logical_or( (points[:,0]<clips[0]), (points[:,0]>clips[1]) )
        if opts[1] == CMConst.CLIP_INCLUDE :
            m2 = np.logical_and( (points[:,1]>clips[2]), (points[:,1]<clips[3]) )
        else :
            m2 = np.logical_or( (points[:,1]<clips[2]), (points[:,1]>clips[3]) )
        if opts[2] == CMConst.CLIP_INCLUDE :
            m3 = np.logical_and( (points[:,2]>clips[4]), (points[:,2]<clips[5]) )
        else :
            m3 = np.logical_or( (points[:,2]<clips[4]), (points[:,2]>clips[5]) )
        if len(cliparea)==8 :
            if opts[3] == CMConst.CLIP_INCLUDE :
                m4 = np.logical_and( (points[:,idx_doppler]>clips[6]), (points[:,idx_doppler]<clips[7]) )
            else :
                m4 = np.logical_or( (points[:,idx_doppler]<clips[6]), (points[:,idx_doppler]>clips[7]) )
            mask = np.logical_and( np.logical_and(m1, m2), np.logical_and(m3, m4))
        else :
            mask = np.logical_and( np.logical_and(m1, m2), m3)
        arr = points[mask]
        return arr
    return points


# # -------------------------------------
# # translate axis position and angle
# def process_axis_translate(points, pos, angle):
#     if len(list(filter(lambda pi : pi > 0.001, [pos[0],pos[1],pos[2], angle[0], angle[1], angle[2]]))) < 0 :
#         return points

#     points[:,0] += pos[0]
#     points[:,1] += pos[1]
#     points[:,2] += pos[2]

#     yaw = angle[0]
#     elev = angle[1]
#     # case 1
#     # new_position = [ [sin(El)*con(Az), sin(Az), cos(El)*con(Az)], [sin(El)*sin(Az), -cos(Az), cos(El)*sin(Az)], [cos(El), 0, -sin(El) ]] @ [x,y,z]
#     tr = [ [np.sin(elev)*np.cos(azim), np.sin(azim), np.cos(elev)*np.cos(azim)]
#         , [np.sin(elev)*np.sin(azim), -np.cos(azim), np.cos(elev)*np.sin(azim)]
#         , [np.cos(elev), 0, -np.sin(elev)] ]
    
    

# -------------------------------------
# generate eular matrix
# https://kr.mathworks.com/matlabcentral/answers/500680-how-to-convert-euler-angle-roll-pitch-yaw-to-position-x-y-z
# case 1
# dcm = zeros(3,3);
# dcm(1,1) = cosd(yaw)*cosd(pitch);
# dcm(1,2) = cosd(yaw)*sind(pitch)*sind(roll) - sind(yaw)*cosd(roll);
# dcm(1,3) = cosd(yaw)*sind(pitch)*cosd(roll) + sind(yaw)*sind(roll);
# dcm(2,1) = sind(yaw)*cosd(pitch);
# dcm(2,2) = sind(yaw)*sind(pitch)*sind(roll) + cosd(yaw)*cosd(roll);
# dcm(2,3) = sind(yaw)*sind(pitch)*cosd(roll) - cosd(yaw)*sind(roll);
# dcm(3,1) = -sind(pitch);
# dcm(3,2) = cosd(pitch)*sind(roll);
# dcm(3,3) = cosd(pitch)*cosd(roll);
# case 2
# dcm = zeros(3,3);
# dcm(1,1) = cosd(pitch)*cosd(yaw);
# dcm(1,2) = cosd(pitch)*sind(yaw);
# dcm(1,3) = -sind(pitch);
# dcm(2,1) = sind(roll)*sind(pitch)*cosd(yaw) - cosd(roll)*sind(yaw);
# dcm(2,2) = sind(roll)*sind(pitch)*sind(yaw) + cosd(roll)*cosd(yaw);
# dcm(2,3) = sind(roll)*cosd(pitch);
# dcm(3,1) = cosd(roll)*sind(pitch)*cosd(yaw) + sind(roll)*sind(yaw);
# dcm(3,2) = cosd(roll)*sind(pitch)*sind(yaw) - sind(roll)*cosd(yaw);
# dcm(3,3) = cosd(roll)*cosd(pitch);

def generate_eular_matrix(angle):
    if len(angle) != 3 :
        return None
    
    yaw = angle[0]*np.pi/180    ;   sinyaw = np.sin(yaw)    ;   cosyaw = np.cos(yaw)
    roll = angle[1]*np.pi/180   ;   sinroll = np.sin(roll)  ;   cosroll = np.cos(roll)
    pitch = angle[2]*np.pi/180  ;   sinpitch = np.sin(pitch);   cospitch = np.cos(pitch)
    # case 1
    tr = np.array([[ (cosyaw*cospitch), (cosyaw*sinpitch*sinroll - sinyaw*cosroll), (cosyaw*sinpitch*cosroll + sinyaw*sinroll) ]
                ,[(sinyaw*cospitch), (sinyaw*sinpitch*sinroll+cosyaw*cosroll), (sinyaw*sinpitch*cosroll - cosyaw*sinroll)]
                ,[(-sinpitch), (cospitch*sinroll), (cospitch*cosroll)]])
    # case 2
    # tr = np.array([[ (cospitch*cosyaw), (cospitch*sinyaw), (-sinpitch) ]
    #             ,[(sinroll*sinpitch*cosyaw-cosroll*sinyaw), (sinroll*sinpitch*sinyaw), (sinroll*cospitch) ]
    #             ,[(cosroll*sinpitch*cosyaw+sinroll*sinyaw), (cosroll*sinpitch*sinyaw-sinroll*cosyaw), (cosroll*cospitch) ]])
    return tr
# -------------------------------------
# translate axis position and angle
def process_axis_translate(points, pos, tr):
    if(points is None or pos is None or tr is None):
        return None  # or points

    points[:,0] += pos[0]
    points[:,1] += pos[1]
    points[:,2] += pos[2]

    npoints = (tr @ points.T).T
    return npoints
