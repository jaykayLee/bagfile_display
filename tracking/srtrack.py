import sys
import datetime
import time
import struct
import os
import string
import typing
import numpy as np
from numpy.linalg import inv

class SRTrkCANFD():
    def __init__(self):
        self.track_state : np.array = np.zeros((64))         # 0 : 미감지 / 1 : 신규 타겟 / 2: 기존 타겟 / 3 : TBD
        self.track_count : np.array = np.zeros((64))         # Count 횟수, 0 ~ 63
        self.track_range : np.array = np.zeros((64))         # 상대 거리
        self.track_angle : np.array = np.zeros((64))         # 상대 각도
        self.track_type : np.array = np.zeros((64))          # 1 : 차량 / 2 : 사람 / 3 : TBD
        self.track_activity : np.array = np.zeros((64))      # 1 : Moving / 2 : Static / 3 : unknown
        self.track_reliability : np.array = np.zeros((64))   # 0 : 미감지 / 1 : 장애물 최초검출 / 2 : 장애물 추적 중 / 3 : 장애물 추적실패 / 4 : 장애물 추적 여러번 실패
        self.track_lost_cnt : np.array = np.zeros((64))      # Target lost count

# class SRFilteredTrk():
#     def __init__(self):
#         self.track_longpos_F : np.array = np.zeros((64))        # Filtered Long pos
#         self.track_latpos_F : np.array = np.zeros((64))         # Filtered Lat pos
#         self.track_longvel_F : np.array = np.zeros((64))        # Filtered Long vel
#         self.track_latvel_F : np.array = np.zeros((64))         # Filtered Lat vel
#         self.track_Headangle_F : np.array = np.zeros((64))          # Filtered Heading angle

class SRTrkTrackHist():
    def __init__(self):
        self.Num : int = 0
        self.track_numpoint : np.array = np.zeros((64,15))
        self.track_longpos : np.array = np.ones((64,15))*255 # LongPos 값은 Default를 큰 값으로 두어야 한다. 
        self.track_latpos : np.array = np.zeros((64,15))
        self.track_longvel : np.array = np.zeros((64,15)) # LongPos 값은 Default를 큰 값으로 두어야 한다. 
        self.track_latvel : np.array = np.zeros((64,15))
        self.track_score : np.array = np.zeros((64,15))
        self.track_width : np.array = np.zeros((64,15))
        self.track_length : np.array = np.zeros((64,15))
        self.track_doppler : np.array = np.zeros((64,15))
        self.track_power : np.array = np.zeros((64,15))
        self.track_type : np.array = np.zeros((64,15))


class SRTrkClusterParam():
    def __init__(self):
        self.pts_type_noise = -1
        self.pts_type_unclassified = 0
        self.pts_type_classified = 1
        self.max_numpts = 5000
        self.num_obj = 0
        self.max_numcluster = 500
        self.USRmeter = 1
        self.LRmeter = 7
        self.minptsUSR = 30
        self.minptsLR = 3 
        self.minpts = 30 
        self.eps = 2.5 
        self.x : np.array = np.zeros((self.max_numpts,1))
        self.y : np.array = np.zeros((self.max_numpts,1))
        self.z : np.array = np.zeros((self.max_numpts,1))
        self.range_bias : np.array = np.zeros((self.max_numpts,1))

class SRTrkRadarPosition():
    def __init__(self):
        self.RD1_xoffset = 0    # unit : m
        self.RD1_yoffset = 0    # unit : m
        self.RD1_zoffset = 0.6  # unit : m
        self.RD1_roll = np.deg2rad(0) # config unit : degree -> radian 
        self.RD1_pitch = np.deg2rad(0) # config unit : degree -> radian 
        self.RD1_yaw = np.deg2rad(0) # config unit : degree -> radian 
        self.RD1_rotM = np.array([ [np.cos(self.RD1_yaw)*np.cos(self.RD1_pitch), \
                                    np.cos(self.RD1_yaw)*np.sin(self.RD1_pitch)*np.sin(self.RD1_roll)-np.sin(self.RD1_yaw)*np.cos(self.RD1_roll), \
                                    np.cos(self.RD1_yaw)*np.sin(self.RD1_pitch)*np.cos(self.RD1_roll)+np.sin(self.RD1_yaw)*np.sin(self.RD1_roll)], \
                                 [np.sin(self.RD1_yaw)*np.cos(self.RD1_pitch), \
                                     np.sin(self.RD1_yaw)*np.sin(self.RD1_pitch)*np.sin(self.RD1_roll)+np.cos(self.RD1_yaw)*np.cos(self.RD1_roll), \
                                     np.sin(self.RD1_yaw)*np.sin(self.RD1_pitch)*np.cos(self.RD1_roll)-np.cos(self.RD1_yaw)*np.sin(self.RD1_roll)], \
                                 [-np.sin(self.RD1_pitch), np.cos(self.RD1_pitch)*np.sin(self.RD1_roll), np.cos(self.RD1_pitch)*np.cos(self.RD1_roll)] ])

class SRTargetType():
    UNKNOWN   : typing.Final[int] = 0
    CAR       : typing.Final[int] = 1
    PED       : typing.Final[int] = 2
    BICYCLE   : typing.Final[int] = 3
    NAME      : typing.Final[list] = ["Unknown", "Car", "Ped", "Bicycle"]
    CHAR      : typing.Final[list] = ["U", "C", "P", "B"]

# SVM_Model_Parameter_12F_Linear_Car_Full(self):
class SVM_Model_Parameter_12F_Linear_Car_Full():
    def __init__(self):
        self.Mu = np.array([0.114277155645637,0.927134734088259,13.0453705200645,1.57806221495339,-0.113180642774643,1.14822659095567,13.7992820915624,1.83152873031853,0.270532959331483,0.524805498404117,0.331477017639048,50785.2009694908])
        self.Sigma = np.array([4.35612298830269,0.878088102259427,9.30797361828219,1.25900485977203,0.463664034058548,0.667211676961562,9.25286515757668,1.47427942177754,0.280955625128986,0.387064312138405,0.196549191383815,246838.079960553])
        self.Beta = np.array([[-0.147010357328438,-0.484325014471537,2.10561255028895,-0.837870347591198,-0.209658154700113,0.432465635595789,-2.39036352418264,0.0508223363375847,-0.179143651008460,-0.449528707366104,-0.190964679826838,-2.08512792506574]]).T
        self.Bias = 0.611043273850989
        self.Scale = 1.215827598973137

class SVM_Model_Parameter_12F_Linear_Ped_Full():
    def __init__(self):
        self.Mu = np.array([-0.0112444916376715,0.631469469359835,13.4046283655607,1.07371895056493,-0.134618948507791,1.03427752933931,14.0153337816098,1.70806412447586,0.183784957078665,0.377061600744284,0.306814572566248,7606.27123026655])
        self.Sigma = np.array([4.09180289496193,0.473846646152721,9.01503971736264,0.493100843805565,0.414864852214100,0.562451013621032,9.02538025800208,1.23428875412659,0.155850660220676,0.158327056910501,0.177604239616399,37397.2390438208])
        self.Beta = np.array([[-0.0531901641043759,-0.651524906452908,-0.695547946562516,1.14502373087903,-0.355150646451635,0.585273818913524,1.21763672678279,1.99151664445059,1.67455993176087,-0.326476422243240,-0.733960659626061,0.241448897957018]]).T
        self.Bias = -0.571637520883690
        self.Scale = 1.659573672844918
    
class Track_Type_Size_Parameters():
    def __init__(self): 
        # Longitudinal direction, Target Heading Angle -> Rotation Matrix should be multiplied (Currently Not used)
        self.Car_Size = np.array([1.9, 4.5, 1.8]) # width, length, height (m)
        self.Truck_Size = np.array([2.0, 7.5, 2.0]) # width, length, height (m)
        self.Bicycle_Size = np.array([1.2, 3.0, 1.8]) # width, length, height (m)
        self.Cyclist_Size = np.array([0.8, 2.5, 1.8]) # width, length, height (m)
        self.PedA_Size = np.array([0.8, 0.8, 1.75]) # width, length, height (m), Adult
        self.PedB_Size = np.array([0.5, 0.5, 0.9]) # width, length, height (m), Baby


class SRTrackUnit():
    def __init__(self):
        self.reset()

    def reset(self):
        # self.trk_num = 0
        self.dt = 0.05 # R-4FTN : 20fps(0.05) / R-4F : 8fps (0.125)
        self.A = np.array([[1,  1,  0,  0],
                           [0,  1,  0,  0],
                           [0,  0,  1,  1],
                           [0,  0,  0,  1]])
        # self.A = np.array([[1, self.dt, 0, 0],
        #                    [0, 1, 0, 0],
        #                    [0, 0, 1, self.dt],
        #                    [0,  0, 0, 1]])
        self.H = np.array([[ 1,  0,  0,  0],
                           [ 0,  0,  1,  0]])
        self.Q = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
                        #    20*np.eye(4)
        self.R = np.array([[1, 0],
                           [0, 1]])

        self.FilteredTrk = np.array([0, 0, 40, 0]) # Lat pos : 0 / Lat vel : 0 / Long pos : 40 / Long vel : 0 / inital value 
        self.P_mat = 20*np.eye(4) # check

class SRSTracker():
    def __init__(self):
        self.unit_list = []
        for i in range(64) :
            self.unit_list.append(SRTrackUnit())
 
        self._test : int = 0
        self.frame_number = 1
        self.num_obj = 0 

        self.CANFD = SRTrkCANFD()
        self.TrackHist = SRTrkTrackHist()
        self.clusterParams = SRTrkClusterParam()
        self.RD1 = SRTrkRadarPosition()
        # self.EKF_Track = SRFilteredTrk()

        ## previous track list 
        self._track_list = []
        self._file_snapshot_name = ""

    def E_Kalman_Filter(self, idx, curr_num):

        meusured_track = np.array([self.TrackHist.track_latpos[idx, curr_num], self.TrackHist.track_longpos[idx, curr_num]])

        # (1) Prediction.
        x_pred = self.unit_list[idx].A @ self.unit_list[idx].FilteredTrk
        P_pred = self.unit_list[idx].A @ self.unit_list[idx].P_mat @ self.unit_list[idx].A.T + self.unit_list[idx].Q 
        
        # (2) Kalman Gain.
        K = P_pred @ self.unit_list[idx].H.T @ inv(self.unit_list[idx].H @ P_pred @ self.unit_list[idx].H.T + self.unit_list[idx].R)

        # (3) Estimation.
        # x_esti = x_pred + K @ (meusured_track - self.unit_list[idx].H @ x_pred)
        self.unit_list[idx].FilteredTrk = x_pred + K @ (meusured_track - self.unit_list[idx].H @ x_pred)
        # (4) Error Covariance.
        # P = P_pred - K @ self.unit_list[idx].H @ P_pred
        self.unit_list[idx].P_mat = P_pred - K @ self.unit_list[idx].H @ P_pred

        # return x_esti, P

    def get_last_tracks(self):
        return self._track_list[len(self._track_list)-1] if len(self._track_list) > 0 else None

    def put_current_tracks(self, trk_list_item) :
        self._track_list.append(trk_list_item)
        if len(self._track_list) > 5 :
            self._track_list.remove(self._track_list[0])

    def process(self, points, idx_doppler, fname=None):
        prev_track = self.get_last_tracks()
        self._file_snapshot_name = fname
        if prev_track is None :
            pass
        else :
            pass

        if self._test == 0 :
            pass
        elif self._test == 1 :
            pass
        pc_polar = np.zeros((np.shape(points)[0],5))
        RD1_rot_points = points[:,0:3] @ self.RD1.RD1_rotM
        # RD1_rot_points = np.zeros((np.shape(points)[0],3))
        # for pi in points :
        #     RD1_rot_points[:] = pi[0:3] @ self.RD1.RD1_rotM
        
        # RD1_pointcloud_x = RD1_rot_points[:,0] + self.RD1.RD1_xoffset
        # RD1_pointcloud_y = RD1_rot_points[:,1] + self.RD1.RD1_yoffset
        # RD1_pointcloud_z = RD1_rot_points[:,2] + self.RD1.RD1_zoffset
        RD1_pointcloud = RD1_rot_points
        RD1_pointcloud[:,0] += self.RD1.RD1_xoffset
        RD1_pointcloud[:,1] += self.RD1.RD1_yoffset
        RD1_pointcloud[:,2] += self.RD1.RD1_zoffset

        RD1_doppler = points[:,idx_doppler]
        RD1_power = points[:,idx_doppler-1]
        # RD1_power = np.log10(points[:,idx_doppler-1])

        # self.clusterParams.x = RD1_rot_points[:,0] + self.RD1.RD1_xoffset
        # self.clusterParams.y = RD1_rot_points[:,1] + self.RD1.RD1_yoffset
        # self.clusterParams.z = RD1_rot_points[:,2] + self.RD1.RD1_zoffset

        mov_tgt_ind = np.where((RD1_doppler < -0.01) | (RD1_doppler > 0.01)) # (abs(RD1_doppler) > 0.005)
        mov_pointcloud = RD1_pointcloud[:][mov_tgt_ind]
        mov_doppler = RD1_doppler[mov_tgt_ind]
        mov_power = RD1_power[mov_tgt_ind]

        clusterID_check, cluster_info = self.PeopleClustering(mov_pointcloud, mov_doppler, prev_track)
        self.TrackID_Decision(clusterID_check, mov_pointcloud, prev_track, cluster_info)
        Mov_numTrk, Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin = self.Track_Classification(clusterID_check, mov_pointcloud, mov_doppler, mov_power, prev_track)
        return Mov_numTrk, Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin


    def PeopleClustering(self, mov_pointcloud, mov_doppler, prev_track):
        numobj = np.size(mov_doppler)
        point_type = np.zeros((numobj))
        clusterID_check = np.zeros((numobj), dtype=int)
        clusterID = 0
        seedset = np.zeros(self.clusterParams.max_numpts, dtype=int)
        cluster_info = np.zeros((32,5))
        cluster_cnt = 0

        if prev_track is None or prev_track[0] is None or prev_track[0] == 0:
            prev_trk_pos = [[0, 255]] # lat / long
            prev_trk_dop = [0]
            prev_trk_size = [[0, 0]]
            prev_ID_chk =[]
            prev_trk_type = [0]
            prev_track_num = 0
        else :
            prev_track_num = prev_track[0]
            prev_trk_pos = np.zeros((len(prev_track[3][0]),2)) # lat / long
            prev_trk_dop = np.zeros(len(prev_track[3][0]))
            prev_trk_size = np.zeros((len(prev_track[3][0]),2))
            prev_ID_chk = np.zeros(len(prev_track[3][0]), dtype=int)
            prev_trk_type = np.zeros(len(prev_track[3][0]))
            for tt in range(len(prev_track[3][0])):
                prev_trk_pos[tt] = prev_track[1][tt][0:2]
                prev_trk_dop[tt] = prev_track[1][tt][4]
                prev_trk_size[tt] = prev_track[1][tt][5:]
                prev_ID_chk[tt] = prev_track[3][0][tt]
                prev_trk_type[tt] = prev_track[2][prev_ID_chk[tt]]  # Future works : gating decision for each type


        for ii in range(numobj):
            if point_type[ii] != self.clusterParams.pts_type_unclassified :
                continue

            min_pts = 5

            number_neighbor, neighbor_set, cl_info = self.RegionQuery(ii, numobj, mov_pointcloud, mov_doppler)

            if (number_neighbor < min_pts):
                point_type[ii] = self.clusterParams.pts_type_noise
                continue

            if prev_track_num == 0:
                clusterID += 1
                cluster_cnt += 1
            else:
                mismatch_flag = 0
                for tt in range(len(prev_track[3][0])):

                    prev_pos_x = prev_trk_pos[tt][0]
                    prev_pos_y = prev_trk_pos[tt][1]
                    prev_dop = prev_trk_dop[tt]

                    if (prev_trk_size[tt][0] > 0.5):
                        gating_x = prev_trk_size[tt][0]
                    else:
                        gating_x = 0.5

                    if (prev_trk_size[tt][1] > 0.5):
                        gating_y = prev_trk_size[tt][1] 
                    else:
                        gating_y = 0.5

                    gating_d = 1.8   # 1.8 mps 

                    if ((abs(cl_info[0] - prev_pos_x) < gating_x) & (abs(cl_info[1] - prev_pos_y) < gating_y) & (abs(cl_info[3] - prev_dop) < gating_d)):
                        clusterID = prev_track[3][0][tt]
                        cluster_cnt += 1
                        mismatch_flag = 0
                        break
                    else:
                        mismatch_flag = 1
                
                if mismatch_flag == 1:
                    clusterID = max(prev_track[3][0][:]) + 1
                    cluster_cnt += 1
                    # if clusterID > 32:
                    #     clusterID = 13 # 1 to 12 ID is main Target ID
                        
            # clusterID += 1
            clusterID_check[ii] = clusterID 
            point_type[ii] = self.clusterParams.pts_type_classified

            for jj in range(number_neighbor):
                seedset[jj] = neighbor_set[jj]
                clusterID_check[(seedset[jj])] = clusterID
                point_type[(seedset[jj])] = self.clusterParams.pts_type_classified

            seedcnt = 0 
            while seedcnt < number_neighbor :
                current_point = seedset[seedcnt]
                nEps, neighbor_set_2nd, cl_info = self.RegionQuery(current_point, numobj, mov_pointcloud, mov_doppler)
             
                if nEps >= min_pts :
                    for kk in range(nEps):
                        neighborID = neighbor_set_2nd[kk]
                        if point_type[neighborID] != self.clusterParams.pts_type_classified :
                            if point_type[neighborID] == self.clusterParams.pts_type_unclassified :
                                number_neighbor += 1
                                seedset[number_neighbor] = neighborID
                            clusterID_check[neighborID] = clusterID
                            point_type[neighborID] = self.clusterParams.pts_type_classified 
                seedcnt += 1
            cluster_info[cluster_cnt,:] = np.append( clusterID, cl_info)

            if clusterID >= self.clusterParams.max_numpts :
                break
    
        return clusterID_check, cluster_info

    # def TrackID_Decision(self, clusterID_check, prev_track):

    #     if prev_track is None or prev_track[0] is None or prev_track[0]==0:
    #         prev_trk_pos = [[0, 255]] # lat / long
    #         prev_trk_dop = [0]
    #         prev_trk_size = [[0, 0]]
    #         prev_ID_chk =[]
    #         prev_trk_type = [0]
    #     else :
    #         prev_trk_pos = np.zeros((len(prev_track[3][0]),2)) # lat / long
    #         prev_trk_dop = np.zeros(len(prev_track[3][0]))
    #         prev_trk_size = np.zeros((len(prev_track[3][0]),2))
    #         prev_ID_chk = np.zeros(len(prev_track[3][0]), dtype=int)
    #         prev_trk_type = np.zeros(len(prev_track[3][0]))
    #         for tt in range(len(prev_track[3][0])):
    #             prev_trk_pos[tt] = prev_track[1][tt][0:2]
    #             prev_trk_dop[tt] = prev_track[1][tt][4]
    #             prev_trk_size[tt] = prev_track[1][tt][5:]
    #             prev_ID_chk[tt] = prev_track[3][0][tt]
    #             prev_trk_type[tt] = prev_track[2][prev_ID_chk[tt]]  # Future works : gating decision for each type
                
    #     gating_x = prev_trk_size[0][0]
    #     gating_y = prev_trk_size[0][1] 
    #     gating_d = 0.5   # 1.8 kph 


    #     region_x = np.average(mov_pointcloud[neighbor_set[:number_neighbor],0])
    #     region_y = np.average(mov_pointcloud[neighbor_set[:number_neighbor],1])
    #     region_d = np.average(mov_doppler[neighbor_set[:number_neighbor]])

    #     if ((abs(prev_trk_pos[tt][0] - region_x) < gating_x) & (abs(prev_trk_pos[tt][1] - region_y) < gating_y) & (abs(prev_trk_dop[tt] - region_d) < gating_d)):
    #         clusterID = prev_ID_chk[tt]
    #         clusterID_check[ii] = clusterID 
    #         point_type[ii] = self.clusterParams.pts_type_classified
    #         if (number_neighbor < min_pts):
    #             clusterID = prev_ID_chk[tt]
    #             clusterID_check[ii] = clusterID 
    #             point_type[ii] = self.clusterParams.pts_type_classified
    #         else:
    #             point_type[ii] = self.clusterParams.pts_type_noise
    #             continue
    #     else:
    #         point_type[ii] = self.clusterParams.pts_type_noise
    #         continue

    def RegionQuery(self, obj_idx, numobj, mov_pointcloud, mov_doppler):
        if True :
            # m1 = np.logical_and( (points[:,0]>clips[0]), (points[:,0]<clips[1]) )
            pcs = np.copy(mov_pointcloud)
            pcs_cal = np.zeros(mov_pointcloud.shape)
            dops = np.copy(mov_doppler)
            dops_cal = np.zeros(mov_doppler.shape)
            dists = np.zeros((len(mov_pointcloud)))
            position = pcs[obj_idx]
            doppler = dops[obj_idx]
            pcs_cal[:] = np.abs(pcs[:] - position)
            dops_cal[:] = np.abs(dops[:] - doppler)
            ## ? if add x,y,z filter by self.clusterParams.eps, more performance or array iteration overhead
            dists[:] = np.sqrt(pcs_cal[:,0]**2 + pcs_cal[:,1]**2 + pcs_cal[:,2]**2)
            mask = np.logical_and(dists<=self.clusterParams.eps, dops_cal < 1.8) # 1.2
            dop_arr = dops[mask]
            dop_avg = np.average(dop_arr)
            pcs_arr = pcs[mask]
            center_x = np.average(pcs_arr[:,0])
            center_y = np.average(pcs_arr[:,1])
            center_z = np.average(pcs_arr[:,2])
            neighborset = np.where(mask)[0]
            n_pts = len(neighborset)
            return n_pts, neighborset, (center_x, center_y, center_z, dop_avg)
        else :
            neighborset = np.zeros((self.clusterParams.max_numpts), dtype=int)

            n_pts = 0

            for ii in range(numobj):
                if obj_idx == ii :
                    continue

                dx = abs(mov_pointcloud[obj_idx, 0] - mov_pointcloud[ii, 0])
                dy = abs(mov_pointcloud[obj_idx, 1] - mov_pointcloud[ii, 1])
                dz = abs(mov_pointcloud[obj_idx, 2] - mov_pointcloud[ii, 2])
                dDop = abs(mov_doppler[obj_idx] - mov_doppler[ii])

                dist = np.sqrt(dx*dx + dy*dy + dz*dz)

                if dist <= self.clusterParams.eps:
                    if (dDop < 0.3):
                        neighborset[n_pts] = ii
                        n_pts += 1
                return n_pts, neighborset

    def Track_snapshot2file(self, id, id2, outdata):
        if self._file_snapshot_name is None or len(self._file_snapshot_name) < 1 :
            return
        fname = self._file_snapshot_name + f"_{id}_{id2}.csv"
        #np.savetxt(fname, outdata, delimiter=",")
        np.savetxt(fname, outdata, fmt="%.5f", delimiter=",")
        # with open(fname, 'w') as f :
        #     for oi in outdata :
        #         f.write('')

    def Track_Classification(self, clusterID_check, mov_pointcloud, mov_doppler, mov_power, prev_track):

        if self.TrackHist.Num == 0:
            curr_num = self.TrackHist.Num
            prev_num = 14
        else:
            curr_num = self.TrackHist.Num
            prev_num = self.TrackHist.Num - 1

        # initialize 
        self.TrackHist.track_numpoint[:, curr_num] = 0  
        self.TrackHist.track_longpos[:, curr_num] = 255 
        self.TrackHist.track_latpos[:, curr_num] = 0 
        self.TrackHist.track_longvel[:, curr_num] = 0 
        self.TrackHist.track_latvel[:, curr_num] = 0 
        self.TrackHist.track_score[:, curr_num] = self.TrackHist.track_score[:, prev_num]
        self.TrackHist.track_width[:, curr_num] = 0
        self.TrackHist.track_length[:, curr_num] = 0 
        self.TrackHist.track_doppler[:, curr_num] = 0 
        self.TrackHist.track_power[:, curr_num] = 0 
        self.TrackHist.track_type[:, curr_num] = 0 
        #
        trk_rect = None
        trk_info = None
        numTrk = None
        x_point_fin = None
        y_point_fin = None
        xv_point_fin = None
        yv_point_fin = None
        Width_fin = None
        Length_fin = None

        chk_ind = np.unique(clusterID_check)
        Trk_Type_New = np.zeros(32)


        # Target information 
        x_point = np.zeros((np.size(chk_ind)))
        y_point = np.zeros((np.size(chk_ind)))
        z_point = np.zeros((np.size(chk_ind)))
        dop_point = np.zeros((np.size(chk_ind)))
        pow_point = np.zeros((np.size(chk_ind)))
        #

        inval_cnt = 0
        trkid_fin = []
        Valid_ID = []

        # Point Cloud #
        xx = mov_pointcloud[:,0]
        yy = mov_pointcloud[:,1]
        zz = mov_pointcloud[:,2]
        dd = mov_doppler[:]
        pp = mov_power[:]

        # numobj = np.size(mov_doppler)

        # Classification #
        MySVM_Model_Car = SVM_Model_Parameter_12F_Linear_Car_Full()
        MySVM_Model_Ped = SVM_Model_Parameter_12F_Linear_Ped_Full()
        if np.size(chk_ind) < 1 :
            for jj in range(32):
                pass
        else:
            for ii in range(np.size(chk_ind)):
                if chk_ind[ii] == 0 :
                    continue
                pt_chk = np.where(clusterID_check == chk_ind[ii])
                if pt_chk is None :
                    x_point[ii] = 0
                    y_point[ii] = 255
                    z_point[ii] = 0
                    dop_point[ii] = 0
                    pow_point[ii] = 0
                    inval_cnt = inval_cnt + 1
                else:
                    outdata = np.zeros((len(pt_chk[0]) ,5))
                    if ((np.sum(np.diff(np.sort(xx[pt_chk])) > 3) < 1) & (np.sum(np.diff(np.sort(yy[pt_chk])) > 3) < 1)):
                        x_point[ii] = np.sum(xx[pt_chk])/np.size(xx[pt_chk])
                        outdata[:,0] = xx[pt_chk]
                        y_point[ii] = np.sum(yy[pt_chk])/np.size(yy[pt_chk])
                        outdata[:,1] = yy[pt_chk]
                        z_point[ii] = np.sum(zz[pt_chk])/np.size(zz[pt_chk])
                        outdata[:,2] = zz[pt_chk]
                        dop_point[ii] = np.sum(dd[pt_chk])/np.size(dd[pt_chk])
                        outdata[:,3] = dd[pt_chk]
                        pow_point[ii] = np.sum(pp[pt_chk])/np.size(pp[pt_chk])
                        outdata[:,4] = pp[pt_chk]

                        self.Track_snapshot2file(ii, chk_ind[ii], outdata)
                        #Valid_ID[ii-inval_cnt] = ii
                        Valid_ID.append(ii)

                        labels, pscore = self.SVM_Target_Est_Linear_Linear(MySVM_Model_Car, MySVM_Model_Ped, outdata)

                        self.TrackHist.track_numpoint[chk_ind[ii], curr_num] = np.size(pt_chk)  
                        self.TrackHist.track_longpos[chk_ind[ii], curr_num] = y_point[ii] 
                        self.TrackHist.track_latpos[chk_ind[ii], curr_num] = x_point[ii]
                        if ((abs(self.TrackHist.track_longpos[chk_ind[ii], prev_num]) > 0 ) & (abs(self.TrackHist.track_longpos[chk_ind[ii], prev_num]) != 255)):
                            self.TrackHist.track_longvel[chk_ind[ii], curr_num] = (self.TrackHist.track_longpos[chk_ind[ii], curr_num] - self.TrackHist.track_longpos[chk_ind[ii], prev_num])/0.05 # 20 fps 
                            self.TrackHist.track_latvel[chk_ind[ii], curr_num] = (self.TrackHist.track_latpos[chk_ind[ii], curr_num] - self.TrackHist.track_latpos[chk_ind[ii], prev_num])/0.05 # 20 fps
                        else : 
                            self.TrackHist.track_longvel[chk_ind[ii], curr_num] = 0
                            self.TrackHist.track_latvel[chk_ind[ii], curr_num] = 0
                        self.TrackHist.track_width[chk_ind[ii], curr_num] = np.max(xx[pt_chk])-np.min(xx[pt_chk])
                        self.TrackHist.track_length[chk_ind[ii], curr_num] = np.max(yy[pt_chk])-np.min(yy[pt_chk])
                        self.TrackHist.track_doppler[chk_ind[ii], curr_num] = dop_point[ii]
                        self.TrackHist.track_power[chk_ind[ii], curr_num] = pow_point[ii]
                        self.TrackHist.track_type[chk_ind[ii], curr_num] = labels 
                    else:
                        x_point[ii] = 0
                        y_point[ii] = 255
                        z_point[ii] = 0
                        dop_point[ii] = 0
                        pow_point[ii] = 0
                        inval_cnt = inval_cnt + 1
            
            # if Valid_ID is None :
            #     pass
            if len(Valid_ID) == 0 :
                pass
            else:
                chk_ind = chk_ind[:][Valid_ID]
                x_point = x_point[:][Valid_ID]
                y_point = y_point[:][Valid_ID]
                z_point = z_point[:][Valid_ID]
                dop_point = dop_point[:][Valid_ID]
                pow_point = pow_point[:][Valid_ID]
            
        for jj in range(64):
            self.E_Kalman_Filter(jj, curr_num)
            # EKF_valid = (self.TrackHist.track_longpos[jj, curr_num] != 255) & (self.TrackHist.track_longpos[jj, prev_num] != 255) & (self.CANFD.track_count[jj] == 0) 
            # if EKF_valid == False:
            #     continue
            # else:
            #     if (self.TrackHist.track_longpos[jj, prev_num] == 255) & (self.CANFD.track_count[jj] != 0): # New Track
            #         Init_Esti = self.EKF_Param.Init_PosVel
            
        for jj in range(32):
            gating_valid = (self.TrackHist.track_longpos[jj, curr_num] != 255) & (abs(self.TrackHist.track_longpos[jj, curr_num] - self.TrackHist.track_longpos[jj, prev_num]) < 3) & (abs(self.TrackHist.track_latpos[jj, curr_num] - self.TrackHist.track_latpos[jj, prev_num]) < 2)
            if gating_valid == True :
                self.CANFD.track_count[jj] = self.CANFD.track_count[jj] + 1
                if (self.CANFD.track_count[jj] > 100) :
                    self.CANFD.track_count[jj] = 100 # count limit -> 100 
                if (self.CANFD.track_count[jj] <= 5):
                    self.CANFD.track_state[jj] = 1
                elif ((self.CANFD.track_count[jj] > 5) & ((abs(self.TrackHist.track_doppler[jj,prev_num]) > 0) & (abs(self.TrackHist.track_doppler[jj,curr_num]) < 0.01))) :
                    self.TrackHist.track_doppler[jj,curr_num] = self.TrackHist.track_doppler[jj,prev_num]
                    self.TrackHist.track_type[jj,curr_num] = self.TrackHist.track_type[jj,prev_num]
                    self.CANFD.track_reliability[jj] = 2
                    self.CANFD.track_state[jj] = 2
                elif (self.CANFD.track_count[jj] > 15):
                    if (((self.TrackHist.track_type[jj, curr_num]) != (self.TrackHist.track_type[jj, prev_num])) & (self.TrackHist.track_longpos[jj, curr_num] > 15)):
                        self.CANFD.track_count[jj] = self.CANFD.track_count[jj]/2
                    elif ((self.TrackHist.track_type[jj, curr_num]) != (self.TrackHist.track_type[jj, prev_num])):
                        self.CANFD.track_count[jj] = self.CANFD.track_count[jj] - 5
                    self.TrackHist.track_type[jj,curr_num] = self.TrackHist.track_type[jj,prev_num]
                    self.CANFD.track_reliability[jj] = 2
                    self.CANFD.track_state[jj] = 2
            elif ((gating_valid == False) & (self.CANFD.track_count[jj] > 10)):
                self.CANFD.track_reliability[jj] = 2
                self.CANFD.track_state[jj] = 2
                if (abs(self.TrackHist.track_longpos[jj, curr_num] - self.TrackHist.track_longpos[jj, prev_num]) > 25) & (self.TrackHist.track_longpos[jj, curr_num] != 255)  & (self.TrackHist.track_longpos[jj, prev_num] != 255)  : 
                    self.CANFD.track_count[jj] = self.CANFD.track_count[jj] / 3 # track 값 존재, 다른 위치 인 경우 -> track 변경으로 판단
                else : 
                    self.TrackHist.track_numpoint[jj, curr_num] = self.TrackHist.track_numpoint[jj, prev_num]  
                    self.TrackHist.track_longpos[jj, curr_num] = self.TrackHist.track_longpos[jj, prev_num] 
                    self.TrackHist.track_latpos[jj, curr_num] = self.TrackHist.track_latpos[jj, prev_num] 
                    self.TrackHist.track_longvel[jj, curr_num] = self.TrackHist.track_longvel[jj, prev_num] 
                    self.TrackHist.track_latvel[jj, curr_num] = self.TrackHist.track_latvel[jj, prev_num] 
                    self.TrackHist.track_score[jj, curr_num] = self.TrackHist.track_score[jj, prev_num]
                    self.TrackHist.track_width[jj, curr_num] = self.TrackHist.track_width[jj, prev_num]
                    self.TrackHist.track_length[jj, curr_num] = self.TrackHist.track_length[jj, prev_num] 
                    self.TrackHist.track_doppler[jj, curr_num] = self.TrackHist.track_doppler[jj, prev_num] 
                    self.TrackHist.track_power[jj, curr_num] = self.TrackHist.track_power[jj, prev_num] 
                    self.TrackHist.track_type[jj, curr_num] = self.TrackHist.track_type[jj, prev_num] 
                    self.CANFD.track_count[jj] = self.CANFD.track_count[jj] - 5
            elif ((gating_valid == False) & (self.CANFD.track_count[jj] <= 10) & (self.TrackHist.track_longpos[jj, curr_num] != 255)): # New Track 
                self.CANFD.track_count[jj] = 1
                self.CANFD.track_reliability[jj] = 1
                self.CANFD.track_state[jj] = 1
            else:
                self.CANFD.track_count[jj] = 0
                self.CANFD.track_reliability[jj] = 3
                self.CANFD.track_state[jj] = 0

            if ((abs(self.TrackHist.track_doppler[jj,curr_num]) > 2.5) & (abs(self.TrackHist.track_longpos[jj,curr_num]) < 40)):
                self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] + 5
                if ((self.TrackHist.track_type[jj,curr_num] != self.TrackHist.track_type[jj,prev_num]) & (self.TrackHist.track_type[jj,prev_num] != 0)):
                    self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] - 3
            elif ((abs(self.TrackHist.track_doppler[jj,curr_num]) > 0.75) & (abs(self.TrackHist.track_longpos[jj,curr_num]) < 40)):
                self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] + 3
                if ((self.TrackHist.track_type[jj,curr_num] != self.TrackHist.track_type[jj,prev_num]) & (self.TrackHist.track_type[jj,prev_num] != 0)):
                    self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] - 2
            elif ((abs(self.TrackHist.track_doppler[jj,curr_num]) > 0) & (abs(self.TrackHist.track_longpos[jj,curr_num]) < 40)):
                self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] + 1
                if ((self.TrackHist.track_type[jj,curr_num] != self.TrackHist.track_type[jj,prev_num]) & (self.TrackHist.track_type[jj,prev_num] != 0)):
                    self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] - 1
            else:
                if (self.TrackHist.track_score[jj,prev_num] > 10):
                    self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] - 3
                    if (self.TrackHist.track_score[jj,curr_num] < 0):
                        self.TrackHist.track_score[jj,curr_num] = 0 
                else:
                    self.TrackHist.track_score[jj,curr_num] = 0

            if (self.TrackHist.track_score[jj,curr_num] > 255) :
                self.TrackHist.track_score[jj,curr_num] = 255  # score limit

            if ((self.TrackHist.track_score[jj,curr_num] >= 10) & (self.TrackHist.track_type[jj,curr_num] == self.TrackHist.track_type[jj,prev_num])):
                Trk_Type_New[jj] = self.TrackHist.track_type[jj,curr_num]
            elif ((self.TrackHist.track_score[jj,curr_num] >= 10) & (self.TrackHist.track_type[jj,curr_num] != self.TrackHist.track_type[jj,prev_num]) & (self.TrackHist.track_type[jj,prev_num] != 0)):
                if ((self.TrackHist.track_score[jj,curr_num] <= 20) & (self.TrackHist.track_type[jj,curr_num] == 2) & (self.TrackHist.track_type[jj,prev_num] == 3)):
                    self.TrackHist.track_type[jj,curr_num] = 3
                elif ((self.TrackHist.track_score[jj,curr_num] > 20) & (self.TrackHist.track_type[jj,curr_num] == 3)):
                    self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num]/2
                    self.TrackHist.track_type[jj,curr_num] = self.TrackHist.track_type[jj,prev_num]
                else:
                    self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,curr_num] - 2
                    if (self.TrackHist.track_score[jj,curr_num] < 0):
                        self.TrackHist.track_score[jj,curr_num] = 0 
                    self.TrackHist.track_type[jj,curr_num] = self.TrackHist.track_type[jj,prev_num]                                           
                Trk_Type_New[jj] = self.TrackHist.track_type[jj,curr_num]
            elif ((self.TrackHist.track_score[jj,curr_num] >= 10) & (self.TrackHist.track_type[jj,curr_num] != self.TrackHist.track_type[jj,prev_num]) & (self.TrackHist.track_type[jj,prev_num] == 0)):
                self.TrackHist.track_score[jj,curr_num] = self.TrackHist.track_score[jj,prev_num] - 2
                if (self.TrackHist.track_score[jj,curr_num] < 0):
                    self.TrackHist.track_score[jj,curr_num] = 0
                self.TrackHist.track_type[jj,curr_num] = self.TrackHist.track_type[jj,prev_num]
            else: 
                Trk_Type_New[jj] = 0

        trkid_fin = np.nonzero(self.TrackHist.track_longpos[:,curr_num] < 255)
        numTrk = np.size(trkid_fin)
    
        # if (numTrk > 0):
        trk_info = np.zeros((numTrk, 7))
        x_point_fin = self.TrackHist.track_latpos[trkid_fin,curr_num]
        y_point_fin = self.TrackHist.track_longpos[trkid_fin,curr_num]
        xv_point_fin = self.TrackHist.track_latvel[trkid_fin,curr_num]
        yv_point_fin = self.TrackHist.track_longvel[trkid_fin,curr_num]
        d_point_fin = self.TrackHist.track_doppler[trkid_fin,curr_num]
        Width_fin = self.TrackHist.track_width[trkid_fin,curr_num]
        Length_fin = self.TrackHist.track_length[trkid_fin,curr_num]


        for kk in range(numTrk):
            trk_info[kk,:] = [x_point_fin[0,kk], y_point_fin[0,kk], xv_point_fin[0,kk], yv_point_fin[0,kk], d_point_fin[0,kk], Width_fin[0,kk], Length_fin[0,kk]]

        # else:
        #     numTrk = prev_track[0]
        #     trk_info = np.zeros((numTrk, 7))
        #     for kk in range(numTrk):
        #         if (prev_track[2][prev_track[3][0][kk]] != 0) :
        #             trk_info[kk,:] = [prev_track[1][kk][0] + (prev_track[1][kk][2])*0.05, prev_track[1][kk][1] + (prev_track[1][kk][3])*0.05, prev_track[1][kk][2], prev_track[1][kk][3], prev_track[1][kk][4], prev_track[1][kk][5], prev_track[1][kk][6]]
        #             Trk_Type_New = prev_track[2]
        #             trkid_fin = prev_track[3] 
            # if np.sum(prev_track[2][prev_track[3][0][kk]] == 0):
                
        if (self.TrackHist.Num < 14):
            self.TrackHist.Num = self.TrackHist.Num + 1
        else:
            self.TrackHist.Num = self.TrackHist.Num - 14 

        print(f">>TrackInfo num:{numTrk}")
        print(f">>trk_info:{trk_info}")
        print(f">>type_new:{Trk_Type_New}") 
        print(f">>id_fin:{trkid_fin}")
        #self._track_list.remove()
        self.put_current_tracks( (numTrk, trk_info, Trk_Type_New, trkid_fin) )
        return numTrk, trk_info, Trk_Type_New, trkid_fin

    def TrackID_Decision(self, clusterID_check, mov_pointcloud, prev_track, cluster_info):

        if prev_track is None or prev_track[0] is None or prev_track[0]==0:
            prev_trk_pos = [[0, 255]] # lat / long
            prev_trk_dop = [0]
            prev_trk_size = [[0, 0]]
            prev_ID_chk =[]
            prev_trk_type = [0]
            prev_track_num = 0
        else :
            prev_track_num = prev_track[0]
            prev_trk_pos = np.zeros((len(prev_track[3][0]),2)) # lat / long
            prev_trk_dop = np.zeros(len(prev_track[3][0]))
            prev_trk_size = np.zeros((len(prev_track[3][0]),2))
            prev_ID_chk = np.zeros(len(prev_track[3][0]), dtype=int)
            prev_trk_type = np.zeros(len(prev_track[3][0]))
            for tt in range(len(prev_track[3][0])):
                prev_trk_pos[tt] = prev_track[1][tt][0:2]
                prev_trk_dop[tt] = prev_track[1][tt][4]
                prev_trk_size[tt] = prev_track[1][tt][5:]
                prev_ID_chk[tt] = prev_track[3][0][tt]
                prev_trk_type[tt] = prev_track[2][prev_ID_chk[tt]]  # Future works : gating decision for each type
                
        trk_chk = np.unique(clusterID_check) # Current clusterID_check
        int_cluster_info = np.zeros([32, 5])

        if prev_track_num == 0 :
            int_cluster_info = cluster_info[trk_chk]
        else :
            for ii in range(len(prev_track[3][0])):

                prev_pos_x = prev_trk_pos[ii][0]
                prev_pos_y = prev_trk_pos[ii][1]
                prev_dop = prev_trk_dop[ii]
                
                # curr_pos_x = cluster_info[trk_chk[ii]][1]
                # curr_pos_y = cluster_info[trk_chk[ii]][2]
                # curr_dop = cluster_info[trk_chk[ii]][4]
                ID_match = 0

                for tt in range(len(trk_chk)):
                    gating_x = prev_trk_size[ii][0]
                    gating_y = prev_trk_size[ii][1] 
                    gating_d = 1.8 #0.5   # 1.8 kph 
                    if ((abs(cluster_info[trk_chk[tt]][1] - prev_pos_x) < gating_x) & (abs(cluster_info[trk_chk[tt]][2] - prev_pos_y) < gating_y) & (abs(cluster_info[trk_chk[tt]][4] - prev_dop) < gating_d)):
                        tmp_clusterID = prev_track[3][0][ii]
                        if tmp_clusterID == cluster_info[trk_chk[tt]][0]:
                            cluster_info[trk_chk[tt]][0] = tmp_clusterID
                            ID_match = ID_match + 1
                            continue
                        else :
                            chk_clusterID = cluster_info[trk_chk[tt]][0]
                            cluster_info[trk_chk[tt]][0] = tmp_clusterID
                            cluster_info[trk_chk[tt]][0] = max(cluster_info[:,0]) + 1 
                    else:
                        if max(cluster_info[:,0]) + 1 > 31:
                            cluster_info[trk_chk[tt]][0] = 24 # 0 ~ 23 : Main Track / 24 ~ 32 : Subtrack
                        else:
                            cluster_info[trk_chk[tt]][0] = max(cluster_info[:,0]) + 1
                    

                
    
    def SVM_Target_Est_Linear_Linear(self, MySVM_Model_Car, MySVM_Model_Ped, outdata):
        
        x_mean, x_size, y_mean, y_size, z_mean, z_size, rng_p, d_mean, x_std, y_std, z_std, p_std = self.FeatureExtraction_12F_1(outdata)
        E_data = [x_mean, x_size, y_mean, y_size, z_mean, z_size, rng_p, d_mean, x_std, y_std, z_std, p_std]

        X = E_data - MySVM_Model_Car.Mu
        X = X/MySVM_Model_Car.Sigma 
        pscore = (X/MySVM_Model_Car.Scale)@MySVM_Model_Car.Beta + MySVM_Model_Car.Bias

        if pscore < 0:
            labels = 1
        else:
            E_data_1 = E_data

            if (((x_size < 0.75) & (y_size > 1.50)) | (x_size > 1.50)):
                labels = 3
            else:
                X = E_data_1 - MySVM_Model_Ped.Mu
                X = X/MySVM_Model_Ped.Sigma 
                pscore = (X/MySVM_Model_Ped.Scale)@MySVM_Model_Ped.Beta + MySVM_Model_Ped.Bias
                
                if (pscore < 0):
                    labels = 2
                else:
                    labels = 3

        return labels, pscore

    
    def FeatureExtraction_12F_1(self, outdata):

        x_min = np.min(outdata[:,0])
        x_max = np.max(outdata[:,0])
        x_mean = np.mean(outdata[:,0])
        x_size = abs(x_max - x_min)

        y_min = np.min(outdata[:,1])
        y_max = np.max(outdata[:,1])
        y_mean = np.mean(outdata[:,1])
        y_size = abs(y_max - y_min)

        z_min = np.min(outdata[:,2])
        z_max = np.max(outdata[:,2])
        z_mean = np.mean(outdata[:,2])
        z_size = abs(z_max - z_min)

        rng_p = np.sqrt(x_mean**2 + y_mean**2 + z_mean**2)

        d_mean = np.mean(outdata[:,3])

        x_std = np.std(outdata[:,0])
        y_std = np.std(outdata[:,1])
        z_std = np.std(outdata[:,2])
        p_std = np.std(outdata[:,4])

        return x_mean, x_size, y_mean, y_size, z_mean, z_size, rng_p, d_mean, x_std, y_std, z_std, p_std


        # xs = points[:,0];   ys = points[:,1];   zs = points[:,2]
        # pc_polar[:,0] = np.sqrt(xs**2+ys**2)    # range
        # pc_polar[:,1] = np.arctan2(ys,xs)       # azimuth
        # pc_polar[:,2] = 0                       # elevation, not use
        # if points.shape[1] == 12 :
        #     pc_polar[:,3] = points[:,9]
        #     pc_polar[:,4] = points[:,8]
        # else :
        #     pc_polar[:,3] = points[:,3]
        #     pc_polar[:,4] = points[:,4]

        #trk.update_point_cloud(pc_polar[:,0], pc_polar[:,1], pc_polar[:,3], pc_polar[:,4] )
        #target_desc, target_num = trk.step()
        #return pc_polar, target_num, target_desc
        return None
    

# ----------------------------------
if __name__ == '__main__':
    trk = SRSTracker()
    pts = np.array([[1,1,1,0,0,0,0,0,10,5,0,0]])
    trk.process(pts)
    np.matrix()
