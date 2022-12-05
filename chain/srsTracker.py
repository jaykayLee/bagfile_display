from dataclasses import dataclass
import sys
from typing import final
#import typing
import numpy as np

@final
class SRTrkConst:
    ASSOCIATED_TRACK_ID: Final[int] = 6144
    MAX_TRACKING_DATA: Final[int] = 128
    MAX_LINK_NUM : Final[int] = 34
    MAX_TRACKS : Final[int] = 32
    LEN_HISTORY : Final[int] = 5
    HISTORY_LEN : Final[int] = 10
    MAX_TRACK_HISTORY : Final[int] = 5
    LIMIT_SNR_PEAK : Final[int] = 1000000000
    NUM_STATE : Final[int] = 6   # enum {X_POS, X_VEL,  X_ACC, Y_POS, Y_VEL, Y_ACC, PWR};
    MAX_OBJECTS : Final[int] = 260
    MAX_CLUSTER : Final[int] = 32
    NUM_CLUSTER_FRAME : Final[int] = 1

@final
class SRObjType:
    _UNKNOWN : Final[int] = 0
    _VEHICLE : Final[int] = 1
    _PEDESTRIAN : Final[int] = 2
    _CYCLIST : Final[int] = 3
    _MAX_TRACKING_TYPE : Final[int] = 4

@final
class SRObjState:
    _STATIC : Final[int] = 0
    _STOPPED : Final[int] = 1
    _MOVING : Final[int] = 2
    _MAX_OBJ_STATE : Final[int] = 3


# struct TrackingObjectState
# {
#     bool valid;
#     ObjectState state;                 // 0 : static, 1 : Stopped, 2 : Moving
#     ObjectType type;
#     Position pos;
#     P_Region region;
#     float doppler;
# };
# struct SRS_Tracker
# {
#     uint8_t index;
#     TrackingObjectState track[LEN_HISTORY][MAX_TRACKS];
# };


# typedef struct
# {
# 	uint16_t               frameIdx;
# 	int16_t                nClusters;
# 	int16_t       		   nDetectedObjects[NUM_CLUSTER_FRAMES];
# 	TYPE_Object			   detectedObjects[NUM_CLUSTER_FRAMES][MAX_OBJECTS];
# 	TYPE_ClusterCheck      clusterCheck[MAX_OBJECTS];
# 	TYPE_ClusterInfo       clusterInfo[MAX_CLUSTERS];
# 	TYPE_Track             trackInfo[MAX_TRACKS]; 
# 	TYPE_ValidTracks       confirmedTracks[MAX_TRACKS];  
# 	float 	      		   m3Pk1[MAX_TRACKS][NUM_STATES][NUM_STATES];
# 	int16_t 	   		   managedTrackIdx[MAX_TRACKS];
# 	int16_t 		       nTracks;
# 	TYPE_ROI               roiInfo;
# 	TYPE_ClusteringCfg     clusteringCfg;
# 	TYPE_TrkScoreCfg       trkScoreCfg;
# 	TYPE_TrkManageCfg      trkManageCfg;
# 	TYPE_TraffMonitorCfg   traffMonitorCfg;
# 	TYPE_StaticZoneCfg     staticZoneCfg;
# 	TYPE_AxisInvertCfg     axisInvertCfg;

# 	uint32_t 			   baseTrackID;
# 	float				   timeFrameAcc;
# 	float				   timeFrame;
# 	float 				   r00, r11;
# 	float				   p00v, p01v, p02v, p11v, p12v, p22v;
# 	float				   p00h, p01h, p02h, p11h, p12h, p22h;
# } AlgorithmFxn_RadarTracker;

class SRTracker:

    def __init__(self):
        self.cur_frame_id = 0
        self.prev_frame_id = -1
        # -----------------
        ## data receiver.
        
        self.frame_number = 0
        self.num_points = 0
        self.num_tracks = 0
        # point = x, y, z, doppler, snr
        self.points = None # input point data
        self.detobjs = None # detectedObjects,  internal point data
        self.num_detobjs = 0  # nDetectedObjects
        # self.track_ids = ,    # track_id ,assume none from radar
        # numTracks, assume 0 from radar
        self.num_tracks = 0,  

        # track_history = num_points, long_pos, lat_pos, type_score, azimuth, width, length, doppler, power, type
        # typeinfo_out = state, count, range, angel, type, activity, reliability
        # type_track trackInfo[MAX_TRACKS] = rx, vx, ax, ry, vy, ay, history[history_len][num_state+1], track_state
        #               power, cntHit, cntMiss, lifeTime, numAssoc, staticCount, motionStatus, reliability, trackId
        self.track_info = np.zeros((SRTrkConst.MAX_TRACKS, 16))
        self.track_info_history = np.zeros((SRTrkConst.MAX_TRACKS, SRTrkConst.HISTORY_LEN, SRTrkConst.NUM_STATE+1))
        # type_validtrack = rx, vx, ax, ry, vy, ay, power, bufferidx, blockage_count, motionstatus, blockagestate, 
        #               reliability, tackid, rXtx, rYtx

        # tpe_cluster_check clustercheck[MAX_OBJECTS] = frameIdx, objectIdx, clusterID, pointType
        self.cluster_check = np.zeros((SRTrkConst.MAX_OBJECTS, 4))
        # type_cluster_info clusterinfo[MAX_OBJECTS] = x, y, range, doppler, power
        self.cluster_info = np.zeros((SRTrkConst.MAX_OBJECTS, 5))

    def process(self, points):
        self.points = points
        self.num_detobjs = points.shape[0]
        self.detobjs = np.zeros((self.num_detobjs, 5))  # no need, associatedTrackIdx
        self.detobjs[:,0] = points[:,0]*100 # X
        self.detobjs[:,1] = points[:,1]*100 # Y
        self.detobjs[:,2] = points[:,2]*100 # Z
        self.detobjs[:,3] = points[:,9]*100 # doppler
        self.detobjs[:,4] = np.clip(points[:,8], 0, SRTrkConst.LIMIT_SNR_PEAK)  # snr
        
        ## 2 condition assume
        ##      1. detobjs.associatedTrackIdx is always 0
        ##      2. numTracks is always 0

        nobj = self.process_tracking()

    def process_tracking(self):

        if self.num_tracks == 0:
            self.track_info[:,0:16] = 0
            self.track_info_history[:,:,:] = 0


        else :
            pass


    def subprocess_clustering(self):
        self.cluster_check[:,:] = 0
        self.cluster_info[:,:] = 0

        pass