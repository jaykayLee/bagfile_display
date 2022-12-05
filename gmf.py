import sys
import datetime
import time
import struct
import os
import string
import numpy as np
import queue
import multiprocessing as mp
from multiprocessing import shared_memory as shm
import copy
import ctypes
import pathlib
import typing
from PyQt5.QtCore import QDateTime, Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
    QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
    QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget, QFileDialog, QButtonGroup, QMessageBox)
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from bag_parse import BagFileReader
from chain import SRDataStep
from gcm import CMTrkType, ConfigManager, ConfigData
compare_chain = False
if compare_chain :
    from chain import SRGUIStep
    from chainx import SRGUIStep as SRGUIStepX
else:
    from chain import SRGUIStep
from tracking import ekf, srtrack
from gtrack import titrk
vcdbg_on = True ## vscode debugger
if vcdbg_on :
    import debugpy

###################################
### global message flow

# ----------------------------------
# ----------------------------------
class QueCleaner(QThread):
    _priority=QThread.HighestPriority-2
    _do_run = False
    _call_count = 0
    _stack_size = 0


    # ----------------------------------
    def __init__(self, inque, stack_size):
        QThread.__init__(self)
        self._in_q = inque
        self._stack_size = stack_size

    def reset(self):
        self._call_count = 0

    def stop(self):
        print('>>Stop QueCleaner')
        self._do_run = False
        self.wait()

    def run(self):
        if vcdbg_on : debugpy.debug_this_thread()
        print('>>Start QueCleaner.Run')
        self._do_run = True
        while self._do_run :
            try :
                #if self._in_q.qsize() > self._stack_size :
                item = self._in_q.get(timeout=0.5)
            except queue.Empty:
                continue
            except Exception as e:
                print(e)
            else:
                if item is not None :
                    del item
                    item = None
                    self._call_count += 1
        print('>>Exit QueCleaner.Run')

# ----------------------------------
def process_gtrack(trk : ekf.EKF, points):
    pc_polar = np.zeros((np.shape(points)[0],5))
    xs = points[:,0]
    ys = points[:,1]
    zs = points[:,2]
    pc_polar[:,0] = np.sqrt(xs**2+ys**2)    # range
    #pc_polar[:,1] = np.arctan2(ys,xs)       # azimuth
    pc_polar[:,1] = np.arctan2(xs,ys)       # azimuth
    pc_polar[:,2] = 0                       # elevation, not use
    if points.shape[1] == 12 :
        pc_polar[:,3] = points[:,9]
        pc_polar[:,4] = points[:,8]
    else :
        pc_polar[:,3] = points[:,3]
        pc_polar[:,4] = points[:,4]
    trk.update_point_cloud(pc_polar[:,0], pc_polar[:,1], pc_polar[:,3], pc_polar[:,4] )
    #trk.update_point_cloud_pnc(points.shape[0], points, pc_polar)
    target_desc, target_num = trk.step()
    return pc_polar, target_num, target_desc

# ----------------------------------
def ProcessDataChain(datachain, _inq, _outq, _gcm_name, _gcm_chaned, _is_run):
    ##  _bagf:BagFileReader, _play_mode:(1:normal,2:reverse), _outq:Queue, is_run(mp.Value)
    if vcdbg_on : debugpy.debug_this_thread()
    shmem = None
    cfg = None
    tracker = None
    trMatrix = None
    stack_list = []
    mp.current_process().name = "DataChain"
    print(f">>START ProcessDataChain, pid={mp.current_process().pid}")
    #try :
    while _is_run.value:
        try :
            if _outq.qsize() > 5 :
                time.sleep(0.1)
                continue
            item = _inq.get(timeout=0.5)
        except queue.Empty:
            continue
        except Exception as ie:
            print(ie)
        if cfg == None or _gcm_chaned.value :
            if shmem == None : shmem = shm.SharedMemory(_gcm_name)
            cfg = ConfigData.from_buffer_copy(shmem.buf)
            if cfg.tracker_type == CMTrkType.GTRACKER:
                tracker = ekf.EKF()
            elif cfg.tracker_type == CMTrkType.SRTRACKER:
                tracker = srtrack.SRSTracker()
            elif cfg.tracker_type == CMTrkType.TIGTRACKER:
                tracker = titrk.TiGTracker()
                if cfg.tracker_cfg is not None and len(cfg.tracker_cfg) > 1 :
                    pcfg = tracker.load_config_file(cfg.tracker_cfg)
                else :
                    pcfg = tracker.get_default_config()
                tracker.create(pcfg)
            else :
                tracker = None
            _gcm_chaned.value = False
        if item == None :
            pass
        elif item['topic'] == '/point_cloud':

            ## 1. clipping process
            item['dataprocessed'] = item['parsed']  # item[6]..
            
            if len(list(filter(lambda pi : abs(pi) > 0.001, [cfg.axis_pos[0],cfg.axis_pos[1],cfg.axis_pos[2], cfg.axis_angle[0], cfg.axis_angle[1], cfg.axis_angle[2]]))) > 0 :
                ##if trMatrix is None :
                trMatrix = SRDataStep.generate_eular_matrix((cfg.axis_angle[0], cfg.axis_angle[1], cfg.axis_angle[2]))
                item['dataprocessed'][4][:,0:3] = SRDataStep.process_axis_translate(item['dataprocessed'][4][:,0:3], (cfg.axis_pos[0], cfg.axis_pos[1], cfg.axis_pos[2]), trMatrix)
            item['dataprocessed'][4] = SRDataStep.process_clipping_by_area(item['dataprocessed'][4], cfg.clips, cfg.clip_opt, cfg.idx_doppler)

            # ----------------------------------
            def fname_by(ci, sec, nsec):
                dt = datetime.datetime.fromtimestamp(sec)
                return datachain._snapshot_dir + os.path.sep + f"{ci}_" + dt.strftime('%Y%m%d_%H%M%S_') + str(nsec)

            ## 2. tracker process
            if tracker != None :
                if type(tracker).__name__ == 'EKF': ## for gtracker data
                    pc_polar, tnum, targets = process_gtrack(tracker, item['dataprocessed'][4])
                    item['dataprocessed'].append(('EKF', int(tnum), targets, pc_polar))
                    # if tnum > 0 :  print(f'...GTrack target num is {tnum}')
                elif type(tracker).__name__ == 'SRSTracker':
                    if cfg.stack_size == 0 :
                        Mov_numTrk, Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin = tracker.process(item['dataprocessed'][4], cfg.idx_doppler)
                        #Mov_numTrk, Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin = tracker.process(item['dataprocessed'][4], cfg.idx_doppler, fname_by(item['chunk_idx'], item["time"][0], item["time"][1]))
                        item['dataprocessed'].append(('SRSTracker', int(Mov_numTrk) if Mov_numTrk is not None else 0 , Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin ))
                    else :
                        if len(stack_list) >= cfg.stack_size :
                            rem_item = stack_list[0]
                            stack_list.remove(rem_item)
                            #del rem_item
                        stack_list.append(item['dataprocessed'][4])

                        stack_total : int = 0
                        for mi in stack_list:
                            stack_total += len(mi)
                        num_column = item['dataprocessed'][4].shape[1]
                        pc_data = np.zeros((stack_total, num_column+1))
                        offset = 0
                        fidx = 0
                        for mi in stack_list :
                            pc_data[offset:offset+len(mi), 0:num_column] = mi[:,0:num_column]
                            pc_data[offset:offset+len(mi), num_column] = fidx
                            fidx += 1
                            offset += len(mi)
                        Mov_numTrk, Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin = tracker.process(pc_data, cfg.idx_doppler)
                        item['dataprocessed'].append(('SRSTracker', int(Mov_numTrk) if Mov_numTrk is not None else 0 , Mov_trk_info, Mov_Trk_Type, Mov_trkid_fin ))


                elif type(tracker).__name__ == 'TiGTracker':
                    num_trk, trk_info = tracker.process(item['dataprocessed'][4], cfg.idx_doppler)
                    item['dataprocessed'].append(('TiGTracker', num_trk.value, copy.copy(trk_info[0:num_trk.value])))
        _outq.put(item)
        datachain._call_count += 1
    # except Exception as e:
    #     print(e)
    if cfg != None : del cfg
    if shmem != None : shmem.close()
    print('<<EXIT ProcessDataChain')

# ----------------------------------
class DataChain():
    _recent_process_ms = 0
    _call_count = 0
    _do_clip_mask = 0
    _gcm = None
    _is_run = mp.RawValue(ctypes.c_bool, False)
    _is_gcm_changed = mp.RawValue(ctypes.c_bool, False)
    _data_process = None
    _snapshot_dir = None

    # ----------------------------------
    def __init__(self, inque, outque, _cm):
        self._in_q = inque
        self._out_q = outque
        self._call_count = 0
        self._gcm = _cm

    def set_gcm(self, cm):
        self._gcm = cm
        self._is_gcm_changed.value = True

    def reset(self):
        self._call_count = 0

    def stop(self):
        print('>>Stop DataChain')
        self._is_run.value = False
        self._data_process.join()

    def reload_gcm(self, mask):
        if mask == 1 :
            self._do_clip_mask = 1

    def start(self):
        self._is_run.value = True
        self._snapshot_dir = os.getcwd() + os.path.sep + f"snapshot"
        if pathlib.Path(self._snapshot_dir).is_dir() == False :
            os.makedirs(self._snapshot_dir)
        self._data_process = mp.Process(target=ProcessDataChain, args=(self, self._in_q, self._out_q, self._gcm.shared_name(), self._is_gcm_changed, self._is_run ))
        self._data_process.name = "DataChain"
        self._data_process.start()


# ----------------------------------
# ----------------------------------
def ProcessGraphData(gupd, _inq, _outq, _gcm_name, _gcm_chaned, _is_run):
    ##  _bagf:BagFileReader, _play_mode:(1:normal,2:reverse), _outq:Queue, is_run(mp.Value)
    if vcdbg_on : debugpy.debug_this_thread()
    shmem = None
    cfg = None
    color_table = None
    print('>>START ProcessGraphData')
    # try :
    while _is_run.value:
        try :
            if _outq.qsize() > 5 :
                time.sleep(0.1)
                continue
            item = _inq.get(timeout=0.5)
        except queue.Empty:
            continue
        except Exception as ie:
            print(ie)
        if cfg == None or _gcm_chaned.value :
            clips = None
            if shmem == None : shmem = shm.SharedMemory(_gcm_name)
            cfg = ConfigData.from_buffer_copy(shmem.buf)
            clips = cfg.clips
            color_table = np.copy(cfg.get_color_table(shmem.buf))
            _gcm_chaned.value = False
        if item == None :
            pass
        elif item['topic'] == '/point_cloud':
            points_msg = item['dataprocessed']
            cur_pc_data = points_msg[4][:, 0:3]
            cur_pc_color = np.zeros((np.shape(cur_pc_data)[0], 4))
            out_color = pg.glColor('r')

            #for i, cur_pc in enumerate(cur_pc_data):
            #    cur_pc_color[i]=self._gcm._color_table[int((cur_pc[2]/color_range)*10)] if cur_pc[2]>=self._gcm._z_range[0] and cur_pc[2]<=self._gcm._z_range[1] else pg.glColor('r')
            # -----------------------------------
            def set_pointcloud_color_by_zindex(zmin, zmax, color_table, len_pc, pc_xyz, pc_color, out_color):
                color_range = zmax+abs(zmin)
                color_index = 0
                for i in range(len_pc):
                    if pc_xyz[i][2] < zmin or pc_xyz[i][2] > zmax :
                        pc_color[i] = out_color
                    else:
                        #color_index = int((pc_xyz[i][2]/color_range)*10)
                        color_index = int((pc_xyz[i][2]-zmin)*10)
                        pc_color[i] = color_table[color_index]
            # set_pointcloud_color_by_zindex(self._gcm._z_range[0], self._gcm._z_range[1], self._gcm._color_table, len(cur_pc_data), cur_pc_data, cur_pc_color, out_color)
            if compare_chain :
                stp0 = time.time_ns()
                SRGUIStep.set_pointcloud_color_by_zindex(cfg.pc_color_zmin, cfg.pc_color_zmax, color_table, len(cur_pc_data), cur_pc_data, cur_pc_color, np.asarray(out_color))
                stp1 = time.time_ns()
                SRGUIStepX.set_pointcloud_color_by_zindex(cfg.pc_color_zmin, cfg.pc_color_zmax, color_table, len(cur_pc_data), cur_pc_data, cur_pc_color, np.asarray(out_color))
                stp2 = time.time_ns()
                print(f'  compare time consume.. pc:{len(cur_pc_data)} Python:Cython = {int(stp1-stp0)/1000000}:{int(stp2-stp1)/1000000} '  )
            else:
                #SRGUIStep.set_pointcloud_color_by_zindex(cfg.pc_color_zmin, cfg.pc_color_zmax, color_table, len(cur_pc_data), cur_pc_data, cur_pc_color, np.asarray(out_color))
                set_pointcloud_color_by_zindex(cfg.pc_color_zmin, cfg.pc_color_zmax, color_table, len(cur_pc_data), cur_pc_data, cur_pc_color, np.asarray(out_color))


            if cfg.tracker_type == CMTrkType.INTERNAL:
                if 'sub' in item :  ## exist sub data ?
                    if 'targets' in item['sub'] : ##  has target data ?
                        tracker_type = item['sub']['tracker_type']
                        cur_tgt_data = item['sub']['targets']
                        cur_tgt_num = len(cur_tgt_data)
                        tgt_label = []
                        tgt_data = np.zeros((cur_tgt_num, 3))
                        if tracker_type == 0x20 :
                            for i in range(cur_tgt_num):
                                tgt_data[i,0] = cur_tgt_data[i,0]
                                tgt_data[i,1] = cur_tgt_data[i,1]
                                tgt_label.append(f"{cur_tgt_data[i,6]}({tgt_data[i,0]:.1f},{tgt_data[i,1]:.1f})")
                            tgt_color = pg.glColor('#FFFF0080')
                        elif tracker_type == 0x21 :
                            for i in range(cur_tgt_num):
                                tgt_data[i,0] = cur_tgt_data[i,2]
                                tgt_data[i,1] = cur_tgt_data[i,3]
                                tgt_label.append(f"{cur_tgt_data[i,0]}:{cur_tgt_data[i,1]}({tgt_data[i,0]:.1f},{tgt_data[i,1]:.1f})")
                            tgt_color = pg.glColor('#FFFF0080')
                        item['graphed'] = [cur_pc_data, cur_pc_color, tgt_data, tgt_color, tgt_label]
            ## ('EKF', tnum, targets, pc_polar)
            elif len(points_msg) > 5:
                if points_msg[5][1] == 0 :
                    item['graphed'] = [cur_pc_data, cur_pc_color, None, None, None]
                elif points_msg[5][0] == 'EKF':
                    cur_tgt_num = points_msg[5][1]
                    cur_tgt_data = points_msg[5][2]
                    #tgt_data = cur_tgt_data[:, 0:3]
                        # self.uid = 0
                        # self.tid = 0
                        # self.S = np.zeros(shape=(6,), dtype=np.float32)
                        # self.EC = np.zeros(shape=(9,), dtype=np.float32)
                        # self.G = 0
                        ## target_info = uid, tid, x, y, z, vx, vy, vz, G
                    if cur_tgt_num > 0 :
                        tgt_data = np.zeros((cur_tgt_num, 3))
                        tgt_label = []
                        for i in range(cur_tgt_num):
                            tgt_data[i][0] = cur_tgt_data[i].S[0]
                            tgt_data[i][1] = cur_tgt_data[i].S[1]
                            tgt_label.append(f"{cur_tgt_data[i].uid}:{cur_tgt_data[i].tid}({tgt_data[i,0]:.1f},{tgt_data[i,1]:.1f})")
                            #tgt_data[i][2] = 0
                        tgt_color = pg.glColor('#00FF0080')
                        item['graphed'] = [cur_pc_data, cur_pc_color, tgt_data, tgt_color, tgt_label]
                elif points_msg[5][0] == 'SRSTracker' :
                    # 'SRSTracker', numTrk, trk_info, Trk_Type_New, trkid_fin
                    ##            trk_info : [x_point[kk], y_point[kk], z_point[kk], tgt_width[kk], tgt_length[kk], tgt_height[kk], dop_point[kk], target_type[kk]]
                    cur_tgt_num = points_msg[5][1]
                    tgt_data = np.zeros((cur_tgt_num,3))
                    tgt_data[:,0] = points_msg[5][2][:,0]
                    tgt_data[:,1] = points_msg[5][2][:,1]
                    tgt_data[:,2] = 0
                    tgt_color = pg.glColor('#FF000080')
                    tgt_label = []
                    ti : int = 0
                    for li in range(len(points_msg[5][3])):
                        if points_msg[5][3][li] != 0 :
                            tgt_label.append(f"{srtrack.SRTargetType.CHAR[int(points_msg[5][3][li])]}{li}({tgt_data[ti,0]:.1f},{tgt_data[ti,1]:.1f})")
                            ti += 1
                            if ti >= cur_tgt_num:
                                break
                    # for i in range(cur_tgt_num):
                    #     tgt_label.append(f"{srtrack.SRTargetType.CHAR[int(points_msg[5][3][i+1])]}{i+1}({tgt_data[i,0]:.1f},{tgt_data[i,1]:.1f})")
                    item['graphed'] = [cur_pc_data, cur_pc_color, tgt_data, tgt_color, tgt_label]
                elif points_msg[5][0] == 'TiGTracker':
                    cur_tgt_num = points_msg[5][1]
                    cur_tgt_data = points_msg[5][2]
                    tgt_label = []
                    tgt_data = np.zeros((cur_tgt_num, 3))
                    for i in range(cur_tgt_num):
                        tgt_data[i][0] = cur_tgt_data[i].s[0]
                        tgt_data[i][1] = cur_tgt_data[i].s[1]
                        tgt_label.append(f"{cur_tgt_data[i].uid}:{cur_tgt_data[i].tid}({tgt_data[i,0]:.1f},{tgt_data[i,1]:.1f})")
                    tgt_color = pg.glColor('#FFFF0080')
                    item['graphed'] = [cur_pc_data, cur_pc_color, tgt_data, tgt_color, tgt_label]
            if ('graphed' in item.keys()) == False :
                item['graphed'] = [cur_pc_data, cur_pc_color]
        elif item['topic'] == '/velodyne_points':
            velodyne_msg = item['parsed']
            if True :  ## 1. X axis and Y axis swap, 2. X axis flip, 3. Z axis add 2.1
                cur_pc_data = np.dstack((-velodyne_msg[4][:,1], velodyne_msg[4][:,0], velodyne_msg[4][:,2]+2.1 ))[0]
                #cur_pc_data = np.dstack((velodyne_msg[4][:,2], velodyne_msg[4][:,1], velodyne_msg[4][:,0]))[0]
            else :
                cur_pc_data = velodyne_msg[4][:, 0:3]
            cur_pc_color = np.zeros((np.shape(cur_pc_data)[0], 4))
            cur_pc_color[:,:] = pg.glColor('w')
            #cur_pc_color[:,:] = pg.glColor('g')
            # cur_pc_color[:,:] = pg.glColor('#000040FF')
            item['graphed'] = [cur_pc_data, cur_pc_color]
            pass
        else:
            pass
        _outq.put(item)
        gupd._call_count += 1
#    except Exception as e:
#        print(e)


    if color_table is not None : del color_table
    if cfg != None : del cfg
    if shmem != None : shmem.close()
    print('<<EXIT ProcessGraphData')


# 
# ----------------------------------
class GraphDataUpdater():
    _invalidate_graph = pyqtSignal('PyQt_PyObject')
    _gcm = None  #  ConfigManager
    _is_run = mp.RawValue(ctypes.c_bool, False)
    _is_gcm_chaned = mp.RawValue(ctypes.c_bool, False)
    _graph_process = None
    _recent_process_ms = 0
    _call_count = 0

    # ----------------------------------
    def __init__(self, inque, outque, _cm):
        self._in_q = inque
        self._out_q = outque
        self._call_count = 0
        self._gcm = _cm

    def set_gcm(self, cm):
        self._gcm = cm
        self._is_gcm_chaned.value = True

    def reset(self):
        self._call_count = 0

    def stop(self):
        print('>>Stop GraphDataUpdater')
        self._is_run.value = False
        self._graph_process.join()

    def start(self):
        self._is_run.value = True
        self._graph_process = mp.Process(target=ProcessGraphData, args=(self, self._in_q, self._out_q, self._gcm.shared_name(), self._is_gcm_chaned, self._is_run))
        self._graph_process.start()

# ----------------------------------
# ----------------------------------
class GuiUpdater(QThread):
    _invalidate_gui = pyqtSignal('PyQt_PyObject')
    _update_console = pyqtSignal(object)
    _priority=QThread.HighestPriority-2
    _do_run = False
    _call_count = 0
    _stack_size = 0

    # ----------------------------------
    def __init__(self, inque, outque, stack_size):
        QThread.__init__(self)
        self._in_q = inque
        self._out_q = outque
        self._stack_size = stack_size

    def reset(self):
        self._call_count = 0

    def stop(self):
        print('>>Stop GuiUpdater')
        self._do_run = False
        self.wait()

    def run(self):
        if vcdbg_on : debugpy.debug_this_thread()
        self._do_run = True
        print('>>START GuiUpdater')
        while self._do_run :
            try :
                if self._stack_size > 0 : ## stacking
                    item = self._in_q.get(timeout=0.5)
                    if item != None :
                        self._out_q.put(item)
                    if type(item).__name__ == 'dict' :
                        if self._invalidate_gui != None: self._invalidate_gui.emit(item)
                    elif item != None :
                        if self._update_console != None: self._update_console.emit(item)
                    elif item == None : ## eof signal.. ?? replace ??
                        self._invalidate_gui.emit(item)
                else :
                    item = self._in_q.get(timeout=0.5)
                    if type(item).__name__ == 'dict' :
                        if self._invalidate_gui != None: self._invalidate_gui.emit(item)
                    elif item != None :
                        if self._update_console != None: self._update_console.emit(item)
                    elif item == None : ## eof signal.. ?? replace ??
                        self._invalidate_gui.emit(item)
                    self._out_q.put(item)
                self._call_count += 1
            except IndexError:
                self.msleep(50)
            except queue.Empty:
                 continue
            except Exception as e :
                print( "Exception, GuiUpdater=",  e)
        print('<<EXIT GuiUpdater')
