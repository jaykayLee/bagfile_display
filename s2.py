import sys
import datetime
import time
import math
import struct
import os
import string
import re
import numpy as np
import multiprocessing as mp
import ctypes
from collections import deque
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal, QMutex, QMutexLocker, QPoint
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
    QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy, QMainWindow,
    QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit, QPlainTextEdit,
    QVBoxLayout, QWidget, QFileDialog, QButtonGroup, QMessageBox, QSplitter, QFrame, QDialogButtonBox, QSpacerItem)
from PyQt5.QtGui import QPixmap, QImage, QMouseEvent
import OpenGL.GL as GL
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import cv2
from bag_parse import BagFileReader, str_bag_time
from sriowrapper import *
from chain import SRDataStep
from gcm import ConfigManager
from gmf import QueCleaner, DataChain, GraphDataUpdater, GuiUpdater
from srwidget import *
from srcam import SRCamera
from srscatter import SRTargetList, SRTargetScatter
vcdbg_on = True ## vscode debugger
if vcdbg_on :
    import debugpy


# ----------------------------------
# ----------------------------------
class ConfigPortDialog(QDialog):
    _ipaddr = None
    _config_port = None
    _data_port = None
    _camera_port = None
    def __init__(self, par=None):
        super(ConfigPortDialog, self).__init__(par)
        self.setWindowTitle('Config Port')

        glay = QGridLayout(self)
        grid_row = 0
        # ip address
        glay.addWidget(QLabel('IP Address'), grid_row, 0, 1, 1)
        self._ipaddr_combo = QComboBox()
        self._ipaddr_combo.addItems("None,127.0.0.1,192.168.137.15,192.168.137.16,192.168.137.17,192.168.137.18,192.168.137.19,192.168.137.20,192.168.30.15".split(','))
        self._ipaddr_combo.setEditable(True)
        glay.addWidget(self._ipaddr_combo, grid_row, 1, 1, 1); grid_row+=1
        # config port
        glay.addWidget(QLabel('Config Port'), grid_row, 0, 1, 1)
        self._cfgport_combo = QComboBox()
        self._cfgport_combo.addItems("None,50000,9000,14000,2525".split(','))
        self._cfgport_combo.setEditable(True)
        glay.addWidget(self._cfgport_combo, grid_row, 1, 1, 1); grid_row+=1
        # data port
        glay.addWidget(QLabel('Data Port'), grid_row, 0, 1, 1)
        self._dataport_combo = QComboBox()
        self._dataport_combo.addItems("None,29172,15000,15001,15002".split(','))
        self._dataport_combo.setEditable(True)
        glay.addWidget(self._dataport_combo, grid_row, 1, 1, 1); grid_row+=1
        # camera port
        glay.addWidget(QLabel('Camera'), grid_row, 0, 1, 1)
        self._cam_combo = QComboBox()
        self._cam_combo.addItem("None")
        for ci in SRCamera.scan() :
            self._cam_combo.addItem( str(ci[0]) )
        glay.addWidget(self._cam_combo, grid_row, 1, 1, 1); grid_row+=1
        ###
        glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding), grid_row, 1, 1, 2)
        grid_row += 1

        self.btnbox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
        glay.addWidget(self.btnbox, grid_row, 0, 1, 2)
        self.setLayout(glay)
        self.btnbox.accepted.connect(self.click_ok)
        self.btnbox.rejected.connect(self.click_cancel)
        
        ## additional setting
        
        # self._lidar_on.setChecked(self._dict['lidar_on'])
        # self._lidar_pc_size_combo.setCurrentText(str(self._dict['lidar_pc_size']))
        # self._pc_size_combo.setCurrentText(str(self._dict['pc_size']))
        # self._pc_color_combo.setEnabled(False)

    def click_ok(self):
        if True :
            self._ipaddr = self._ipaddr_combo.currentText().strip()
            self._config_port = self._cfgport_combo.currentText().strip()
            self._data_port = self._dataport_combo.currentText().strip()
            self._camera_port = self._cam_combo.currentText().strip()
            
            self.accept()
        else :
            self._ipaddr = self._ipaddr_combo.currentText()
            self._config_port = self._cfgport_combo.currentText()
            self._data_port = self._dataport_combo.currentText()
            self._camera_port = self._cam_combo.currentText()
            self.accept()
        
    def click_cancel(self):
        self.reject()

class ConsoleDialog(QDialog):
    _send_command_cb = pyqtSignal(object)
    def __init__(self, par=None):
        super(ConsoleDialog, self).__init__(par)
        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle('Radar Console')

        glay = QGridLayout(self)
        grid_row = 0
        glay.addWidget(QLabel("Console Output"), grid_row, 0, 1, 1)
        glay.addWidget(QLabel("Command lines"), grid_row, 5, 1, 1); grid_row+=1
        self._console_output = QPlainTextEdit(self)
        glay.addWidget(self._console_output, grid_row, 0, 1, 5)
        self._commandset_edit = QPlainTextEdit(self)
        glay.addWidget(self._commandset_edit, grid_row, 5, 1, 3); grid_row+=1

        self._clear_button = QPushButton("Clear", self)
        self._load_button = QPushButton("Load", self)
        self._save_button = QPushButton("Save", self)
        self._send_button = QPushButton("Send", self)
        glay.addWidget(self._clear_button, grid_row, 0, 1, 1)
        glay.addWidget(self._load_button, grid_row, 5, 1, 1)
        glay.addWidget(self._save_button, grid_row, 6, 1, 1)
        glay.addWidget(self._send_button, grid_row, 7, 1, 1)
        grid_row+=1
        self._command_edit = QLineEdit()
        self._cmd_button = QPushButton("Cmd", self)
        glay.addWidget(self._command_edit, grid_row, 0, 1, 7)
        glay.addWidget(self._cmd_button, grid_row, 7, 1, 1)
        ###
        glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding), grid_row, 0, 1, 1)
        grid_row += 1

        self._close_button = QPushButton("Close", self)
        glay.addWidget(self._close_button, grid_row, 6, 1, 1)
        glay.setRowStretch(1, 20)
        self.setLayout(glay)
        #self.btnbox = QDialogButtonBox(QDialogButtonBox.Ok, Qt.Horizontal, self)
        #self.btnbox.accepted.connect(self.click_ok)

        self._clear_button.clicked.connect(self.clicked_clear_button)
        self._load_button.clicked.connect(self.clicked_load_button)
        self._save_button.clicked.connect(self.clicked_save_button)
        self._cmd_button.clicked.connect(self.clicked_cmd_button)
        self._send_button.clicked.connect(self.clicked_send_button)
        self._command_edit.returnPressed.connect(self.clicked_cmd_button)
        self._close_button.clicked.connect(self.clicked_close_button)

        self._cmd_button.setDefault(True)

    def clicked_clear_button(self):
        self._console_output.clear()

    def clicked_load_button(self):
        fd = QFileDialog()
        filt = "cfg(*.cfg)"
        filename = fd.getOpenFileName(filter=filt)
        if filename == None or os.path.exists(filename[0]) != True :
            return
        self._commandset_edit.clear()
        fp = open(filename[0], 'r')
        lines = fp.readlines()
        for ln in lines:
            ln = ln.strip()
            self._commandset_edit.appendPlainText(ln)
        fp.close()

    def clicked_save_button(self):
        pass

    def clicked_send_button(self):
        lines = re.split('\r|\n', self._commandset_edit.toPlainText())
        newlines = []
        for ln in lines:
            ln = ln.strip()
            if len(ln) < 1 or ln[0] == '%' :
                continue
            newlines.append(ln+'\r')
        self._send_command_cb.emit(newlines)

    def clicked_cmd_button(self):
        ln = self._command_edit.text().strip()
        if len(ln) < 1 :
            return
        print(f"cmds:{ln}")
        self._command_edit.setText("")
        self._send_command_cb.emit(ln+'\r')
        
    def clicked_close_button(self):
        self.accept()
    
    def append_console_data(self, msgs):
        if type(msgs).__name__ != 'str' : # bytes, bytearray
            msgs = msgs.decode()
        self._console_output.appendPlainText(msgs)

# ----------------------------------
class PCDataDialog(QDialog):
    def __init__(self, par=None):
        super(PCDataDialog, self).__init__(par)
        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle('Point Cloud Data')

        glay = QGridLayout(self)
        grid_row = 0
        # pcd enable/disable
        self._pcd_update_on = QCheckBox('on/off')
        glay.addWidget(self._pcd_update_on, grid_row, 0, 1, 1); grid_row+=1
        ## text area
        self._pcd_text_area = QPlainTextEdit(self)
        glay.addWidget(self._pcd_text_area, grid_row, 0, 1, 1)
        ###
        glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding), grid_row, 0, 1, 1)
        grid_row += 1

        self.btnbox = QDialogButtonBox(QDialogButtonBox.Ok, Qt.Horizontal, self)
        glay.addWidget(self.btnbox, grid_row, 0, 1, 1)
        glay.setRowStretch(1, 10)
        self.setLayout(glay)
        self.btnbox.accepted.connect(self.click_ok)
        self._pcd_update_on.setChecked(True)
        

    def click_ok(self):
        self.accept()

    def invalidate_pcd(self, records, clips):
        if not self._pcd_update_on.isChecked() :
            return

        if type(records).__name__ == 'list' :
            pcd = records[len(records)-1]
            str = f"chunk={pcd['chunk_idx']}  topic={pcd['topic']}  time={ str_bag_time(pcd['time'][0], pcd['time'][1]) }\n"
            str += f"stacking size : {len(records)}\n"
            for ri in range(len(records)-1, -1, -1):
                str += f"## stack index : {ri}\n"
                pcs = records[ri]['dataprocessed'][4]
                if clips != None :
                    pts = SRDataStep.process_clipping_by_area(pcs, clips, [0,0,0,g_cm._cfg.clip_opt[3]], g_cm._cfg.idx_doppler)
                    str += f"   Clipping Points:{len(pts)}/{len(pcs)}  X({clips[0]:.2f}~{clips[1]:.2f}) Y({clips[2]:.2f}~{clips[3]:.2f}) Z({clips[4]:.2f}~{clips[5]:.2f})\n"
                    pcs = pts
                else :
                    str += f"   Points:{pcs.shape[0]} =\n"
                str += "    " + "-"*80; str+="\n"
                str += "    IDX=[  X   Y   Z   --   X.D   Y.D   Z.D   --   PWR   Doppler   --   -- ]\n"
                str += "    " + "-"*80; str+="\n"
                i = 0
                for pc in pcs:
                    str += f"   {i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
                    i += 1

                #str += f"{pcs.shape[0]} = {np.array2string(pcs, separator=',', formatter={'float_kind':lambda pcs: '%.2f' % pcs})}"
                str += "    " + "-"*80; str+="\n"

                str += f"   Mean[{np.mean(pcs[:,0]):.3f}, {np.mean(pcs[:,1]):.3f}, {np.mean(pcs[:,2]):.3f}, {np.mean(pcs[:,8]):.3f}, {np.mean(pcs[:,9]):.3f}]\n"
                str += f"    STD[{np.std(pcs[:,0]):.3f}, {np.std(pcs[:,1]):.3f}, {np.std(pcs[:,2]):.3f}, {np.std(pcs[:,8]):.3f}, {np.std(pcs[:,9]):.3f}]\n"

            self._pcd_text_area.setPlainText(str)
            return
        
        ## elif type(records).__name__ == 'dict' :
        # {'chunk_idx':chunk_idx, 'conn':sub_hds['conn'], 'topic':topic \
        #                      , 'time':sub_hds['time'], 'message':mdata}
        pcd = records
        app_type = pcd['app_type']
        if app_type == SR4AppType.AT_R4BAG :
            str = f"chunk={pcd['chunk_idx']}  topic={pcd['topic']}  time={ str_bag_time(pcd['time'][0], pcd['time'][1]) }\n"
            point_str = ""
            pcs = pcd['dataprocessed'][4]

            if clips != None :
                pts = SRDataStep.process_clipping_by_area(pcs, clips, [0,0,0,g_cm._cfg.clip_opt[3]], g_cm._cfg.idx_doppler)
                str += f"Clipping Points:{len(pts)}/{len(pcs)}  X({clips[0]:.2f}~{clips[1]:.2f}) Y({clips[2]:.2f}~{clips[3]:.2f}) Z({clips[4]:.2f}~{clips[5]:.2f})\n"
                pcs = pts
            else :
                str += f"Points:{pcs.shape[0]} =\n"

            point_str += "-"*80; point_str+="\n"
            point_str += "IDX=[  X   Y   Z   --   X.D   Y.D   Z.D   --   PWR   Doppler   --   -- ]\n"
            point_str += "-"*80; point_str+="\n"
            i = 0
            for pc in pcs:
                point_str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
                i += 1

            #str += f"{pcs.shape[0]} = {np.array2string(pcs, separator=',', formatter={'float_kind':lambda pcs: '%.2f' % pcs})}"
            str += "-"*80; str+="\n"
            if len(pcs) > 0 :
                _posx = [ np.min(pcs[:,0]), np.max(pcs[:,0]), 0 ];   _posx[2] = _posx[1] - _posx[0]
                _posy = [ np.min(pcs[:,1]), np.max(pcs[:,1]), 0 ];   _posy[2] = _posy[1] - _posy[0]
                _posz = [ np.min(pcs[:,2]), np.max(pcs[:,2]), 0 ];   _posz[2] = _posz[1] - _posz[0]
                _pwr = [ np.min(pcs[:,8]), np.max(pcs[:,8]), 0, np.sum(pcs[:,8])];  _pwr[2] = _pwr[1] - _pwr[0]
                _dop = [ np.min(pcs[:,9]), np.max(pcs[:,9]), 0 ];    _dop[2] = _dop[1] - _dop[0]
                str += f"X[{_posx[0]:.3f} {_posx[1]:.3f} {_posx[2]:.3f}] Y[{_posy[0]:.3f} {_posy[1]:.3f} {_posy[2]:.3f}] Z[{_posz[0]:.3f} {_posz[1]:.3f} {_posz[2]:.3f}]\n"
                str += f"Power[{_pwr[0]:.3f} {_pwr[1]:.3f} {_pwr[2]:.3f} sum={_pwr[3]:.3f}] Doppler[{_dop[0]:.3f} {_dop[1]:.3f} {_dop[2]:.3f}]\n"
                
                str += f"Mean[{np.mean(pcs[:,0]):.3f}, {np.mean(pcs[:,1]):.3f}, {np.mean(pcs[:,2]):.3f}, {np.mean(pcs[:,8]):.3f}, {np.mean(pcs[:,9]):.3f}]\n"
                str += f" STD[{np.std(pcs[:,0]):.3f}, {np.std(pcs[:,1]):.3f}, {np.std(pcs[:,2]):.3f}, {np.std(pcs[:,8]):.3f}, {np.std(pcs[:,9]):.3f}]\n"
            str += point_str
            ## mean, std, for range, x, y, z, eng, dopp and...???
        else : #   SR4AppType.AT_R4FDEV, SR4AppType.AT_R4FFILE
            str = f"FrameNum={pcd['chunk_idx']}  time={ str_bag_time(pcd['time'][0], pcd['time'][1]) }\n"
            pint_str = ""
            pcs = pcd['dataprocessed'][4]

            if clips != None :
                pts = SRDataStep.process_clipping_by_area(pcs, clips, [0,0,0,g_cm._cfg.clip_opt[3]], g_cm._cfg.idx_doppler)
                str += f"Clipping Points:{len(pts)}/{len(pcs)}  X({clips[0]:.2f}~{clips[1]:.2f}) Y({clips[2]:.2f}~{clips[3]:.2f}) Z({clips[4]:.2f}~{clips[5]:.2f})\n"
                pcs = pts
            else :
                str += f"Points:{pcs.shape[0]} =\n"

            pint_str += "-"*80; point_str+="\n"
            pint_str += "IDX=[  X   Y   Z  Doppler   PWR   ]\n"
            pint_str += "-"*80; point_str+="\n"
            i = 0
            for pc in pcs:
                rng = np.sqrt(pc[0]**2+pc[1]**2+pc[3]**2)
                ang = np.arctan(pc[2]/rng)
                ang = np.rad2deg(ang)
                #ang = math.atan(pc[2]/np.sqrt(pc[0]**2+pc[1]**2+pc[3]**2))
                #ang = math.degrees(ang)
                pint_str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}, [{rng:.2f}, {ang:.2f}]\n"
                i += 1

            #str += f"{pcs.shape[0]} = {np.array2string(pcs, separator=',', formatter={'float_kind':lambda pcs: '%.2f' % pcs})}"
            str += "-"*80; str+="\n"
            str += pint_str
            
            
        self._pcd_text_area.setPlainText(str)

    # def update_pcd_in_area(self, pts, clips):
    #     if not self._pcd_update_on.isChecked() :
    #         return
    #     if pts == None or len(pts) < 1 :
    #         return
    #     str = f"Points:{len(pts)}  X({clips[0]}~{clips[1]}) Y({clips[2]}~{clips[3]}) Z({clips[4]}~{clips[5]})"
    #     str += "-"*80; str+="\n"
    #     str += "  1.X   2.Y   3.Z  4.RSV  5.doppler-X 6.doppler-Y 7.doppler-Z 8.RSV 9.PWR  10.Doppler 11.RSV 12.RSV"
    #     str += "-"*80; str+="\n"
    #     i = 0
    #     for pc in pts:
    #         #str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=200, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
    #         str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
    #         i += 1
    #     self._pcd_text_area.setPlainText(str)

        
# ----------------------------------
# ----------------------------------
class SetGUIDialog(QDialog):
    def __init__(self, _dict, par=None):
        super(SetGUIDialog, self).__init__(par)
        self.setWindowTitle('Config GUI')
        self._dict = _dict

        glay = QGridLayout(self)
        grid_row = 0
        # lidar point cloud on/off
        self._lidar_on = QCheckBox('Lidar PC On/off (if exist)')
        glay.addWidget(self._lidar_on, grid_row, 0, 1, 2); grid_row+=1
        ## lidar point cloud size
        glay.addWidget(QLabel('Lidar PC Size is '), grid_row, 0, 1, 1)
        self._lidar_pc_size_combo = QComboBox()
        self._lidar_pc_size_combo.addItems("1,2,3,4,5".split(','))
        glay.addWidget(self._lidar_pc_size_combo, grid_row, 1, 1, 1); grid_row+=1
        ###
        glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding), grid_row, 1, 1, 2)
        grid_row += 1
        # radar point cloud size
        glay.addWidget(QLabel('PC Size is '), grid_row, 0, 1, 1)
        self._pc_size_combo = QComboBox()
        self._pc_size_combo.addItems("1,2,3,4,5".split(','))
        glay.addWidget(self._pc_size_combo, grid_row, 1, 1, 1); grid_row+=1
        # radar point cloud color index
        glay.addWidget(QLabel('PC color by '), grid_row, 0, 1, 1)
        self._pc_color_combo = QComboBox()
        self._pc_color_combo.addItems("height,snr,doppler".split(','))
        glay.addWidget(self._pc_color_combo, grid_row, 1, 1, 1); grid_row+=1
        ###
        glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding), grid_row, 1, 1, 2)
        grid_row += 1

        self.btnbox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
        glay.addWidget(self.btnbox, grid_row, 0, 1, 2)
        self.setLayout(glay)
        self.btnbox.accepted.connect(self.click_ok)
        self.btnbox.rejected.connect(self.click_cancel)
        
        ## additional setting
        
        self._lidar_on.setChecked(self._dict['lidar_on'])
        self._lidar_pc_size_combo.setCurrentText(str(self._dict['lidar_pc_size']))
        self._pc_size_combo.setCurrentText(str(self._dict['pc_size']))
        self._pc_color_combo.setEnabled(False)

    def click_ok(self):
        if self._dict != None and type(self._dict).__name__ == 'dict' :
            self._dict['lidar_on'] = self._lidar_on.isChecked()
            self._dict['lidar_pc_size'] = int(self._lidar_pc_size_combo.currentText())
            self._dict['pc_size'] = int(self._pc_size_combo.currentText())
            self._dict['pc_color_index'] = self._pc_color_combo.currentText()
        self.accept()

    def click_cancel(self):
        self.reject()


# ----------------------------------
class SRMainWindow(QDialog):
    _pc_car = None
    _bagf = None
    _record_message = None
    _color_table = None
    _gui_cfg = dict()
    _pcd_win = None
    _console_win = None
    _invalidate_pcd = None # pyqtSignal('PyQt_PyObject')
    _start_time = None
    _end_time = None
    _dur_time_str = ""
    _cur_rec_msg = None
    _cur_rec_clips = None
    _cur_lidar_pcs = None
    _pc_w_mode = 0 # 0:view 1:edit
    _camera = None
    _ind_que : mp.Queue = None
    #_stack_que : mp.Queue = None
    _stack_list = []
    _app_type = SR4AppType.AT_UNKNOWN
    _frame_list = None
    _chunk_list = None
    _chunk_offset = 0
    _trk_info_summary = None
    _io_name = ''

    # --- workers' queue
    # _message_que = None     #  source : play to datachain
    # _data_que = None        #  source : datachain to graph_data
    # _graph_que = None       #  source : graph_data to gui_updater
    # _gui_que = None         #  source : gui_updatoer to cleaner.. (inter-operation is invalidate..)
    # --- workers
    # _player = None
    # _data_chain = None
    # _graph_data_updater = None
    # _gui_updater = None
    # _que_cleaner = None

    def __init__(self, parent=None, size=[]):
        super(SRMainWindow, self).__init__(parent)
        #self._z_range=g_cm._z_range
        g_cm.set_win(self)

        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle("SRS Visual")

        self.load_configs()

        self.prepare_graph_layout()
        self.prepare_control_layout()
        self.prepare_status_box()
        self.prepare_image_layout()
        self.prepare_log_layout()

        vbox = QVBoxLayout()
        self._splitter1 = QSplitter(Qt.Vertical)
        self._splitter2 = QSplitter(Qt.Horizontal)
        self._splitter3 = QSplitter(Qt.Vertical)

        self._splitter1.addWidget(self._control_box)
        self._splitter1.addWidget(self._status_box)
        self._splitter1.addWidget(self._image_box)
        self._splitter1.setSizes([int(self._splitter1.size().height() * 0.3), int(self._splitter1.size().height() * 0.2), int(self._splitter1.size().height() * 0.5)])

        self._splitter2.addWidget(self._splitter1)
        self._splitter2.addWidget(self._pc_box)
        self._splitter2.setSizes([int(self._splitter2.size().width() * 0.1), int(self._splitter2.size().width() * 0.9)])

        self._splitter3.addWidget(self._splitter2)
        self._splitter3.addWidget(self._hlbox)
        self._splitter3.setSizes([int(self._splitter3.size().height() * 0.9), int(self._splitter3.size().height() * 0.1)])

        vbox.addWidget(self._splitter3)
        self.setLayout(vbox)
        self._splitter2.setStretchFactor(1, 8)
        self._splitter3.setStretchFactor(0, 10)
        ## for updater
        g_gui_updater._invalidate_gui.connect(self.invalidate_graph)
        g_gui_updater._update_console.connect(self.update_message)
        self._camera = SRCamera()

    def do_finalize(self):
        self._camera.enable(False)
        if self._pcd_win != None :
            self._pcd_win.close()
            del self._pcd_win
            self._pcd_win = None
        if self._console_win != None :
            self._console_win.close()
            del self._console_win
            self._console_win = None

    def prepare_log_layout(self):
        self._hlbox = QGroupBox()
        hlgbox = QGridLayout()

        #self._hlbox.setSizeIncrement()

        self._slog_clear_button = QPushButton("Cl")
        self._slog_clear_button.clicked.connect(self.logclear)
        self._slog_clear_button.setFixedWidth(20)
        self._slog_viewer = QPlainTextEdit()

        hlgbox.addWidget(self._slog_clear_button, 0, 0)
        hlgbox.addWidget(self._slog_viewer, 0, 1, 1, 10)
        self._hlbox.setLayout(hlgbox)

    def logout(self, fmsg):
        self._slog_viewer.appendPlainText(fmsg)

    def logclear(self):
        self._slog_viewer.clear()

    def do_log_view(self):
        #self._hbox.
        pass

    def prepare_graph_layout(self):
        self._pc_box = QGroupBox()

        self._gw = pg.GradientWidget(orientation='right')
        self._gw.restoreState({'ticks': [(1.0, (0, 255, 255, 255)), (0.0, (255, 255, 0, 255))], 'mode': 'hsv'})

        self._pc_w = SRViewWidget()
        self._pc_w._select_region_cb.connect(self.selected_pc_w_region)
        self._pc_w._clicked_cb.connect(self.clicked_pc_w)
        #self._pc_w = gl.GLViewWidget()
        # def pcw_mousePressEvent(self, a0: QMouseEvent) -> None:
        #     pos = a0.pos()
        #     print(f"Mouse Press {a0.type}, [{pos.x}, {pos.y}]")
        #     return super().mousePressEvent(a0)
        # self._pc_w.mouseMoveEvent = pcw_mousePressEvent
        self._dummy_pos = np.zeros((1,3))
        #self._dummy_pos[0,0] = 1.0; self._dummy_pos[0,1] = 1.0
        self._stacked_colors = np.array((0.0,0.0,0.5,1))
        #self._grayed_colors = pg.glColor('w')

        self._pc_axis = gl.GLAxisItem(glOptions='opaque')
        self._pc_axis.setSize(x=10, y=20, z=5)
        self._pc_ground = gl.GLGridItem()
        self._pc_ground.translate(0, 0, 0)
        self._pc_ground.setSize(200, 200, 0)
        self._pc_wground = gl.GLGridItem( color=(255,255,255,0), glOptions='opaque')
        #self._pc_wground.setColor('y')
        self._pc_wground.setSpacing(10,10,0)
        self._pc_wground.setSize(200, 200, 0)
        self._pc_scatter = gl.GLScatterPlotItem(size=3)
        self._pc_scatter.setData(pos=self._dummy_pos)
        self._veldyne_scatter = gl.GLScatterPlotItem(size=1)
        self._veldyne_scatter.setData(pos=self._dummy_pos)
        # self._target_scatter = SRTargetScatter()
        # self._target_scatter.setData(pos=self._dummy_pos)
        self._targetbox_list = SRTargetList()

        qimg = QImage()
        ##if qimg.load('./resource/car.png') :
        if qimg.load('./resource/center.png') :
            #qimg = qimg.convertToFormat(QImage.Format.Format_RGBA8888)
            imgarr = pg.imageToArray(qimg, copy=True, transpose=True)
            ##imgarr = np.flip(imgarr, axis=1)
            self._pc_car = gl.GLImageItem(imgarr)
            self._pc_car.scale(x=0.008, y=0.008, z=1)  ## adjust roughly as Tivoli( Y: 4,225mm, X: 1,810mm, Z: 1,615~1,620mm )
            self._pc_car.translate(-(imgarr.shape[0]*0.008/2), -(imgarr.shape[1]*0.008/2), 0)
        
        if self._pc_car != None :
            self._pc_w.addItem(self._pc_car)
        self._pc_w.addItem(self._pc_scatter)
        #self._pc_w.addItem(self._target_scatter)
        self._targetbox_list.add_to_parent(self._pc_w)
        self._pc_w.addItem(self._veldyne_scatter)
        self._pc_w.addItem(self._pc_axis)
        self._pc_w.addItem(self._pc_wground)
        self._pc_w.addItem(self._pc_ground)
        layout = QGridLayout()
        layout.addWidget(self._pc_w, 0, 0)
        layout.addWidget(self._gw, 0, 1)
        layout.setColumnStretch(0,6)
        self._pc_box.setLayout(layout)


    def prepare_control_layout(self):
        self._control_box = QGroupBox()
        clay = QHBoxLayout()
        self._app_type_combo = QComboBox()
        self._app_type_combo.addItems(SR4AppType.NAME)
        self._connect_button = QPushButton("Connect", self)
        self._console_button = QPushButton("Cmds", self)
        clay.addWidget(self._app_type_combo)
        clay.addWidget(self._connect_button)
        clay.addWidget(self._console_button)

        s0lay = QHBoxLayout()
        self._mode_button = QPushButton("VIEW", self)
        self._slog_button = QPushButton("slog", self)
        s0lay.addWidget(self._mode_button)
        s0lay.addWidget(self._slog_button)
        s1lay = QHBoxLayout()
        self._config_button = QPushButton("Config", self)
        self._gui_button = QPushButton("Gui", self)
        self._data_button = QPushButton("Data", self)
        s1lay.addWidget(self._config_button)
        s1lay.addWidget(self._gui_button)
        s1lay.addWidget(self._data_button)
        self._playstep_combo = QComboBox()
        self._playstep_combo.addItems("1,2,3,4,5,6,7,8,10,15,20,25,30,50,80,100,200,300,400,500,600,700,800,900,1000".split(','))
        self._play_button = QPushButton("Play", self)
        self._prev_button = QPushButton("Prev", self)
        self._next_button = QPushButton("Next", self)
        self._play_info = QLabel('')
        self._play_time = QLabel('')
        self._play_slider = QSlider(Qt.Orientation.Horizontal)
        hlay = QHBoxLayout()
        hlay.addWidget(self._playstep_combo)
        hlay.addWidget(self._prev_button)
        hlay.addWidget(self._play_button)
        hlay.addWidget(self._next_button)
        layout = QVBoxLayout()
        layout.addLayout(clay)
        layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        layout.addLayout(s0lay)
        layout.addLayout(s1lay)
        layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        layout.addWidget(self._play_info)
        layout.addWidget(self._play_time)
        layout.addWidget(self._play_slider)
        layout.addLayout(hlay)
        self._control_box.setLayout(layout)

        self._connect_button.clicked.connect(self.do_connect)
        self._console_button.clicked.connect(self.do_console_win)
        self._play_button.clicked.connect(self.do_play_message)
        self._prev_button.clicked.connect(self.do_prev_message)
        self._next_button.clicked.connect(self.do_next_message)
        self._play_slider.valueChanged.connect(self.do_play_slider_changed)
        self._play_slider.setEnabled(False)

        self._config_button.clicked.connect(self.do_configure)
        self._mode_button.clicked.connect(self.do_change_mode)
        self._slog_button.clicked.connect(self.do_log_view)
        self._gui_button.clicked.connect(self.do_config_gui)
        self._data_button.clicked.connect(self.do_load_data_win)


    def load_configs(self):
        self._gui_cfg['lidar_on'] = True
        self._gui_cfg['lidar_pc_size'] = 1
        self._gui_cfg['pc_size'] = 4
        self._gui_cfg['pc_color_index'] = 'height'

    def do_console_win(self):
        if self._console_win != None :
            print(f'Console Window exist, showing..')
            self._console_win.show()
            return
        def done_console_win(event):
            self._console_win = None
        self._console_win = ConsoleDialog()
        self._console_win.closeEvent = done_console_win
        self._console_win._send_command_cb.connect(self.send_radar_command)
        self._console_win.show()

    def do_load_data_win(self):
        if self._pcd_win != None :
            print(f'Point Cloud Window exist, showing..')
            self._pcd_win.show()
            return
        def done_data_win(event):
            self._pcd_win = None
        self._pcd_win = PCDataDialog()
        self._pcd_win.closeEvent = done_data_win
        self._pcd_win.show()

    def do_config_gui(self):
        ##
        # dict['lidar_on'] = self._lidar_on.isChecked()
        # dict['lidar_pc_size'] = int(self._lidar_pc_size_combo.currentText())
        # dict['pc_size'] = int(self._lidar_pc_size_combo.currentText())
        # dict['pc_color_index'] = self._pc_color_combo.currentText()
        guicfgdlg = SetGUIDialog(self._gui_cfg)
        if guicfgdlg.exec_() == QDialog.Accepted :
            print(f'GUI Config = {self._gui_cfg}')

    def do_change_mode(self):
        #self._pc_w_mode = 0
        if self._pc_w_mode == 0 : # normal view
            self._mode_button.setText("Search")
            self._pc_w_mode = 1   # edit mode
            self._pc_w.set_clipmode(True)
            self.do_load_data_win()
            self._cur_rec_clips = None
            #self._pcd_win._pcd_update_on = T
        else : # self._pc_w_mode == 1 :
            self._mode_button.setText("View")
            self._pc_w_mode = 0   # view mode
            self._pc_w.set_clipmode(False)
            self._cur_rec_clips = None
        #self._target_scatter = 
        # pos = np.random.random(size=(100000,3))
        # pos *= [10,-10,10]
        # pos[0] = (0,0,0)
        # color = np.ones((pos.shape[0], 4))
        # d2 = (pos**2).sum(axis=1)**0.5
        # size = np.random.random(size=pos.shape[0])*10
        # self._target_scatter.setData(pos=pos, color=color, size=size)

    def selected_pc_w_region(self, pos):
        start = pos[0]
        end = np.copy(pos[1])
        start[2] = -5.0
        if end[2]==0. : end[2] = 15.0
        clips = [0,0,0,0,0,0, g_cm._cfg.clips[6], g_cm._cfg.clips[7] ]
        if start[0]<end[0] : clips[0]=start[0]; clips[1]=end[0]
        else : clips[1]=start[0]; clips[0]=end[0]
        if start[1]<end[1] : clips[2]=start[1]; clips[3]=end[1]
        else : clips[3]=start[1]; clips[2]=end[1]
        if start[2]<end[2] : clips[4]=start[2]; clips[5]=end[2]
        else : clips[5]=start[2]; clips[4]=end[2]
        if self._pcd_win != None :
            self._pcd_win.invalidate_pcd(self._cur_rec_msg, clips)
            self._cur_rec_clips = clips
        # for pt in pts :
        #     print(pt)

    def clicked_pc_w(self, btn, lpos):
        self.do_change_mode()

    def do_play_slider_changed(self, v):
        if g_player.is_playing() :
            return
        print(f'value changed : {v}')
        self.do_change_play_pos(v)
        ##self._play_info.setText(f"{v}/{self._play_slider.maximum()}  {self._dur_time_str}")

    def do_configure(self):
        if g_cm.do_configure_set() :
            self.get_gw_color_table()
            g_data_chain.set_gcm(g_cm)
            g_graph_data_updater.set_gcm(g_cm)
            

    def prepare_status_box(self):
        self._status_box = QGroupBox()
        self._que_info = QLabel('Que=')
        self._cam_info = QLabel('CAM=')
        self._pc_info = QLabel('PC=')
        layout = QVBoxLayout()
        layout.addWidget(self._que_info)
        layout.addWidget(self._cam_info)
        layout.addWidget(self._pc_info)
        self._status_box.setLayout(layout)

    def prepare_image_layout(self):
        self._image_box = QGroupBox()
        self._image_w = QLabel()
        self._image_pixmap = QPixmap()
        layout = QVBoxLayout()
        layout.addWidget(self._image_w)
        self._image_box.setLayout(layout)

    def get_gw_color_table(self):
        # if self._color_table != None:
        #     return

        #color_range = self._z_range[1]+abs(self._z_range[0]) 
        color_range = g_cm._cfg.pc_color_zmax+abs(g_cm._cfg.pc_color_zmin)
        crange = int(color_range*10)
        self._color_table = np.zeros((crange, 4))
        for i in range(crange):
            #cur_pc_color[i]=pg.glColor(self._gw.getColor(abs(zs/colorRange)))
            self._color_table[i] = pg.glColor(self._gw.getColor(i/crange))
        #g_cm._color_table = self._color_table
        g_cm.update_color_table(self._color_table)

    def do_connect(self):
        if self._connect_button.text() == 'Connect':
            res = False
            app_type_index = self._app_type_combo.currentIndex()
            res = self.do_connect_io(app_type_index)
            if res != True :
                self._app_type = SR4AppType.AT_UNKNOWN
                return
            self._connect_button.setText("Disconnect")
            self._app_type_combo.setEnabled(False)
        else :
            self._connect_button.setText("Connect")
            self.do_disconnect_io()
            self._app_type_combo.setEnabled(True)

    def do_connect_io(self, app_type):
        try:
            if app_type == SR4AppType.AT_R4FDEV :
                cfgdlg = ConfigPortDialog()
                if cfgdlg.exec_() != QDialog.Accepted :
                    return False
                if cfgdlg._camera_port != 'None' :
                    self._camera.set(cfgdlg._camera_port)
                    self._camera._capture_cb.connect(self.invalidate_cam)
                    self._camera.enable(True)
                g_player.open_socket(app_type, cfgdlg._ipaddr, int(cfgdlg._config_port) if cfgdlg._config_port!='None' else 0, int(cfgdlg._data_port))
                self._start_time = get_cur_timestamp()
                g_cm._cfg.idx_doppler = 3
                self._io_name = f"{cfgdlg._ipaddr}-{cfgdlg._config_port}-{datetime.datetime.fromtimestamp(self._start_time[0]).strftime('%Y%m%d_%H%M%S')}"
            elif app_type == SR4AppType.AT_R4FFILE :
                fd = QFileDialog()
                filt = "bin(*.bin)"
                filename = fd.getOpenFileName(filter=filt)
                if filename == None or  os.path.exists(filename[0]) != True :
                    return False
                g_player.open_binfile(app_type, filename[0])
                time.sleep(0.5)
                flist = g_player.get_frame_list()
                if flist is not None :
                    self._start_time = flist[0][1]
                    self._end_time = flist[len(flist)-1][1]
                    self._frame_list = flist
                    self._chunk_offset = flist[0][0]
                    self._dur_time_str = f"{datetime.datetime.fromtimestamp(self._start_time[0]).strftime('%Y/%m/%d %H:%M:%S')}~{datetime.datetime.fromtimestamp(self._end_time[0]).strftime('%H:%M:%S')}"
                    self._play_slider.setValue(0)
                    self._play_slider.setRange(0, len(flist))
                    self._play_slider.setEnabled(True)
                else :
                    self._start_time = get_cur_timestamp()
                    self._dur_time_str = f"unknown"
                self._io_name = f"{filename[0]}-{datetime.datetime.fromtimestamp(self._start_time[0]).strftime('%Y%m%d_%H%M%S')}"
                g_cm._cfg.idx_doppler = 3
            elif app_type == SR4AppType.AT_R4BAG :
                fd = QFileDialog()
                filt = "bag(*.bag)"
                filename = fd.getOpenFileName(filter=filt)
                if filename == None or  os.path.exists(filename[0]) != True :
                    return False
                bagfile = BagFileReader()
                if bagfile.open(filename[0]) :
                    bagfile.load( ('/usb_cam/image_raw/compressed', '/image_raw/compressed', '/point_cloud', '/velodyne_points') )
                    g_player.open_bagfile(bagfile, (False, False, True, False))
                    self._start_time = bagfile._chunks[0][2]
                    self._end_time = bagfile._chunks[len(bagfile._chunks)-1][3]
                    self._chunk_list = bagfile._chunks
                    self._chunk_offset = 0
                    self._dur_time_str = f"{datetime.datetime.fromtimestamp(self._start_time[0]).strftime('%Y/%m/%d %H:%M:%S')}~{datetime.datetime.fromtimestamp(self._end_time[0]).strftime('%H:%M:%S')}"
                    self._play_slider.setValue(0)
                    self._play_slider.setRange(0, len(bagfile._chunks))
                    self._play_slider.setEnabled(True)
                self._io_name = f"{filename[0]}-{datetime.datetime.fromtimestamp(self._start_time[0]).strftime('%Y%m%d_%H%M%S')}"
                g_cm._cfg.idx_doppler = 9

            self._app_type = app_type
            self.get_gw_color_table()
            g_data_chain.set_gcm(g_cm)
            g_graph_data_updater.set_gcm(g_cm)
            while not g_message_que.empty() :
                item = g_message_que.get(); del item
            while not g_data_que.empty() :
                item = g_data_que.get(); del item
            while not g_graph_que.empty() :
                item = g_graph_que.get(); del item
            while not g_gui_que.empty() :
                item = g_gui_que.get(); del item
            #self._static_scatter.setData(pos=self._dummy_pos)
            self._pc_scatter.setData(pos=self._dummy_pos)
            #self._target_scatter.setData(pos=self._dummy_pos)
            self._targetbox_list.reset()
            #self._targetbox_list.set_data(pos=self._dummy_pos, color=pg.glColor('#00FF0080'), size=(1,1,1) )
            #self._pc_w.paintGL()
            self._trk_info_summary = None
        except Exception as e:
            print(e)
            return False

        return True

    def do_disconnect_io(self):
        if g_player.is_playing() :
            g_player.pause()
        self._play_slider.setValue(0)
        self._play_slider.setRange(0, 1)
        self._play_slider.setEnabled(False)        
        if self._camera.is_enabled():
            self._camera.enable(False)
            self._camera._capture_cb.disconnect(self.invalidate_cam)
        g_player.close()
        self._play_slider.setEnabled(False)

    def do_prev_message(self):
        step = int(self._playstep_combo.currentText())
        g_player.prev(-step)

    def do_next_message(self):
        step = int(self._playstep_combo.currentText())
        g_player.next(step)

    def do_play_message(self):
        if self._play_button.text() == 'Play' :
            self._play_button.setText("Pause")
            g_player.play()
        else:
            #_player.
            g_player.pause()
            self._play_button.setText("Play")

    def do_change_play_pos(self, pos):
        if self._play_button.text() != 'Play' :
            g_player.pause()
            self._play_button.setText("Play")
        g_player.set_chunk_pos(pos)

    def invalidate_cam(self, img):
        self._image_pixmap.loadFromData(img)
        size = self._image_w.size()
        self._image_w.setPixmap( self._image_pixmap.scaled(size) )
        self._image_w.show()
        if self._app_type == SR4AppType.AT_R4FDEV:
            g_player.append_cam_data(self._camera.frame_num(), img)
            #g_player.append_cam_data(self._camera.frame_num(), h, w, c, imgarr.tobytes())

    def invalidate_graph(self, rec_msg):
        if g_cm._cfg.stack_size > 0 : # stacking
            self._invalidate_stacked_graph(rec_msg)
        else :
            self._in_invalidate_graph(rec_msg)

    def _in_invalidate_graph(self, rec_msg):
        if rec_msg == None :
            QMessageBox.about(self, 'Message', 'no more data')
            return
        # [chunk_id, conn,topic,start,end,message]
        self._que_info.setText(f'[{g_data_chain._call_count}:{g_data_chain._recent_process_ms}, \
{g_graph_data_updater._call_count}:{g_graph_data_updater._recent_process_ms}], \
[{g_message_que.qsize()}, {g_data_que.qsize()}, {g_graph_que.qsize()}, {g_gui_que.qsize()}]')

        if rec_msg['topic'] == '/usb_cam/image_raw/compressed' or rec_msg['topic']=='/image_raw/compressed':
            cam_msg = rec_msg['parsed']
            self._cam_info.setText(f'CAM={cam_msg[1]} {str_bag_time(cam_msg[2], cam_msg[3])}')
            self.invalidate_cam(cam_msg[4])
            if False:
                f = open(f'd:\\image_{rec_msgs[mi][0]}.jpg', 'wb')
                f.write(rec_msgs[mi][4])
                f.close()
        elif rec_msg['topic'] == '/point_cloud' :
            self._play_info.setText(f'{rec_msg["chunk_idx"]}, {rec_msg["chunk_idx"]-self._chunk_offset}/{self._play_slider.maximum()}  {self._dur_time_str}')
            diff_time = datetime.datetime.fromtimestamp(rec_msg['time'][0]) - datetime.datetime.fromtimestamp(self._start_time[0])
            self._play_time.setText(f"sec={int(diff_time.total_seconds())}    {int(diff_time.total_seconds()/60)}:{int(diff_time.total_seconds()%60)}  ")

            #print(f'invalidate - {rec_msg["chunk_idx"]}, {rec_msg["topic"]}')
            if g_player.is_playing() :
                self._play_slider.setValue(rec_msg["chunk_idx"] - self._chunk_offset)
                #self._play_slider.setTickPosition(rec_msg["chunk_idx"])

            parsed_msg = rec_msg['parsed']
            points_msg = rec_msg['graphed']
            cur_pc_data = points_msg[0]
            cur_pc_color = points_msg[1]
            cur_pc_size = self._gui_cfg['pc_size']
            self._pc_scatter.setData(pos=cur_pc_data, color=cur_pc_color, size=cur_pc_size) #, size=2 
            self._pc_info.setText(f'PC={parsed_msg[1]} {len(cur_pc_data)}/{len(parsed_msg[4])} {str_bag_time(parsed_msg[2], parsed_msg[3])}')
            if False:
                f = open(f'd:\\pc_{rec_msg[0]}.bin', 'wb')
                f.write(rec_msg[4])
                f.close()
            # if True :
            #     pc_polar = rec_msg['dataprocessed'][5][:, 0:3]
            #if self._invalidate_pcd != None :  self._invalidate_pcd.emit(rec_msg)
            if len(points_msg) > 4 :
                cur_tgt_data = points_msg[2]
                cur_tgt_color = points_msg[3]
                cur_tgt_label = points_msg[4]
                cur_tgt_size = (1,1,1)
                if cur_tgt_data is not None :
                    #self._target_scatter.setData(pos=cur_tgt_data, color=cur_tgt_color, size=cur_tgt_size) #, size=2 
                    self._targetbox_list.set_data(pos=cur_tgt_data, color=cur_tgt_color, size=cur_tgt_size, label=cur_tgt_label)
                    if len(rec_msg['dataprocessed']) > 5 and rec_msg['dataprocessed'][5][0] == 'SRSTracker' :
                        # '', numTrk, trk_info, Trk_Type_New, trkid_fin
                        ##str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
                        msg = f"CI:{rec_msg['chunk_idx']} Target=num:{rec_msg['dataprocessed'][5][1]}" \
                                    f" trkinfo:{np.array2string(rec_msg['dataprocessed'][5][2], separator=',', max_line_width=400)}\n" \
                                    f" trktype:{np.array2string(rec_msg['dataprocessed'][5][3], separator=',', max_line_width=400)} trkid:{rec_msg['dataprocessed'][5][4]}"
                        self.logout(msg)
                    else :
                        self.logout(f"CI:{rec_msg['chunk_idx']} Target=num:{len(cur_tgt_data)}")
                else :
                    #self._target_scatter.setData(pos=self._dummy_pos)
                    self._targetbox_list.reset()
                    self.logout(f"CI:{rec_msg['chunk_idx']} Target=num:0")
                if len(rec_msg['dataprocessed']) > 5 and rec_msg['dataprocessed'][5][0] == 'SRSTracker' :
                    class srtrk_trkinfo :
                        def __init__(self, count=32):
                            self._frame_count = 0
                            self._trk_info = np.zeros((count, 4), dtype=int)  ## per track,  order is count, car, ped, bicycle
                            self._total_trk = np.zeros((3), dtype=int)  ## total track order is car, ped, bicycle
                            self._range_y = 0
                            self._curr_picked = False
                        
                        def apply(self, trkinfo, range_y):
                            self._curr_picked = False
                            if trkinfo is not None :
                                for ti in range(len(trkinfo)):
                                    if trkinfo[ti] != 0 :
                                        self._trk_info[ti, 0] += 1
                                        self._trk_info[ti, int(trkinfo[ti])]  += 1
                                        self._total_trk[int(trkinfo[ti])-1] += 1
                                        self._curr_picked = True
                                        break
                                if self._curr_picked == True :
                                    self._frame_count += 1
                                    self._range_y = range_y

                        def out_str(self):
                            if self._curr_picked == True :
                                ostr = f"FN:{self._frame_count} = "
                                if True :
                                    ostr += f"({self._total_trk[0]}/{self._frame_count} {self._total_trk[1]}/{self._frame_count} {self._total_trk[2]}/{self._frame_count}), {self._range_y:.2f}"
                                else :
                                    for ti in range(self._trk_info.shape[0]):
                                        ostr += f"{ti}({self._trk_info[ti,1]} {self._trk_info[ti,2]} {self._trk_info[ti,3]})/{self._trk_info[ti,0]} "
                            else :
                                ostr = ""
                            return ostr

                    if self._trk_info_summary is None :
                        self._trk_info_summary = srtrk_trkinfo(len(rec_msg['dataprocessed'][5][3]))
                        #os.remove(f'{self._io_name}-srtracker_trkinfo_summary.txt')
                    if cur_tgt_data is not None :
                        self._trk_info_summary.apply(rec_msg['dataprocessed'][5][3], cur_tgt_data[0,1])
                    else :
                        self._trk_info_summary.apply(None, 0)
                    outstr = self._trk_info_summary.out_str()
                    if len(outstr) > 1 :
                        with open(f'{self._io_name}-srtracker_trkinfo_summary.txt', 'a') as file:
                            file.write( outstr + "\n")
                            self.logout( outstr )
                            #print(self._trk_info_summary.out_str())

            else :
                self._targetbox_list.reset()
            #    self.logout(f"FN:{rec_msg['chunk_idx']} Target=None")
            if self._pcd_win != None and not g_player.is_playing() : self._pcd_win.invalidate_pcd(rec_msg, self._cur_rec_clips)
            self._cur_rec_msg = rec_msg
        elif rec_msg['topic'] == '/velodyne_points' :
            if self._gui_cfg['lidar_on'] == True :
                points_msg = rec_msg['graphed']
                cur_pc_data = points_msg[0]
                cur_pc_color = points_msg[1]
                cur_pc_size = self._gui_cfg['lidar_pc_size']
                self._veldyne_scatter.setData(pos=cur_pc_data, color=cur_pc_color, size=cur_pc_size)
                self._cur_lidar_pcs = [cur_pc_data, cur_pc_color, cur_pc_size]
            else :
                self._veldyne_scatter.setData(pos=self._dummy_pos)
        self.update()

    def _invalidate_stacked_graph(self, rec_msg):
        if rec_msg == None :
            QMessageBox.about(self, 'Message', 'no more data')
            return
        
        # [chunk_id, conn,topic,start,end,message]
        self._que_info.setText(f'Stacking:{g_cm._cfg.stack_size}  [{g_data_chain._call_count}:{g_data_chain._recent_process_ms}, \
{g_graph_data_updater._call_count}:{g_graph_data_updater._recent_process_ms}], \
[{g_message_que.qsize()}, {g_data_que.qsize()}, {g_graph_que.qsize()}, {g_gui_que.qsize()}]')

        if rec_msg['topic'] == '/usb_cam/image_raw/compressed' or rec_msg['topic']=='/image_raw/compressed':
            cam_msg = rec_msg['parsed']
            self._cam_info.setText(f'CAM={cam_msg[1]} {str_bag_time(cam_msg[2], cam_msg[3])}')
            self.invalidate_cam(cam_msg[4])
            if False:
                f = open(f'd:\\image_{rec_msgs[mi][0]}.jpg', 'wb')
                f.write(rec_msgs[mi][4])
                f.close()
        elif rec_msg['topic'] == '/point_cloud' :

            if len(self._stack_list) >= g_cm._cfg.stack_size :
                rem_item = self._stack_list[0]
                self._stack_list.remove(rem_item)
                del rem_item
            self._stack_list.append(rec_msg)

            self._play_info.setText(f'{rec_msg["chunk_idx"]}, {rec_msg["chunk_idx"]-self._chunk_offset}/{self._play_slider.maximum()}  {self._dur_time_str}')
            diff_time = datetime.datetime.fromtimestamp(rec_msg['time'][0]) - datetime.datetime.fromtimestamp(self._start_time[0])
            self._play_time.setText(f"sec={int(diff_time.total_seconds())}    {int(diff_time.total_seconds()/60)}:{int(diff_time.total_seconds()%60)}  ")

            #print(f'invalidate - {rec_msg["chunk_idx"]}, {rec_msg["topic"]}')
            if g_player.is_playing() :
                self._play_slider.setValue(rec_msg["chunk_idx"] - self._chunk_offset)
                #self._play_slider.setTickPosition(rec_msg["chunk_idx"])

            parsed_msg = rec_msg['parsed']
            points_msg = rec_msg['graphed']
            cur_pc_size = self._gui_cfg['pc_size']
            filtered_count = 0
            total_count = 0


            for mi in self._stack_list:
                mi_parsed_msg = mi['parsed']       ## 1:
                mi_points_msg = mi['graphed']
                mi_pc_data = mi_points_msg[0]     ## 
                mi_pc_color = mi_points_msg[1]    ## 
                filtered_count += len(mi_pc_data)
                total_count += len(mi_parsed_msg[4])

            filtered_pc_data = np.zeros((filtered_count,3))
            filtered_pc_color = np.zeros((filtered_count,4))
            offset = 0
            for mi in self._stack_list :
                mi_parsed_msg = mi['parsed']       ## 1:
                mi_points_msg = mi['graphed']
                mi_pc_data = mi_points_msg[0]     ## 
                mi_pc_color = mi_points_msg[1]    ## 
                filtered_pc_data[offset:offset+len(mi_pc_data)] = mi_points_msg[0][:]
                if self._stack_list[len(self._stack_list)-1] == mi :
                    filtered_pc_color[offset:offset+len(mi_pc_data)] = mi_points_msg[1][:]
                else :
                    filtered_pc_color[offset:offset+len(mi_pc_data),:] = self._stacked_colors
                offset += len(mi_pc_data)

            self._pc_scatter.setData(pos=filtered_pc_data, color=filtered_pc_color, size=cur_pc_size) #, size=2 
            ## PC=time filtered_pc_count/all_pc_count time
            ##self._pc_info.setText(f'PC={parsed_msg[1]} {len(cur_pc_data)}/{len(parsed_msg[4])} {str_bag_time(parsed_msg[2], parsed_msg[3])}')
            self._pc_info.setText(f'PC={parsed_msg[1]} {filtered_count}/{total_count} {str_bag_time(parsed_msg[2], parsed_msg[3])}')


            if False:
                f = open(f'd:\\pc_{rec_msg[0]}.bin', 'wb')
                f.write(rec_msg[4])
                f.close()
            if len(points_msg) > 4 :
                cur_tgt_data = points_msg[2]
                cur_tgt_color = points_msg[3]
                cur_tgt_label = points_msg[4]
                cur_tgt_size = (1,1,1)
                if cur_tgt_data is not None :
                    #self._target_scatter.setData(pos=cur_tgt_data, color=cur_tgt_color, size=cur_tgt_size) #, size=2 
                    self._targetbox_list.set_data(pos=cur_tgt_data, color=cur_tgt_color, size=cur_tgt_size, label=cur_tgt_label)
                    if len(rec_msg['dataprocessed']) > 5 and rec_msg['dataprocessed'][5][0] == 'SRSTracker' :
                        # '', numTrk, trk_info, Trk_Type_New, trkid_fin
                        ##str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
                        msg = f"CI:{rec_msg['chunk_idx']} Target=num:{rec_msg['dataprocessed'][5][1]}" \
                                    f" trkinfo:{np.array2string(rec_msg['dataprocessed'][5][2], separator=',', max_line_width=400)}\n" \
                                    f" trktype:{np.array2string(rec_msg['dataprocessed'][5][3], separator=',', max_line_width=400)} trkid:{rec_msg['dataprocessed'][5][4]}"
                        self.logout(msg)
                    else :
                        self.logout(f"CI:{rec_msg['chunk_idx']} Target=num:{len(cur_tgt_data)}")
                else :
                    #self._target_scatter.setData(pos=self._dummy_pos)
                    self._targetbox_list.reset()
                    self.logout(f"CI:{rec_msg['chunk_idx']} Target=num:0")
                if len(rec_msg['dataprocessed']) > 5 and rec_msg['dataprocessed'][5][0] == 'SRSTracker' :
                    class srtrk_trkinfo :
                        def __init__(self, count=32):
                            self._frame_count = 0
                            self._trk_info = np.zeros((count, 4), dtype=int)  ## per track,  order is count, car, ped, bicycle
                            self._total_trk = np.zeros((3), dtype=int)  ## total track order is car, ped, bicycle
                            self._range_y = 0
                            self._curr_picked = False
                        
                        def apply(self, trkinfo, range_y):
                            self._curr_picked = False
                            if trkinfo is not None :
                                for ti in range(len(trkinfo)):
                                    if trkinfo[ti] != 0 :
                                        self._trk_info[ti, 0] += 1
                                        self._trk_info[ti, int(trkinfo[ti])]  += 1
                                        self._total_trk[int(trkinfo[ti])-1] += 1
                                        self._curr_picked = True
                                        break
                                if self._curr_picked == True :
                                    self._frame_count += 1
                                    self._range_y = range_y

                        def out_str(self):
                            if self._curr_picked == True :
                                ostr = f"FN:{self._frame_count} = "
                                if True :
                                    ostr += f"({self._total_trk[0]}/{self._frame_count} {self._total_trk[1]}/{self._frame_count} {self._total_trk[2]}/{self._frame_count}), {self._range_y:.2f}"
                                else :
                                    for ti in range(self._trk_info.shape[0]):
                                        ostr += f"{ti}({self._trk_info[ti,1]} {self._trk_info[ti,2]} {self._trk_info[ti,3]})/{self._trk_info[ti,0]} "
                            else :
                                ostr = ""
                            return ostr

                    if self._trk_info_summary is None :
                        self._trk_info_summary = srtrk_trkinfo(len(rec_msg['dataprocessed'][5][3]))
                        #os.remove(f'{self._io_name}-srtracker_trkinfo_summary.txt')
                    if cur_tgt_data is not None :
                        self._trk_info_summary.apply(rec_msg['dataprocessed'][5][3], cur_tgt_data[0,1])
                    else :
                        self._trk_info_summary.apply(None, 0)
                    outstr = self._trk_info_summary.out_str()
                    if len(outstr) > 1 :
                        with open(f'{self._io_name}-srtracker_trkinfo_summary.txt', 'a') as file:
                            file.write( outstr + "\n")
                            self.logout( outstr )
                            #print(self._trk_info_summary.out_str())

            else :
                self._targetbox_list.reset()
            #    self.logout(f"FN:{rec_msg['chunk_idx']} Target=None")
            if self._pcd_win != None and not g_player.is_playing() : self._pcd_win.invalidate_pcd(self._stack_list, self._cur_rec_clips)
            self._cur_rec_msg = rec_msg
        elif rec_msg['topic'] == '/velodyne_points' :
            if self._gui_cfg['lidar_on'] == True :
                points_msg = rec_msg['graphed']
                cur_pc_data = points_msg[0]
                cur_pc_color = points_msg[1]
                cur_pc_size = self._gui_cfg['lidar_pc_size']
                self._veldyne_scatter.setData(pos=cur_pc_data, color=cur_pc_color, size=cur_pc_size)
                self._cur_lidar_pcs = [cur_pc_data, cur_pc_color, cur_pc_size]
            else :
                self._veldyne_scatter.setData(pos=self._dummy_pos)
        self.update()


    #@pyqtSlot(object)
    def send_radar_command(self, cmds):
        g_player.send_cs_command(cmds)
        ##pass
        #if self._srtdev != None :
        #    self._srtdev.send_config(cmds)

    def update_message(self, console_msg):
        if self._console_win != None :
            self._console_win.append_console_data(console_msg)


def test():
    print('test....')

    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    size = screen.size()
    main = PCDataDialog()
    main.show()
    sys.exit(app.exec_())

# ----------------------------------
if __name__ == '__main__':

    # test case
    if False :
        test()
        sys.exit(0)

    # -- config manager
    g_cm = ConfigManager(None)
    # --- message que
    manager = mp.Manager()
    #g_console_que = mp.Queue()
    g_message_que = mp.Queue()
    g_data_que = mp.Queue()
    g_graph_que = mp.Queue()
    g_gui_que = mp.Queue()
    g_ind_que = mp.Queue()
    # --- create wokers.
    g_player = SRIOWrapper(manager, g_message_que, g_graph_que, g_ind_que)
    g_data_chain = DataChain(g_message_que, g_data_que, g_cm)
    g_graph_data_updater = GraphDataUpdater(g_data_que, g_graph_que, g_cm)
    g_gui_updater = GuiUpdater(g_graph_que, g_gui_que, g_cm._cfg.stack_size)
    #g_gui_updater._invalidate_gui.connect(ginvalidate_graph)
    g_que_cleaner = QueCleaner(g_gui_que, g_cm._cfg.stack_size)
    # --- start workers.
    g_data_chain.start()
    g_graph_data_updater.start()
    g_gui_updater.start()
    g_que_cleaner.start()
    g_player.start()

    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    size = screen.size()
    main = SRMainWindow(size=size)
    main._ind_que = g_ind_que
    #main._stack_que = g_gui_que
    def done_close_event(event):
        main.do_finalize()
    main.closeEvent = done_close_event

    main.show()

    errcode = app.exec_()
    g_player.stop()
    g_data_chain.stop()
    g_graph_data_updater.stop()
    g_gui_updater.stop()
    g_que_cleaner.stop()
    del g_cm
    sys.exit(errcode)
