import sys
import os
import numpy as np
import datetime
import time
import multiprocessing as mp
from multiprocessing import shared_memory as shm
import ctypes
import typing
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import ( QDialog, QGridLayout, QLabel, QLineEdit, QDialogButtonBox, QApplication, QSpacerItem, QSizePolicy, QComboBox, QFileDialog)
#import debugpy

###################################
### global config manager..
class CMConst:
    CLIP_INCLUDE    : typing.Final[int] = 0
    CLIP_EXCLUDE    : typing.Final[int] = 1
    PATH_SIZE       : typing.Final[int] = 256

class CMTrkType:
    NONE            : typing.Final[int] = 0
    INTERNAL        : typing.Final[int] = 1
    GTRACKER        : typing.Final[int] = 2
    SRTRACKER       : typing.Final[int] = 3
    TIGTRACKER      : typing.Final[int] = 4
    NAME            : typing.Final[list] = ["None", "Internal", "GTrack(Ili)", "SRSTracker", "GTrack(TI)-DLL"]

# # ----------------------------------
class ConfigData(ctypes.Structure):
    _fields_ = [('pc_color_zmin', ctypes.c_double), ('pc_color_zmax', ctypes.c_double)
            , ('idx_doppler', ctypes.c_int)
            , ('tracker_type', ctypes.c_int )
            , ('tracker_cfg', ctypes.c_char*CMConst.PATH_SIZE)
            , ('axis_pos', ctypes.c_float*3)
            , ('axis_angle', ctypes.c_float*3)
            , ('clips', ctypes.c_double*8)
            , ('clip_opt', ctypes.c_int8*4)
            , ('stack_size', ctypes.c_int)
            , ('color_table_size', ctypes.c_int*2)  # ndarray[?,4]
            , ('etc_size', ctypes.c_int*3)          # ndarray[?,?] for test
            #### static offset for data field
            ## i removed below..
    ]

    ### use ConfigData.from_buf( sharedmemory.buf ) 
    # def deserialize(cls, buf):
    #     inst = cls.from_buffer(buf)
    #     return inst

    def get_color_table(self, shm_buf):
        offset = ctypes.sizeof(ConfigData) + 0
        size = ctypes.sizeof(ctypes.c_double)*self.color_table_size[0]*self.color_table_size[1]
        ctbl = np.ndarray([self.color_table_size[0], self.color_table_size[1]], dtype=ctypes.c_double, buffer=shm_buf[offset:offset+size])
        return ctbl

    def get_etc(self, shm_buf):
        preoffset = ctypes.sizeof(ConfigData) + ctypes.sizeof(ctypes.c_double)*self.color_table_size[0]*self.color_table_size[1]
        offset = ctypes.sizeof(ConfigData) + preoffset
        size = ctypes.sizeof(ctypes.c_double)*self.etc_size[0]*self.etc_size[1]*self.etc_size[2]
        cetc = np.ndarray([self.etc_size[0], self.etc_size[1], self.etc_size[2]], dtype=ctypes.c_double, buffer=shm_buf[offset:offset+size])
        return cetc


#  bytearray(cfg)
# sys.getsizeof(cfg)
# ctypes.sizeof(ConfigData)
# bytes(cfg).hex()
# shmem = shm.SharedMemory(create=True, size=1024)
# shmem = shm.SharedMemory( ? )
# cfg = ConfigData.from_buffer(shmem.buf)
# cfg = ConfigData.from_buffer_copy(shmem.buf)
# ConfigData.color_table.offset
# offset = ctypes.sizeof(ConfigData) + 0
# size = ctypes.sizeof(ctypes.c_double)*cfg.color_table_size[0]*cfg.color_table_size[1]
# ctbl = np.ndarray([cfg.color_table_size[0],cfg.color_table_size[1]], dtype=ctypes.c_double, buffer=shmem.buf[offset:offset+size])


# # ----------------------------------
# class ConfigData(ctypes.Structure):
#     _fields_ = [('z_min', ctypes.c_double), ('z_max', ctypes.c_double), ('z_max', ctypes.c_double) ]

# ----------------------------------
class ConfigManager():
    _shmem = None
    _color_table = None
    _cfg = None
    _MASK_PC = 1
    _MASK_CAM = 2
    _gui_mask = (_MASK_PC|_MASK_CAM)  # 1:point_cloud, 2:camera, 3:??
    _clip_items = ( 'X-Range', 'Y-Range', 'Z-Range', 'Doppler' )
    _clip_opt_name = ( 'Include', 'Exclude' )
    _win = None
    def __init__(self, par):
        self._shmem = shm.SharedMemory(create=True, size=32768)
        print(f'**ConfigManager. shared={self._shmem.name}')
        self._cfg = ConfigData.from_buffer(self._shmem.buf)
        ## set default values.
        self._cfg.pc_color_zmin = 0
        self._cfg.pc_color_zmax = 4
        self._cfg.idx_doppler = 9 # 3:r4fxxx, 9:bagf
        self._cfg.tracker_type = CMTrkType.INTERNAL
        self._cfg.tracker_cfg = b''
        self._cfg.stack_size = 0
        self._cfg.clips[:] = [ float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan') ]
        self._cfg.clip_opt[:] = [CMConst.CLIP_INCLUDE, CMConst.CLIP_INCLUDE, CMConst.CLIP_INCLUDE, CMConst.CLIP_EXCLUDE]

    def __del__(self):
        print(f'**ConfigManager.Close. shared={self._shmem.name}')
        del self._cfg
        self._shmem.close()
        self._shmem.unlink()

    def shared_name(self):
        return self._shmem.name

    def gui_mask(self, v):
        return True if (self._gui_mask & v)!=0 else False

    def set_win(self, win):
        self._win = win

    def update_color_table(self, color_table) :
        self._cfg.color_table_size[0] = color_table.shape[0]
        self._cfg.color_table_size[1] = color_table.shape[1]
        self._color_table = self._cfg.get_color_table(self._shmem.buf)
        self._color_table[:,:] = color_table[:,:]
        ## print(f'Color Table ({self._cfg.color_table_size[0]},{self._cfg.color_table_size[1]}) {len(self._color_table)} = {self._color_table}')

    # ----------------------------------
    class ClipDialog(QDialog):
        def __init__(self, outer, par):
            super(ConfigManager.ClipDialog, self).__init__(par)
            self._outer = outer
            self.setWindowTitle('Clipping Config')
            self._vitems = []
            self._vopts = []
            glay = QGridLayout(self)
            for i in range(len(self._outer._clip_items)):
                name = self._outer._clip_items[i]
                opt_name = QLabel(name)
                opt_combo = QComboBox()
                opt_combo.addItems("Include,Exclude".split(','))
                self._vopts.append(opt_combo)
                opt_combo.setCurrentIndex(self._outer._cfg.clip_opt[i])
                opt_v_min = QLineEdit(str(self._outer._cfg.clips[i*2+0]) if not np.isnan(self._outer._cfg.clips[i*2+0])  else '' )
                opt_v_max = QLineEdit(str(self._outer._cfg.clips[i*2+1]) if not np.isnan(self._outer._cfg.clips[i*2+1])  else '' )
                if name.find('Doppler') == -1 :
                    opt_v_min.setPlaceholderText('meter');  opt_v_max.setPlaceholderText('meter')
                self._vitems.append(opt_v_min);     self._vitems.append(opt_v_max)
                glay.addWidget(opt_name,   i, 0, 1, 1)
                glay.addWidget(opt_combo,  i, 1, 1, 1)
                glay.addWidget(opt_v_min,  i, 2, 1, 1)
                glay.addWidget(opt_v_max,  i, 3, 1, 1)
            
            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
            row_offset = len(self._outer._clip_items) + 1
            opt_name = QLabel('Point Color Range(Z)')
            self._pc_color_zmin = QLineEdit(str(self._outer._cfg.pc_color_zmin))
            self._pc_color_zmax = QLineEdit(str(self._outer._cfg.pc_color_zmax))
            glay.addWidget(opt_name,   row_offset, 0, 1, 2)
            glay.addWidget(self._pc_color_zmin,  row_offset, 2, 1, 1)
            glay.addWidget(self._pc_color_zmax,  row_offset, 3, 1, 1)

            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
            self.btnbox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
            glay.addWidget(self.btnbox, row_offset+3, 0, 1, 2)
            self.setLayout(glay)
            self.btnbox.accepted.connect(self.click_ok)
            self.btnbox.rejected.connect(self.click_cancel)

        def click_ok(self):
            for i in range(len(self._vitems)):
                vstr = self._vitems[i].text()
                #if  vstr != None and len(vstr) > 0 : self._outer._cfg.clips[i] = float(vstr)
                self._outer._cfg.clips[i] = float(vstr) if  vstr != None and len(vstr) > 0 else np.nan
                if i%2 == 0:
                    self._outer._cfg.clip_opt[i//2] = self._vopts[i//2].currentIndex()
            vstr = self._pc_color_zmin.text()
            if  vstr != None and len(vstr) > 0 : self._outer._cfg.pc_color_zmin = float(vstr)
            vstr = self._pc_color_zmax.text()
            if  vstr != None and len(vstr) > 0 : self._outer._cfg.pc_color_zmax = float(vstr)
            self.accept()

        def click_cancel(self):
            self.reject()

    # ----------------------------------
    def do_area_clip(self):
        clipdlg = self.ClipDialog(self, self._win)
        if clipdlg.exec_() == QDialog.Accepted :
            print(f'dialog accepted, {id(clipdlg)}')
            print(f'       {self._cfg.clips[:]}')
            return True
        else :
            print(f'dialog rejected, {id(clipdlg)}')
        return False

    # ----------------------------------
    class SetConfigDialog(QDialog):
        def __init__(self, outer, par):
            super(ConfigManager.SetConfigDialog, self).__init__(par)
            self._outer = outer
            self.setWindowTitle('Set Config')
            self._vitems = []
            self._vopts = []
            glay = QGridLayout(self)

            row_offset = 0

            ## target config - tracker_type, config
            tgt_type_label = QLabel('Target')
            self.tgt_type_combo = QComboBox()
            self.tgt_type_combo.addItems(CMTrkType.NAME)
            self.tgt_type_combo.setCurrentIndex(self._outer._cfg.tracker_type)
            self.tgt_type_combo.currentIndexChanged.connect(self.tgt_type_combo_changed)
            self.tgt_type_config_line = QLineEdit()
            self.tgt_type_config_line.setPlaceholderText('Json Tracker Config File')
            self.tgt_type_combo_changed(self._outer._cfg.tracker_type)
            #self.tgt_type_config_line.focusInEvent.connect(self.tgt_type_config_focused)
            #self.tgt_type_combo.
            glay.addWidget(tgt_type_label,   row_offset, 0, 1, 1)
            glay.addWidget(self.tgt_type_combo,  row_offset, 1, 1, 1)
            glay.addWidget(self.tgt_type_config_line, row_offset, 2, 1, 2)
            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
            row_offset += 1

            ## axis config - position, angle            
            self.axis_pos_label = QLabel("Axis_Pos")
            self.axis_angle_label = QLabel("Eular_angle")
            self.axis_pos_line = [ QLineEdit(str(self._outer._cfg.axis_pos[0]) if not np.isnan(self._outer._cfg.axis_pos[0])  else '' )
                    , QLineEdit(str(self._outer._cfg.axis_pos[1]) if not np.isnan(self._outer._cfg.axis_pos[1])  else '' )
                    , QLineEdit(str(self._outer._cfg.axis_pos[2]) if not np.isnan(self._outer._cfg.axis_pos[2])  else '' ) ]
            #map( lambda x: x.setPlaceholderText('meter'), self.axis_pos_line)
            self.axis_pos_line[0].setPlaceholderText('x.meter');self.axis_pos_line[1].setPlaceholderText('y.meter');self.axis_pos_line[2].setPlaceholderText('z.meter')
            self.axis_angle_line = [ QLineEdit(str(self._outer._cfg.axis_angle[0]) if not np.isnan(self._outer._cfg.axis_angle[0])  else '' )
                    , QLineEdit(str(self._outer._cfg.axis_angle[1]) if not np.isnan(self._outer._cfg.axis_angle[1])  else '' )
                    , QLineEdit(str(self._outer._cfg.axis_angle[2]) if not np.isnan(self._outer._cfg.axis_angle[2])  else '' ) ]
            #map( lambda x: x.setPlaceholderText('degree'), self.axis_angle_line)
            self.axis_angle_line[0].setPlaceholderText('Yaw.degree');self.axis_angle_line[1].setPlaceholderText('Roll.degree');self.axis_angle_line[2].setPlaceholderText('Pitch.degree')
            glay.addWidget(self.axis_pos_label,  row_offset, 0)
            glay.addWidget(self.axis_pos_line[0],  row_offset, 1)
            glay.addWidget(self.axis_pos_line[1],  row_offset, 2)
            glay.addWidget(self.axis_pos_line[2],  row_offset, 3)
            row_offset += 1
            glay.addWidget(self.axis_angle_label,  row_offset, 0)
            glay.addWidget(self.axis_angle_line[0],  row_offset, 1)
            glay.addWidget(self.axis_angle_line[1],  row_offset, 2)
            glay.addWidget(self.axis_angle_line[2],  row_offset, 3)
            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))

            row_offset += 1
            for i in range(len(self._outer._clip_items)):
                name = self._outer._clip_items[i]
                opt_name = QLabel(name)
                opt_combo = QComboBox()
                opt_combo.addItems("Include,Exclude".split(','))
                self._vopts.append(opt_combo)
                opt_combo.setCurrentIndex(self._outer._cfg.clip_opt[i])
                opt_v_min = QLineEdit(str(self._outer._cfg.clips[i*2+0]) if not np.isnan(self._outer._cfg.clips[i*2+0])  else '' )
                opt_v_max = QLineEdit(str(self._outer._cfg.clips[i*2+1]) if not np.isnan(self._outer._cfg.clips[i*2+1])  else '' )
                if name.find('Doppler') == -1 :
                    opt_v_min.setPlaceholderText('meter');  opt_v_max.setPlaceholderText('meter')
                self._vitems.append(opt_v_min);     self._vitems.append(opt_v_max)
                glay.addWidget(opt_name,   row_offset, 0, 1, 1)
                glay.addWidget(opt_combo,  row_offset, 1, 1, 1)
                glay.addWidget(opt_v_min,  row_offset, 2, 1, 1)
                glay.addWidget(opt_v_max,  row_offset, 3, 1, 1)
                row_offset += 1
            
            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
            #row_offset = len(self._outer._clip_items) + 1
            opt_name = QLabel('Point Color Range(Z)')
            self._pc_color_zmin = QLineEdit(str(self._outer._cfg.pc_color_zmin))
            self._pc_color_zmax = QLineEdit(str(self._outer._cfg.pc_color_zmax))
            glay.addWidget(opt_name,   row_offset, 0, 1, 2)
            glay.addWidget(self._pc_color_zmin,  row_offset, 2, 1, 1)
            glay.addWidget(self._pc_color_zmax,  row_offset, 3, 1, 1)
            row_offset += 1

            #self._cfg.stack_size
            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
            #row_offset = len(self._outer._clip_items) + 1
            opt_name = QLabel('Point Stack Size')
            self._pc_stack_size = QLineEdit(str(self._outer._cfg.stack_size))
            glay.addWidget(opt_name,   row_offset, 0, 1, 2)
            glay.addWidget(self._pc_stack_size,  row_offset, 2, 1, 1)
            row_offset += 1

            glay.addItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding))
            self.btnbox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
            glay.addWidget(self.btnbox, row_offset, 0, 1, 2)
            #row_offset += 1
            self.setLayout(glay)
            self.btnbox.accepted.connect(self.click_ok)
            self.btnbox.rejected.connect(self.click_cancel)

        def tgt_type_combo_changed(self, index):
            #print(f"Param:{index}, {self.tgt_type_combo.currentIndex()}")
            if index == CMTrkType.TIGTRACKER :
                self.tgt_type_config_line.setEnabled(True)
                fd = QFileDialog()
                filt = "json(*.json)"
                filename = fd.getOpenFileName(filter=filt)
                if filename == None or os.path.exists(filename[0]) != True :
                    return
                self.tgt_type_config_line.setText(filename[0])
            else :
                if self.tgt_type_config_line.isEnabled() :
                    self.tgt_type_config_line.setText("")
                    self.tgt_type_config_line.setEnabled(False)

        #def tgt_type_config_focused(self, event):
        #    print(f"tgt config, {self.tgt_type_combo.currentIndex()}")
            

        def click_ok(self):
            self._outer._cfg.tracker_type = self.tgt_type_combo.currentIndex()
            if self._outer._cfg.tracker_type == CMTrkType.TIGTRACKER :
                self._outer._cfg.tracker_cfg = self.tgt_type_config_line.text().strip().encode()
            else :
                self._outer._cfg.tracker_cfg = b''

            vstr = self.axis_pos_line[0].text(); self._outer._cfg.axis_pos[0] = float(vstr) if  vstr != None and len(vstr) > 0 else 0
            vstr = self.axis_pos_line[1].text(); self._outer._cfg.axis_pos[1] = float(vstr) if  vstr != None and len(vstr) > 0 else 0
            vstr = self.axis_pos_line[2].text(); self._outer._cfg.axis_pos[2] = float(vstr) if  vstr != None and len(vstr) > 0 else 0
            vstr = self.axis_angle_line[0].text(); self._outer._cfg.axis_angle[0] = float(vstr) if  vstr != None and len(vstr) > 0 else 0
            vstr = self.axis_angle_line[1].text(); self._outer._cfg.axis_angle[1] = float(vstr) if  vstr != None and len(vstr) > 0 else 0
            vstr = self.axis_angle_line[2].text(); self._outer._cfg.axis_angle[2] = float(vstr) if  vstr != None and len(vstr) > 0 else 0
            
            for i in range(len(self._vitems)):
                vstr = self._vitems[i].text()
                #if  vstr != None and len(vstr) > 0 : self._outer._cfg.clips[i] = float(vstr)
                self._outer._cfg.clips[i] = float(vstr) if  vstr != None and len(vstr) > 0 else np.nan
                if i%2 == 0:
                    self._outer._cfg.clip_opt[i//2] = self._vopts[i//2].currentIndex()
            vstr = self._pc_color_zmin.text()
            if  vstr != None and len(vstr) > 0 : self._outer._cfg.pc_color_zmin = float(vstr)
            vstr = self._pc_color_zmax.text()
            if  vstr != None and len(vstr) > 0 : self._outer._cfg.pc_color_zmax = float(vstr)
            vstr = self._pc_stack_size.text()
            if  vstr != None and len(vstr) > 0 : self._outer._cfg.stack_size = int(vstr)

            self.accept()

        def click_cancel(self):
            self.reject()

    # ----------------------------------
    def do_configure_set(self):
        clipdlg = self.SetConfigDialog(self, self._win)
        if clipdlg.exec_() == QDialog.Accepted :
            print(f'dialog accepted, {id(clipdlg)}')
            print(f'       {self._cfg.clips[:]}')
            return True
        else :
            print(f'dialog rejected, {id(clipdlg)}')
        return False



#####
# ----------------------------------
if __name__ == '__main__':
    gcm = ConfigManager(None)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    size = screen.size()
    #main = ConfigManager.ClipDialog(gcm, None)
    main = ConfigManager.SetConfigDialog(gcm, None)
    main.show()
    errcode = app.exec_()

    points = np.zeros((2,3))
    new_points = np.zeros((2,3))
    points[0,0] = 0
    points[0,1] = 0
    points[0,2] = 0
    points[1,0] = 1
    points[1,1] = 1
    points[1,2] = 1

    pos = [ gcm._cfg.axis_pos[0], gcm._cfg.axis_pos[1], gcm._cfg.axis_pos[2]]
    angle = [ gcm._cfg.axis_angle[0], gcm._cfg.axis_angle[1], gcm._cfg.axis_angle[2] ]

    ## real code..
    points[:,0] += pos[0]
    points[:,1] += pos[1]
    points[:,2] += pos[2]

    tr = generate_eular_matrix(angle)
    npoints = process_axis_translate(points, pos, tr)

    sys.exit(errcode)
