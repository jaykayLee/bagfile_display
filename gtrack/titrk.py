import sys
import datetime
import time
import struct
import os
import string
import typing
import ctypes
import numpy as np
import json

GTRACK2D : typing.Final[bool] = True
libtitrk : ctypes.CDLL = None

# ----------------------------------
class GTrkConst() :
    MAX_BOUNDARY_BOXES      : typing.Final[int] = 2
    MAX_STATIC_BOXES        : typing.Final[int] = 2
    MAX_OCCUPANCY_BOXES     : typing.Final[int] = 2
    MEASUREMENT_VEC_2D_SIZE : typing.Final[int] = 3
    MEASUREMENT_VEC_3D_SIZE : typing.Final[int] = 4
    MEASUREMENT_VEC_SIZE    : int               = MEASUREMENT_VEC_2D_SIZE if GTRACK2D else MEASUREMENT_VEC_3D_SIZE
    STATE_VEC_2D_SIZE       : typing.Final[int] = 6
    STATE_VEC_3D_SIZE       : typing.Final[int] = 9
    STATE_VEC_SIZE          : int               = STATE_VEC_2D_SIZE if GTRACK2D else STATE_VEC_3D_SIZE
    BENCHMARK_SIZE          : int               = 10
    
# ----------------------------------
class GtrkSensorPosition(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float),
                ("z", ctypes.c_float)
                ]

# ----------------------------------
class GtrkSensorOrientation(ctypes.Structure):
    _fields_ = [("azim_tilt", ctypes.c_float),
                ("elev_tilt", ctypes.c_float)
                ]

# ----------------------------------
class GtrkBoundaryBox(ctypes.Structure):
    _fields_ = [("x1", ctypes.c_float),
                ("x2", ctypes.c_float),
                ("y1", ctypes.c_float),
                ("y2", ctypes.c_float),
                ("z1", ctypes.c_float),
                ("z2", ctypes.c_float)
                ]

# ----------------------------------
class GtrkGatingLimits(ctypes.Structure):
    _fields_ = [("depth", ctypes.c_float),
                ("width", ctypes.c_float),
                ("height", ctypes.c_float),
                ("vel", ctypes.c_float)
                ]

# ----------------------------------
class GtrkGatingParams(ctypes.Structure):
    _fields_ = [("gain", ctypes.c_float),
                ("depth", ctypes.c_float),
                ("width", ctypes.c_float),
                ("height", ctypes.c_float),
                ("vel", ctypes.c_float)
                ]

# ----------------------------------
class GtrkAllocationParams(ctypes.Structure):
    _fields_ = [("snr_thre", ctypes.c_float),
                ("snr_thre_obscured", ctypes.c_float),
                ("velocity_thre", ctypes.c_float),
                ("points_thre", ctypes.c_ushort),
                ("maxdistance_thre", ctypes.c_float),
                ("maxvel_thre", ctypes.c_float)
                ]

# ----------------------------------
class GtrkStateParams(ctypes.Structure):
    _fields_ = [("det2act_thre", ctypes.c_ushort),
                ("det2free_thre", ctypes.c_ushort),
                ("active2free_thre", ctypes.c_ushort),
                ("static2free_thre", ctypes.c_ushort),
                ("exit2free_thre", ctypes.c_ushort),
                ("sleep2free_thre", ctypes.c_ushort)
                ]

# ----------------------------------
class GtrkScenaryParams(ctypes.Structure):
    _fields_ = [("sensor_position", GtrkSensorPosition),
                ("sensor_orientation", GtrkSensorOrientation),
                ("num_boundary_boxes", ctypes.c_int8),
                ("boundary_box", GtrkBoundaryBox*GTrkConst.MAX_BOUNDARY_BOXES),
                ("num_static_boxes", ctypes.c_int8),
                ("static_box", GtrkBoundaryBox*GTrkConst.MAX_STATIC_BOXES)
                ]

# ----------------------------------
class GtrkPresenceParams(ctypes.Structure):
    _fields_ = [("points_thre", ctypes.c_ushort),
                ("velocity_thre", ctypes.c_float),
                ("on2off_thre", ctypes.c_ushort),
                ("num_occupancy_boxes", ctypes.c_uint8),
                ("occupancy_box", GtrkBoundaryBox*GTrkConst.MAX_OCCUPANCY_BOXES)
                ]

# ----------------------------------
class GtrkAdvancedParams(ctypes.Structure):
    _fields_ = [("gating_params", ctypes.POINTER(GtrkGatingParams)),
                ("allocation_params", ctypes.POINTER(GtrkAllocationParams)),
                ("state_params", ctypes.POINTER(GtrkStateParams)),
                ("scenary_params", ctypes.POINTER(GtrkScenaryParams)),
                ("presence_params", ctypes.POINTER(GtrkPresenceParams)),
                ]

# ----------------------------------
class GtrkModuleConfig(ctypes.Structure):
    _fields_ = [("state_vector_type", ctypes.c_int),
                ("verbose", ctypes.c_int),
                ("max_num_point", ctypes.c_ushort),
                ("max_num_track", ctypes.c_ushort),
                ("initial_radial_velocity", ctypes.c_float),
                ("max_radial_velocity", ctypes.c_float),
                ("radial_velocity_resolution", ctypes.c_float),
                ("max_accellation", ctypes.c_float*3),
                ("delta_t", ctypes.c_float),
                ("adv_params", ctypes.POINTER(GtrkAdvancedParams))
                ]


# ----------------------------------
class GtrkMeasurementPoint2D(ctypes.Structure):
    _fields_ = [("range", ctypes.c_float),
                ("angle", ctypes.c_float),
                ("doppler", ctypes.c_float),
                ("snr", ctypes.c_float)
                ]
    ## 
# ----------------------------------
class GtrkMeasurementPoint3D(ctypes.Structure):
    _fields_ = [("range", ctypes.c_float),
                ("angle", ctypes.c_float),
                ("elev", ctypes.c_float),
                ("doppler", ctypes.c_float),
                ("snr", ctypes.c_float)
                ]

# ----------------------------------
class GtrkMeasurementVector2D(ctypes.Structure):
    _fields_ = [("range", ctypes.c_float),
                ("angle", ctypes.c_float),
                ("doppler", ctypes.c_float),
                ]

# ----------------------------------
class GtrkMeasurementVector3D(ctypes.Structure):
    _fields_ = [("range", ctypes.c_float),
                ("angle", ctypes.c_float),
                ("elev", ctypes.c_float),
                ("doppler", ctypes.c_float),
                ]

# ----------------------------------
class GtrkStateVectorPosVelAcc2D(ctypes.Structure):
    _fields_ = [("pos_x", ctypes.c_float),
                ("pos_y", ctypes.c_float),
                ("vel_x", ctypes.c_float),
                ("vel_y", ctypes.c_float),
                ("acc_x", ctypes.c_float),
                ("acc_y", ctypes.c_float),
                ]

# ----------------------------------
class GtrkStateVectorPosVelAcc3D(ctypes.Structure):
    _fields_ = [("pos_x", ctypes.c_float),
                ("pos_y", ctypes.c_float),
                ("pos_z", ctypes.c_float),
                ("vel_x", ctypes.c_float),
                ("vel_y", ctypes.c_float),
                ("vel_z", ctypes.c_float),
                ("acc_x", ctypes.c_float),
                ("acc_y", ctypes.c_float),
                ("acc_z", ctypes.c_float),
                ]

# ----------------------------------
class GtrkTargetDesc(ctypes.Structure):
    _fields_ = [("uid", ctypes.c_uint8),
                ("tid", ctypes.c_uint32),
                ("s", ctypes.c_float*GTrkConst.STATE_VEC_SIZE),
                ("ec", ctypes.c_float*GTrkConst.MEASUREMENT_VEC_SIZE*GTrkConst.MEASUREMENT_VEC_SIZE),
                ("g", ctypes.c_float),
                ("dim", ctypes.c_float*GTrkConst.MEASUREMENT_VEC_SIZE),
                ("u_center", ctypes.c_float*GTrkConst.MEASUREMENT_VEC_SIZE),
                ("confidence_level", ctypes.c_float),
                ]

# ----------------------------------
# ----------------------------------
# ----------------------------------
def LoadTiGTracker():
    global libtitrk
    if libtitrk is not None :
        return libtitrk
    if sys.platform == 'win32' :
        #libtitrk = ctypes.cdll.LoadLibrary("./gtrack/gtrack_c/bld_win/Debug/titrk.dll")
        ## for vs debugger,  below path is visual c++ debug output path,
        #libtitrk = ctypes.cdll.LoadLibrary("./gtrack/gtrack_c/out/build/x64-Debug/titrk.dll")
        libtitrk = ctypes.cdll.LoadLibrary("./gtrack/gtrack_c/lib/titrk.dll")
    else :
        #libtitrk = ctypes.cdll.LoadLibrary("./gtrack/gtrack_c/bld_linux/libtitrk.so")
        libtitrk = ctypes.cdll.LoadLibrary("./gtrack/gtrack_c/lib/libtitrk.so")

    #libtitrk.gtrack_default_module_config.restype = ctypes.POINTER(GtrkModuleConfig)
    libtitrk.gtrack_default_module_config.restype = ctypes.c_void_p
    libtitrk.gtrack_create.argtypes = [ctypes.POINTER(GtrkModuleConfig), ctypes.POINTER(ctypes.c_int32)]
    libtitrk.gtrack_create.restype = ctypes.c_void_p
    # FWIN_EXPORT void gtrack_step(void *handle, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t mNum
    #               , GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint8_t *uIndex, uint8_t *presence, uint32_t *bench);
    # libtitrk.gtrack_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(GtrkMeasurementPoint2D*1000), ctypes.POINTER(GtrkMeasurementVector2D*1000), ctypes.c_ushort,
    #                             ctypes.POINTER(GtrkTargetDesc*100), ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
    #                             ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint32) ]
    # libtitrk.gtrack_array_test.argtypes = [ctypes.c_void_p, ctypes.POINTER(GtrkMeasurementPoint2D*1000), ctypes.POINTER(GtrkMeasurementVector2D*1000), ctypes.c_ushort,
    #                             ctypes.POINTER(GtrkTargetDesc*100), ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
    #                             ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint32) ]
    libtitrk.gtrack_delete.argtypes = [ctypes.c_void_p]
    libtitrk.gtrack_alloc.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
    libtitrk.gtrack_alloc.restype = ctypes.c_void_p
    libtitrk.gtrack_free.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    return libtitrk



# ----------------------------------
## not called, why??
class CfgEncoder(json.JSONEncoder):
    # def default(self, obj):
    #     print("called cfgencoder.default")
    #     if isinstance(obj, float):
    #         return f"{obj:.2f}"
    #     return json.JSONEncoder.default(self, obj)
    def iterencode(self, o, _one_shot=False):
        INFINITY = float('inf')
        if self.check_circular:
            markers = {}
        else:
            markers = None
        if self.ensure_ascii:
            _encoder = json.encoder.encode_basestring_ascii
        else:
            _encoder = json.encoder.encode_basestring

        def floatstr(o, allow_nan=self.allow_nan,
                _repr=float.__repr__, _inf=INFINITY, _neginf=-INFINITY):
            # Check for specials.  Note that this type of test is processor
            # and/or platform-specific, so do tests which don't depend on the
            # internals.

            if o != o:
                text = 'NaN'
            elif o == _inf:
                text = 'Infinity'
            elif o == _neginf:
                text = '-Infinity'
            else:
                return f"{o:.2f}"
                # return _repr(o)

            if not allow_nan:
                raise ValueError(
                    "Out of range float values are not JSON compliant: " +
                    repr(o))

            return text


        if (_one_shot and json.encoder.c_make_encoder is not None
                and self.indent is None):
            _iterencode = json.encoder.c_make_encoder(
                markers, self.default, _encoder, self.indent,
                self.key_separator, self.item_separator, self.sort_keys,
                self.skipkeys, self.allow_nan)
        else:
            _iterencode = json.encoder._make_iterencode(
                markers, self.default, _encoder, self.indent, floatstr,
                self.key_separator, self.item_separator, self.sort_keys,
                self.skipkeys, _one_shot)
        return _iterencode(o, 0)

# ----------------------------------
class TiGTrkJsonConfig():
    def __init__(self):
        global libtitrk
        if libtitrk is None :
            LoadTiGTracker()
        self._defconfig_ptr : ctypes.c_void_p = None
        self._defconfig : ctypes.POINTER(GtrkModuleConfig) = None
        self._cfg_strt : GtrkModuleConfig = None
        self._cfg_dict : dict = None
        self._cfg_json : str = None
        ## must load default.. it's local( python App and DLL) memory scope.
        self.load_default()

    def load_default(self) -> dict:
        self._defconfig_ptr = libtitrk.gtrack_default_module_config()
        self._defconfig = ctypes.cast(self._defconfig_ptr, ctypes.POINTER(GtrkModuleConfig))
        self._cfg_strt = self._defconfig.contents
        self._cfg_dict = convert_structure_to_dict(self._cfg_strt)
        self._cfg_json = json.dumps(self._cfg_dict, indent=2, cls=CfgEncoder)
        return self._cfg_dict

    def save(self, filepath):
        if self._cfg_strt is None :
            return False
        self._cfg_dict = convert_structure_to_dict(self._cfg_strt)
        self._cfg_json = json.dumps(self._cfg_dict, indent=2, cls=CfgEncoder)
        with open(filepath, 'w') as wf :
            wf.write(self._cfg_json)
            
    def load_from_file(self, filepath):
        parsed_obj = None
        with open(filepath, 'r') as rf :
            parsed_obj = json.load(rf)
            # if isinstance(parsed_obj, dict) :
            #     self.apply_to(parsed_obj)
            #     return True
            # else :
            #     return False
        return parsed_obj

    def apply_to(self, pdict : dict):
        self.apply_dict_to_strt(pdict, self._cfg_strt)

    def apply_dict_to_strt(self, _dict, _strt):
        for ik, iv in _dict :
            pass
        pass


# ----------------------------------
class TiGTracker():
# GTRACK_measurementPoint pointCloud[GTRACK_NUM_POINTS_MAX];
# GTRACK_targetDesc targetDescr[GTRACK_NUM_TRACKS_MAX];
    
    def __init__(self):
        global libtitrk
        if libtitrk is None :
            LoadTiGTracker()
        self._handle : ctypes.c_void_p = None
        self._tgt_ptr  : ctypes.c_void_p = None #: ctypes.POINTER(GtrkTargetDesc) = None
        self._tgt_arr : ctypes.POINTER(GtrkTargetDesc) = None #: ctypes.POINTER(GtrkTargetDesc) = None
        self._pc_ptr  : ctypes.c_void_p = None # : ctypes.POINTER(GtrkMeasurementPoint2D) = None
        self._pc_arr : ctypes.POINTER(GtrkMeasurementPoint2D) = None # : ctypes.POINTER(GtrkMeasurementPoint2D) = None
        self._pc_nparr = None
        self._pc_index_ptr  : ctypes.c_void_p = None # : ctypes.POINTER(ctypes.c_uint8) = None
        self._pc_index_arr : ctypes.POINTER(ctypes.c_uint8) = None # : ctypes.POINTER(ctypes.c_uint8) = None
        self._pc_uindex_ptr : ctypes.c_void_p = None 
        self._pc_uindex_arr  : ctypes.POINTER(ctypes.c_uint8) = None 
        self._bench_ptr : ctypes.c_void_p = None 
        self._bench_arr  : ctypes.POINTER(ctypes.c_uint32) = None 
        self._bench_max_ns = 0

        self._tgt_num : ctypes.c_uint16 = ctypes.c_ushort(0)
        self._pc_num : ctypes.c_uint16 = ctypes.c_ushort(0)
        self._max_tgt_num : int = 0
        self._max_pc_num : int = 0

        ## for debug
        self._defconfig_ptr : ctypes.c_void_p = None
        self._defconfig : ctypes.POINTER(GtrkModuleConfig) = None

    def get_default_config(self) -> ctypes.POINTER(GtrkModuleConfig) :
        self._defconfig_ptr = libtitrk.gtrack_default_module_config()
        self._defconfig = ctypes.cast(self._defconfig_ptr, ctypes.POINTER(GtrkModuleConfig))
        return self._defconfig

    def load_config_file(self, file_path):
        tjconfig = TiGTrkJsonConfig()
        pdict = tjconfig.load_from_file(file_path)
        apply_dict_to_strt( pdict, tjconfig._cfg_strt )
        return tjconfig._defconfig

    def create(self, mod_config:ctypes.POINTER(GtrkModuleConfig)) :
        err_flag : ctypes.c_int32 = ctypes.c_int32(0)

        self._max_tgt_num = mod_config.contents.max_num_track
        self._max_pc_num = mod_config.contents.max_num_point
        #mod_config.contents.verbose = 5
        #mod_config.contents.verbose = 5
        self._handle = libtitrk.gtrack_create(mod_config, ctypes.byref(err_flag))
        if err_flag.value != 0 :
            print(f">>ERR:gtrack_create failed. err:{err_flag.value}")
            return False
        self._tgt_ptr = libtitrk.gtrack_alloc(ctypes.sizeof(GtrkTargetDesc), self._max_tgt_num)
        self._tgt_arr = ctypes.cast(self._tgt_ptr, ctypes.POINTER(GtrkTargetDesc*self._max_tgt_num))[0]
        self._pc_ptr = libtitrk.gtrack_alloc(ctypes.sizeof(GtrkMeasurementPoint2D), self._max_pc_num)
        self._pc_arr = ctypes.cast(self._pc_ptr, ctypes.POINTER(GtrkMeasurementPoint2D*self._max_pc_num))[0]
        pcarr_pntr = ctypes.cast(self._pc_ptr, ctypes.POINTER(ctypes.c_float))
        self._pc_nparr = np.ctypeslib.as_array(pcarr_pntr, shape=(self._max_pc_num, len(GtrkMeasurementPoint2D._fields_)))
        self._pc_index_ptr = libtitrk.gtrack_alloc(ctypes.sizeof(ctypes.c_uint8), self._max_pc_num)
        self._pc_index_arr = ctypes.cast(self._pc_index_ptr, ctypes.POINTER(ctypes.c_uint8*self._max_pc_num))[0]
        arr_size = (self._max_pc_num-1)>>3
        self._pc_uindex_ptr = libtitrk.gtrack_alloc(ctypes.sizeof(ctypes.c_uint8), arr_size )
        self._pc_uindex_arr = ctypes.cast(self._pc_uindex_ptr, ctypes.POINTER(ctypes.c_uint8*arr_size))[0]
        self._bench_ptr = libtitrk.gtrack_alloc(ctypes.sizeof(ctypes.c_uint32),  GTrkConst.BENCHMARK_SIZE)
        self._bench_arr = ctypes.cast(self._bench_ptr, ctypes.POINTER(ctypes.c_uint32* GTrkConst.BENCHMARK_SIZE))[0]

        libtitrk.gtrack_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(GtrkMeasurementPoint2D*self._max_pc_num), ctypes.POINTER(GtrkMeasurementVector2D*self._max_pc_num), ctypes.c_ushort,
                                    ctypes.POINTER(GtrkTargetDesc*self._max_tgt_num), ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
                                    ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint32) ]
        libtitrk.gtrack_array_test.argtypes = [ctypes.c_void_p, ctypes.POINTER(GtrkMeasurementPoint2D*self._max_pc_num), ctypes.POINTER(GtrkMeasurementVector2D*self._max_pc_num), ctypes.c_ushort,
                                    ctypes.POINTER(GtrkTargetDesc*self._max_tgt_num), ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
                                    ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint32) ]
        return True

    def process(self, pc_cart, idx_doppler) :
        # gtrack_step(void *handle, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t mNum
        #            , GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint8_t *uIndex, uint8_t *presence, uint32_t *bench);
        num = len(pc_cart) if len(pc_cart) < self._max_pc_num else self._max_pc_num
        if False :
            xs = pc_cart[:num,0]
            ys = pc_cart[:num,1]
            zs = pc_cart[:num,2]
            for i in range(num):
                self._pc_nparr[i,0] = np.sqrt(xs[i]**2 + ys[i]**2)    # range
                self._pc_nparr[i,1] = np.arctan2(xs[i], ys[i])       # azimuth
                self._pc_nparr[i,2] = pc_cart[i,idx_doppler]
                self._pc_nparr[i,3] = pc_cart[i,idx_doppler-1]
        else:
            xs = pc_cart[:num,0]
            ys = pc_cart[:num,1]
            zs = pc_cart[:num,2]
            self._pc_nparr[:num,0] = np.sqrt(xs**2+ys**2)    # range
            self._pc_nparr[:num,1] = np.arctan2(xs,ys)       # azimuth
            self._pc_nparr[:num,2] = pc_cart[:num,idx_doppler]
            self._pc_nparr[:num,3] = pc_cart[:num, idx_doppler+1 if idx_doppler == 3 else idx_doppler-1 ]  ## stupid...

        libtitrk.gtrack_step(self._handle, self._pc_arr, None, num, self._tgt_arr, ctypes.byref(self._tgt_num), self._pc_index_arr, self._pc_uindex_arr, None, self._bench_arr)
        if True :
            if self._bench_max_ns < self._bench_arr[6]-self._bench_arr[0] :
                self._bench_max_ns = self._bench_arr[6]-self._bench_arr[0]
            print(f"Target is {self._tgt_num}, bench({self._bench_arr[6]-self._bench_arr[0]}(max:{self._bench_max_ns}):{self._bench_arr[0]}~{self._bench_arr[6]})[0 {self._bench_arr[1]-self._bench_arr[0]} {self._bench_arr[2]-self._bench_arr[1]} {self._bench_arr[3]-self._bench_arr[2]} {self._bench_arr[4]-self._bench_arr[3]} {self._bench_arr[5]-self._bench_arr[4]} {self._bench_arr[6]-self._bench_arr[5]}]")
        return self._tgt_num, self._tgt_arr

    def test(self):
        libtitrk.gtrack_array_test(self._handle, self._pc_arr, None, 256, self._tgt_arr, ctypes.byref(self._tgt_num), self._pc_index_arr, self._pc_uindex_arr, None, self._bench_arr)
        print(f"Target is {self._tgt_num}")

# ----------------------------------
def print_structure(pre, data):
    print(f"{pre}---Structure : {type(data).__name__}---")
    for f in data._fields_:
        val = getattr(data, f[0])
        if isinstance(val, ctypes.Structure):
            print(f"{pre}\tFN:{f[0]} FT:{f[1]} --Structure-- ")
            print_structure(pre+"\t", val)
        elif isinstance(val, ctypes.Array) :
            print(f"{pre}\tFN:{f[0]} FT:{f[1]} --Array-- ")
            for v in val:
                if isinstance(v, ctypes.Structure):
                    print_structure(pre+"\t\t", v)
                elif isinstance(v, ctypes._Pointer):
                    if v :
                        print(f"{pre}\t\t--Pointer-- ")
                        print_structure(pre+"\t", v.contents)
                    else :
                        print(f"{pre}\t\tpointer.NULL")
                else:
                    print(f"{pre}\t\tFT:{type(v)} FV:{v}")
        elif isinstance(val, ctypes._Pointer):
            if val :
                print(f"{pre}\tFN:{f[0]} --Pointer-- ")
                print_structure(pre+"\t", val.contents)
            else :
                print(f"{pre}\tpointer.NULL")
        else :
            print(f"{pre}\tFN:{f[0]} FT:{f[1]} FV:{val}")
    print(f'{pre}-----------------')

def convert_structure_to_dict(data):
    _dict = dict()
    for f in data._fields_:
        val = getattr(data, f[0])
        if isinstance(val, ctypes.Structure) :
            _dict[f[0]] = convert_structure_to_dict(val)
        elif isinstance(val, ctypes.Array) :
            _sub = list()
            for v in val :
                if isinstance(v, ctypes.Structure):
                    _sub.append(convert_structure_to_dict(v))
                elif isinstance(v, ctypes._Pointer):
                    if v :
                        _sub.append(convert_structure_to_dict(v.contents))
                    else :
                        _sub.append(None)
                else :
                    _sub.append(v)
            _dict[f[0]] = _sub
        elif isinstance(val, ctypes._Pointer):
            if val :
                _dict[f[0]] = convert_structure_to_dict(val.contents)
            else :
                _dict[f[0]] = None
        else :
            _dict[f[0]] = getattr(data, f[0])
    return _dict

# ----------------------------------
def apply_dict_to_strt(pdict, strt : ctypes.Structure):
    for fi, fname in enumerate(pdict) :
        idl = [ idx for idx, fitem in enumerate(strt._fields_) if fitem[0] == fname ]
        if len(idl) == 1 :
            # fname = strt._fields_[idl[0]][0]
            val = getattr(strt, fname)
            if isinstance(val, ctypes.Structure) :
                if isinstance(pdict[fname], dict) :
                    apply_dict_to_strt(pdict[fname], val)
                    ##_dict[f[0]] = convert_structure_to_dict(val)
                else :
                    print(f"ERR.{type(pdict[fname].__name__)}, STRT != DICT")
                    break
            elif isinstance(val, ctypes.Array) :
                if isinstance(pdict[fname], list) :
                    i = 0
                    dval = pdict[fname]
                    for v in val :
                        if isinstance(v, ctypes.Structure):
                            if isinstance(dval[i], dict) :
                                apply_dict_to_strt(dval[i], v)
                            else :
                                print(f"ERR.{type(dval.__name__)} in list, STRT != DICT")
                                break
                        elif isinstance(v, ctypes._Pointer):
                            if v :
                                apply_dict_to_strt(dval[i], v.contents)
                            else :
                                print(f"ERR.{type(dval.__name__)} in list, Pointer is None")
                        else :
                            val[i] = dval[i]
                        i += 1
                else :
                    print(f"ERR.{type(pdict[fname].__name__)}, Array != List")
            elif isinstance(val, ctypes._Pointer):
                if val :
                    apply_dict_to_strt(pdict[fname], val.contents)
                else :
                    print(f"ERR.{type(dval.__name__)}, Pointer is None")
                pass
            else :
                setattr(strt, fname, pdict[fname])

# ----------------------------------
# ----------------------------------
# ----------------------------------


if __name__ == '__main__':
    # tigtrk = TiGTracker()
    # libtitrk.gtrack_log(0, "test0--going...\r\n".encode("ascii") )
    # pcfg = tigtrk.get_default_config()
    # if tigtrk.create(pcfg) :
    #     tigtrk.test()
    # libtitrk.gtrack_log(1, "test1--going...\r\n".encode("ascii") )

    tjconfig = TiGTrkJsonConfig()
    tjconfig.save("d:\\titrk_def.json")
    # pdict = tjconfig.load_from_file("d:\\test2.json")
    # apply_dict_to_strt( pdict, tjconfig._cfg_strt )
    # print_structure("--", tjconfig._cfg_strt)
    # tjconfig.save("d:\\test3.json")






