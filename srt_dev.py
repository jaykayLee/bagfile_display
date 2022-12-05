import struct
import sys
import serial
import binascii
import time
import numpy as np
import math
import os
import datetime
import pathlib
import typing
import threading
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal, QMutex, QMutexLocker, QPoint, QObject, pyqtSlot

lpdbg_on = False    ## line debugger
vcdbg_on = False    ## vscode debugger
if vcdbg_on :
    import debugpy

## for debugging
def ldprint(msgs):
    if lpdbg_on :
        print(msgs)

# ----------------------------------
class SRTDevice(QObject):
    pass

# ----------------------------------
class SRTAppType:
    UNKNOWN          : typing.Final[int] = -1
    AREASCAN_DEV     : typing.Final[int] = 0
    AREASCAN_BIN     : typing.Final[int] = 1
    NAMES            : typing.Final[list] = ["AREASCAN-DEV", "AREASCAN-BIN"]

# ----------------------------------
class SRTDumper(QObject):
    _dump_filepath = None
    _dump_file = None
    _dump_length = -1
    _lock = threading.Lock()

    def open(self, filepath):
        self._dump_filepath = filepath
        self._dump_length = 0
        self._dump_file = open(filepath, 'wb')
        return True

    def close(self) :
        self._dump_file.close()

    def write(self, data:bytes):
        with self._lock :
            self._dump_file.write(data)
            self._dump_file.flush()

# ----------------------------------
class tlvparser():
    def __init__(self,type=SRTAppType.AREASCAN_DEV):
        self._magics_bin = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        self._magics_num = 0x708050603040102
        self._radar_platform_bin = b'\x43\x68\x0A\x00'
        self._radar_platform_num = 0x000A6843
        self._cam_platform_bin = b'\x01\x00\x0C\x00'
        self._cam_platform_num = 0x000C0001
        self._bytes = None
        self._preload_len = 44
        self._num_req = 10000
        self._fail = 0
        self._dev = None
        self._tlvheader_len = 8  # struct.calcsize('2I')
        self._dumper = None
        self.setup_app_type(type)


    def setup_app_type(self, type=SRTAppType.AREASCAN_DEV, dumper=None):
        if type == SRTAppType.AREASCAN_DEV or type == SRTAppType.AREASCAN_BIN:
            self._dumper = dumper
            self._app_type = type
            self._preload_len = 44
            self._num_req = self._preload_len
        else:
            self._app_type = SRTAppType.UNKNOWN

    def build_cam_data(self, fn:int, data:bytes):
        strt_fmt = 'Q4I'  # magic, version, totalpkt_len, platform, frame_num
            # w = version & 0xFF
            # h = (version & 0xFF00 ) >> 16
        hdsize = struct.calcsize(strt_fmt)
        pkt_bin = struct.pack(strt_fmt, self._magics_num, 0, hdsize+len(data), self._cam_platform_num, fn )
        pkt_bin += data
        return pkt_bin

    #decode People Counting TLV Header
    def tlvHeaderDecode(self, data):
        #print(len(data))
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength

    def polar2Cart3D(self, num_obj, pobjs, cobjs):
        for n in range(0, num_obj):
            cobjs[n,2] = pobjs[n,0]*np.sin(pobjs[n,2])  # z
            cobjs[n,0] = pobjs[n,0]*np.cos(pobjs[n,2])*np.sin(pobjs[n,1])  # x
            cobjs[n,1] = pobjs[n,0]*np.cos(pobjs[n,2])*np.cos(pobjs[n,1])  # y

    def parse_areascan(self):
        #strt_fmt = 'Q9I'  # split Q4I 5I
        strt_fmt = 'Q4I'  # split Q4I 5I
        error_code = 0
        strt_size = struct.calcsize(strt_fmt)
        off = 0
        if strt_size > len(self._bytes):
            self._num_req = self._preload_len
            ldprint(f"too short ... retry return")
            return None
        try :
            #magic, version, totalpkt_len, platform, frame_num, cpu_cycle, num_detobj, num_tlv, subf_num, num_static_detobj = struct.unpack(strt_fmt, self._bytes[:strt_size])
            magic, version, totalpkt_len, platform, frame_num = struct.unpack(strt_fmt, self._bytes[:strt_size])
        except:
            self._fail = 1
            self._bytes = None
            ldprint(f"unknown error drops...")
            return None

        off = strt_size
        ldprint(f"***FN:{frame_num} pkt_len:{totalpkt_len} ")
        #data = self._bytes[strt_size:]
        remain_len = totalpkt_len - len(self._bytes)
        retry = 0
        while remain_len > 0 and retry < 5 :
            ndata = self._dev.read(remain_len)
            ldprint(f"READ.S.UART={len(ndata)}, pre_size=[{ len(self._bytes) }], req_total={totalpkt_len}, req_size={remain_len}, retry={retry}")
            remain_len = totalpkt_len - strt_size - len(self._bytes) - len(ndata)
            self._bytes += ndata
            retry += 1
        if retry >= 5 or remain_len > 0 :
            self._bytes = None
            self._num_req = self._preload_len
            ldprint(f"warning... skip packet receivning...")
            return None

        ## case of camera
        if platform == self._cam_platform_num :
            img = self._bytes[off:totalpkt_len]
            self._bytes = self._bytes[totalpkt_len:]
            ldprint(f".. consume by cam {strt_size+totalpkt_len} ..remains..{len(self._bytes)}")
            return (frame_num, img)

        ## else radar...
        strt_fmt = '5I'
        strt_size = struct.calcsize(strt_fmt)
        try :
            #magic, version, totalpkt_len, platform, frame_num, cpu_cycle, num_detobj, num_tlv, subf_num, num_static_detobj = struct.unpack(strt_fmt, self._bytes[:strt_size])
            cpu_cycle, num_detobj, num_tlv, subf_num, num_static_detobj = struct.unpack("<5I", self._bytes[off:off+strt_size])
        except:
            self._fail = 1
            self._bytes = None
            ldprint(f"unknown error 2 drops...")
            return None

        off += strt_size
        data = self._bytes[off:]
        #items = [frame_num, subf_num, cpu_cycle, num_tlv, num_detobj, num_static_detobj ]
        items = dict()
        items['frame_num'] = frame_num
        items['subf_num'] = subf_num
        items['cpu_cycle'] = cpu_cycle
        items['num_tlv'] = num_tlv
        items['num_detobj'] = num_detobj
        items['num_static_detobj'] = num_static_detobj
        items['detobjs'] = np.zeros((num_detobj, 6))
        items['detobjs_cart'] = np.zeros((num_detobj, 3))
        items['static_detobj'] = np.zeros((num_static_detobj, 6))
        items['num_target'] = 0 ## yet unknown, fix in tlv
        items['azhmap'] = None
        items['dophmap'] = None
        for i in range(num_tlv):
            try :
                tlv_type, tlv_length = struct.unpack('2I', data[:self._tlvheader_len] )
                ldprint(f"TLV.{i}/{num_tlv}, TLV:{tlv_type} len:{tlv_length}")
            except Exception as e:
                ldprint(f"ERROR.tlv parsing.[{e}]" )
                error_code = -99
                break
            data = data[self._tlvheader_len:]
            if tlv_type == 1 : # MMWDEMO_OUTPUT_MSG_DETECTED_POINTS
                # range, azimuthAngle, elevAngle, velocity, + sideinfo( snr, noise )
                it_size = struct.calcsize('4f')
                it_count = tlv_length//it_size
                if it_count != num_detobj : 
                    ldprint(f"..TLV detobj size check  {num_detobj} != {it_count}")
                    error_code = -101
                    break
                dpt = items['detobjs']
                for i in range(it_count):
                    dpt[i,0:4] = struct.unpack('4f', data[it_size*i:it_size*(i+1)])
                self.polar2Cart3D(it_count, items['detobjs'], items['detobjs_cart'])
            elif tlv_type == 2: # MMWDEMO_OUTPUT_MSG_RANGE_PROFILE
                pass
            elif tlv_type == 3: # MMWDEMO_OUTPUT_MSG_NOISE_PROFILE
                pass
            elif tlv_type == 4: # MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP
                it_count = tlv_length//4
                harr = struct.unpack(f'{it_count}i', data[:tlv_length])
                # str += f"{i} = {np.array2string(pc, separator=',  ', max_line_width=400, formatter={'float_kind':lambda x: '%.2f' % x})}\n"
                #harr = np.array(harr)
                hmap = np.array(harr).reshape(64,32)
                for r in range(hmap.shape[0]):
                    ldprint( f"{r}={hmap[r]}")
                #print( "AzMap=" + np.array2string(harr, separator=', ', max_line_width=200) )
                ldprint("-"*80)
                #items['azhmap'] = hmap = np.array(harr).reshape(64,32)
                #hmap.ravel()[]
                #hmap
                #items['azhmap_image'] = 
                pass
            elif tlv_type == 5: # MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP
                rbin_size = 128
                dbin_size = 32
                it_count = tlv_length//2
                if it_count == rbin_size*dbin_size :
                    harr = struct.unpack(f'{it_count}H', data[:tlv_length])
                    hmap = np.array(harr).reshape(rbin_size, dbin_size)
                    items['dophmap'] = hmap
                else:
                    ldprint(f"..TLV range-doppler heatmap size check  {rbin_size*dbin_size} != {it_count}")
                    pass
                pass
            elif tlv_type == 6: # MMWDEMO_OUTPUT_MSG_STATS
                pass
            elif tlv_type == 7: # MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO
                it_size = struct.calcsize('2h')
                it_count = tlv_length//it_size
                if it_count != num_detobj : 
                    ldprint(f"..TLV detobj_si size check  {num_detobj} != {it_count}")
                    error_code = -107
                    break
                dpt = items['detobjs']
                for i in range(it_count):
                    dpt[i,4:6] = struct.unpack('2h', data[it_size*i:it_size*(i+1)])
            elif tlv_type == 8: # MMWDEMO_OUTPUT_MSG_STATIC_DETECTED_POINTS
                # x, y, z, velocity, + sideinfo( snr, noise )
                it_size = struct.calcsize('4f')
                it_count = tlv_length//it_size
                if it_count != num_static_detobj : 
                    ldprint(f"..TLV static_detobj size check  {num_static_detobj} != {it_count}")
                    error_code = -108
                    break
                spt = items['static_detobj']
                for i in range(it_count):
                    spt[i,0:4] = struct.unpack('4f', data[it_size*i:it_size*(i+1)])
            elif tlv_type == 9: # MMWDEMO_OUTPUT_MSG_STATIC_DETECTED_POINTS_SIDE_INFO
                it_size = struct.calcsize('2h')
                it_count = tlv_length//it_size
                if it_count != num_static_detobj : 
                    ldprint(f"..TLV static_detobj_si size check  {num_static_detobj} != {it_count}")
                    error_code = -109
                    break
                dpt = items['static_detobj']
                for i in range(it_count):
                    dpt[i,4:6] = struct.unpack('2h', data[it_size*i:it_size*(i+1)])
            elif tlv_type == 10: # MMWDEMO_OUTPUT_MSG_TRACKERPROC_OUTPUT_TARGET_LIST
                it_size = struct.calcsize('I9f')
                it_count = tlv_length//it_size
                items['num_target'] = it_count
                tgt = np.zeros((it_count, 10))
                items['targets'] = tgt
                for i in range(it_count):
                    tgt[i] = struct.unpack('I9f', data[it_size*i:it_size*(i+1)])
                    ## targetId, posX, posY, velX, velY, accX, accY, posZ, velZ, accZ, [EC[16], g]
                items['targets_cart'] = np.zeros((it_count, 3))
                items['targets_cart'][:,0:2] = items['targets'][:,1:3]
                items['targets_cart'][:,2] = items['targets'][:,7]
            elif tlv_type == 11: # MMWDEMO_OUTPUT_MSG_TRACKERPROC_OUTPUT_TARGET_INDEX
                items['num_indices'] = tlv_length
                indices = np.zeros((tlv_length))
                items['indices'] = indices
                it_count = tlv_length
                if it_count != len(data[:tlv_length]) : 
                    ldprint(f"..TLV indices size check  {it_count} != {len(data[:tlv_length])}")
                    error_code = -111
                    break
                indices[:] = struct.unpack(f'{it_count}B', data[:tlv_length])

            data = data[tlv_length:]
        if error_code < 0 :
            ldprint(f"parsing Error{error_code} drop FN:{frame_num}")
            self._bytes = None
            return None
        
        if self._dumper != None:
            self._dumper.write(self._bytes[:totalpkt_len])
        self._bytes = self._bytes[totalpkt_len:]
        ldprint(f".. consume by radar {strt_size+totalpkt_len} ..remains..{len(self._bytes)}")
        return items

    def process_dev(self, upper, dev):
        self._dev = dev
        data = dev.read(self._num_req)
        if data == None or len(data) < 1 :
            return None
        ldprint(f"READ.UART={len(data)}, pre_size=[{ 0 if self._bytes == None else len(self._bytes) }]")

        if self._bytes == None :
            self._bytes = data
        else:
            self._bytes += data
        
        off = self._bytes.find(self._magics_bin)
        if off == -1 :
            ldprint(f"no magic, drop all {len(self._bytes)}[{self._bytes}]")
            self._bytes = None
            return

        self._bytes = self._bytes[off:]
        ldprint(f"..Find.Magic={off}, after size=[{ len(self._bytes) }]")
        if self._app_type == SRTAppType.AREASCAN_DEV or self._app_type == SRTAppType.AREASCAN_BIN :
            tlvs = self.parse_areascan()

        if tlvs != None :
            if type(tlvs).__name__ == 'dict' :
                upper.data_rx_cb.emit(tlvs)
            else :
                upper._capture_cb.emit(tlvs[1])
        return None

    def do_preload(self, dev):
        self._dev = dev
        hdr_size = 0
        frame_count = 0
        frame_list = []
        frame_pos_list = []
        offset = 0

        dev.seek(0, 2)
        file_length = dev.tell()
        dev.seek(0, 0)

        if self._app_type == SRTAppType.AREASCAN_DEV or self._app_type == SRTAppType.AREASCAN_BIN :
            #strt_fmt = 'Q9I'
            strt_fmt = 'Q4I'
            hdr_size = struct.calcsize(strt_fmt)
            offset = 0

            while offset < file_length :
                data = dev.read(hdr_size)
                if data == None :
                    break
                off = data.find(self._magics_bin)
                if off != 0 :
                    ldprint(f"no magic, drop all {len(data)}[{data}], current file offset is {offset}")
                    return
                try :
                    #magic, version, totalpkt_len, platform, frame_num, cpu_cycle, num_detobj, num_tlv, subf_num, num_static_detobj = struct.unpack(strt_fmt, data)
                    magic, version, totalpkt_len, platform, frame_num = struct.unpack(strt_fmt, data)
                except:
                    self._fail = 1
                    return None
                # if platform == self._radar_platform_num :
                frame_list.append(frame_num)
                frame_pos_list.append(offset)
                # else,  camera or others..
                dev.seek(totalpkt_len-hdr_size, 1)
                offset += totalpkt_len
            dev.seek(0, 0)
            return file_length, frame_list, frame_pos_list
        return None

# ----------------------------------
class SRTDevice(QObject):
    _capture_cb = pyqtSignal(object) # for camera data
    config_rx_cb = pyqtSignal(bytes)
    data_rx_cb = pyqtSignal(object)
    #_is_replay = False
    _dump_filepath = None
    _dumper = None
    _app_type = SRTAppType.UNKNOWN
    _cfg_dev_name = None
    _data_dev_name = None
    _data_dev = None
    _cfg_dev = None
    _run_on = False
    _cfg_lines = []
    _event = threading.Event()
    _event_mode = 0
    _parser = None
    _log_file_length = -1
    _log_frame_list = None
    _log_frame_pos_list = None
    _log_frame_index = 0  ## only seeking mode.
    _data_rx_thread = None
    _cfg_rx_thread = None

    def __init__(self, manager, outq=None):
        super(SRTDevice, self).__init__()

    def open_dev(self, cfg_port_name, data_port_name):
        self.close()
        #self._is_replay = False
        self._cfg_dev_name = cfg_port_name
        self._data_dev_name = data_port_name
        self._app_type = SRTAppType.AREASCAN_DEV
        self._parser = tlvparser(self._app_type)

        try:
            if self._cfg_dev_name != None and self._cfg_dev_name != 'None' :
                self._cfg_dev = serial.Serial(self._cfg_dev_name, 115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.1)
                self._cfg_dev.reset_input_buffer()
                self._cfg_rx_thread = ConfigRxThread(self, None, None)
            if self._data_dev_name != None and self._cfg_dev_name != 'None' :
                #self._data_dev = serial.Serial(self._data_dev_name, 921600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.025)
                self._data_dev = serial.Serial(self._data_dev_name, 921600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.033)
                self._data_dev.reset_input_buffer()
                self._data_rx_thread = DataRxThread(self, None, None)
            if self._data_dev.is_open :
                self._dump_filepath = os.getcwd() + os.path.sep + "bin_"  + SRTAppType.NAMES[self._app_type]
                if pathlib.Path(self._dump_filepath).is_dir() == False :
                    os.makedirs(self._dump_filepath)
                filepath = self._dump_filepath + os.path.sep + SRTAppType.NAMES[self._app_type] + datetime.datetime.now().strftime("_%Y%m%d_%H%M%S.bin")
                self._dumper = SRTDumper()
                self._dumper.open(filepath)
                return True
        except Exception as e:
            print("ERROR.Open Dev.", e)
        
        if self._cfg_dev != None :
            self._cfg_dev.close()
            self._cfg_dev = None
        if self._data_dev != None :
            self._data_dev = None
        return False

    def open_file(self, filepath):
        self.close()
        if os.path.isfile(filepath) == False :
            return False

        self._dump_filepath = None
        self._dumper = None
        #self._is_replay = True
        self._cfg_dev_name = None
        self._data_dev_name = filepath
        self._cfg_dev = None
        self._app_type = SRTAppType.AREASCAN_BIN
        self._data_dev = open(self._data_dev_name, 'rb')
        self._parser = tlvparser(self._app_type)

        res = self._parser.do_preload(self._data_dev)
        if res != None :
            self._log_file_length = res[0]
            self._log_frame_list = res[1]
            self._log_frame_pos_list = res[2]

        self._event_mode = 0
        self._event.clear()
        self._data_rx_thread = DataRxThread(self, None, None)
        return True
        

    def close(self):
        self.stop()
        self._dump_filepath = None
        #self._is_replay = False
        self._app_type = SRTAppType.UNKNOWN
        if self._dumper != None :
            self._dumper.close()
            self._dumper = None
        if self._cfg_dev != None :
            self._cfg_dev.close()
            self._cfg_dev = None
            self._cfg_rx_thread.wait
        if self._data_dev != None :
            self._data_dev.close()
            self._data_dev = None
        self._event.clear()

    def send_config(self, lines):
        if self._cfg_dev == None:
            return
        if type(lines).__name__ == 'list':
            self._cfg_lines.extend(lines)
        elif type(lines).__name__ == 'str':
            self._cfg_lines.append(lines)

    def is_open(self):
        # if self._is_replay :
        #     return self._data_dev != None
        # return True if self._data_dev != None and self._data_dev.is_open else False
        return self._app_type != SRTAppType.UNKNOWN

    def start(self):
        if self.is_open() == False :
            return
        self._run_on = True
        if self._cfg_dev != None:
            self._cfg_rx_thread.start()

        if self._data_dev != None:
            self._parser.setup_app_type(self._app_type, self._dumper)
            self._data_rx_thread.start()

    def stop(self):
        self._run_on = False
        if self._cfg_rx_thread != None :
            if self._cfg_rx_thread.wait(500) == False:
                print(f'Config rx thread is not terminated..')
        if self._data_rx_thread != None:
            if self._data_rx_thread.wait(500) == False:
                print(f'data rx thread is not terminated..')


    def is_playing(self) :
        return self._app_type==SRTAppType.AREASCAN_BIN and self._event_mode == 101

    def prev(self, step=-1):
        if self._app_type!=SRTAppType.AREASCAN_BIN : return
        print('not implement : prev')

    def next(self, step=1):
        if self._app_type!=SRTAppType.AREASCAN_BIN : return
        self._event_mode = 102
        self._event.set()

    def pause(self):
        if self._app_type!=SRTAppType.AREASCAN_BIN : return
        self._event_mode = 100
        self._event.set()

    def play(self):
        if self._app_type!=SRTAppType.AREASCAN_BIN : return
        self._event_mode = 101
        self._event.set()

    def reverse_play(self):
        if self._app_type!=SRTAppType.AREASCAN_BIN : return
        print('not implement : reverse_play')

    def set_chunk_pos(self, pos):
        if self._app_type!=SRTAppType.AREASCAN_BIN : return
        #print('not implement : set_chunk_pos')
        self._log_frame_index = pos
        self._event_mode = 105
        self._event.set()
        time.sleep(0.2)

    def append_cam_data(self, fn:int, data:bytes):
        if self._dumper != None and self._parser != None :
            sitem = self._parser.build_cam_data(fn, data)
            if sitem != None :
                self._dumper.write(sitem)

# ----------------------------------
class ConfigRxThread(QThread):
    _upper : SRTDevice = None

    def __init__(self, upper, manager, outq=None):
        super(ConfigRxThread, self).__init__()
        self._upper = upper

    def run(self):
        if vcdbg_on : debugpy.debug_this_thread()
        print("--- start run_cfg_rx ---")
        while self._upper._run_on:
            data = self._upper._cfg_dev.read(1024)
            if data != None and len(data) > 0 :
                #print(f"<<{len(data)}=" + str(data) )
                if self._upper.config_rx_cb != None: self._upper.config_rx_cb.emit(data)
            if len(self._upper._cfg_lines) > 0 :
                for ln in self._upper._cfg_lines:
                    ln += '\r'
                    self._upper._cfg_dev.write(ln.encode())
                    #print(f">>{len(ln)}>>" + ln )
                    while True:
                        rln = self._upper._cfg_dev.readline()
                        if rln == None or len(rln) < 1:
                            break
                        self._upper.config_rx_cb.emit(rln)
                        #print(f"<<{len(rln)}=" + str(rln) )
                self._upper._cfg_lines.clear()
        print("--- exit run_cfg_rx ---")

# ----------------------------------
class DataRxThread(QThread):
    _upper : SRTDevice  = None

    def __init__(self, upper, manager, outq=None):
        super(DataRxThread, self).__init__()
        self._upper = upper

    def run(self):
        if vcdbg_on : debugpy.debug_this_thread()
        print("--- start run_data_rx ---")
        parser = self._upper._parser
        while self._upper._run_on :
            if self._upper._app_type==SRTAppType.AREASCAN_BIN :
                if self._upper._event.wait(0.1) != True :
                    continue
                self._upper._event.clear()
                if self._upper._event_mode == 100 : # pause
                    print(f"DataRxThread event_mode is pause:{self._upper._event_mode}")
                    continue
                elif self._upper._event_mode == 101 : # play
                    print(f"DataRxThread event_mode is play:{self._upper._event_mode}")
                    while self._upper._run_on :
                        if self._upper._event_mode != 101 :
                            print(f"DataRxThread event_mode is changed:{self._upper._event_mode}")
                            break
                        parser.process_dev(self._upper, self._upper._data_dev)
                        fpos = self._upper._data_dev.tell()
                        if fpos >= self._upper._log_file_length :
                            self._upper._event_mode = 100
                            self._upper.data_rx_cb.emit(None)
                        time.sleep(0.01)
                elif self._upper._event_mode == 102 : # next
                    print(f"DataRxThread event_mode is next:{self._upper._event_mode}")
                    parser.process_dev(self._upper, self._upper._data_dev)
                elif self._upper._event_mode == 105 : # go pos
                    pos = self._upper._log_frame_pos_list[self._upper._log_frame_index]
                    self._upper._data_dev.seek(pos, 0)
                    parser.process_dev(self._upper, self._upper._data_dev)
            else :
                parser.process_dev(self._upper, self._upper._data_dev)
        print("--- exit run_data_rx ---")

# ----------------------------------
if __name__ == '__main__':
    class TestReceiver(QObject):
        def __init__(self):
            super(TestReceiver, self).__init__()

        @pyqtSlot(bytes)
        def config_rx(self, str):
            print(f"RXED-CFG={len(str)}[{str}]")
            
        @pyqtSlot(bytes)
        def data_rx(self, bin):
            print(f"RXED-DATA={len(bin)}[{bin}]")

        def assign_cb(self, srt):
            srt.config_rx_cb.connect(self.config_rx)
            srt.data_rx_cb.connect(self.data_rx)


    srt = SRTDevice(None, None)
    trx = TestReceiver()
    # trx.init(srt)

    srt.open_dev('COM4', 'COM3')
    trx.assign_cb(srt)
    srt.start()
    while True:
        instr = input(">>")
        if instr == 'quit':
            break
        else :
            srt.send_cfg(instr)
        time.sleep(0.5)
    
    srt.stop()
    sys.exit(0)
