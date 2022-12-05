from io import SEEK_END, SEEK_SET, BufferedWriter, BufferedReader
from lib2to3.pgen2.token import NAME
from multiprocessing.pool import CLOSE
import queue
import sys
import datetime
import time
import struct
import os
import string
import numpy as np
import multiprocessing as mp
import threading
import ctypes
import socket
import typing
import pathlib
import abc as ABC # abstract base classes
from bag_parse import BagFileReader

lpdbg_on = True    ## line debugger
vcdbg_on = True    ## vscode debugger
if vcdbg_on :
    import debugpy

## for debugging
def ldprint(msgs):
    if lpdbg_on :
        print(msgs)


# ----------------------------------
def getstr_timestamp(prefix, tm_sec, tm_nsec):
        return f"{prefix} time:{datetime.datetime.fromtimestamp(tm_sec).strftime('%Y/%m/%d %H:%M:%S.')} + {str(tm_nsec)} "

def get_cur_timestamp():
    ct = time.time()
    return [int(ct), int((ct-int(ct))*1000000000)]

# ----------------------------------
class SRPEvent:
    PAUSE       : typing.Final[int] = 0
    PLAY        : typing.Final[int] = 1
    RPLAY       : typing.Final[int] = 2
    GO          : typing.Final[int] = 3
    BAGFILE     : typing.Final[int] = 100
    SOCK        : typing.Final[int] = 101
    BINFILE     : typing.Final[int] = 102
    CLOSE       : typing.Final[int] = 120
    CS_CMD      : typing.Final[int] = 130

class SRPErrCode:
    ERR_EVENT_CODE_MIN  : typing.Final[int] = 0
    ERR_EVENT_CODE_MAX  : typing.Final[int] = 499
    ERR_EOF             : typing.Final[int] = -500
    ERR_NOT_RUN         : typing.Final[int] = -1
    ERR_UNKNOWN_EVT     : typing.Final[int] = -2

class SR4PKT:
    #PRE_HDSIZE          : typing.Final[int] = 176
    NET_HDSIZE          : typing.Final[int] = 36
    FRAME_HDSIZE        : typing.Final[int] = 176
    PAYLOAD_HDSIZE      : typing.Final[int] = 40
    ## index from Net Header
    ## Buf_num(dim), Magic, Payload_type, channel(fn), data_size, width(r), height(r), pitch1(r), pitch2(r)
    IDX_NET_BUFNUM      : typing.Final[int] = 0
    IDX_NET_DIM         : typing.Final[int] = 0
    IDX_NET_MAGIC       : typing.Final[int] = 1
    IDX_NET_PTYPE       : typing.Final[int] = 2
    IDX_NET_CHANNEL     : typing.Final[int] = 3
    IDX_NET_FRAMENUM    : typing.Final[int] = 3
    IDX_NET_DATA_SIZE   : typing.Final[int] = 4
    IDX_NET_WIDTH       : typing.Final[int] = 5
    IDX_NET_IP          : typing.Final[int] = 5
    IDX_NET_HEIGHT      : typing.Final[int] = 6
    IDX_NET_PORT        : typing.Final[int] = 6
    IDX_NET_PITCH1      : typing.Final[int] = 7
    IDX_NET_PITCH2      : typing.Final[int] = 8
    IDX_NET_TMSTMP1     : typing.Final[int] = 7
    IDX_NET_TMSTMP2     : typing.Final[int] = 8
    OFFSET_NET_TMSTMP   : typing.Final[int] = 28
    ## index from frame header
    IDX_FHD_MAGIC       : typing.Final[int] = 0
    IDX_FHD_PROTOCOL_VER: typing.Final[int] = 1
    IDX_FHD_HDR_SIZE    : typing.Final[int] = 2
    IDX_FHD_PAYLOAD_SIZE: typing.Final[int] = 3
    IDX_FHD_ANT_MODEL   : typing.Final[int] = 4
    IDX_FHD_SBL_VER     : typing.Final[int] = 5
    IDX_FHD_FIRM_VER    : typing.Final[int] = 6
    IDX_FHD_SERIAL      : typing.Final[int] = 7
    IDX_FHD_RSV_TMSTAMP : typing.Final[int] = 8
    ## index from Payload header
    IDX_PLHD_FRAME_NUM  : typing.Final[int] = 0
    IDX_PLHD_TIMESTAMP  : typing.Final[int] = 1
    IDX_PLHD_TGTINFO_TYPE : typing.Final[int] = 2
    IDX_PLHD_TGT_NUM    : typing.Final[int] = 3
    IDX_PLHD_HDR_SIZE   : typing.Final[int] = 4


class SR4AddData:
    RADAR               : typing.Final[int] = 0
    CAM_STILL           : typing.Final[int] = 1

class SR4AppType:
    AT_UNKNOWN          : typing.Final[int] = -1
    AT_R4FDEV           : typing.Final[int] = 0
    AT_R4FFILE          : typing.Final[int] = 1
    AT_R4BAG            : typing.Final[int] = 2
    NAME                : typing.Final[list] = ["R4F-DEV", "R4F-BIN", "R4F-BAG"]

class SR4ERR:
    NORMAL              : typing.Final[int] = 0
    DT_RCV_ERR          : typing.Final[int] = -100
    DT_READ_ERR         : typing.Final[int] = -101
    DT_SOCK_ERR         : typing.Final[int] = -102


class WRAPPER:
    INVALID : typing.Final[int] = -1
    SOCKET  : typing.Final[int] = 0
    BINF    : typing.Final[int] = 1
    BAGF    : typing.Final[int] = 2

## -----------------------------
## -----------------------------
class sr4base_dev(metaclass=ABC.ABCMeta):
    _wtype : int = WRAPPER.INVALID
    _run_count : int = 0
    _dict : dict = None
    _outq : mp.Queue = None
    ##
    def get_type(self):
        return self._wtype

    @ABC.abstractmethod
    def open(self, params:list, param_count:int) -> bool:
        return NotImplemented
    @ABC.abstractmethod
    def close(self):
        return NotImplemented
    @ABC.abstractmethod
    def oper_pause(self, params:list, param_count:int) :
        return NotImplemented
    @ABC.abstractmethod
    def oper_play(self, evtid:int, params:list, param_count:int, is_run, is_play_on, interval):
        return NotImplemented
    @ABC.abstractmethod
    def oper_go(self, params:list, param_count:int):
        return NotImplemented
    @ABC.abstractmethod
    def start(self, params:list, param_count:int):
        return NotImplemented
    @ABC.abstractmethod
    def stop(self):
        return NotImplemented
    @ABC.abstractmethod
    def send_console(self, cmds):
        return NotImplemented


# ----------------------------------
class SRTDumper():
    _dump_filepath : str = None
    _dump_file : BufferedWriter = None
    _dump_length : int = -1
    _inq = None
    _dumper_on = False
    _dumper_thread = None
    _parser = None

    def open(self, filepath, inq, parser):
        self._dump_filepath = filepath
        self._dump_length = 0
        self._dump_file = open(filepath, 'wb')
        self._inq = inq
        self._parser = parser
        self._dumper_on = True
        self._dumper_thread = threading.Thread(target=self.write_run)
        self._dumper_thread.start()
        return True

    def close(self) :
        self._dumper_on = False
        ### 
        self._dump_file.close()

    def write(self, data:bytes):
        self._inq.put(data)

    # def write_ind(self, item:list): ## [CAM, fn, img]
    #     if item[0] == SR4AddData.CAM_STILL :
    #         sitem = self._parser.build_dump_cam_data(item[1], item[2])
    #         if sitem != None :
    #             self._inq.put(sitem)

    ## -----------------------------
    def write_run(self):
        if vcdbg_on : debugpy.debug_this_thread()
        print(">>Enter In Dumper Thread--")
        while self._dumper_on :
            try :
                item = self._inq.get(timeout=0.5)
            except queue.Empty:
                continue
            except Exception as ie:
                print(ie)
            #print(f"InQ Data is {item}")
            if type(item).__name__ == 'bytes' :
                data = item
            elif type(item).__name__ == 'list'  and len(item) == 3 and item[0] == SR4AddData.CAM_STILL :
                if item[2] is None or len(item[2]) < 1 :
                    print(f">>><<<>>><<< ERROR >>><<<>>><<<")
                data = self._parser.build_dump_cam_data(item[1], item[2])
            if data is not None and len(data) > 0 and self._dump_file.closed != True :
                self._dump_file.write(data)
                self._dump_file.flush()

        print(">>Exit In Dumper Thread--")


## -----------------------------
def ip2int(addr):
    return struct.unpack("!I", socket.inet_aton(addr))[0]


def int2ip(addr):
    return socket.inet_ntoa(struct.pack("!I", addr))

## -----------------------------
## -----------------------------
class SR4DataParser():
    #-- Frame Header
    #   Magic(8B), protocol_ver(4B), Header_size(4B), payload_size(4B), antenna(4B), SBL_ver(4B), firm_ver(4B), serial(8B), reserved(128B)
    #           Q,           Uint32,          uint32,           uint32,          4B,          4B,           4B,         8B,           128B
    #-- payload header
    #   Frame_counter(4B), Time_stamp(8B), Target_info_type(4B),  Target_num(4B)
    #              uint32,         uint64,                   4B,          uint32
    # 

    def __init__(self,type=SR4AppType.AT_R4FDEV, dumper:SRTDumper=None):
        self._net_magic_bin : bytes = b'\x01\x00\x00\x00\x21\x43\xcd\xab'
        self._net_magic_num : int = 0xabcd4321
        self._net_data_length : int = 0
        self._magics_bin : bytes = b'\x01\x02\x03\x04\x05\x06\x07\x08'
        self._magics_num : np.int64 = 0x0807060504030201
        self._dummy_magic_bin : bytes = b'\x00\x00\x00\x00\xEC\x1C\x00\x00'
        self._dummy_magic_num : np.int64 = 0x1cec00000000
        self._radar_platform_bin : bytes = b'\x43\x68\x0A\x00'
        self._radar_platform_num : int = 0x000A6843
        self._cam_platform_bin : bytes = b'\x01\x00\x0C\x00'
        self._cam_platform_num : int = 0x000C0001
        self._bytes : bytes = None
        self._points = None
        self._pc_indices = None
        self._targets = None
        self._subdata_on = False
        self._num_req : int = 65535*2
        self._preload_len : int = SR4PKT.NET_HDSIZE    # begin with header size.
        self._fail : int = 0
        self._dev = None
        self._tlvheader_len : int = 8  # struct.calcsize('2I')
        self._dumper : SRTDumper = dumper
        self._netheader : list = None
        self._frameheader : list = None
        self._payloadheader : list = None
        self._err_count : int = 0
        self._err_max : int = 20
        self._err_code : int = 0
        self._prev_frame_number : int = 0
        self._frame_number : int = 0
        self._addrinfo = (0,0)
        self.set_app_type(type)

    def set_app_type(self, type=SR4AppType.AT_R4FDEV):
        if type == SR4AppType.AT_R4FDEV:
            self._app_type = SR4AppType.AT_R4FDEV # 
            self._preload_len = SR4PKT.NET_HDSIZE
        elif type == SR4AppType.AT_R4FFILE :
            self._app_type = SR4AppType.AT_R4FFILE #
        else:
            self._app_type = SR4AppType.AT_UNKNOWN

    def set_addr_info(self, ipaddr:int, port:int):
        self._addrinfo = (ipaddr, port)

    def build_dump_cam_data(self, fn:int, data:bytes):
        if fn < 0 :
            return None
        strt_fmt : string = "<9I"   ## Buf_num(dim), Magic, Payload_type, channel(fn), data_size, width, height, pitch1, pitch2
        ct = get_cur_timestamp()
        pkt_bin = struct.pack(strt_fmt, 1, self._net_magic_num, SR4AddData.CAM_STILL, fn, len(data), 0, 0, ct[0], ct[1] )
        pkt_bin += data
        return pkt_bin

    def build_dump_radar_net_header(self, orig_nh, fn, ip, port): # self._netheader
        strt_fmt : string = "<9I"   ## Buf_num(dim), Magic, Payload_type, channel(fn), data_size, width, height, pitch1, pitch2
        ct = get_cur_timestamp()
        netheader = struct.pack(strt_fmt, orig_nh[SR4PKT.IDX_NET_BUFNUM], self._net_magic_num, orig_nh[SR4PKT.IDX_NET_PTYPE], \
                    fn, orig_nh[SR4PKT.IDX_NET_DATA_SIZE], \
                    ip, port, ct[0], ct[1])
        return netheader

    def get_bytes_timestamp(self, tms:int, tmns:int):
        return tms.to_bytes(4, byteorder='little') + tmns.to_bytes(4, byteorder='little')

    def parse_netheader(self, data:bytes) -> int:
        strt_fmt : string = "<9I"   ## Buf_num(dim), Magic, Payload_type, channel(fn), data_size, width(r), height(r), pitch1(r), pitch2(r)
        hdr_size : int = struct.calcsize(strt_fmt)
        self._netheader = struct.unpack(strt_fmt, data[:hdr_size])
        return hdr_size  # used size

    def parse_frameheader(self, data:bytes) -> int:
        strt_fmt : string = '<Q6I16s2I15Q' ## Magic, protocol, head_size, data_size, antenna, sbl_ver, firm_ver, serial, reserved
        hdr_size : int = struct.calcsize(strt_fmt)
        self._frameheader = struct.unpack(strt_fmt, data[:hdr_size])
        return hdr_size  # used size

    def parse_payloadheader(self, data:bytes) -> int:
        strt_fmt : string = '<IQ3I16s'  ## fn, timestamp, tgt_info_type, tgt_num,  payload_head_size, reserved
        hdr_size : int = struct.calcsize(strt_fmt)
        self._payloadheader = struct.unpack(strt_fmt, data[:hdr_size])
        return hdr_size  # used size

    def parse_targetinfo_pc(self, data:bytes, count:int) -> list:
        off : int = 0
        pc_item_struct_fmt : str = '<5f'
        pc_item_step = struct.calcsize(pc_item_struct_fmt)
        point_clouds = np.zeros((count,5))
        for i in range(0, count) :
            point_clouds[i] = struct.unpack(pc_item_struct_fmt, data[off:off+pc_item_step])
            off += pc_item_step
        return off, point_clouds

    def parse_subheader(self, data:bytes) -> list:
        # pc_index is byte array..
        off : int = 0
        subhd_struct_fmt : str = '<3I'
        subhd_size = struct.calcsize(subhd_struct_fmt)
        subhd = struct.unpack(subhd_struct_fmt, data[off:off+subhd_size])
        return subhd

    def parse_targetinfo_pc_indices8(self, subhd, data:bytes):
        # pc_index is byte array..
        off : int = 0
        item_strt_fmt : str = f'<{subhd[1]//2}B'
        item_strt_size = struct.calcsize(item_strt_fmt)
        pci_arr = struct.unpack(item_strt_fmt, data[off:off+item_strt_size])
        return pci_arr

    def parse_targetinfo_pc_indices16(self, subhd, data:bytes):
        # pc_index is byte array..
        off : int = 0
        item_strt_fmt : str = f'<{subhd[1]//2}H'
        item_strt_size = struct.calcsize(item_strt_fmt)
        pci_arr = struct.unpack(item_strt_fmt, data[off:off+item_strt_size])
        return pci_arr

    def parse_targetinfo_tgt_srtrk(self, subhd, data:bytes):
        off : int = 0
        #pc_item_struct_fmt : str = '<BI6f9ff3f3ff' # uid, tid, S[6], EC[9], G, dim[3], uCenter[3], confidential
        item_struct_fmt : str = '3iIifh'
        item_step = struct.calcsize(item_struct_fmt)
        if subhd[1]//item_step != subhd[1]/item_step :
            print(f"..parse_targetinfo_tgt_srtrk item not match? {subhd[1]//item_step} != {subhd[1]/item_step}")
        item_count = subhd[1]//item_step
        tgt_arr = np.zeros((item_count,7))
        for i in range(0, item_count) :
            items = struct.unpack(item_struct_fmt, data[off:off+item_step])
            tgt_arr[i][0] = items[0] #x
            tgt_arr[i][1] = items[1] #y
            tgt_arr[i][2] = items[2] #z
            tgt_arr[i][3] = items[3] #range
            tgt_arr[i][4] = items[4] #doppler
            tgt_arr[i][5] = items[5] #power
            tgt_arr[i][6] = items[6] #ati
            off += item_step
        return tgt_arr

    def parse_targetinfo_tgt_titrk(self, subhd, data:bytes):
        off : int = 0
        #pc_item_struct_fmt : str = '<BI6f9ff3f3ff' # uid, tid, S[6], EC[9], G, dim[3], uCenter[3], confidential
        item_struct_fmt : str = 'BI23f'
        item_step = struct.calcsize(item_struct_fmt)
        if subhd[1]//item_step != subhd[1]/item_step :
            print(f"..parse_targetinfo_tgt_titrk item not match? {subhd[1]//item_step} != {subhd[1]/item_step}")
        item_count = subhd[1]//item_step
        tgt_arr = np.zeros((item_count,9))
        for i in range(0, item_count) :
            items = struct.unpack(item_struct_fmt, data[off:off+item_step])
            tgt_arr[i][0] = int(items[0]) #uid
            tgt_arr[i][1] = int(items[1]) #tid
            tgt_arr[i][2] = items[2] #posX
            tgt_arr[i][3] = items[3] #posY
            tgt_arr[i][4] = items[4] #velX
            tgt_arr[i][5] = items[5] #velY
            tgt_arr[i][6] = items[6] #accX
            tgt_arr[i][7] = items[7] #accY
            tgt_arr[i][8] = items[17] #G
            off += item_step
        return tgt_arr

    def print_header(self, nethd, frmhd, plhd) :
        if nethd != None or frmhd != None or plhd != None :
            ldprint("="*30)
        if nethd != None :
            ldprint(f"    Net: bn={nethd[0]} magic={hex(nethd[SR4PKT.IDX_NET_MAGIC])} ptype={nethd[2]} ch={nethd[3]} size={nethd[SR4PKT.IDX_NET_DATA_SIZE]} whpp={nethd[5]}, {nethd[6]}, {nethd[7]}{nethd[8]} ")
        if frmhd != None :
            ldprint(f"    FRAME: magic={hex(frmhd[0])} protocol={hex(frmhd[1])} hdsize={frmhd[2]} plsize={frmhd[3]} ant={hex(frmhd[4])} ver={hex(frmhd[5])},{hex(frmhd[6])}  serial={frmhd[7]}")
        if plhd != None :
            ldprint(f"    PAYLOAD: FN:{plhd[0]} timestamp:{plhd[1]} tgtinfo_type:{plhd[2]} tgt_maxnum:{plhd[3]} ")

    def process_file(self, file:BufferedReader ):
        self._dev = file
        off : int = 0
        self._bytes = b''
        self._netheader = self._frameheader = self._payloadheader = None

        data = self._dev.read(SR4PKT.NET_HDSIZE)
        if data == None or len(data) < SR4PKT.NET_HDSIZE:
            self._err_count += 1
            if self._err_count > 100 :
                self._err_code = SR4ERR.DT_READ_ERR
                raise ValueError('bin file read failed..!')
            return None

        self._err_count = 0  # clear...
        self._bytes = data
        off = self.parse_netheader(self._bytes)
        if self._netheader[SR4PKT.IDX_NET_MAGIC] != self._net_magic_num :
            print(f"-->ERR.NetHeader Magic {hex(self._netheader[SR4PKT.IDX_NET_MAGIC])} ")
            self._bytes = None
            return None
        #off = SR4PKT.NET_HDSIZE
        self._net_data_length = self._netheader[SR4PKT.IDX_NET_DATA_SIZE]

        data = self._dev.read(self._net_data_length)
        if data == None or len(data) != self._net_data_length:
            print(f"-->ERR.Frame Read {len(data) if data != None else 'None'}")
            return None
        self._bytes += data

        ## for camera data
        if self._netheader[SR4PKT.IDX_NET_PTYPE] == SR4AddData.CAM_STILL :
            ## Buf_num(dim), Magic, Payload_type, channel(fn), data_size, width(r), height(r), pitch1(r), pitch2(r)
            data = self._bytes[off:]
            chunk_idx = self._netheader[SR4PKT.IDX_NET_FRAMENUM]
            ct = [0,0]
            ct[0] = self._netheader[SR4PKT.IDX_NET_TMSTMP1]
            ct[1] = self._netheader[SR4PKT.IDX_NET_TMSTMP2]
            ldprint(f".. consume by cam {off+self._net_data_length}.")
            addr = int2ip(self._netheader[SR4PKT.IDX_NET_IP]) + ":" + str(self._netheader[SR4PKT.IDX_NET_PORT])
            self._bytes = None # next time, do by netheader..
            message = {'chunk_idx':chunk_idx, 'app_type':self._app_type, 'devid':addr, 'topic':'/image_raw/compressed' \
                    , 'time':ct, 'nethd':self._netheader, 'payload':data \
                    , 'parsed':['/image_raw/compressed', chunk_idx, ct[0], ct[1], data]}
            return message
            #return (self._netheader[SR4PKT.IDX_NET_FRAMENUM], w, h, data)

        #else... radar
        off += self.parse_frameheader(self._bytes[off:])
        if self._frameheader[SR4PKT.IDX_FHD_MAGIC] != self._magics_num :
            print(f"-->ERR.FrameHeader Magic {hex(self._frameheader[SR4PKT.IDX_FHD_MAGIC])} ")
            self._bytes = None
            return None

        payload_hdsize = self.parse_payloadheader(self._bytes[off:])
        off += payload_hdsize

        ### 
        self._frame_number = self._payloadheader[SR4PKT.IDX_PLHD_FRAME_NUM]
        self._payloaddata = self._bytes[off:]
        used_size, self._points = self.parse_targetinfo_pc(self._bytes[off:], self._payloadheader[SR4PKT.IDX_PLHD_TGT_NUM])
        self._targets = None
        self._pc_indices = None
        tracker_type = 0
        if used_size != len(self._payloaddata) :
            #print(f"-- F:{self._frame_number} has addition sub data size {len(self._payloaddata) - used_size}")
            while( (len(self._payloaddata) - used_size) >= 12):
                subhd = self.parse_subheader(self._bytes[off+used_size:])
                used_size += 12
                if (subhd[1] == 0) or (subhd[0] == 0x00) :
                    continue
                elif subhd[0] == 0x10 : ## point_cloud_indices
                    self._subdata_on = True
                    self._pc_indices = self.parse_targetinfo_pc_indices8(subhd, self._bytes[off+used_size:])
                    #print(f" FN:{self._frame_number} - PC_indices={self._pc_indices}")
                elif subhd[0] == 0x11 : ## point_cloud_indices
                    self._subdata_on = True
                    self._pc_indices = self.parse_targetinfo_pc_indices16(subhd, self._bytes[off+used_size:])
                    #print(f" FN:{self._frame_number} - PC_indices={self._pc_indices}")
                elif subhd[0] == 0x20 : ## target info
                    self._subdata_on = True
                    tracker_type = 0x20
                    self._targets = self.parse_targetinfo_tgt_srtrk(subhd, self._bytes[off+used_size:])
                    #print(f" FN:{self._frame_number} - targets={self._targets}")
                elif subhd[0] == 0x21 : ## target info
                    self._subdata_on = True
                    tracker_type = 0x21
                    self._targets = self.parse_targetinfo_tgt_titrk(subhd, self._bytes[off+used_size:])
                    #print(f" FN:{self._frame_number} - targets={self._targets}")
                else :
                    print(f" FN:{self._frame_number} unknown.sub:{subhd[0]} ..{self._bytes[off+used_size:off+used_size+subhd[1]:]} ")
                used_size += subhd[1]

        chunk_idx = self._frame_number
        ct = [0,0]
        ct[0] = self._netheader[SR4PKT.IDX_NET_TMSTMP1]
        ct[1] = self._netheader[SR4PKT.IDX_NET_TMSTMP2]
        addr = int2ip(self._netheader[SR4PKT.IDX_NET_IP]) + ":" + str(self._netheader[SR4PKT.IDX_NET_PORT])
        message = {'chunk_idx':chunk_idx, 'app_type':self._app_type, 'devid':addr, 'topic':'/point_cloud' \
                , 'time':ct, 'nethd':self._netheader, 'frmhd':self._frameheader, 'plhd':self._payloadheader, 'payload':self._payloaddata \
                , 'parsed':['/point_cloud', self._frame_number, ct[0], ct[1], self._points]}

        if self._subdata_on :
            message['sub'] = dict()
            if self._pc_indices != None :
                message['sub']['pc_indices'] = self._pc_indices
            if self._targets is not None :
                message['sub']['tracker_type'] = tracker_type
                message['sub']['targets'] = self._targets

        off += self._frameheader[SR4PKT.IDX_FHD_PAYLOAD_SIZE] - payload_hdsize
        if False :
            self.print_header(self._netheader, self._frameheader, self._payloadheader)
            #ldprint(f"    FrameSkip={self._frame_number-self._prev_frame_number} all_packet_size={len(self._bytes)}, cur_off={off}, remain={len(self._bytes)-off}")
        if (self._prev_frame_number!=0) and (self._frame_number-self._prev_frame_number != 1) :
            print(f"WARNING.FRAMENUM.MISSMATCH,  new={self._frame_number} prev:{self._prev_frame_number} diff={self._frame_number-self._prev_frame_number}")
        self._bytes = None # next time, do by netheader..
        self._prev_frame_number = self._frame_number
        return message

    def process_sock(self, sock: socket.socket):
        self._dev = sock
        off : int = 0
        pos : int = 0
        req_size : int = 0
        self._netheader = self._frameheader = self._payloadheader = None
        ## -----------------------------
        ## internal only function
        def read_sock(sock:socket.socket, size:int, retry_max:int) -> bytes :
            retry_count : int = 0
            data : bytes = bytes()
            if size <= 0 :
                print(f"--req_size:{size}  < 0 ")
                return b''
            while retry_count < retry_max :
                try :
                    rxdata = sock.recv(size - len(data))
                    with open(f'rx_dumpper.dat', 'a') as file:
                        file.write( f"\nREQ:{size}, len:{len(data)}, retry:{retry_count}\n")
                        file.write(rxdata.hex())
                    data += rxdata
                    if len(data) < size :
                        retry_count += 1
                        continue
                    break
                except socket.timeout :
                    retry_count += 1
                    continue
                except socket.error as e :
                    print(f"SockExecption:{e}")
                    return b''
                except InterruptedError as e :
                    print(f"InterruptExecption:{e}")
                    raise
            # if len(data) > 0 and self._dumper != None:
            #     self._dumper.write(data)
            return data
            # retry_count : int = 0
            # data : bytes = bytes()
            # if size <= 0 :
            #     print(f"--req_size:{size}  < 0 ")
            #     return b''
            # while retry_count < retry_max :
            #     try :
            #         data += sock.recv(size - len(data))
            #         if len(data) < size :
            #             retry_count += 1
            #             continue
            #         break
            #     except socket.timeout :
            #         retry_count += 1
            #         continue
            #     except InterruptedError as e :
            #         raise
            #     except socket.error as e :
            #         print(f"Execption:{e}")
            #         return b''
            # # if len(data) > 0 and self._dumper != None:
            # #     self._dumper.write(data)
            # return data
        ## -----------------------------

        while self._err_count < self._err_max :
            pos = -1
            if self._bytes == None : 
                req_size = SR4PKT.NET_HDSIZE 
                self._bytes = read_sock(sock, req_size, 3)
            elif len(self._bytes) < SR4PKT.NET_HDSIZE:
                req_size =  SR4PKT.NET_HDSIZE - len(self._bytes)
                self._bytes += read_sock(sock, req_size, 3)
            else :
                req_size = 1024*50
                self._bytes += read_sock(sock, req_size, 3)
            if self._bytes != None and len(self._bytes) >= SR4PKT.NET_HDSIZE :
                pos = self._bytes.find(self._net_magic_bin)
                if pos >= 0 :   # found net_magic_bin..
                    self._bytes = self._bytes[pos:]
                    off = self.parse_netheader(self._bytes)
                    if self._netheader[SR4PKT.IDX_NET_MAGIC] == self._net_magic_num :
                        self._net_data_length = self._netheader[SR4PKT.IDX_NET_DATA_SIZE]
                        req_size = self._net_data_length + SR4PKT.NET_HDSIZE - len(self._bytes)  # off
                        req_size = req_size if req_size > 0 else 0
                        break   ## okay.. normal oparation.. 
            print(f"-->ERR.NetHeader Magic required:{req_size} size:{len(self._bytes)} pos:{pos} magic:{hex(self._netheader[SR4PKT.IDX_NET_MAGIC]) if self._netheader != None else 'None' }")
            self._err_count += 1

        if self._err_count >= self._err_max :
            self._bytes = b'' # clear all
            self._err_code = SR4ERR.DT_RCV_ERR
            raise ValueError('Radar Data port recv failed. C3 ..!')
    
        self._err_count = 0
        if req_size > 0 :
            data = read_sock(sock, req_size, 10)
            if data == None or len(data) != req_size:
                print(f"-->ERR.Frame Read {len(data) if data != None else 'None'}, requred:{req_size}")
                self._bytes = b'' # clear all
                return None
            self._bytes += data

        off += self.parse_frameheader(self._bytes[off:])
        if self._frameheader[SR4PKT.IDX_FHD_MAGIC] != self._magics_num :
            if self._frameheader[SR4PKT.IDX_FHD_MAGIC] != self._dummy_magic_num :
                print(f"-->ERR.FrameHeader Magic {hex(self._frameheader[SR4PKT.IDX_FHD_MAGIC])}, skip {len(self._bytes)} ")
            self._bytes = b'' # clear all
            return None

        payload_hdsize = self.parse_payloadheader(self._bytes[off:])
        off += payload_hdsize

        ### 
        self._frame_number = self._payloadheader[SR4PKT.IDX_PLHD_FRAME_NUM]
        self._payloaddata = self._bytes[off:]
        used_size, self._points = self.parse_targetinfo_pc(self._bytes[off:], self._payloadheader[SR4PKT.IDX_PLHD_TGT_NUM])
        chunk_idx = self._frame_number
        ct = get_cur_timestamp()
        self._targets = None
        self._pc_indices = None
        tracker_type = 0
        if used_size != len(self._payloaddata) :
            #print(f"-- F:{self._frame_number} has addition sub data size {len(self._payloaddata) - used_size}")
            while( (len(self._payloaddata) - used_size) >= 12):
                subhd = self.parse_subheader(self._bytes[off+used_size:])
                used_size += 12
                if (subhd[1] == 0) or (subhd[0] == 0x00) :
                    if True :
                        print(f"Optional Break Out, subheader is none, guess maybe fixed size..")
                        break
                    continue
                elif subhd[0] == 0x10 : ## point_cloud_indices
                    self._subdata_on = True
                    self._pc_indices = self.parse_targetinfo_pc_indices8(subhd, self._bytes[off+used_size:])
                    print(f" FN:{self._frame_number} - PC_indices8={self._pc_indices}")
                elif subhd[0] == 0x11 : ## point_cloud_indices
                    self._subdata_on = True
                    self._pc_indices = self.parse_targetinfo_pc_indices16(subhd, self._bytes[off+used_size:])
                    print(f" FN:{self._frame_number} - PC_indices16={self._pc_indices}")
                elif subhd[0] == 0x20 : ## target info
                    self._subdata_on = True
                    tracker_type = 0x20
                    self._targets = self.parse_targetinfo_tgt_srtrk(subhd, self._bytes[off+used_size:])
                    print(f" FN:{self._frame_number} -srtrk targets={self._targets}")
                elif subhd[0] == 0x21 : ## target info
                    self._subdata_on = True
                    tracker_type = 0x21
                    self._targets = self.parse_targetinfo_tgt_titrk(subhd, self._bytes[off+used_size:])
                    print(f" FN:{self._frame_number} -titrk targets={self._targets}")
                else :
                    print(f"Break.do FN:{self._frame_number} unknown.sub:{subhd[0]} ..{self._bytes[off+used_size:off+used_size+subhd[1]:]} ")
                    if True :
                        print(f"Optional Break, subheader is invalid, guess maybe fixed size..")
                        break
                used_size += subhd[1]

        message = {'chunk_idx':chunk_idx, 'app_type':self._app_type, 'devid':'x.x.x.x:x', 'topic':'/point_cloud' \
                , 'time':ct, 'nethd':self._netheader, 'frmhd':self._frameheader, 'plhd':self._payloadheader, 'payload':self._payloaddata \
                , 'parsed':['/point_cloud', self._frame_number, ct[0], ct[1], self._points]}

        if self._subdata_on :
            message['sub'] = dict()
            if self._pc_indices != None :
                message['sub']['pc_indices'] = self._pc_indices
            if self._targets is not None :
                message['sub']['tracker_type'] = tracker_type
                message['sub']['targets'] = self._targets

        off += self._frameheader[SR4PKT.IDX_FHD_PAYLOAD_SIZE] - payload_hdsize
        if False :
            self.print_header(self._netheader, self._frameheader, self._payloadheader)
            #ldprint(f"    FrameSkip={self._frame_number-self._prev_frame_number} all_packet_size={len(self._bytes)}, cur_off={off}, remain={len(self._bytes)-off}")
        if (self._prev_frame_number!=0) and (self._frame_number-self._prev_frame_number != 1) :
            print(f"WARNING.FRAMENUM.MISSMATCH,  new={self._frame_number} prev:{self._prev_frame_number} diff={self._frame_number-self._prev_frame_number}")
        if len(self._bytes) > 0 and self._dumper != None:
            ## replace to timestamp from net_header.pitch1,2
            # dumphdr = self._bytes[:SR4PKT.OFFSET_NET_TMSTMP]
            # dumphdr += self.get_bytes_timestamp(ct[0], ct[1])
            dumpnh = self.build_dump_radar_net_header(self._netheader, chunk_idx, self._addrinfo[0], self._addrinfo[1]) # self._netheader
            dumpnh += self._bytes[SR4PKT.NET_HDSIZE:]
            self._dumper.write(dumpnh)
            ## self._dumper.write(self._bytes[SR4PKT.NET_HDSIZE:])
        if len(self._bytes) > off :
            self._bytes = self._bytes[off:]
        else :
            self._bytes = None # next time, do by netheader..
        self._prev_frame_number = self._frame_number
        return message


    def do_preload(self, dev):
        self._dev = dev
        frame_count  : int = 0
        frame_list : list  = []
        foffset : int = 0
        off : int = 0

        dev.seek(0, 2)
        file_length = dev.tell()
        dev.seek(0, 0)

        if self._app_type == SR4AppType.AT_R4FFILE :

            foffset = 0
            while foffset < file_length :
                data = self._dev.read(SR4PKT.NET_HDSIZE)
                if data == None or len(data) < SR4PKT.NET_HDSIZE:
                    break
                off = self.parse_netheader(data)
                if self._netheader[SR4PKT.IDX_NET_MAGIC] != self._net_magic_num :
                    print(f"no magic, current file offset is {foffset}")
                    break
                
                if self._netheader[SR4PKT.IDX_NET_PTYPE] == SR4AddData.RADAR :
                    frame_list.append((self._netheader[SR4PKT.IDX_NET_FRAMENUM], (self._netheader[SR4PKT.IDX_NET_TMSTMP1], self._netheader[SR4PKT.IDX_NET_TMSTMP2]), foffset))
                    frame_count += 1
                dev.seek(self._netheader[SR4PKT.IDX_NET_DATA_SIZE], 1)
                foffset += (off + self._netheader[SR4PKT.IDX_NET_DATA_SIZE])
            dev.seek(0, 0)
            return file_length, frame_list
        return -1, None


## -----------------------------
## -----------------------------
class sr4bagf_dev(sr4base_dev):
    _bagf : BagFileReader = None
    _filepath : str = None
    _topics : list = None
    _check_topics : list = None
    
    def __init__(self, pdict:dict, outque:mp.Queue):
        self._wtype = WRAPPER.BAGF
        self._bagf = None
        self._filepath = None
        self._topics = None
        self._check_topics = None
        self._run_count = 0
        self._dict = pdict
        self._outq : mp.Queue = outque

    def open(self, params:list, param_count:int):
        if self._bagf != None :
            self._bagf.close()
            del self._bagf
            self._bagf = None
        self._filepath = params[0]
        self._topics = str(params[1]).split(',')
        self._check_topics = params[2]
        self._bagf = BagFileReader()
        self._bagf.open(self._filepath)
        self._bagf.load(self._topics)
        if self._bagf.is_valid_bag() :
            print(f'player open {self._filepath} with topcis:{self._topics}')
            self._run_count = 0
            self._dict['errcode'] = SRPEvent.BAGFILE
            return False

        print(f'player failed open {self._filepath} with topcis:{self._topics}')
        self._dict['errcode'] = -SRPEvent.BAGFILE
        return True

    def close(self):
        print(f'player close {self._filepath}')
        if self._bagf != None :
            self._bagf.close()
            del self._bagf
            self._bagf = None
        self._run_count = 0
        self._dict['errcode'] = SRPEvent.CLOSE

    def oper_pause(self, params:list, param_count:int) :
        step : int = np.abs(params[0])
        # ... do step operation, next, prev
        if step == 0:
            print('player stopped')
        else:
            print(f'go { "next" if params[0] > 0 else "prev"} {step} step, count is {self._run_count}')
            #self._check_topics = self._dict['check_topics']
            try :
                while step > 0 :
                    if self._bagf != None and self._bagf.is_valid_bag() :
                        record_message = self._bagf.get_next_message() if params[0] > 0 else self._bagf.get_prev_message()
                        if record_message != None :
                            self._dict['errcode'] = SRPEvent.PAUSE
                            #print(f'player go - {record_message["chunk_idx"]}, {record_message["topic"]}')
                            self._outq.put(record_message)
                            self._run_count += 1
                        else:
                            self._outq.put(None)
                            self._dict['errcode'] = SRPErrCode.ERR_EOF
                            break
                    else:
                        break
                    if self._check_topics == None or record_message['topic'] in self._check_topics :
                        step -= 1
            except Exception as e:
                print(e)

    def oper_play(self, evtid:int, params:list, param_count:int, is_run, is_play_on, interval):
        self._dict['errcode'] = evtid
        while is_run.value:
            # stop if mode is changed
            if is_play_on.value != True : #evtid != self._dict['eventid'] :
                print(f'player mode chaned {evtid} -> ? { int(self._dict["eventid"])}')
                break

            ## waiting... que is ready state..
            if self._outq.qsize() > 5 :
                time.sleep(interval*10)
                continue
            #... 
            #print(f'play { "next" if l_evtid==1 else "prev"}, count is {_run_count}')
            try :
                if self._bagf.is_valid_bag() :
                    record_message = self._bagf.get_next_message() if evtid == SRPEvent.PLAY else self._bagf.get_prev_message()
                    if record_message != None :
                        #print(f'player play - {record_message["chunk_idx"]}, {record_message["topic"]}')
                        self._outq.put(record_message)
                        self._run_count += 1
                    else:
                        self._outq.put(None)
                        self._dict['eventid'] = evtid = SRPEvent.PAUSE
                        self._dict['errcode'] = SRPErrCode.ERR_EOF
                        break
            except Exception as e:
                print(e)
            time.sleep(interval)

    def oper_go(self, params:list, param_count:int):
        chunk_idx = params[0]
        try :
            if self._bagf.is_valid_bag() :
                self._bagf.go_chunk_record(chunk_idx)
                record_message = self._bagf.get_next_message()
                if record_message != None :
                    #print(f'player chunk - {record_message["chunk_idx"]}, {record_message["topic"]}')
                    self._outq.put(record_message)
                    self._run_count += 1
                else:
                    self._dict['errcode'] = SRPErrCode.ERR_EOF
                    self._outq.put(None)
        except Exception as e:
            print(e)
        #l_evtid = SRPEvent.PAUSE # change to pause mode
        #l_params[0] = 0

    def start(self, params:list, param_count:int):
        pass

    def stop(self):
        pass

    def send_console(self, cmds):
        pass


## -----------------------------
## -----------------------------
class sr4binf_dev(sr4base_dev):
    def __init__(self, pdict:dict, outque:mp.Queue):
        self._wtype = WRAPPER.BINF
        self._filepath = None
        self._file : BufferedReader = None
        self._file_length : int = -1
        self._run_count = 0
        self._dict = pdict
        self._outq : mp.Queue = outque
        self._parser : SR4DataParser = None
        self._frame_list : list = None
        self._frame_idx : int = 0

    def open(self, params:list, param_count:int):
        self._dev_type = params[0]
        if pathlib.Path(params[1]).is_file() :
            self._filepath = params[1]
            self._file = open(self._filepath, "rb")
            self._parser = SR4DataParser(self._dev_type, None)
            self._dict['errcode'] = SRPEvent.BINFILE
            self._file_length, self._frame_list = self._parser.do_preload(self._file)
            if self._frame_list is not None :
                print(f">> Bin File:[{self._filepath}] length:{self._file_length} frame_count:{len(self._frame_list)}")
                self._frame_idx = -1
                return True
            else :
                print(f">>ERR. Bin File:[{self._filepath}] length:{self._file_length}, pre_load failed.")
                self._file.close()
                self._file = None
        if self._file == None :
            self._dict['errcode'] = -SRPEvent.BINFILE
            return False

    def close(self):
        if self._file != None :
            self._file.close()
            self._file = None
            self._file_length = 0
            print(">> Bin File Close..")
        self._dict['errcode'] = SRPEvent.CLOSE

    def oper_pause(self, params:list, param_count:int) :
        step : int = np.abs(params[0])
        is_forwared : bool = (params[0] > 0)
        if step == 0:
            print('player stopped')
        else:
            print(f'go { "next" if params[0] > 0 else "prev"} {step} step, count is {self._run_count}')
            try :
                while step > 0 :
                    #if is_reverse :
                    n_idx = self._frame_idx + ( 1 if is_forwared else  -1 )
                    if n_idx < 0 or n_idx >= len(self._frame_list) :
                        print(f'WARNING. position is out range.. {n_idx}, current:{self._frame_idx}')
                        break

                    print(f'..... {n_idx}, prev:{self._frame_idx}, {params[0]}')
                    
                    self._frame_idx = n_idx
                    self._file.seek( self._frame_list[self._frame_idx][2], SEEK_SET )
                    resp = self._parser.process_file(self._file)
                    if resp != None :
                        if resp['topic'] == '/point_cloud' :
                            #if self._frame_list[self._frame_idx][1][0] != resp['time'] :
                            if self._frame_list[self._frame_idx][0] != resp['chunk_idx'] :
                                print(f"ERR.Frame Align mismatch. {self._frame_list[self._frame_idx][0]} != {resp['chunk_idx']}")
                        self._outq.put(resp)
                        self._run_count += 1
                        step -= 1
                    else:
                        self._outq.put(None)
                        self._dict['errcode'] = SRPErrCode.ERR_EOF
                        break
            except Exception as e:
                print(e)

    def oper_play(self, evtid:int, params:list, param_count:int, is_run, is_play_on, interval):
        self._dict['errcode'] = evtid
        while is_run.value:
            # stop if mode is changed
            if is_play_on.value != True : #evtid != self._dict['eventid'] :
                print(f'player mode chaned {evtid} -> ? { int(self._dict["eventid"])}')
                break

            ## waiting... que is ready state..
            if self._outq.qsize() > 5 :
                time.sleep(interval*10)
                continue

            try :
                resp = self._parser.process_file(self._file)
                if resp != None :
                    if resp['topic'] == '/point_cloud' :
                        if self._frame_list[self._frame_idx+1][0] != resp['chunk_idx'] :
                            print(f"ERR.Frame Align mismatch. {self._frame_list[self._frame_idx+1][0]} != {resp['chunk_idx']}")
                        self._frame_idx += 1
                    self._outq.put(resp)
                    self._run_count += 1
                else:
                    self._outq.put(None)
                    self._dict['eventid'] = evtid = SRPEvent.PAUSE
                    self._dict['errcode'] = SRPErrCode.ERR_EOF
                    break
            except Exception as e:
                print(e)
            time.sleep(interval)

    def oper_go(self, params:list, param_count:int):
        n_idx = params[0]
        try :
            print(f'..... {n_idx}, prev:{self._frame_idx}, {params[0]}')
            self._frame_idx = n_idx
            self._file.seek( self._frame_list[self._frame_idx][2], SEEK_SET )
            resp = self._parser.process_file(self._file)
            if resp != None :
                if resp['topic'] == '/point_cloud' :
                    #if self._frame_list[self._frame_idx][1][0] != resp['time'] :
                    if self._frame_list[self._frame_idx][0] != resp['chunk_idx'] :
                        print(f"ERR.Frame Align mismatch. {self._frame_list[self._frame_idx][0]} != {resp['chunk_idx']}")
                self._outq.put(resp)
                self._run_count += 1
            else:
                self._outq.put(None)
                self._dict['errcode'] = SRPErrCode.ERR_EOF
        except Exception as e:
            print(e)
    def start(self, params:list, param_count:int):
        pass
    def stop(self):
        pass
    def send_console(self, cmds):
        pass

## -----------------------------
class sr4socket_dev(sr4base_dev) :
    pass

## -----------------------------
def receive_socket_cs(upper:sr4socket_dev) :
    # nonlocal upper._dict
    # nonlocal l_sock_run
    if vcdbg_on : debugpy.debug_this_thread()
    upper._dict['cs_err_code'] = 0
    print(">>Enter Console Thread--")
    while upper._sock_run :
        try :
            msgs = upper._cs_sock.recv(1024)
        except socket.timeout :
        # print("CS Timeout : ")
            continue
        except ConnectionResetError as e :
            print(">> CS SOCK Connection Reset ERROR : " + e)
            upper._dict['cs_err_code'] = SR4ERR.DT_SOCK_ERR
            break
        except InterruptedError as e :
            print(">> CS SOCK ERROR : " + e)
            upper._dict['cs_err_code'] = SR4ERR.DT_SOCK_ERR
            break
        if msgs != None and len(msgs) > 0 :
            #pass
            upper._csq.put(msgs)
            #print(f">>Console={len(msgs)}[{str(msgs)}]")
    print(">>Exit Console Thread--")

## -----------------------------
def receive_socket_data(upper:sr4socket_dev):
    if vcdbg_on : debugpy.debug_this_thread()
    upper._dict['dt_err_code'] = 0
    print(">>Enter Data Thread--")
    while upper._sock_run :
        try:
            res = upper._parser.process_sock(upper._dt_sock)
            if res != None :
                upper._outq.put(res)
        except InterruptedError as e :
            print(">> DATA SOCK ERROR : " + e)
            upper._dict['dt_err_code'] = SR4ERR.DT_SOCK_ERR
            break
        except ValueError as e :
            print( e )
            if upper._parser._err_code < 0 :
                upper._dict['dt_err_code'] = upper._parser._err_code
                print( ">> Exception from recv data socket..")
                break
    print(">>Exit Data Thread--")

# ## -----------------------------
# def receive_inq_data(upper:sr4socket_dev):
#     if vcdbg_on : debugpy.debug_this_thread()
#     print(">>Enter In Data Thread--")
#     while upper._sock_run :
#         try :
#             item = upper._inq.get(timeout=0.5)
#         except queue.Empty:
#             continue
#         except Exception as ie:
#             print(ie)
#         #print(f"InQ Data is {item}")
#         if item[0] == SR4AddData.CAM_STILL :
#             sitem = upper._parser.build_dump_cam_data(item[1], item[2])
#             if sitem != None :
#                 upper._dumper.write(sitem)

#     print(">>Exit In Data Thread--")

## -----------------------------
class sr4socket_dev(sr4base_dev):
    def __init__(self, pdict:dict, outque:mp.Queue, csque:mp.Queue, inque:mp.Queue):
        self._wtype = WRAPPER.SOCKET
        self._run_count = 0
        self._dict = pdict
        self._outq : mp.Queue = outque
        self._csq : mp.Queue = csque
        self._inq : mp.Queue = inque
        ##
        self._dev_type = None
        self._parser : SR4DataParser = None
        self._dumper : SRTDumper = None
        self._cs_port = 50000
        self._dt_port = 29172 #27000
        self._radar_ip = '192.168.137.15'
        self._cs_sock = None
        self._dt_sock = None
        self._cs_thread = None
        self._dt_thread = None
        self._in_thread = None
        self._sock_run = False

    def open(self, params:list, param_count:int):
        self._dev_type = params[0]
        self._radar_ip = params[1]
        self._cs_port = params[2]
        self._dt_port = params[3]

        self._dumper = SRTDumper()
        path = os.getcwd() + os.path.sep + f"bin_{SR4AppType.NAME[self._dev_type]}"
        if pathlib.Path(path).is_dir() == False :
            os.makedirs(path)
        filepath = path + os.path.sep + SR4AppType.NAME[self._dev_type] + datetime.datetime.now().strftime("_%Y%m%d_%H%M%S.bin")
        self._parser = SR4DataParser(self._dev_type, self._dumper)
        self._parser.set_addr_info( ip2int(self._radar_ip), self._dt_port)
        self._dumper.open(filepath, self._inq, self._parser)
        try:
            self._cs_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) ## socket.IPPROTO_TCP
            self._dt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._cs_sock.settimeout(5)
            self._dt_sock.settimeout(5)
            self._cs_sock.connect((self._radar_ip, self._cs_port))
            self._dt_sock.connect((self._radar_ip, self._dt_port))
            self._sock_run = True
            self._cs_sock.settimeout(1)
            self._dt_sock.settimeout(1)
            if self._csq != None :
                self._cs_thread = threading.Thread(target=receive_socket_cs, args=(self,))
                self._cs_thread.start()
            self._dt_thread = threading.Thread(target=receive_socket_data, args=(self,))
            self._dt_thread.start()
            # if self._inq != None :
            #     self._in_thread = threading.Thread(target=receive_inq_data, args=(self,))
            #     self._in_thread.start()
            print(">>Socket Opened..")
            return True
        except Exception as e :
            self.close()
            print(f"SOCKET ERROR:{e}")
            self._dict['errcode'] = - SRPEvent.SOCK
            return False

    def close(self):
        self._sock_run = False
        if self._cs_thread != None : self._cs_thread.join(); self._cs_thread = None
        if self._dt_thread != None : self._dt_thread.join(); self._dt_thread = None
        if self._in_thread != None : self._in_thread.join(); self._in_thread = None
        if self._dumper != None : self._dumper.close(); self._dumper = None;    self._parser._dumper = None

        if self._cs_sock != None:
            self._cs_sock.close()
            self._cs_sock = None
        if self._dt_sock != None :
            self._dt_sock.close()
            self._dt_sock = None
        self._dict['errcode'] = SRPEvent.CLOSE
        print(">>Socket Close Completed..")
        
    def oper_pause(self, params:list, param_count:int) :
        pass
    def oper_play(self, evtid:int, params:list, param_count:int, is_run, is_play_on, interval):
        pass
    def oper_go(self, params:list, param_count:int):
        pass
    def start(self, params:list, param_count:int):
        pass
    def stop(self):
        pass
    def send_console(self, cmds):
        if type(cmds).__name__ == 'str' :
            a = cmds.encode("ascii")
            self._cs_sock.send(a)
        elif type(cmds).__name__ == 'bytes' :
            self._cs_sock.send(cmds)
        elif type(cmds).__name__ == 'list' :
            for li in cmds :
                if type(li).__name__ == 'str' :
                    self._cs_sock.send(li.encode("ascii"))
                elif type(li).__name__ == 'bytes' :
                    self._cs_sock.send(li)
                

## -----------------------------
## -----------------------------
# ----------------------------------
#   4Fx data process main routine
# ----------------------------------
def ProcessRecvParsing(_dict, _event, _cs_q, _outq, _inq, _is_run, _is_play_on):
    ##  _bagf:BagFileReader, _play_mode:(1:normal,2:reverse), _outq:Queue, is_run(mp.Value)
    if vcdbg_on : debugpy.debug_this_thread()
    _run_count = 0
    l_parser = None
    l_evtid = _dict['eventid']
    l_params = [None,None,None,None,None,None]
    l_param_count = 0
    l_interval = _dict['interval']
    l_dev = None

    ## -----------------------------
    _dict['errcode'] = SRPErrCode.ERR_NOT_RUN
    print(f'>>Start Controller.Process')
    try :
        while _is_run.value:
            if _event.wait(l_interval) != True :
                continue

            _event.clear()
            l_evtid = _dict['eventid'] # _play_mode.value
            l_params = [ _dict['param1'], _dict['param2'], _dict['param3'], _dict['param4'], _dict['param5'], _dict['param6'] ]
            l_param_count = _dict['param_count']
            l_interval = _dict['interval'] #_play_interval.value
            # step = 0:None  +num : formward, -num:reverse.

            ## wrapper common call
            if l_evtid == SRPEvent.PAUSE :  # pause
                if l_dev != None :
                    l_dev.oper_pause(l_params, l_param_count)

            elif l_evtid == SRPEvent.PLAY or l_evtid==SRPEvent.RPLAY : # play or reverse play
                if l_dev != None :
                    l_dev.oper_play(l_evtid, l_params, l_param_count, _is_run, _is_play_on, l_interval)
                    # evtid:int, params:list, param_count:int, is_run, interval

            elif l_evtid == SRPEvent.GO : ## go to chunk
                if l_dev != None :
                    l_dev.oper_go(l_params, l_param_count)
                l_evtid = SRPEvent.PAUSE # change to pause mode
                l_params[0] = 0
            elif l_evtid == SRPEvent.CLOSE : # file close
                if l_dev != None :
                    l_dev.close()
                continue

            ## wrapper specific call
            elif l_evtid == SRPEvent.BAGFILE : # file open
                if l_dev != None :
                    l_dev.close()
                    del l_dev
                l_dev = sr4bagf_dev(_dict, _outq)
                l_dev.open(l_params, l_param_count)
                continue
            elif l_evtid == SRPEvent.SOCK :
                if l_dev != None :
                    l_dev.close()
                    del l_dev
                l_dev = sr4socket_dev(_dict, _outq, _cs_q, _inq)
                l_dev.open(l_params, l_param_count )
                continue
            elif l_evtid == SRPEvent.BINFILE :
                if l_dev != None :
                    l_dev.close()
                    del l_dev
                l_dev = sr4binf_dev(_dict, _outq)
                l_dev.open(l_params, l_param_count)
                if l_dev._frame_list is not None :
                    _dict['frame_list'] = l_dev._frame_list
            elif l_evtid == SRPEvent.CS_CMD :
                #l_cs_cmdlist.append(l_params[0])
                if l_dev._wtype == WRAPPER.SOCKET :
                    l_dev.send_console(l_params[0])
                pass
            else :
                _dict['errcode'] = -1
    except Exception as e:
        print(e)
    del _dict
    del _event
    print(f'controller exit with {_run_count}')
    if l_dev != None : l_dev.close()
    print(f'>>Exit Controller.Process')

## -----------------------------
## -----------------------------
# ----------------------------------
class SRIOWrapper():
    _dict = None
    ##  _bagf:BagFileReader, _play_mode:(1:normal,2:reverse), _outq:Queue, is_run(mp.Value)
    _is_run = mp.RawValue(ctypes.c_bool, False)
    _is_play_on = mp.RawValue(ctypes.c_bool, False)
    _play_process = None
    _event = None
    _out_que = None
    _cs_que = None
    _in_que = None
    #_app_type = SR4AppType.AT_UNKNOWN
    # ----------------------------------
    def __init__(self, manager, outq=None, csq=None, inq=None):
        self._cs_que = csq
        self._out_que = outq
        self._in_que = inq
        self._dict = manager.dict()
        self._dict['eventid'] = SRPEvent.PAUSE
        self._dict['param1'] = 0
        self._dict['param2'] = 0
        self._dict['param3'] = 0
        self._dict['param4'] = 0
        self._dict['param5'] = 0
        self._dict['param6'] = 0
        self._dict['param_count'] = 0
        self._dict['interval'] = 0.001
        self._dict['cs_err_code'] = 0
        self._dict['dt_err_code'] = 0
        self._event = mp.Event()

    def open_bagfile(self, bagf, check_list):
        self._dict['param1'] = bagf._filepath
        self._dict['param2'] = ','.join(bagf._topic_list)
        self._dict['param3'] = list(np.array(bagf._topic_list)[np.array(check_list)])
        self._dict['param_count'] = 3
        self._dict['eventid'] = SRPEvent.BAGFILE
        self._event.set()

    def open_socket(self, ptype, ip_addr, cs_port, dt_port):
        self._dict['param1'] = ptype
        self._dict['param2'] = ip_addr
        self._dict['param3'] = cs_port
        self._dict['param4'] = dt_port
        self._dict['param_count'] = 4
        self._dict['eventid'] = SRPEvent.SOCK
        self._event.set()

    def open_binfile(self, ptype, binfilepath):
        self._dict['param1'] = ptype
        self._dict['param2'] = binfilepath
        self._dict['param_count'] = 2
        self._dict['eventid'] = SRPEvent.BINFILE
        self._event.set()

    def close(self) :
        self._dict['param_count'] = 0
        self._dict['eventid'] = SRPEvent.CLOSE
        self._event.set()


    def prev(self, step=-1):
        print(f'>>Go Prev to {step}')
        self._dict['eventid'] = SRPEvent.PAUSE
        self._dict['param1'] = step
        self._dict['param_count'] = 1
        self._event.set()

    def next(self, step=1):
        print(f'>>Go Next to {step}')
        self._dict['eventid'] = SRPEvent.PAUSE
        self._dict['param1'] = step
        self._dict['param_count'] = 1
        self._event.set()

    def pause(self):
        print('>>Pause Player')
        self._dict['eventid'] = SRPEvent.PAUSE
        self._dict['param1'] = 0
        self._dict['param_count'] = 1
        self._is_play_on.value = False
        self._event.set()

    def play(self):
        print('>>Play forward')
        self._dict['eventid'] = SRPEvent.PLAY
        self._dict['param_count'] = 0
        self._is_play_on.value = True
        self._event.set()

    def reverse_play(self):
        print('>>Play Reverse')
        self._dict['eventid'] = SRPEvent.RPLAY
        self._dict['param_count'] = 0
        self._event.set()

    def set_chunk_pos(self, pos):
        self._dict['eventid'] = SRPEvent.GO
        self._dict['param1'] = pos
        self._dict['param_count'] = 1
        self._event.set()

    def start(self):
        print('>>Start Controller')
        self._is_run.value = True
        #def ProcessPlay(_dict, _event, _outq, _is_run):
        self._play_process = mp.Process(target=ProcessRecvParsing, args=(self._dict, self._event, self._cs_que, self._out_que, self._in_que, self._is_run, self._is_play_on))
        self._play_process.start()

    def stop(self):
        if self._is_run.value == False :
            print('>>already stopped.')
            return
        print('>>Stop Controller')
        self.pause()
        time.sleep(0.5)
        self._is_run.value = False
        self._event.set()
        time.sleep(0.5)
        if self._play_process.join(1) == None :
            print(f'..exit code is {self._play_process.is_alive()}, exit:{self._play_process.exitcode} ')
            self._play_process.terminate()
        del self._play_process
        self._play_process = None

    def is_playing(self):
        return True if self._dict['eventid'] == SRPEvent.PLAY or self._dict['eventid'] == SRPEvent.RPLAY else False

    def send_cs_command(self, cmdlist):
        self._dict['eventid'] = SRPEvent.CS_CMD
        self._dict['param1'] = cmdlist
        self._dict['param_count'] = 1
        self._event.set()

    def get_err_code(self):
        return (self._dict['cs_err_code'], self._dict['dt_err_code'])

    def append_cam_data(self, fn:int, data:bytes):
        if self._in_que != None :
            self._in_que.put([SR4AddData.CAM_STILL, fn, data])

    def get_frame_list(self):
        if 'frame_list' in self._dict :
            return self._dict['frame_list']

# ----------------------------------
def test_bag_play():
    bagfile_path = "d:/2021-12-28-14-02-41.bag"
    outq = mp.Queue()
    manager = mp.Manager()
    bagf = BagFileReader()
    bagf.open(bagfile_path)
    bagf.load(('/usb_cam/image_raw/compressed', '/image_raw/compressed', '/point_cloud', '/velodyne_points'))
    count = 0
    if bagf.is_valid_bag() :
        player = SRIOWrapper(manager, outq, None)
        player.start()
        time.sleep(1)
        player.set_file(bagf, (True, True, True, True))
        time.sleep(1)
        if player._dict['errcode'] == 100 :
            while True:
                player.next()
                item = outq.get()
                print(f'Next out_queue is {item["chunk_idx"]}, {item["conn"]}, {item["time"]} ')
                if count  == 5 :
                    break
                time.sleep(0.01)
                count += 1

            player.pause()
            player.play()
            while count < 20:
                item = outq.get()
                count += 1
                print(f'PLay out_queue is {item["chunk_idx"]}, {item["conn"]}, {item["time"]}')

            player.pause()
            player.reverse_play()
            while count < 30:
                item = outq.get()
                count += 1
                print(f'Reverse Play out_queue is {item["chunk_idx"]}, {item["conn"]}, {item["time"]}')
                

        time.sleep(1)
        player.stop()
        time.sleep(1)

def test_sock_receiver():
    cs_port = 50000
    dt_port = 29172 # 27000
    sr4d_ip = "192.168.137.15"
    manager = mp.Manager()
    cs_que = mp.Queue()
    dt_que = mp.Queue()
    is_close = False

    sr4d = SRIOWrapper(manager, dt_que, cs_que)
    sr4d.start()
    ## set_socket(self, ptype, ip_addr, cs_port, dt_port)
    sr4d.open_socket( SR4AppType.AT_R4FDEV, sr4d_ip, cs_port, dt_port)
    print("*"*80)
    print("--- Start ---")
    print("*"*80)

    def receiver(is_close, cs_que, dt_que) :
        if vcdbg_on : debugpy.debug_this_thread()
        cs_count : int = 0
        dt_count : int = 0
        print(">>start cs/data receiver Thread--")
        while is_close == False:
            cs_err, dt_err = sr4d.get_err_code()
            if dt_err == SR4ERR.DT_RCV_ERR :
                print(f">>ERR.RCV.DT.FAILED..")
                break
            try :
                cs_data = cs_que.get(timeout=0.1)
                if cs_data != None :
                    print(f">>Console:{cs_count}={len(cs_data)}[{str(cs_data)}]")
                    cs_count += 1
            except queue.Empty :
                pass
            except Exception as e:
                print(e)
            
            try :
                ds_data = dt_que.get(timeout=0.1)
                if ds_data != None :
                    print(f">>Data:{dt_count}={len(ds_data)} [ FN:{ds_data[2][SR4PKT.IDX_PLHD_FRAME_NUM]}  ]")
                    dt_count += 1
            except queue.Empty :
                pass
            except Exception as e:
                print(e)
        print(">>Exit cs/data receiver Thread--")
        is_close = True

    l_receiver = threading.Thread(target=receiver, args=(is_close, cs_que, dt_que))
    l_receiver.start()
    while True:
        instr = input(">>")
        if instr == 'quit'  or is_close == True :
            is_close = True
            break
        elif instr == 'cmd' :
            sr4d.send_cs_command("test\r\n")
        else :
            print(f"Que={cs_que.qsize()},{dt_que.qsize()}")
        time.sleep(0.5)
    l_receiver.join()
    sr4d.stop()
    print("*"*80)


def test_binf():
    manager = mp.Manager()
    cs_que = mp.Queue()
    dt_que = mp.Queue()
    is_close = False

    sr4d = SRIOWrapper(manager, dt_que, cs_que)
    sr4d.start()
    ## set_socket(self, ptype, ip_addr, cs_port, dt_port)
    sr4d.open_binfile( SR4AppType.AT_R4FFILE, '')
    print("*"*80)
    print("--- Start ---")
    print("*"*80)

    def receiver(is_close, cs_que, dt_que) :
        if vcdbg_on : debugpy.debug_this_thread()
        cs_count : int = 0
        dt_count : int = 0
        print(">>start cs/data receiver Thread--")
        while is_close == False:
            cs_err, dt_err = sr4d.get_err_code()
            if dt_err == SR4ERR.DT_RCV_ERR :
                print(f">>ERR.RCV.DT.FAILED..")
                break
            try :
                cs_data = cs_que.get(timeout=0.1)
                if cs_data != None :
                    print(f">>Console:{cs_count}={len(cs_data)}[{str(cs_data)}]")
                    cs_count += 1
            except queue.Empty :
                pass
            except Exception as e:
                print(e)
            
            try :
                ds_data = dt_que.get(timeout=0.1)
                if ds_data != None :
                    print(f">>Data:{dt_count}={len(ds_data)} [ FN:{ds_data[2][SR4PKT.IDX_PLHD_FRAME_NUM]}  ]")
                    dt_count += 1
            except queue.Empty :
                pass
            except Exception as e:
                print(e)
        print(">>Exit cs/data receiver Thread--")
        is_close = True

    l_receiver = threading.Thread(target=receiver, args=(is_close, cs_que, dt_que))
    l_receiver.start()
    while True:
        instr = input(">>")
        if instr == 'quit'  or is_close == True :
            is_close = True
            break
        elif instr == 'cmd' :
            sr4d.send_cs_command("test\r\n")
        else :
            print(f"Que={cs_que.qsize()},{dt_que.qsize()}")
        time.sleep(0.5)
    l_receiver.join()
    sr4d.stop()
    print("*"*80)



if __name__ == '__main__':
    test_binf()
    sys.exit(0)
