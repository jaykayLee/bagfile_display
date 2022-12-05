import sys
import struct
import numpy as np
import datetime

#bagfile_path = "d:/w0/log/s0/2021-12-28-14-02-41.bag"
#bagfile_path = "d:/2021-12-28-14-02-41.bag"
bagfile_path = "d:/2021-06-21-15-26-39_json.bag"
LM_INFO = 1
LM_ERR = 2
LM_WARN = 4
LM_DBG = 8
#_LOG_MASK_ = (LM_INFO|LM_ERR|LM_WARN|LM_DBG)
_LOG_MASK_ = (LM_INFO|LM_ERR|LM_WARN)
# ----------------------------------
def Logger(maskbit, msgstr):
    if True:
        mask = _LOG_MASK_ & maskbit
        if mask == LM_INFO :
            print("[I]", msgstr)
        elif mask == LM_ERR :
            print("[E]", msgstr)
        elif mask == LM_WARN :
            print("[W]", msgstr)
        elif mask == LM_DBG :
            print("[D]", msgstr)
        elif mask != 0 :
            print("[U]", msgstr)
    else:
        print(msgstr)

# ----------------------------------
def str_bag_time(sec, nsec):
    dt = datetime.datetime.fromtimestamp(sec)
    return "" + dt.strftime('%Y/%m/%d %H:%M:%S.') + str(nsec)

# ----------------------------------
def BagRawMessage(topic, msgd):
    if topic == '/usb_cam/image_raw/compressed' or topic =='/image_raw/compressed' :
        #header.seq, time[0,1]
        head = struct.unpack('3I', msgd[0:12])
        off = 12
        # header.frame_id.len + header.frame_id.str
        off += 4 + int.from_bytes(msgd[off:off+4], byteorder='little')
        # format.len + format.str
        off += 4 + int.from_bytes(msgd[off:off+4], byteorder='little')
        data_length = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        return [topic, head[0], head[1], head[2], msgd[off:off+data_length]]
    elif topic == '/point_cloud' :
        #return get_bag_pointcloud_raw(msgd)
        head = struct.unpack('3I', msgd[0:12])
        off = 12
        off += 4 + int.from_bytes(msgd[off:off+4], byteorder='little')
        height = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        width = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        pfcount = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        for pi in range(0, pfcount):
            off += 4 + int.from_bytes(msgd[off:off+4], byteorder='little')
            off += 4 + 1 + 4
        off += 1
        point_step = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        row_step = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        data_length = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        data = msgd[off:off+data_length]
        off += data_length
        is_dense = msgd[off:off+1]

        off = 0 ## data parse offset
        pc_item_struct_fmt = '12f'
        # same as point_step
        pc_item_size = struct.calcsize(pc_item_struct_fmt)
        pc_count = data_length//point_step
        point_clouds = np.zeros((pc_count,12))
        for pi in range(0, pc_count):
            point_clouds[pi] = struct.unpack(pc_item_struct_fmt,data[off:off+point_step])
            off += point_step
        return [topic, head[0], head[1], head[2], point_clouds]
    elif topic == '/velodyne_points':
        # header.seq, time[2]
        head = struct.unpack('3I', msgd[0:12])
        off = 12
        # header.frame_id.len & header.frame_id.char_p
        off += 4 + int.from_bytes(msgd[off:off+4], byteorder='little')
        # height
        height = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        # width
        width = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        # PointField.Length
        pfcount = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        # PointField[]
        for pi in range(0, pfcount):
            # pointField[].name.len
            # pointFiled[].name.char_p[len]
            off += 4 + int.from_bytes(msgd[off:off+4], byteorder='little')
            # pointFiled[].offset
            # pointFiled[].datatype
            # pointField[].count
            off += 4 + 1 + 4
        # is_bigendian
        off += 1
        # point_step
        point_step = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        # row_step
        row_step = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        # data.length
        data_length = int.from_bytes(msgd[off:off+4], byteorder='little')
        off += 4
        # data.byte_p
        data = msgd[off:off+data_length]
        off += data_length
        # is_dense.
        is_dense = msgd[off:off+1]

        off = 0 ## data parse offset
        pc_item_struct_fmt = '5fI2f'
        # same as point_step
        pc_item_size = struct.calcsize(pc_item_struct_fmt)
        pc_count = data_length//point_step
        point_clouds = np.zeros((pc_count,8))
        for pi in range(0, pc_count):
            point_clouds[pi] = struct.unpack(pc_item_struct_fmt,data[off:off+point_step])
            off += point_step
        return [topic, head[0], head[1], head[2], point_clouds]
    return None

# ----------------------------------
class BagFileReader :
    _file = None
    _filepath = None
    _BAGMARK_STR_ = b"#ROSBAG V2.0\n"
    _OP_MESSAGE_DATA = 2
    _OP_BAG = 3
    _OP_INDEX_DATA = 4
    _OP_CHUNK = 5
    _OP_CHUNK_INFO = 6
    _OP_CONNECTION = 7
    _LOG_MASK_ = 0

    ## data set
    _cur_record = None  # [_record_index, f_pos, head_len, header, data_len, data, headers_parsed=_cur_headers]
    _cur_headers = None
    _file_length = 0
    _record_index = 0
    #
    _bag_record = None
    _end_offset = None
    _topic_list = []

    ## file info
    # from op=7,_OP_CONNECTION
    _connections = []  # [[connection_idx,topic_name,type],[...],[...],[...]]
    # from op=6,_OP_CHUNK_INFO
    _chunks = []    # [[conn, count, start_time, end_time],[...],[...],[...]]
    _chunk_idx = 0
    _chunk_messages = []  # [[chunk_id, conn,topic,start,message,parsed],[...],[...],...]
    _chunk_message_idx = 0

    # ----------------------------------
    def __init__(self, mask=0):
        _LOG_MASK_ = mask


    # ----------------------------------
    def open(self, filepath):
        self._file = None
        try:
            self._file = open(filepath, 'rb')
            self._file.seek(0, 2)
            self._file_length = self._file.tell()
            self._file.seek(0, 0)
            ver_str = self._file.readline()
            self._record_index = 0
            self._connections.clear()
            self._chunks.clear()
            self._chunk_idx = 0
            self._chunk_messages.clear()
            self._chunk_message_idx = 0
            self._filepath = filepath
            #if bagmark_str == ver_str.decode('ascii') :
            if self._BAGMARK_STR_ == ver_str :
                Logger(LM_INFO, f"Open {filepath}, size:{self._file_length} {ver_str.decode('ascii')}")
                return True
            else :
                Logger(LM_ERR, f"..File is not Bag File for SRS\n")
                Logger(LM_DBG, f"..{type(self._BAGMARK_STR_)} != {type(ver_str)}")
        except IOError:
            Logger(LM_ERR, f"Exception file open {filepath}\n")
        if self._file != None:
            self._file.close()
            self._file = None
        return False

    # ----------------------------------
    def close(self):
        if self._file != None:
            self._file.close()
            self._file = None

    # ----------------------------------
    def is_open(self):
        if self._file != None:
            return True
        return False

    # ----------------------------------
    def is_valid_record(self):
        if self._cur_record == None or self._cur_record[2] == 0 :
            return False
        return True

    # ----------------------------------
    def read_record(self):
        # if _cur_record is None:
        #     _cur_record = []
        f_pos = self._file.tell()
        head_len = int.from_bytes(self._file.read(4), byteorder='little')
        if head_len == 0 :
            self._cur_record = None
            return False
        header = self._file.read(head_len)
        data_len = int.from_bytes(self._file.read(4), byteorder='little')
        data = self._file.read(data_len)
        self._cur_record = [self._record_index, f_pos, head_len, header, data_len, data, None]
        # len(record_offset) and record_index increase togegher..
        return True

    # ----------------------------------
    def parse_bag_header(self, data, dlen):
        off = 0
        header = dict()
        while off < dlen:
            flen = int.from_bytes(data[off:off+4], byteorder='little')
            off += 4
            pos_eq = data.find(b'=', off)
            field_name = data[off:pos_eq]
            field_value = data[pos_eq+1:off+flen]
            off += flen
            header[field_name.decode("ascii")] = field_value
        return header

    # ----------------------------------
    def parse_sub_record(self, chunk_idx, data, dlen):
        self._chunk_messages.clear()  # [[chunk_id, conn,topic,time,message, parsed],[...],[...],...]
        self._chunk_message_idx = 0
        off = 0
        sub_rec_idx = 0
        sub_rec_off = 0
        while off < dlen:
            sub_rec_off = off
            sub_rec = [sub_rec_idx, sub_rec_off, None, None, None]  # rec_idx, rec_off, headers, data_len, data
            rec_len = int.from_bytes(data[off:off+4], byteorder="little")
            off += 4
            Logger(LM_DBG, f'    {sub_rec_idx}.chunk_record length:{rec_len} offset:{sub_rec_off}')
            # sub_records..
            sub_hds = self.parse_bag_header(data[off:off+rec_len], rec_len)
            for k, v in sub_hds.items():
                sub_hds[k] = self.translate_bag_field_value(k, v)
            Logger(LM_DBG, f'        sub_header={sub_hds}')
            sub_rec[2] = sub_hds
            sub_rec_idx += 1
            off += rec_len
            if 'op' in sub_hds and sub_hds['op'] == self._OP_MESSAGE_DATA:
                md_len = int.from_bytes(data[off:off+4], byteorder="little")
                off += 4
                mdata = data[off:off+md_len]
                sub_rec[3] = md_len
                sub_rec[4] = mdata
                Logger(LM_DBG, f'        sub_message_data={md_len}')
                off += md_len
                if len(self._connections) > 0 :
                    conn_id = sub_hds['conn']
                    topic = self.is_valid_message_data(conn_id)
                    if topic != None :
                        # self._chunk_messages.append([chunk_idx, sub_hds['conn'], topic, sub_hds['time'], mdata, None])
                        self._chunk_messages.append( 
                            {'chunk_idx':chunk_idx, 'app_type':2,'conn':sub_hds['conn'], 'topic':topic \
                             , 'time':sub_hds['time'], 'message':mdata} )

    # ----------------------------------
    def translate_bag_field_value(self, key, value):
        if key=="chunk_count" or key=="conn_count" or key=="conn" or key=="count" or key=="size" :
            return int.from_bytes(value, byteorder='little')
        elif key =='ver' :
            #return value
            return int.from_bytes(value, byteorder='little')
        elif key=="op" or key=="latching" :
            return int.from_bytes(value, byteorder='little')
        elif key=="compression" or key=="topic" or key=="type" or key=="callerid" or key=="md5sum" or key=="message_definition":
            return value.decode("ascii")
        elif key=='time' or key=='end_time' or key=='start_time':
            secs = int.from_bytes(value[0:4], byteorder='little')
            nsecs = int.from_bytes(value[4:8], byteorder='little')
            #return [ time.localtime(secs), nsecs ]
            return [ secs, nsecs ]
        elif key=='index_pos' or key=='chunk_pos':
            ## 8 bytes ??
            return int.from_bytes(value, byteorder='little')
        else:
            return value

    def is_valid_message_data(self, conn_id):
        #                                self._connections.append([self._cur_headers['conn'], self._cur_headers['topic'], None])
        # if assume  conn_id == same self._connection[conn_id][0]..
        if conn_id == self._connections[conn_id][0] :
            #if self._connections[conn_id][1] == '/usb_cam/image_raw/compressed' or self._connections[conn_id][1] == '/point_cloud' :
            #    return self._connections[conn_id][1]
            if self._connections[conn_id][1] in self._topic_list:
                return self._connections[conn_id][1] if self._connections[conn_id][1] in self._topic_list else None
        else:
            for conn in self._connections:
                if conn[0] == conn_id :
                    return self._connections[conn_id][1] if self._connections[conn_id][1] in self._topic_list else None
        #return None

    # ----------------------------------
    def load(self, topics):
        if self.is_open() == False:
            return
        self._topic_list = topics
        try:
            # first record, op=3,_OP_BAG
            self._cur_record = None 
            self.read_record()
            if self.is_valid_record() == False:
                Logger(LM_DBG, 'EOF..\n')
                return
            Logger(LM_DBG, f'  {self._cur_record[0]}.record h:{self._cur_record[2]} d:{self._cur_record[4]} offset:0x{self._cur_record[1]:x},{self._cur_record[1]}')
            self._cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])            
            for k, v in self._cur_headers.items():
                self._cur_headers[k] = self.translate_bag_field_value(k, v)
            Logger(LM_DBG, f'    header={self._cur_headers}')
            self._cur_record[6] = self._cur_headers
            if self._cur_headers['op'] == self._OP_BAG:
                self._bag_record = self._cur_record
                # in case of play, index_data is in last, so play until index_pos, not file_length
                self._end_offset = self._bag_record[6]['index_pos']
                # jump to index_record
                self._file.seek(self._end_offset, 0)
                Logger(LM_DBG, f'--> Jump to Index : {self._end_offset}')
                while True:
                    self._cur_record = None
                    self.read_record()
                    if self.is_valid_record() == False:
                        Logger(LM_DBG, 'INdex is Ending? file end ?..\n')
                        break
                    self._cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])     
                    for k, v in self._cur_headers.items():
                        self._cur_headers[k] = self.translate_bag_field_value(k, v)
                    if self._cur_headers['op'] == self._OP_CONNECTION :
                        self._connections.append([self._cur_headers['conn'], self._cur_headers['topic'], None])
                    elif self._cur_headers['op'] == self._OP_CHUNK_INFO :
                        self._chunks.append([self._cur_headers['chunk_pos'], self._cur_headers['count'], self._cur_headers['start_time'], self._cur_headers['end_time']])

            Logger(LM_DBG, "*"*80)
            Logger(LM_INFO, f'CONNECTIONS:{len(self._connections)}={self._connections}')
            Logger(LM_INFO, f'CHUNKS:{len(self._chunks)}={self._chunks}')
            Logger(LM_DBG, "*"*80)
            # jump to end of bag_record
            ## self._file.seek(self._bag_record[1]+8+self._bag_record[2]+self._bag_record[4], 0)
            Logger(LM_DBG, f'--> Jump to First Chunk : {self._chunks[0][0]}')
            self._record_index = 0
        except IOError:
            if self._file_length == self._file.tell():
                Logger(LM_INFO, 'EOF ? Exception\r\n')
            else:
                Logger(LM_INFO,  f'IO Error\n')

    # ----------------------------------
    def is_valid_bag(self):
        return self.is_open() and len(self._connections)>0 and len(self._chunks)>0

    # ----------------------------------
    def get_next_record(self):
        try:
            for cid in range(self._chunk_idx+1, len(self._chunks)):
                chunk = self._chunks[cid]
                self._cur_record = None 
                self._file.seek(chunk[0], 0)
                self.read_record()
                if self.is_valid_record() == False:
                    Logger(LM_DBG, 'EOF..\n')
                    break
                Logger(LM_DBG, f'  {self._cur_record[0]}.record h:{self._cur_record[2]} d:{self._cur_record[4]} offset:0x{self._cur_record[1]:x},{self._cur_record[1]}')
                _cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])
                for k, v in _cur_headers.items():
                    _cur_headers[k] = self.translate_bag_field_value(k, v)
                Logger(LM_DBG, f'    header={_cur_headers}')
                self._record_index += 1
                if 'op' in _cur_headers and _cur_headers['op'] == self._OP_CHUNK:
                    self.parse_sub_record(cid, self._cur_record[5], self._cur_record[4])
                if len(self._chunk_messages) > 0 :
                    self._chunk_idx = cid
                    return self._chunk_messages
        except IOError:
            if self._file_length == self._file.tell():
                Logger(LM_INFO, 'EOF ? Exception\r\n')
            else:
                Logger(LM_INFO,  f'IO Error\n')
        return None

    # ----------------------------------
    def get_prev_record(self):
        try:
            for cid in range(self._chunk_idx-1, 0, -1):
                chunk = self._chunks[cid]
                self._cur_record = None 
                self._file.seek(chunk[0], 0)
                self.read_record()
                if self.is_valid_record() == False:
                    Logger(LM_DBG, 'EOF..\n')
                    break
                Logger(LM_DBG, f'  {self._cur_record[0]}.record h:{self._cur_record[2]} d:{self._cur_record[4]} offset:0x{self._cur_record[1]:x},{self._cur_record[1]}')
                _cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])
                for k, v in _cur_headers.items():
                    _cur_headers[k] = self.translate_bag_field_value(k, v)
                Logger(LM_DBG, f'    header={_cur_headers}')
                self._record_index += 1
                if 'op' in _cur_headers and _cur_headers['op'] == self._OP_CHUNK:
                    self.parse_sub_record(cid, self._cur_record[5], self._cur_record[4])
                if len(self._chunk_messages) > 0 :
                    self._chunk_idx = cid
                    return self._chunk_messages
        except IOError:
            if self._file_length == self._file.tell():
                Logger(LM_INFO, 'EOF ? Exception\r\n')
            else:
                Logger(LM_INFO,  f'IO Error\n')
        return None
    
    # ----------------------------------
    def go_chunk_record(self, cidx):
        self._chunk_idx = cidx
        self.get_next_record()

    # ----------------------------------
    def get_prev_message(self):
        if self._chunk_message_idx-1 >= 0:
            self._chunk_message_idx -= 1
            # parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx][2], self._chunk_messages[self._chunk_message_idx][4])
            # self._chunk_messages[self._chunk_message_idx][5] = parsed
            parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx].get('topic'), self._chunk_messages[self._chunk_message_idx].get('message'))
            self._chunk_messages[self._chunk_message_idx]['parsed'] = parsed
            return self._chunk_messages[self._chunk_message_idx]

        # if else, reparse next record..
        if self.get_prev_record() != None:
            self._chunk_message_idx = len(self._chunk_messages)-1
            # parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx][2], self._chunk_messages[self._chunk_message_idx][4])
            # self._chunk_messages[self._chunk_message_idx][5] = parsed
            parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx].get('topic'), self._chunk_messages[self._chunk_message_idx].get('message'))
            self._chunk_messages[self._chunk_message_idx]['parsed'] = parsed
            return self._chunk_messages[self._chunk_message_idx]
        return None
    # ----------------------------------
    def get_next_message(self):
        if self._chunk_message_idx+1 < len(self._chunk_messages):
            self._chunk_message_idx += 1
            # parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx][2], self._chunk_messages[self._chunk_message_idx][4])
            # self._chunk_messages[self._chunk_message_idx][5] = parsed
            parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx].get('topic'), self._chunk_messages[self._chunk_message_idx].get('message'))
            self._chunk_messages[self._chunk_message_idx]['parsed'] = parsed
            return self._chunk_messages[self._chunk_message_idx]

        # if else, reparse next record..
        if self.get_next_record() != None:
            self._chunk_message_idx = 0
            # parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx][2], self._chunk_messages[self._chunk_message_idx][4])
            # self._chunk_messages[self._chunk_message_idx][5] = parsed
            parsed = BagRawMessage(self._chunk_messages[self._chunk_message_idx].get('topic'), self._chunk_messages[self._chunk_message_idx].get('message'))
            self._chunk_messages[self._chunk_message_idx]['parsed'] = parsed
            return self._chunk_messages[self._chunk_message_idx]
        return None

    # ----------------------------------
    def print_all(self):
        Logger(LM_DBG, "-"*80)
        try:
            while True:
                self._cur_record = None 
                self.read_record()
                if self.is_valid_record() == False:
                    Logger(LM_DBG, 'EOF..\n')
                    break
                Logger(LM_DBG, f'  {self._cur_record[0]}.record h:{self._cur_record[2]} d:{self._cur_record[4]} offset:0x{self._cur_record[1]:x},{self._cur_record[1]}')
                self._cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])
                for k, v in self._cur_headers.items():
                    self._cur_headers[k] = self.translate_bag_field_value(k, v)
                self._cur_record[6] = self._cur_headers
                self._record_index += 1
                Logger(LM_DBG, f'    header={self._cur_headers}')
                if 'op' in self._cur_headers and self._cur_headers['op'] == self._OP_CHUNK:
                    self.parse_sub_record( -1, self._cur_record[5], self._cur_record[4])
        except IOError:
            if self._file_length == self._file.tell():
                Logger(LM_INFO, 'EOF ? Exception\r\n')
            else:
                Logger(LM_INFO,  f'IO Error\n')

    # ----------------------------------
    def print_bag(self):
        Logger(LM_DBG, "-"*80)
        try:
            # first record, op=3,_OP_BAG
            self._cur_record = None 
            self.read_record()
            if self.is_valid_record() == False:
                Logger(LM_DBG, 'EOF..\n')
                return
            Logger(LM_DBG, f'  {self._cur_record[0]}.record h:{self._cur_record[2]} d:{self._cur_record[4]} offset:0x{self._cur_record[1]:x},{self._cur_record[1]}')
            self._cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])            
            for k, v in self._cur_headers.items():
                self._cur_headers[k] = self.translate_bag_field_value(k, v)
            Logger(LM_DBG, f'    header={self._cur_headers}')
            self._cur_record[6] = self._cur_headers
            if self._cur_headers['op'] == self._OP_BAG:
                self._bag_record = self._cur_record
                # in case of play, index_data is in last, so play until index_pos, not file_length
                self._end_offset = self._bag_record[6]['index_pos']
                # jump to index_record
                self._file.seek(self._end_offset, 0)
                Logger(LM_DBG, f'--> Jump to Index : {self._end_offset}')
                while True:
                    self._cur_record = None
                    self.read_record()
                    if self.is_valid_record() == False:
                        Logger(LM_DBG, 'INdex is Ending? file end ?..\n')
                        break
                    self._cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])     
                    for k, v in self._cur_headers.items():
                        self._cur_headers[k] = self.translate_bag_field_value(k, v)
                    if self._cur_headers['op'] == self._OP_CONNECTION :
                        self._connections.append([self._cur_headers['conn'], self._cur_headers['topic'], None])
                    elif self._cur_headers['op'] == self._OP_CHUNK_INFO :
                        self._chunks.append([self._cur_headers['chunk_pos'], self._cur_headers['count'], self._cur_headers['start_time'], self._cur_headers['end_time']])

            Logger(LM_DBG, "*"*80)
            Logger(LM_INFO, f'CONNECTIONS:{len(self._connections)}={self._connections}')
            Logger(LM_INFO, f'CHUNKS:{len(self._chunks)}={self._chunks}')
            Logger(LM_DBG, "*"*80)
            # jump to end of bag_record
            ## self._file.seek(self._bag_record[1]+8+self._bag_record[2]+self._bag_record[4], 0)
            Logger(LM_DBG, f'--> Jump to First Chunk : {self._chunks[0][0]}')
            self._record_index = 0
            cid = 0
            for chunk in self._chunks :
                self._cur_record = None 
                self._file.seek(chunk[0], 0)
                self.read_record()
                if self.is_valid_record() == False:
                    Logger(LM_DBG, 'EOF..\n')
                    break
                Logger(LM_DBG, f'  {self._cur_record[0]}.record h:{self._cur_record[2]} d:{self._cur_record[4]} offset:0x{self._cur_record[1]:x},{self._cur_record[1]}')
                _cur_headers = self.parse_bag_header(self._cur_record[3], self._cur_record[2])
                for k, v in _cur_headers.items():
                    _cur_headers[k] = self.translate_bag_field_value(k, v)
                Logger(LM_DBG, f'    header={_cur_headers}')
                self._record_index += 1
                if 'op' in _cur_headers and _cur_headers['op'] == self._OP_CHUNK:
                    self.parse_sub_record(cid, self._cur_record[5], self._cur_record[4])
                cid += 1

        except IOError:
            if self._file_length == self._file.tell():
                Logger(LM_INFO, 'EOF ? Exception\r\n')
            else:
                Logger(LM_INFO,  f'IO Error\n')


# ----------------------------------
if __name__ == '__main__':
    argument = sys.argv
    del argument[0]
    Logger(LM_INFO, f'args={argument}')
    if len(argument) > 0 :
        bagfile_path = argument[0]
    bagf = BagFileReader()
    bagf.open(bagfile_path)
    bagf.load( ('/usb_cam/image_raw/compressed', '/image_raw/compressed',  '/point_cloud', '/velodyne_points') )
    if bagf.is_valid_bag() :
        while True:
            record_message = bagf.get_next_message()
            if record_message == None :
                break
            ### # [[chunk_id, conn,topic,start,end,message],[...],[...],...]
            print(f"---> message(chunk_id={record_message['chunk_idx']}) cid:{record_message['conn']} topic:{record_message['topic']}")
        bagf.close()

