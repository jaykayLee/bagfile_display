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
from PyQt5.QtCore import QDateTime, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
    QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
    QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget, QFileDialog, QButtonGroup, QMessageBox)
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from bag_parse import BagFileReader
#import debugpy


# ----------------------------------
def getstr_timestamp(prefix, tm_sec, tm_nsec):
        return f"{prefix} time:{datetime.datetime.fromtimestamp(tm_sec).strftime('%Y/%m/%d %H:%M:%S.')} + {str(tm_nsec)} "

class BagPlayer():
    pass

# def TimeTicker(player, tick_event):
#     _tmtick_count = 0
#     tick_event.set()
#     print(f'Time Ticker {_tmtick_count}')
#     _tmtick_count += 1
#     #player._timer.start()

# ----------------------------------
def ProcessPlay(_dict, _event, _outq, _is_run):
    ##  _bagf:BagFileReader, _play_mode:(1:normal,2:reverse), _outq:Queue, is_run(mp.Value)
    #debugpy.debug_this_thread()
    _run_count = 0
    l_mode = _dict['mode'] # _play_mode.value
    l_step = _dict['step'] # _play_step.value
    l_interval = _dict['interval'] #_play_interval.value
    l_bagf = None
    l_filepath = None
    l_topics = None
    l_check_topics = None
    _dict['errcode'] = -1
    print(f'>>Start Player')
    try :
        while _is_run.value:
            if _event.wait(l_interval) != True :
                continue

            _event.clear()
            l_mode = _dict['mode'] # _play_mode.value
            l_step = _dict['step'] # _play_step.value
            l_interval = _dict['interval'] #_play_interval.value
            # mode = 0 # 0:pause, 1:play, 2:reverse_play, 100:file_open 200:file_close(?)
            # step = 0:None  +num : formward, -num:reverse.            
            if l_mode == 0 :  # pause
                step = np.abs(l_step)
                # ... do step operation, next, prev
                if step == 0:
                    print('player stopped')
                else:
                    print(f'go { "next" if l_step > 0 else "prev"} {step} step, count is {_run_count}')
                    l_check_topics = _dict['check_topics']
                    try :
                        while step > 0 :
                            if l_bagf != None and l_bagf.is_valid_bag() :
                                record_message = l_bagf.get_next_message() if l_step > 0 else l_bagf.get_prev_message()
                                if record_message != None :
                                    _dict['errcode'] = l_mode
                                    #print(f'player go - {record_message["chunk_idx"]}, {record_message["topic"]}')
                                    _outq.put(record_message)
                                    _run_count += 1
                                else:
                                    _outq.put(None)
                                    _dict['errcode'] = -500 # file end.
                                    break
                            else:
                                break
                            if record_message['topic'] in l_check_topics :
                                step -= 1
                    except Exception as e:
                        print(e)

            elif l_mode == 1 or l_mode==2 : # play or reverse play
                _dict['errcode'] = l_mode
                while _is_run.value:
                    # stop if mode is changed
                    if l_mode != _dict['mode'] :
                        print(f'player mode chaned {l_mode} -> { int(_dict["mode"])}')
                        break
                    #... 
                    #print(f'play { "next" if l_mode==1 else "prev"}, count is {_run_count}')
                    try :
                        if l_bagf.is_valid_bag() :
                            record_message = l_bagf.get_next_message() if l_mode == 1 else l_bagf.get_prev_message()
                            if record_message != None :
                                #print(f'player play - {record_message["chunk_idx"]}, {record_message["topic"]}')
                                _outq.put(record_message)
                                _run_count += 1
                            else:
                                _outq.put(None)
                                l_mode = 0
                                break
                    except Exception as e:
                        print(e)
                    time.sleep(l_interval)
            elif l_mode == 3 : ## go to chunk
                l_chunk_idx = l_step
                try :
                    if l_bagf.is_valid_bag() :
                        l_bagf.go_chunk_record(l_chunk_idx)
                        record_message = l_bagf.get_next_message()
                        if record_message != None :
                            #print(f'player chunk - {record_message["chunk_idx"]}, {record_message["topic"]}')
                            _outq.put(record_message)
                            _run_count += 1
                        else:
                            _outq.put(None)
                except Exception as e:
                    print(e)
                l_mode = 0 # change to pause mode
                l_step = 0
            elif l_mode == 100 : # file open
                if l_bagf != None :
                    l_bagf.close()
                    del l_bagf
                    l_bagf = None
                l_filepath = _dict['filepath']
                l_topics = str(_dict['topics']).split(',')
                l_bagf = BagFileReader()
                l_bagf.open(l_filepath)
                l_bagf.load(l_topics)
                if l_bagf.is_valid_bag() :
                    print(f'player open {l_filepath} with topcis:{l_topics}')
                    _run_count = 0
                    _dict['errcode'] = 100
                else :
                    print(f'player failed open {l_filepath} with topcis:{l_topics}')
                    _dict['errcode'] = -100
                continue
            elif l_mode == 200 : # file close
                print(f'player close {l_filepath}')
                l_bagf.close()
                del l_bagf
                l_bagf = None
                _run_count = 0
                _dict['errcode'] = 200
                continue
            else :
                _dict['errcode'] = -1


            # if _bagf.is_valid_bag() :
            #     pre_tick = time.time_ns()/1000000
            #     _record_message = _bagf.get_next_message() if _play_mode == 1 else _bagf.get_prev_message()
            #     post_tick = time.time_ns()/1000000
            #     if _record_message == None :
            #         _outq.put(None)
            #         _play_mode = 0
            #         return
            #     _outq._out_que.put(_record_message)

    except Exception as e:
        print(e)
    del _dict
    del _event
    print(f'p..layer exit with {_run_count}')
    if l_bagf != None:
        l_bagf.close()
        del l_bagf
        l_bagf = None
    print(f'>>Exit Player')


# ----------------------------------
class BagPlayer():
    _dict = None
    ##  _bagf:BagFileReader, _play_mode:(1:normal,2:reverse), _outq:Queue, is_run(mp.Value)
    _is_run = mp.RawValue(ctypes.c_bool, False)
    _play_process = None
    _event = None
    _out_que = None
    # ----------------------------------
    def __init__(self, manager, outq=None):
        self._out_que = outq
        self._dict = manager.dict()
        self._dict['mode'] = 0
        self._dict['step'] = 0
        self._dict['interval'] = 0.001
        self._dict['filepath'] = None
        self._dict['topics'] = None
        self._dict['check_topics'] = None
        self._event = mp.Event()

    def set_file(self, bagf, check_list):
        #self._bagf = bagf
        self._dict['filepath'] = bagf._filepath
        self._dict['topics'] = ','.join(bagf._topic_list)
        self._dict['check_topics'] = list(np.array(bagf._topic_list)[np.array(check_list)])
        self._dict['mode'] = 100
        self._event.set()

    def prev(self, step=-1):
        print(f'>>Go Prev to {step}')
        self._dict['mode'] = 0
        self._dict['step'] = step
        self._event.set()

    def next(self, step=1):
        print(f'>>Go Next to {step}')
        self._dict['mode'] = 0
        self._dict['step'] = step
        self._event.set()

    def pause(self):
        print('>>Pause Player')
        self._dict['mode'] = 0
        self._dict['step'] = 0
        self._event.set()

    def play(self):
        print('>>Play forward')
        self._dict['mode'] = 1
        self._dict['step'] = 0
        self._event.set()

    def reverse_play(self):
        print('>>Play Reverse')
        self._dict['mode'] = 2
        self._dict['step'] = 0
        self._event.set()

    def set_chunk_pos(self, pos):
        self._dict['mode'] = 3
        self._dict['step'] = pos
        self._event.set()

    def start(self):
        print('>>Start Player')
        self._is_run.value = True
        #def ProcessPlay(_dict, _event, _outq, _is_run):
        self._play_process = mp.Process(target=ProcessPlay, args=(self._dict, self._event, self._out_que, self._is_run))
        self._play_process.start()

    def stop(self):
        print('>>Stop Player')
        self.pause()
        time.sleep(0.5)
        self._is_run.value = False
        self._event.set()
        if self._play_process.join(1)  == None :
            print(f'..exit code is {self._play_process.is_alive()}, exit:{self._play_process.exitcode} ')
            self._play_process.terminate()
        del self._play_process
        self._play_process = None

    def is_playing(self):
        return True if self._dict['mode'] == 1 or self._dict['mode'] == 2 else False


# ----------------------------------
if __name__ == '__main__':
    bagfile_path = "d:/2021-12-28-14-02-41.bag"
    outq = mp.Queue()
    manager = mp.Manager()
    bagf = BagFileReader()
    bagf.open(bagfile_path)
    bagf.load(('/usb_cam/image_raw/compressed', '/image_raw/compressed', '/point_cloud', '/velodyne_points'))
    count = 0
    if bagf.is_valid_bag() :
        player = BagPlayer(manager, outq)
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
