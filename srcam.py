from time import sleep

qtcam_on = False

if qtcam_on :
    from PyQt5.QtCore import QObject, QDateTime, Qt, QTimer, QThread, pyqtSignal, QMutex, QMutexLocker, QPoint, pyqtSlot
    from PyQt5.QtWidgets import QMessageBox
    from PyQt5.QtMultimedia import *
    from PyQt5.QtMultimediaWidgets import *
    import PyQt5.QtGui as qtgui
    import sys

else :
    import cv2
    from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal, QMutex, QMutexLocker, QPoint, pyqtSlot
    from PyQt5.QtWidgets import QMessageBox

vcdbg_on = False ## vscode debugger
if vcdbg_on :
    import debugpy

if qtcam_on :
    class SRCamera(QObject):
        _cam_list = []
        _cam_on = False
        _camera = None
        _capture_cb = pyqtSignal(object)
        _run_on = False
        _fn_count = 0
        def __init__(self):
            super(SRCamera, self).__init__()

        def set(self, cam_idx) :
            self._camera = None
            if cam_idx == 'default' :
                self._camera = QCameraInfo.defaultCamera()
            elif len(SRCamera._cam_list) > 0 :
                if type(cam_idx).__name__ == 'str':
                    cam_idx = int(cam_idx)
                if cam_idx >=0 and cam_idx < len(SRCamera._cam_list) :
                    self._camera = QCamera(QCameraInfo.availableCameras()[cam_idx])
                else :
                    return False
            else :
                return False
            return True

        def enable(self, onoff):
            self._cam_on = onoff
            if self._camera == None :
                return
            if self._cam_on :
                self._run_on = True
                self._fn_count = 0
                #self.start()
                self._camera.start()
                self._capture = QCameraImageCapture(self._camera)
                self._capture.setCaptureDestination(QCameraImageCapture.CaptureDestination.CaptureToBuffer)
                #self._capture.imageCaptured.connect(self.captured)
                self._capture.error.connect(lambda error_msg, error,
                                        msg: print(f" capture_error : {msg}"))

                # when image captured showing message
                self._capture.imageCaptured.connect(lambda d, i: print(f"Image captured : "
                                                                            + str(self.save_seq)))
                
            else:
                self._run_on = False
                self._fn_count = 0
                #if self.wait(500) == False :
                #    print(f'Camera thread is not terminated..')
                self._capture.cancelCapture()
                self._camera.stop()

        def is_enabled(self):
            return self._run_on

        def frame_num(self):
            return self._fn_count

        def captured(self, id: int, preview: qtgui.QImage):
            self._capture_cb.emit(id, preview)

        # # def stop(self):
        # #     self._run_on = False
        # #     if self.wait(500) == False :
        # #         print(f'Camera thread is not terminated..')
        # def run(self):
        #     if vcdbg_on : debugpy.debug_this_thread()
        #     print("--- start run_camera ---")
        #     cap = cv2.VideoCapture(self._cam_idx)
        #     while self._run_on:
        #         ret, img = cap.read()
        #         if ret:
        #             # imgarr = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        #             ret, img = cv2.imencode('.jpeg', img)
        #             # h,w,c = img.shape
        #             # qImg = QImage(img.data, w, h, w*c, QImage.Format_RGB888)
        #             # pixmap = QPixmap.fromImage(qImg)
        #             if img is None or len(img) < 1 :
        #                 print("<<<>>>><<<<>>>>")
        #             if ret == True :
        #                 self._capture_cb.emit(img.tobytes())
        #                 self._fn_count += 1
        #         else:
        #             QMessageBox.about(None, "Error", "Cannot read frame.")
        #             print("cannot read frame.")
        #             break
        #     cap.release()
        #     self._fn_count = -1
        #     print("--- exit run_camera ---")    

        def scan(scan_max=5):
            available_cameras = QCameraInfo.availableCameras()
            SRCamera._cam_list = []
            _start_idx = 0

            if not available_cameras:
                return SRCamera._cam_list  ## nothing..

            for i in range(len(available_cameras)) :
                print(f"    camera:{i} name:{available_cameras[i].deviceName()} desc:{available_cameras[i].description()}")
                SRCamera._cam_list.append((i, available_cameras[i].deviceName()))
            return SRCamera._cam_list

else :
    # ----------------------------------
    class SRCamera(QThread):
        _cam_on = False
        _cam_idx = -1
        _capture_cb = pyqtSignal(object)
        _run_on = False
        _fn_count = 0

        def __init__(self):
            super(SRCamera, self).__init__()

        def set(self, cam_idx) :
            if cam_idx == 'default' :
                self._cam_idx = 0
            elif cam_idx == 'None':
                return False
            else :
                if type(cam_idx).__name__ == 'str':
                    self._cam_idx = int(cam_idx)
                else : # type(cam_idx).__name__ == 'int':
                    self._cam_idx = cam_idx
            return True

        def enable(self, onoff):
            self._cam_on = onoff
            if self._cam_idx < 0 :
                return
            if self._cam_on :
                self._run_on = True
                self._fn_count = 0
                self.start()
            else:
                self._run_on = False
                self._fn_count = 0
                if self.wait(500) == False :
                    print(f'Camera thread is not terminated..')

        def is_enabled(self):
            return self._run_on

        def frame_num(self):
            return self._fn_count

        # def stop(self):
        #     self._run_on = False
        #     if self.wait(500) == False :
        #         print(f'Camera thread is not terminated..')
        def run(self):
            if vcdbg_on : debugpy.debug_this_thread()
            print("--- start run_camera ---")
            cap = cv2.VideoCapture(self._cam_idx)
            while self._run_on:
                ret, img = cap.read()
                if ret:
                    # imgarr = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
                    ret, img = cv2.imencode('.jpeg', img)
                    # h,w,c = img.shape
                    # qImg = QImage(img.data, w, h, w*c, QImage.Format_RGB888)
                    # pixmap = QPixmap.fromImage(qImg)
                    if img is None or len(img) < 1 :
                        print("<<<>>>><<<<>>>>")
                    if ret == True :
                        self._capture_cb.emit(img.tobytes())
                        self._fn_count += 1
                else:
                    QMessageBox.about(None, "Error", "Cannot read frame.")
                    print("cannot read frame.")
                    break
            cap.release()
            self._fn_count = -1
            print("--- exit run_camera ---")    

        def scan(scan_max=5):
            if True :
                _cam_list = [(0, 640, 480, 30), (1, 640, 480, 30)]
                return _cam_list
            else :
                _cam_list = []
                _start_idx = 0
                if True :  ## assume default 0 camera is always available. so slow in my laptop.
                    _start_idx = 1
                    _cam_list.append((0, 640, 480, 30))
                else :
                    _start_idx = 0

                # for i in range(scan_max) :
                for i in range(_start_idx, scan_max) :
                    try :
                        cap = cv2.VideoCapture(i)
                    except Exception as e :
                        #print(f"    capture index : {i} is not available.")
                        continue
                    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                    fps = cap.get(cv2.CAP_PROP_FPS)
                    if width > 0 and height > 0 and fps > 0 :
                        print(f"    capture index : {i} w:{width} H:{height} FPS:{fps} ")
                        _cam_list.append((i, width, height, fps))
                    else :
                        #print(f"    capture index : {i} is not available, w:{width} H:{height} FPS:{fps} ")
                        pass
                    cap.release()
                return _cam_list



# ----------------------------------
if __name__ == '__main__':
    cap_count = 0
    def capture_image(object) :
        print(f"--captured:{cap_count}")
        cap_count += 1
        if cap_count > 5 :
            return True
        return False

    clist = SRCamera.scan()
    for ci in clist :
        print(f" index:{ci[0]} available.")
    cam = SRCamera()
    cam.set(0)
    cam._capture_cb = capture_image
    cam.enable(True)

    while cap_count < 5 :
        sleep(1)

    sys.exit(0)
