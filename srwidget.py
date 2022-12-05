import sys
import datetime
import time
import math
import struct
import os
import string
import typing
from matplotlib import projections
import numpy as np
import multiprocessing as mp
import ctypes
from collections import deque
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal, QMutex, QMutexLocker, QPoint, QPointF
from PyQt5.QtGui import QPixmap, QImage, QMouseEvent, QMatrix4x4
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import OpenGL.GL as GL
import OpenGL.GLU as GLU

class SRKEYMODE() :
    NORMAL        : typing.Final[int] = 0
    RESIZE        : typing.Final[int] = 1
    MOVE          : typing.Final[int] = 2
    


## copied from GLViewWidget.py and modify.
class SRViewWidget(gl.GLViewWidget):
    _clip_box = None
    _clip_mode = False
    _clip_key_mode : SRKEYMODE = SRKEYMODE.NORMAL
    _clip_key_detail_on : bool = False
    _clip_keys = [Qt.Key.Key_Left, Qt.Key.Key_Right, Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_PageUp, Qt.Key.Key_PageDown]
    _start_pos = None
    _end_pos = None
    _start_xyz = None
    _end_xyz = None
    _select_region_cb = pyqtSignal('PyQt_PyObject')
    _clicked_cb = pyqtSignal('PyQt_PyObject', 'PyQt_PyObject')


    def __init__(self, parent=None):  ## devicePixelRatio=None, rotationMethod='euler'
        ##GL.glEnable(GL.GL_DEPTH_TEST)
        super(SRViewWidget, self).__init__(parent)
        #QtWidgets.QOpenGLWidget.__init__(self, parent)

    def un_project(self, x, y, d, mv, p, v):
        transfer = np.dot(p, mv)
        transfer = transfer.T
        invec = np.array([x, y, d, 1], dtype=np.double).T
        r1 = np.dot(transfer, invec)
        r0 = r1[:]
        if r0[3] == 0 :
            print(f" un_project error {r0}")
            return -1, -1, -1
        r0[3] = 1.0 / r0[3]
        nx = r0[0] * r0[3]
        ny = r0[1] * r0[3]
        nz = r0[2] * r0[3]
        return nx, ny, nz

    def perspective(self):
        region = (0, 0, self.deviceWidth(), self.deviceHeight())
        
        x0, y0, w, h = self.getViewport()
        dist = self.opts['distance']
        fov = self.opts['fov']
        nearClip = dist * 0.001
        farClip = dist * 1000.

        r = nearClip * np.tan(0.5 * np.radians(fov))
        t = r * h / w

        ## Note that X0 and width in these equations must be the values used in viewport
        left  = r * ((region[0]-x0) * (2.0/w) - 1)
        right = r * ((region[0]+region[2]-x0) * (2.0/w) - 1)
        bottom = t * ((region[1]-y0) * (2.0/h) - 1)
        top    = t * ((region[1]+region[3]-y0) * (2.0/h) - 1)
        print(f"prespective.in ({left},{right},{top},{bottom}), r={r}, t={t}")

        # tr = QMatrix4x4()
        # tr.frustum(left, right, bottom, top, nearClip, farClip)
        # return tr

        #  https://stackoverflow.com/questions/46749675/opengl-mouse-coordinates-to-space-coordinates
        #  https://stackoverflow.com/questions/7777913/how-to-render-depth-linearly-in-modern-opengl-with-gl-fragcoord-z-in-fragment-sh
        # r = right, l = left, b = bottom, t = top, n = near, f = far
        #       2*n/(r-l)      0              0               0
        #       0              2*n/(t-b)      0               0
        #       (r+l)/(r-l)    (t+b)/(t-b)    -(f+n)/(f-n)=A  -1    
        #       0              0              -2*f*n/(f-n)=B  0
        ####  http://www.songho.ca/opengl/gl_projectionmatrix.html ???
        tr = np.zeros((4,4))
        tr[0,0] = 2*nearClip/(right-left)
        tr[1,1] = 2*nearClip/(top-bottom)
        tr[2,0] = (right+left)/(right-left)
        tr[2,1] = (top+bottom)/(top-bottom)
        tr[2,2] = -(farClip+nearClip)/(farClip-nearClip)
        tr[2,3] = -1
        tr[3,2] = -2*farClip*nearClip/(farClip-nearClip)
        
        ## z_ndc = ( -z_eye * (f+n)/(f-n) - 2*f*n/(f-n) ) / -z_eye
        #        ndc_? => ( -1, ~~, 1 ) 
        #        depth = (z_ndc + 1) / 2 
        #        depth => ( 0, ~~ , 1)
        ###  z_ndc = 2.0 * depth - 1.0;
        #    z_eye = 2.0 * n * f / (f + n - z_ndc * (f - n));

        return tr

    def get_view_pos_from_space(self, x, y, z):
        modelview = self.viewMatrix();          modelview = np.array(modelview.copyDataTo()).reshape(4,4);      modelview = np.matrix(modelview)
        projection = self.projectionMatrix();   projection = np.array(projection.copyDataTo()).reshape(4,4);    projection = np.matrix(projection)
        viewport = self.getViewport();          viewport = np.array(viewport, dtype=int)
        
        dist = self.opts['distance']
        fov = self.opts['fov']
        nearClip = dist * 0.001
        farClip = dist * 1000.

        vec = np.array([x,y,z,1], dtype=np.double).reshape(4,1)

        eye = modelview*vec        
        clip = projection*eye
        clip = clip.A1
        eye = eye.A1
        ndc = clip[0:3]/clip[3]
        win = np.copy(ndc)
        win[0] = ((viewport[2]-viewport[0])*win[0])/2 + ((viewport[2]-viewport[0])/2+viewport[0])
        win[1] = ((viewport[3]-viewport[1])*win[1])/2 + ((viewport[3]-viewport[1])/2+viewport[1])
        win[2] = ((farClip-nearClip)*win[2])/2 + (farClip-nearClip)/2
        #print(f"get_view_pos by P*MV [{x}, {y}, {z}] --> Win[{win[0]:0.2f}, {win[1]:0.2f}, {win[2]:0.2f}]  ndc=[{ndc}] clip=[{clip}] eye=[{eye}]")
        win[2] = viewport[3] - win[2]
        return win

    def get_space_pos_from_view(self, x, y):
        modelview = self.viewMatrix();          modelview = np.array(modelview.copyDataTo()).reshape(4,4);      modelview = np.matrix(modelview)
        projection = self.projectionMatrix();   projection = np.array(projection.copyDataTo()).reshape(4,4);    projection = np.matrix(projection)
        viewport = self.getViewport();          viewport = np.array(viewport, dtype=int)

        #x = x
        y = viewport[3]-y

        dist = self.opts['distance']
        fov = self.opts['fov']
        nearClip = dist * 0.001
        farClip = dist * 1000.
        tan_fov = np.tan(0.5 * np.radians(fov))
        ratio = viewport[2]/viewport[3]

        ndc_x = x / (viewport[2]/2) - 1.0 
        ndc_y = y / (viewport[3]/2) - 1.0
        #ndc_z = 2.0*depth-1.0
        ndc_z = 0.998  # original axis, z=0 plane
        ndc = np.array([ndc_x,ndc_y,ndc_z,1], dtype=np.double).reshape(4,1)
        # clip = np.copy(ndc)
        # wc = -eye_z
        clip = projection.I*ndc
        eye = modelview.I*clip
        r_clip = clip[0:3]/clip[3]
        r_eye = eye[0:3]/eye[3]
        #print(f"get_space_pos2 by P.I*MV.I [{x},{y}] --> ndc=[{ndc.ravel()}] clip[{clip.A1}, {r_clip.A1}] eye[{eye.A1}, {r_eye.A1}]")
        return r_eye.A1
        # z_eye = 2.0*nearClip*farClip/(farClip+nearClip - ndc_z*(farClip-nearClip))
        # r2 = np.zeros((3))
        # r2[0] = z_eye*ndc_x * ratio * tan_fov
        # r2[1] = z_eye*ndc_y * tan_fov
        # r2[2] = -z_eye
        # print(f"get_space_pos by [{x},{y},{depth}] --> ndc=[{ndc_vec.ravel()}] view[{r2}] ")

    def get_itemsAt(self, region=None):
        """
        Return a list of the items displayed in the region (x, y, w, h)
        relative to the widget.        
        """
        region = (region[0], self.deviceHeight()-(region[1]+region[3]), region[2], region[3])
        
        #buf = np.zeros(100000, dtype=np.uint)
        buf = GL.glSelectBuffer(100000)
        try:
            GL.glRenderMode(GL.GL_SELECT)
            GL.glInitNames()
            GL.glPushName(0)
            self._itemNames = {}
            self.paintGL(region=region, useItemNames=True)
            
        finally:
            hits = GL.glRenderMode(GL.GL_RENDER)
            
        items = [(h.near, h.names[0]) for h in hits]
        items.sort(key=lambda i: i[0])
        return [self._itemNames[i[1]] for i in items]

    def test(self, lpos):
        modelview = self.viewMatrix();          modelview = np.array(modelview.copyDataTo()).reshape(4,4)
        projection = self.projectionMatrix();   projection = np.array(projection.copyDataTo()).reshape(4,4)
        viewport = self.getViewport();          viewport = np.array(viewport, dtype=int)
        modelview = np.matrix(modelview)
        projection = np.matrix(projection)
        perspective_prj = self.perspective()
        dist = self.opts['distance']
        fov = self.opts['fov']
        print("-"*30)
        print(f"pos=({lpos.x()},{lpos.y()}), view=({viewport}), dist={dist}, fov={fov}")
        print(f"c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({fov}), d=({dist})")
        # print(f"projection=({projection})")
        # print(F'perspective_projection=({perspective_prj})')
        print(f"modelview=({modelview})")
        self.get_view_pos_from_space(1, 1, 0)
        # self.get_view_pos_from_space(-1, -1, 0)
        # self.test(0,0,0)'

        nearClip = dist * 0.001
        farClip = dist * 1000.
        zrange = dist*(1000.001)
        # (farClip-nearClip)1000
        #self.get_space_pos_from_view(lpos.x(), (viewport[3]-viewport[1])-lpos.y(), 0.998)
        self.get_space_pos_from_view(lpos.x(), (viewport[3]-viewport[1])-lpos.y())
        #self.get_itemsAt(region=[lpos.x(), lpos.y(), 5, 5])

    def set_clipmode(self, onoff):
        if True :
            if onoff == True :
                #print(f"PrevMode c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({self.opts['fov']}), d=({self.opts['distance']})")
                ## c=(PyQt5.QtGui.QVector3D(0.0, 0.0, 0.0)), r=(PyQt5.QtGui.QQuaternion(1.0, 0.0, 0.0, 0.0)), rm=(euler), a=(-90.0), e=(90.0), f=(60), d=(10.0)        
                ## center, distance is not change..
                #self.opts['center'] = pg.Vector(0,0,0)  ## will always appear at the center of the widget
                #self.opts['distance'] = 10.0         ## distance of camera from center
                self.opts['fov'] = 60                ## horizontal field of view in degrees
                self.opts['elevation'] = 90          ## camera's angle of elevation in degrees
                self.opts['azimuth'] = -90            ## camera's azimuthal angle in degrees 
                                                    ## (rotation around z-axis 0 points along x-axis)
                self.opts['viewport'] = None         ## glViewport params; None == whole widget
                #self.setBackgroundColor(self.getConfigOption('background'))
                self._clip_mode = True
                self._clip_key_mode = SRKEYMODE.NORMAL
                self._clip_key_detail_on = False
                #self._clip_box = gl.GLBoxItem()
                #self.addItem(self._clip_box)
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glLoadIdentity()
                GLU.gluLookAt(0, -55, 50, 0, 0, 0, 0, 0, 1)
                self.update()
                #print(f"CLipMode c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({self.opts['fov']}), d=({self.opts['distance']})")
            else :
                self._clip_mode = False
                self._clip_key_mode = SRKEYMODE.NORMAL
        else :
            if onoff == True :
                #print(f"PrevMode c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({self.opts['fov']}), d=({self.opts['distance']})")
                ## c=(PyQt5.QtGui.QVector3D(0.0, 0.0, 0.0)), r=(PyQt5.QtGui.QQuaternion(1.0, 0.0, 0.0, 0.0)), rm=(euler), a=(-90.0), e=(90.0), f=(60), d=(10.0)        
                ## center, distance is not change..
                #self.opts['center'] = pg.Vector(0,0,0)  ## will always appear at the center of the widget
                #self.opts['distance'] = 10.0         ## distance of camera from center
                self.opts['fov'] = 60                ## horizontal field of view in degrees
                self.opts['elevation'] = 90          ## camera's angle of elevation in degrees
                self.opts['azimuth'] = -90            ## camera's azimuthal angle in degrees 
                                                    ## (rotation around z-axis 0 points along x-axis)
                self.opts['viewport'] = None         ## glViewport params; None == whole widget
                #self.setBackgroundColor(self.getConfigOption('background'))
                self._clip_mode = True
                self._clip_key_mode = SRKEYMODE.NORMAL
                self._clip_key_detail_on = False
                self._clip_box = None
                #self._clip_box = gl.GLBoxItem()
                #self.addItem(self._clip_box)
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glLoadIdentity()
                GLU.gluLookAt(0, -55, 50, 0, 0, 0, 0, 0, 1)
                self.update()
                #print(f"CLipMode c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({self.opts['fov']}), d=({self.opts['distance']})")
            else :
                if self._clip_box != None : 
                    self.removeItem(self._clip_box)
                    del self._clip_box; self._clip_box = None
                self._start_pos = None
                self._end_pos = None
                self._clip_mode = False
                self._clip_key_mode = SRKEYMODE.NORMAL
                self._clip_key_detail_on = False


    def mousePressEvent(self, ev):
        lpos = ev.position() if hasattr(ev, 'position') else ev.localPos()
        self.mousePos = lpos
        #print(f'*mousePressEvent {ev.buttons()} P[{lpos}]')
        if self._clip_mode :
            if ev.buttons() == Qt.MouseButton.LeftButton:
                if self._clip_box == None :
                    self._clip_box = gl.GLBoxItem()
                    self.addItem(self._clip_box)
                    self._clip_box.setColor('b')
                else :
                    self._clip_box.resetTransform()
                self._clip_box.setSize(0.01,0.01,0.01)
                self._start_pos = lpos
                self._start_xyz = self.get_space_pos_from_view(self._start_pos.x(), self._start_pos.y())
                self._start_xyz[2] = 0.0
                self._clip_box.translate(self._start_xyz[0], self._start_xyz[1], self._start_xyz[2])
                
                # _start_xyz = [0,0,0]
                # _end_xyz = [1,1,1]
            #self.test(lpos)
            if ev.buttons() == Qt.MouseButton.RightButton:
                self._clicked_cb.emit(Qt.MouseButton.RightButton, lpos)
        else :
            if ev.buttons() == Qt.MouseButton.RightButton:
                self._clicked_cb.emit(Qt.MouseButton.RightButton, lpos)
                # vxyz = self.get_space_pos_from_view(lpos.x(), lpos.y())
                # wxyz = self.get_view_pos_from_space(vxyz[0], vxyz[1], vxyz[2])
                # print(f"RMP: ({lpos.x()}, {lpos.y()})  X={vxyz[0]}:{wxyz[0]}, Y={vxyz[1]},{wxyz[1]} Z={vxyz[2]},{wxyz[2]}")
                # print(f"mproess c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({self.opts['fov']}), d=({self.opts['distance']})")

        
    def mouseMoveEvent(self, ev):
        lpos = ev.position() if hasattr(ev, 'position') else ev.localPos()
        diff = lpos - self.mousePos
        self.mousePos = lpos

        if self._clip_mode :
            if ev.buttons() == Qt.MouseButton.LeftButton:
                self._end_pos = lpos
                self._end_xyz = self.get_space_pos_from_view(self._end_pos.x(), self._end_pos.y())
                self._end_xyz[2] = 10
                self._clip_box.setSize(self._end_xyz[0]-self._start_xyz[0], self._end_xyz[1]-self._start_xyz[1], self._end_xyz[2]-self._start_xyz[2])
            pass
        else :
            if ev.buttons() == Qt.MouseButton.LeftButton:
                if (ev.modifiers() & Qt.KeyboardModifier.ControlModifier):
                    self.pan(diff.x(), diff.y(), 0, relative='view')
                else:
                    self.orbit(-diff.x(), diff.y())
            elif ev.buttons() == Qt.MouseButton.MiddleButton:
                if (ev.modifiers() & Qt.KeyboardModifier.ControlModifier):
                    self.pan(diff.x(), 0, diff.y(), relative='view-upright')
                else:
                    self.pan(diff.x(), diff.y(), 0, relative='view-upright')
            elif ev.buttons() == Qt.MouseButton.RightButton:
                pass
            #print(f"mmove c=({self.opts['center']}), r=({self.opts['rotation']}), rm=({self.opts['rotationMethod']}), a=({self.opts['azimuth']}), e=({self.opts['elevation']}), f=({self.opts['fov']}), d=({self.opts['distance']})")
            #print(f'*mouseMoveEvent {ev.buttons()} P[{lpos}]')
        
    def mouseReleaseEvent(self, ev):
        lpos = ev.position() if hasattr(ev, 'position') else ev.localPos()
        if self._clip_mode :
            if ev.button() == Qt.MouseButton.LeftButton:
                self._select_region_cb.emit([self._start_xyz, self._end_xyz])
        #print(f'*mouseReleaseEvent {ev.button()} P[{lpos}]')
        pass
        
    def wheelEvent(self, ev):
        delta = ev.angleDelta().x()
        if self._clip_mode :
            pass
        else:
            if delta == 0:
                delta = ev.angleDelta().y()
            if True :
                self.opts['distance'] *= 0.999**delta
            else :
                if (ev.modifiers() & Qt.KeyboardModifier.ControlModifier):
                    self.opts['fov'] *= 0.999**delta
                else:
                    self.opts['distance'] *= 0.999**delta
            self.update()
        #print(f'*wheelEvent {ev.buttons()} D:[{delta}]')

    def keyPressEvent(self, ev):
        ## Key_Shift, Key_Control, Key_Alt, 
        if self._clip_box != None: # self._clip_mode and 
            if ev.key() == Qt.Key.Key_Alt:
                print(f'ALT On - Move D:[{ev.key()}], {self._clip_key_mode == SRKEYMODE.NORMAL}')
                if self._clip_key_mode == SRKEYMODE.NORMAL :
                    self._clip_key_mode = SRKEYMODE.MOVE
            elif ev.key() == Qt.Key.Key_Control:
                print(f'Ctlr On - Resize D:[{ev.key()}], {self._clip_key_mode == SRKEYMODE.NORMAL}')
                if self._clip_key_mode == SRKEYMODE.NORMAL :
                    self._clip_key_mode = SRKEYMODE.RESIZE
            elif ev.key() == Qt.Key.Key_Shift:
                print(f'Shift On - Detail?  D:[{ev.key()}]')
                self._clip_key_detail_on = True
            else:
                #print(f'*keyPressEvent D:[{ev.key()}]')
                pass
            return

        if ev.key() in self.noRepeatKeys:
            ev.accept()
            if ev.isAutoRepeat():
                return
            self.keysPressed[ev.key()] = 1
            self.evalKeyState()
      
    def keyReleaseEvent(self, ev):
        if self._clip_box != None: # self._clip_mode and 
            if ev.key() == Qt.Key.Key_Alt:
                print(f'ALT Off - Move? D:[{ev.key()}], {self._clip_key_mode == SRKEYMODE.MOVE}')
                if self._clip_key_mode == SRKEYMODE.MOVE :
                    self._clip_key_mode = SRKEYMODE.NORMAL
            elif ev.key() == Qt.Key.Key_Control:
                print(f'Ctlr Off - Resize? D:[{ev.key()}, {self._clip_key_mode == SRKEYMODE.RESIZE}]')
                if self._clip_key_mode == SRKEYMODE.RESIZE :
                    self._clip_key_mode = SRKEYMODE.NORMAL
            elif ev.key() == Qt.Key.Key_Shift:
                print(f'Shift Off - Detail?  D:[{ev.key()}]')
                self._clip_key_detail_on = False
            else:
                if ev.key() in self._clip_keys:
                    if self._clip_key_mode == SRKEYMODE.MOVE :
                        self._clip_box.resetTransform()
                        if ev.key() == Qt.Key.Key_Left:
                            self._start_xyz[0] -= 0.5
                            self._end_xyz[0] -= 0.5
                        elif ev.key() == Qt.Key.Key_Right:
                            self._start_xyz[0] += 0.5
                            self._end_xyz[0] += 0.5
                        elif ev.key() == Qt.Key.Key_Up:
                            self._start_xyz[1] += 0.5
                            self._end_xyz[1] += 0.5
                        elif ev.key() == Qt.Key.Key_Down:
                            self._start_xyz[1] -= 0.5
                            self._end_xyz[1] -= 0.5
                        elif ev.key() == Qt.Key.Key_PageUp:
                            self._start_xyz[2] += 0.5
                            self._end_xyz[2] += 0.5
                        elif ev.key() == Qt.Key.Key_PageDown:
                            self._start_xyz[2] -= 0.5
                            self._end_xyz[2] -= 0.5
                        self._clip_box.setSize(self._end_xyz[0]-self._start_xyz[0], self._end_xyz[1]-self._start_xyz[1], self._end_xyz[2]-self._start_xyz[2])
                        self._clip_box.translate(self._start_xyz[0], self._start_xyz[1], self._start_xyz[2])
                        pass
                    elif self._clip_key_mode == SRKEYMODE.RESIZE :
                        #self._start_xyz
                        if ev.key() == Qt.Key.Key_Left:
                            self._end_xyz[0] -= 0.5
                        elif ev.key() == Qt.Key.Key_Right:
                            self._end_xyz[0] += 0.5
                        elif ev.key() == Qt.Key.Key_Up:
                            self._end_xyz[1] += 0.5
                        elif ev.key() == Qt.Key.Key_Down:
                            self._end_xyz[1] -= 0.5
                        elif ev.key() == Qt.Key.Key_PageUp:
                            self._end_xyz[2] += 0.5
                        elif ev.key() == Qt.Key.Key_PageDown:
                            self._end_xyz[2] -= 0.5
                        self._clip_box.setSize(self._end_xyz[0]-self._start_xyz[0], self._end_xyz[1]-self._start_xyz[1], self._end_xyz[2]-self._start_xyz[2])
                        pass
                elif ev.key() == Qt.Key.Key_Return :
                    self._select_region_cb.emit([self._start_xyz, self._end_xyz])
                    # self._clip_box.setSize(0.01,0.01,0.01)
                    # self._start_pos = lpos
                    # self._start_xyz = self.get_space_pos_from_view(self._start_pos.x(), self._start_pos.y())
                    # self._start_xyz[2] = 0.0
                    # self._clip_box.translate(self._start_xyz[0], self._start_xyz[1], self._start_xyz[2])

                #print(f'*keyReleaseEvent D:[{ev.key()}]')
            return

        if ev.key() in self.noRepeatKeys:
            ev.accept()
            if ev.isAutoRepeat():
                return
            try:
                del self.keysPressed[ev.key()]
            except KeyError:
                self.keysPressed = {}
            self.evalKeyState()


    def projectionMatrix(self, region=None):
        if region == None:
            region = (0, 0, self.deviceWidth(), self.deviceHeight())

        tr = QMatrix4x4()
        x0, y0, w, h = self.getViewport()
        dist = self.opts['distance']
        fov = self.opts['fov']
        nearClip = dist * 0.001
        farClip = dist * 1000.

        r = nearClip * np.tan(0.5 * np.radians(fov))
        t = r * h / w

        if False : #self._clip_mode:
            tr.ortho(x0-w/2, x0+(w/2), y0-h/2, y0+(h/2), 10, 100)
        else :
            ## Note that X0 and width in these equations must be the values used in viewport
            left  = r * ((region[0]-x0) * (2.0/w) - 1)
            right = r * ((region[0]+region[2]-x0) * (2.0/w) - 1)
            bottom = t * ((region[1]-y0) * (2.0/h) - 1)
            top    = t * ((region[1]+region[3]-y0) * (2.0/h) - 1)
            tr.frustum(left, right, bottom, top, nearClip, farClip)
        return tr

    # def setProjection(self, region=None):
    #     m = self.projectionMatrix(region)
    #     gl.glMatrixMode(gl.GL_PROJECTION)
    #     gl.glLoadMatrixf(m.data())

    # def paintGL(self, region=None, viewport=None, useItemNames=False):
    #     """
    #     viewport specifies the arguments to glViewport. If None, then we use self.opts['viewport']
    #     region specifies the sub-region of self.opts['viewport'] that should be rendered.
    #     Note that we may use viewport != self.opts['viewport'] when exporting.
    #     """
    #     if viewport is None:
    #         gl.glViewport(*self.getViewport())
    #     else:
    #         gl.glViewport(*viewport)
    #     self.setProjection(region=region)
    #     self.setModelview()
    #     bgcolor = self.opts['bgcolor']
    #     gl.glClearColor(*bgcolor)
    #     gl.glClear( gl.GL_DEPTH_BUFFER_BIT | gl.GL_COLOR_BUFFER_BIT )
    #     self.drawItemTree(useItemNames=useItemNames)


