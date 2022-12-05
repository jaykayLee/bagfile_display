import sys
import datetime
import time
import math
import struct
import os
import string
import PyQt5
from matplotlib import projections
import numpy as np
import multiprocessing as mp
import ctypes
from collections import deque
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal, QMutex, QMutexLocker, QPoint, QPointF
from PyQt5.QtGui import QPixmap, QImage, QMouseEvent, QMatrix4x4, QColor, QFont
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import OpenGL.GL as GL
import OpenGL.GLUT as GLUT
import OpenGL.GLU as GLU

####__all__ = ['SRTargetScatter']

class SRTargetScatter(gl.GLScatterPlotItem):
    """Draws points at a list of 3D positions."""
    
    def __init__(self, **kwds):
        super().__init__()
        glopts = kwds.pop('glOptions', 'additive')
        self.setGLOptions(glopts)
        self.pos = None
        self.size = 10
        self.color = [1.0,1.0,1.0,0.5]
        self.pxMode = True
        self.setData(**kwds)
        self.shader = None
    
    def setData(self, **kwds):
        """
        Update the data displayed by this item. All arguments are optional; 
        for example it is allowed to update spot positions while leaving 
        colors unchanged, etc.
        
        ====================  ==================================================
        **Arguments:**
        pos                   (N,3) array of floats specifying point locations.
        color                 (N,4) array of floats (0.0-1.0) specifying
                              spot colors OR a tuple of floats specifying
                              a single color for all spots.
        size                  (N,) array of floats specifying spot sizes or 
                              a single value to apply to all spots.
        pxMode                If True, spot sizes are expressed in pixels. 
                              Otherwise, they are expressed in item coordinates.
        ====================  ==================================================
        """
        args = ['pos', 'color', 'size', 'pxMode']
        for k in kwds.keys():
            if k not in args:
                raise Exception('Invalid keyword argument: %s (allowed arguments are %s)' % (k, str(args)))
            
        args.remove('pxMode')
        for arg in args:
            if arg in kwds:
                setattr(self, arg, kwds[arg])
                
        self.pxMode = kwds.get('pxMode', self.pxMode)
        self.update()

    def initializeGL(self):
        if self.shader is not None:
            return
        
        ## Generate texture for rendering points
        w = 64
        def genTexture(x,y):
            r = np.hypot((x-(w-1)/2.), (y-(w-1)/2.))
            return 255 * (w / 2 - pg.functions.clip_array(r, w / 2 - 1, w / 2))
        pData = np.empty((w, w, 4))
        pData[:] = 255
        pData[:,:,3] = np.fromfunction(genTexture, pData.shape[:2])
        pData = pData.astype(np.ubyte)
        
        if getattr(self, "pointTexture", None) is None:
            self.pointTexture = GL.glGenTextures(1)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glEnable(GL.GL_TEXTURE_2D)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.pointTexture)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA, pData.shape[0], pData.shape[1], 0, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, pData)
        
        self.shader = gl.shaders.getShaderProgram('pointSprite')
        
    #def setupGLState(self):
        #"""Prepare OpenGL state for drawing. This function is called immediately before painting."""
        ##glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  ## requires z-sorting to render properly.
        #glBlendFunc(GL_SRC_ALPHA, GL_ONE)
        #glEnable( GL_BLEND )
        #glEnable( GL_ALPHA_TEST )
        #glDisable( GL_DEPTH_TEST )
        
        ##glEnable( GL_POINT_SMOOTH )

        ##glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
        ##glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION, (0, 0, -1e-3))
        ##glPointParameterfv(GL_POINT_SIZE_MAX, (65500,))
        ##glPointParameterfv(GL_POINT_SIZE_MIN, (0,))
        
    def paint(self):
        if self.pos is None:
            return

        self.setupGLState()
        
        GL.glEnable(GL.GL_POINT_SPRITE)
        
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glEnable( GL.GL_TEXTURE_2D )
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.pointTexture)
    
        GL.glTexEnvi(GL.GL_POINT_SPRITE, GL.GL_COORD_REPLACE, GL.GL_TRUE)
        #glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE)    ## use texture color exactly
        #glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE )  ## texture modulates current color
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        
            
        with self.shader:
            #glUniform1i(self.shader.uniform('texture'), 0)  ## inform the shader which texture to use
            GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
            try:
                pos = self.pos
                #if pos.ndim > 2:
                    #pos = pos.reshape((-1, pos.shape[-1]))
                GL.glVertexPointerf(pos)
            
                if isinstance(self.color, np.ndarray):
                    GL.glEnableClientState(GL.GL_COLOR_ARRAY)
                    GL.glColorPointerf(self.color)
                else:
                    color = self.color
                    if isinstance(color, PyQt5.QtGui.QColor):
                        color = color.getRgbF()
                    GL.glColor4f(*color)
                
                if not self.pxMode or isinstance(self.size, np.ndarray):
                    GL.glEnableClientState(GL.GL_NORMAL_ARRAY)
                    norm = np.empty(pos.shape)
                    if self.pxMode:
                        norm[...,0] = self.size
                    else:
                        gpos = self.mapToView(pos.transpose()).transpose()
                        if self.view():
                            pxSize = self.view().pixelSize(gpos)
                        else:
                            pxSize = self.parentItem().view().pixelSize(gpos)
                        norm[...,0] = self.size / pxSize
        
                    GL.glNormalPointerf(norm)
                else:
                    GL.glNormal3f(self.size, 0, 0)  ## vertex shader uses norm.x to determine point size
                    #glPointSize(self.size)
                GL.glDrawArrays(GL.GL_POINTS, 0, int(pos.size / pos.shape[-1]))
            finally:
                GL.glDisableClientState(GL.GL_NORMAL_ARRAY)
                GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
                GL.glDisableClientState(GL.GL_COLOR_ARRAY)
                #posVBO.unbind()
                ##fixes #145
                GL.glDisable( GL.GL_TEXTURE_2D )
                                
        #for i in range(len(self.pos)):
            #pos = self.pos[i]
            
            #if isinstance(self.color, np.ndarray):
                #color = self.color[i]
            #else:
                #color = self.color
            #if isinstance(self.color, QtGui.QColor):
                #color = fn.glColor(self.color)
                
            #if isinstance(self.size, np.ndarray):
                #size = self.size[i]
            #else:
                #size = self.size
                
            #pxSize = self.view().pixelSize(QtGui.QVector3D(*pos))
            
            #glPointSize(size / pxSize)
            #glBegin( GL_POINTS )
            #glColor4f(*color)  # x is blue
            ##glNormal3f(size, 0, 0)
            #glVertex3f(*pos)
            #glEnd()



#SRTargetScatter
class SRTargetBox(gl.GLBoxItem):
    _label = ""
    #_label_item = None
    _position = None
    _temp_v = 1
    """
    **Bases:** :class:`GLGraphicsItem <pyqtgraph.opengl.GLGraphicsItem>`
    
    Displays a wire-frame box.
    """
    def __init__(self, size=None, color=None, glOptions='translucent'):
        gl.GLBoxItem.__init__(self)
        if size is None:
            size = PyQt5.QtGui.QVector3D(1,1,1)
        self.setSize(size=size)
        if color is None:
            color = (1,1,1,0.5)
        self.setColor(color)
        self.setGLOptions(glOptions)
        self._position = [0,0,0]
        self._transform = pg.Transform3D()
        #self._label_item = gl.GLTextItem(pos=(0.0, 0.0, 0.0), text=self._label)
        #self.setParentItem(self._label_item)
        self._label_color = Qt.GlobalColor.white
        self._label_font = QFont('Helvetica', 16)

    
    def set_label(self, label):
        self._label = label
        #self._label_item.setData(text=self._label)

    def label(self):
        return self._label

    def set_position(self, x, y, z):
        self._position = [x,y,z]

        self._transform = pg.Transform3D()
        #self._transform.translate(x-self.__size[0]/2 , y-self.__size[1]/2, z-self.__size[2]/2)
        self._transform.translate(x-self.__size[0]/2 , y-self.__size[1]/2, z)

        #self._label_item.setData(pos=self._position)
        # self.update()

    def position(self):
        return self._position

    def transform(self):
        """Return this item's transform object."""
        return self._transform

    def setSize(self, x=None, y=None, z=None, size=None):
        """
        Set the size of the box (in its local coordinate system; this does not affect the transform)
        Arguments can be x,y,z or size=QVector3D().
        """
        if size is not None:
            x = size.x()
            y = size.y()
            z = size.z()
        self.__size = [x,y,z]
        self.update()
        
    def size(self):
        return self.__size[:]
    
    def setColor(self, color):
        """Set the color of the box. Arguments are the same as those accepted by functions.mkColor()"""
        self.__color = color
        
    def color(self):
        return self.__color
    
    def paint(self):
        #glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        #glEnable( GL_BLEND )
        #glEnable( GL_ALPHA_TEST )
        ##glAlphaFunc( GL_ALWAYS,0.5 )
        #glEnable( GL_POINT_SMOOTH )
        #glDisable( GL_DEPTH_TEST )
        self.setupGLState()
        
        GL.glBegin( GL.GL_LINES )
        
        GL.glColor4f(*self.color())
        x,y,z = self.size()
        GL.glVertex3f(0, 0, 0)
        GL.glVertex3f(0, 0, z)
        GL.glVertex3f(x, 0, 0)
        GL.glVertex3f(x, 0, z)
        GL.glVertex3f(0, y, 0)
        GL.glVertex3f(0, y, z)
        GL.glVertex3f(x, y, 0)
        GL.glVertex3f(x, y, z)

        GL.glVertex3f(0, 0, 0)
        GL.glVertex3f(0, y, 0)
        GL.glVertex3f(x, 0, 0)
        GL.glVertex3f(x, y, 0)
        GL.glVertex3f(0, 0, z)
        GL.glVertex3f(0, y, z)
        GL.glVertex3f(x, 0, z)
        GL.glVertex3f(x, y, z)
        
        GL.glVertex3f(0, 0, 0)
        GL.glVertex3f(x, 0, 0)
        GL.glVertex3f(0, y, 0)
        GL.glVertex3f(x, y, 0)
        GL.glVertex3f(0, 0, z)
        GL.glVertex3f(x, 0, z)
        GL.glVertex3f(0, y, z)
        GL.glVertex3f(x, y, z)
        
        GL.glEnd()
        self.__paint_label()

    def __paint_label(self):
        if len(self._label) < 1:
            return
        self.setupGLState()

        modelview = GL.glGetDoublev(GL.GL_MODELVIEW_MATRIX)
        projection = GL.glGetDoublev(GL.GL_PROJECTION_MATRIX)

        viewport = [0, 0, self.view().width(), self.view().height()]
        # text_pos = self._project_label((0,0,0), modelview, projection, viewport)
        ## self._transform.translate(x-self.__size[0]/2 , y-self.__size[1]/2, z)
        text_pos = self._project_label((self.__size[0]/2, self.__size[1]/2, self.__size[2])
                    , modelview, projection, viewport)

        text_pos.setY(viewport[3] - text_pos.y())

        painter = PyQt5.QtGui.QPainter(self.view())
        painter.setPen(self._label_color)
        painter.setFont(self._label_font)
        painter.setRenderHints(PyQt5.QtGui.QPainter.RenderHint.Antialiasing | PyQt5.QtGui.QPainter.RenderHint.TextAntialiasing)
        painter.drawText(text_pos, self._label)
        painter.end()

    def _project_label(self, obj_pos, modelview, projection, viewport):
        obj_vec = np.append(np.array(obj_pos), [1.0])

        view_vec = np.matmul(modelview.T, obj_vec)
        proj_vec = np.matmul(projection.T, view_vec)

        if proj_vec[3] == 0.0:
            return PyQt5.QtCore.QPointF(0, 0)

        proj_vec[0:3] /= proj_vec[3]

        return PyQt5.QtCore.QPointF(
            viewport[0] + (1.0 + proj_vec[0]) * viewport[2] / 2,
            viewport[1] + (1.0 + proj_vec[1]) * viewport[3] / 2
        )



if False :
    def TestFunc():
        ## --------------------------
        ## 8 vertex
        cube_vertexs = np.array([[ .5, .5, .5], 
                                [-.5, .5, .5], 
                                [-.5,-.5, .5], 
                                [ .5,-.5, .5], 
                                [ .5,-.5,-.5], 
                                [ .5, .5,-.5], 
                                [-.5, .5,-.5], 
                                [-.5,-.5,-.5]], dtype=float)
        ## 8 Normal vectors
        cube_normals = np.array([ [ 0.0, 0.0, 1.0],
                                [ 1.0, 0.0, 0.0],
                                [ 0.0, 1.0, 0.0],
                                [-1.0, 0.0, 0.0],
                                [ 0.0,-1.0, 0.0],
                                [ 0.0, 0.0,-1.0]], dtype=float )

        # 6 vertex indices
        cube_indices = np.array([   0,1,2,3,
                                    0,3,4,5,
                                    0,5,6,1,
                                    1,6,7,2,
                                    7,4,3,2,
                                    4,7,6,5  ], dtype=int )

        CUBE_NUM_VERT = len(cube_vertexs)
        CUBE_NUM_FACES = len(cube_normals)
        CUBE_NUM_EDGE_PER_FACE  = 4
        CUBE_VERT_PER_OBJ      = CUBE_NUM_FACES*CUBE_NUM_EDGE_PER_FACE
        CUBE_VERT_ELEM_PER_OBJ = CUBE_VERT_PER_OBJ*3
        CUBE_VERT_PER_OBJ_TRI  = CUBE_VERT_PER_OBJ+CUBE_NUM_FACES*2 ## 2 extra edges per face when drawing quads as triangles

        ## fghDrawGeometryWire(cube_vertexs, cube_normals, CUBE_VERT_PER_OBJ, NULL,CUBE_NUM_FACES, CUBE_NUM_EDGE_PER_FACE,GL_LINE_LOOP, NULL,0,0);
        ## fghDrawGeometryWire20(cube_vertexs, cube_normals, CUBE_VERT_PER_OBJ, NULL, CUBE_NUM_FACES, CUBE_NUM_EDGE_PER_FACE, GL.GL_LINE_LOOP, NULL, 0, 0, attribute_v_coord, attribute_v_normal);


class SRTargetList():
    
    def __init__(self, max_count=100):
        super().__init__()
        self._max_count = max_count
        self._targetbox_list = []
        for i in range(max_count):
            box = SRTargetBox()
            box.hide()
            self._targetbox_list.append(box)

    def add_to_parent(self, w):
        for i in range(len(self._targetbox_list)):
            w.addItem(self._targetbox_list[i])
    
    def reset(self):
        for i in range(len(self._targetbox_list)):
            self._targetbox_list[i].hide()
            self._targetbox_list[i].set_label("")
            self._targetbox_list[i].set_position(0,0,0)

    #def set_data(self, pos=[(0,0,0)], size=[(1,1,1)], color=[(1,1,1,0.5)], label=[""]):
    def set_data(self, pos, size, color, label=""):
        index : int = 0
        tcount = len(pos)
        if isinstance(size, np.ndarray):
            arr_size = size
        else :
            arr_size = np.zeros((tcount, 3))
            arr_size[:,:] = size

        if isinstance(color, np.ndarray):
            arr_color = color
        else :
            arr_color = np.zeros((tcount, 4))
            arr_color[:,] = color
        
        for index in range(len(pos)):
            ##self._targetbox_list[index].set_label( label[index]  if isinstance(label, list)and else label )
            if isinstance(label, str) :
                self._targetbox_list[index].set_label( label )
            else :
                self._targetbox_list[index].set_label( label[index]  if (isinstance(label, list) and (index < len(label))) else "" )
            self._targetbox_list[index].setColor(arr_color[index])
            self._targetbox_list[index].set_position(pos[index, 0], pos[index, 1], pos[index, 2] )
            if self._targetbox_list[index].visible() == False :
                self._targetbox_list[index].show()
            #self._targetbox_list[index].update()
        index += 1
        for index in range(index, len(self._targetbox_list)):
            self._targetbox_list[index].hide()
        


if __name__ == '__main__':
    app = pg.mkQApp("GLScatterPlotItem Example")
    w = gl.GLViewWidget()
    w.show()
    w.setWindowTitle('pyqtgraph example: GLScatterPlotItem')
    w.setCameraPosition(distance=20)
    verstr = GL.glGetString(GL.GL_VERSION)

    g = gl.GLGridItem()
    w.addItem(g)


    ##
    ##  First example is a set of points with pxMode=False
    ##  These demonstrate the ability to have points with real size down to a very small scale 
    ## 
    pos = np.empty((53, 3))
    size = np.empty((53))
    color = np.empty((53, 4))
    pos[0] = (1,0,0); size[0] = 0.5;   color[0] = (1.0, 0.0, 0.0, 0.5)
    pos[1] = (0,1,0); size[1] = 0.2;   color[1] = (0.0, 0.0, 1.0, 0.5)
    pos[2] = (0,0,1); size[2] = 2./3.; color[2] = (0.0, 1.0, 0.0, 0.5)

    z = 0.5
    d = 6.0
    for i in range(3,53):
        pos[i] = (0,0,z)
        size[i] = 2./d
        color[i] = (0.0, 1.0, 0.0, 0.5)
        z *= 0.5
        d *= 2.0
        
    sp1 = SRTargetScatter(pos=pos, size=size, color=color, pxMode=False)
    sp1.translate(5,5,0)
    w.addItem(sp1)


    #
    #  Second example shows a volume of points with rapidly updating color
    #  and pxMode=True
    #

    pos = np.random.random(size=(100000,3))
    pos *= [10,-10,10]
    pos[0] = (0,0,0)
    color = np.ones((pos.shape[0], 4))
    d2 = (pos**2).sum(axis=1)**0.5
    size = np.random.random(size=pos.shape[0])*10
    sp2 = SRTargetScatter(pos=pos, color=(1,1,1,1), size=size)
    phase = 0.

    w.addItem(sp2)


    #
    #  Third example shows a grid of points with rapidly updating position
    #  and pxMode = False
    #

    pos3 = np.zeros((100,100,3))
    pos3[:,:,:2] = np.mgrid[:100, :100].transpose(1,2,0) * [-0.1,0.1]
    pos3 = pos3.reshape(10000,3)
    d3 = (pos3**2).sum(axis=1)**0.5

    sp3 = SRTargetScatter(pos=pos3, color=(1,1,1,.3), size=0.1, pxMode=False)

    w.addItem(sp3)

    tbox = SRTargetBox()
    w.addItem(tbox)
    tbox.set_label("hello")
    tbox.set_position(0, 0, 0)
    tbox._temp_v =1

    def update():
        pos = tbox.position()
        if pos[1] > 10 :
            tbox._temp_v = -1
        elif pos[1] < -10 :
            tbox._temp_v = 1

        if tbox._temp_v == 1 :
            tbox.set_position(pos[0], pos[1]+0.5, pos[2] )
        elif tbox._temp_v == -1 :
            tbox.set_position(pos[0], pos[1]-0.5, pos[2] )
        tbox.update()
        ## update volume colors
        global phase, sp2, d2
        s = -np.cos(d2*2+phase)
        color = np.empty((len(d2),4), dtype=np.float32)
        color[:,3] = pg.functions.clip_array(s * 0.1, 0., 1.)
        color[:,0] = pg.functions.clip_array(s * 3.0, 0., 1.)
        color[:,1] = pg.functions.clip_array(s * 1.0, 0., 1.)
        color[:,2] = pg.functions.clip_array(s ** 3, 0., 1.)
        sp2.setData(color=color)
        phase -= 0.1
        
        ## update surface positions and colors
        global sp3, d3, pos3
        z = -np.cos(d3*2+phase)
        pos3[:,2] = z
        color = np.empty((len(d3),4), dtype=np.float32)
        color[:,3] = 0.3
        color[:,0] = np.clip(z * 3.0, 0, 1)
        color[:,1] = np.clip(z * 1.0, 0, 1)
        color[:,2] = np.clip(z ** 3, 0, 1)
        sp3.setData(pos=pos3, color=color)
        if pos[1] < 0  and sp3.visible():
            sp3.hide()
        elif pos[1] > 0  and sp3.visible()==False:
            sp3.show()
            
        
    t = QTimer()
    t.timeout.connect(update)
    t.start(100)

    pg.exec()
