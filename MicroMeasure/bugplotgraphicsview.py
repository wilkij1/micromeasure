'''
Created on Feb 3, 2022

@author: jeffr
'''
#
# If CAM_DISPLAY_MODE == 
#  "direct" then video is sent to QGraphicsWidget overlaying the 
#    BugPlotGraphicsView
#  "drawBackground" sends to a QVideoSink and then rendered in drawBackground
#  "scene" renders to a QVideoGraphicsItem that is positioned within the 
#    BugPlotGraphicsScene. The graphics item is repositioned whenever the 
#    microscope position is updated so that it is always centered on the viewport 
# TODO: 7/17/2022 the CAM_DISPLAY_MODE == "direct" version does not display overlaid graphics
# TODO: 7/18/2022 need to implement the "scene" option to see if it works.
#
CAM_DISPLAY_MODE = "drawBackground"
# set DRAW_ORIGIN to True to draw a red circle at the origin position
DRAW_ORIGIN = True

import logging
log = logging.getLogger()

import numpy as np
from numpy import (array, atleast_2d, diff, hstack, inf,  # @UnusedImport
    max, min, ndarray, prod, roll, sum, zeros)  # @UnusedImport
from numpy.linalg import norm  # @UnusedImport

from PyQt6.QtCore import (
    Qt, 
    QPointF, 
    QRectF, 
    QModelIndex,
    pyqtSlot, 
    pyqtSignal,  # @UnusedImport
    )
from PyQt6.QtGui import (
    QBrush, 
    QPainter,  
    QPainterPath, 
    QPen, 
    QTransform,  # @UnusedImport
    QPixmap,  # @UnusedImport
    )
from PyQt6.QtMultimediaWidgets import (
    QGraphicsVideoItem,  # @UnusedImport
    )
from PyQt6.QtWidgets import (
    QFrame, 
    QGraphicsScene, 
    QGraphicsView,
    QLabel, 
    )

if CAM_DISPLAY_MODE == "direct":
    from PyQt6.QtMultimediaWidgets import (
        QVideoWidget, 
        )

from microscope_interface import (
    W_TRAVEL, 
    H_TRAVEL, 
    )

def um2mm(x):
    return np.array(x/1000).astype(float)

def mm2um(x):
    return np.array(x*1000).astype(int)

mode_names = 'Point Line Multiline Area'.split()
MODE_POINTS, MODE_LINEAR, MODE_MULTILINE, MODE_AREA = range(len(mode_names))

KEY_LEFT, KEY_RIGHT = range(2)
PT_RAD = mm2um(0.05) #0.015
PEN_WIDTH = mm2um(0.02) #0.005
    
# ==============================================================================
# Tracking modes
# ==============================================================================
TrackRowRole = Qt.ItemDataRole.UserRole

if CAM_DISPLAY_MODE == "direct":
    class BugPlotViewBackground(QVideoWidget):
        '''render the microscope image onto the graphics scene'''
        pass
    
        def __init__(self, parent, *pargs, **kwargs):
            super().__init__(*pargs, **kwargs)

        def paintEvent(self, event):
            log.debug(f"BugPlotViewBackground.paintEvent() starts {event}")
            pt = QPainter()

            pen = QPen(Qt.GlobalColor.red)
            pen.setWidthF(25)
            pt.setPen(pen)
         
            rt = event.rect()
            center = rt.center()
         
            pt.drawRect(center.x()-400, center.y()-20, 800, 40)
         

    class BugPlotViewCrosshairs(QLabel):
        '''draw crosshairs on top of everything else'''
    
        def __init__(self, parent, *pargs, **kwargs):
            super().__init__(*pargs, **kwargs)

            self.setStyleSheet("background-color: transparent")
        def paintEvent(self, event):
    
            log.info(f"BugPlotViewCrosshairs.paintEvent")
    
            pt = QPainter()
            pt.begin(self)
            pen = QPen(Qt.GlobalColor.black)
            pen.setWidthF(3)
            pt.setPen(pen)
    
            rt = event.rect()
            center = rt.center()
            # horizontal...
            pt.drawLine(rt.x(), center.y(), rt.x()+rt.width(), center.y())
            # ...and vertical crosshairs
            pt.drawLine(center.x(), rt.y(), center.x(), rt.y()+rt.height())
    
            pt.end()

class BugPlotGraphicsView(QGraphicsView):
    '''
    classdocs
    '''

    def __init__(self, parent, *pargs, **kwargs):
        '''
        Constructor
        '''
        super().__init__(parent, *pargs, **kwargs)
        
        log.info(f"instantiate BugPlotGraphicsView")
        self._cachedImage = None
        
        self.setBackgroundBrush(QBrush(Qt.BrushStyle.NoBrush))
        self.setForegroundBrush(QBrush(Qt.BrushStyle.NoBrush))
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
#         self.setBackgroundBrush(QBrush(QColor(230, 230, 250)))
        self.setFrameShape(QFrame.Shape.NoFrame)
#         self.scale(1., 1.)
      
        sceneRect = QRectF(-mm2um(W_TRAVEL), -mm2um(H_TRAVEL), 
                           mm2um(2*W_TRAVEL), mm2um(2*H_TRAVEL))
        scene = BugPlotScene(sceneRect, self)
        self.setScene(scene)
        self.setBackgroundBrush(QBrush(Qt.BrushStyle.NoBrush))
        self.setStyleSheet("background: transparent")
        self._videoSink = None
        self._frame = None
        self._frame_epoch = None
        
    if CAM_DISPLAY_MODE == "drawBackground":
        def setVideoSink(self, sink):
            self._videoSink = sink
            
        @pyqtSlot()
        def video_frame_changed(self, *pargs, **kwargs):
    
            frame = self._videoSink.videoFrame()
            if self._frame_epoch:
                t_start = (frame.startTime() - self._frame_epoch)/1e6
                t_end = (frame.endTime() - self._frame_epoch)/1e6 
                log.info(f"frame start {t_start:.6f}, end {t_end:.6f}")
            else:
                self._frame_epoch = frame.startTime()
                log.info(f"frame epoch set to {self._frame_epoch/1e6:.6f}")
                
            self._frame = frame
        
        def drawBackground(self, painter, rect):
            log.debug(f"BugPlotGraphicsView.drawBackground({str(painter)}, {(str(rect))}) starts")
        
            if self._frame:
                painter.save()
                image = self._frame.toImage()
                image2 = image.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatioByExpanding)
                log.debug(f"rect.size = {rect.size()}, size1 = {image.size()}, size2 = {image2.size()}")
                painter.drawImage(rect, image2)
                #
                # the following code attempts to map into a QPixmap and
                #  be sensitive to the viewport size (above code is stretched
                #  and distorted on resize
                #
                # pixmap = QPixmap(image)
                # vpr = self.viewport().rect()
                # log.debug(f"pixmap size = {pixmap.size()}, rect = {pixmap.rect()}, {type(vpr)}")
                # try:
                #     painter.drawPixmap(vpr, pixmap, rect)
                # except Exception as e:
                #     log.debug(f"{e}")
                # self._frame = None
                painter.restore()
            else:
                log.debug(f"self._frame is None")
            log.debug(f"BugPlotGraphicsView.drawBackground() ends")
        
        def drawForeground(self, painter, rect):
                    
            log.debug(f"BugPlotGraphicsView.drawForeground({painter}, {rect})")
    
            painter.save()
            
            pen = QPen(Qt.GlobalColor.red)
            pen.setWidthF(25)
            painter.setPen(pen)
            
            center = rect.center()
            # horizontal...
            painter.drawLine(rect.x(), center.y(), rect.x()+rect.width(), center.y())
            # ...and vertical crosshairs
            painter.drawLine(center.x(), rect.y(), center.x(), rect.y()+rect.height())
            
            painter.restore()
            
    def resizeEvent(self, event):
        
        log.info(f"BugPlotGraphicsView.resizeEvent({event.size()})")
        
        # find the app window
        obj = self
        while obj.parent() is not None:
            obj = obj.parent()
        appw = obj

        um = appw.FOV()
        vp = self.viewport()
        vp_w, vp_h = vp.width(), vp.height()
        um_w, um_h = um.width(), um.height()
        
        ratio_w = vp_w/um_w
        ratio_h = vp_h/um_h
        
        ratio = ratio_w if ratio_w >= ratio_h else ratio_h
        log.info(f"BugPlotGraphicsView.resizeEvent -> VP {vp_h}x{vp_w}, {um_h}x{um_w}, ratio {ratio}")
        xf = QTransform(ratio, 0., 0., ratio, 0., 0.)
        self.setTransform(xf)

        # DEBUG DEBUG DEBUG
        xf = self.transform()
        vs_label = appw.statusViewScaleLabel
        vs_label.setText(f"scale=({xf.m11():.3f},{xf.m22():.3f})")
        log.debug(f"BugPlotGraphicsView.resizeEvent() ends")
        
    def setMagnification(self, microscope, magnification):
        '''set the view's scale when the''' 
        w_fov, h_fov = self.parent().FOV()
        vp = self.viewport().rect()
        w_ratio = vp.width()/w_fov
        h_ratio = vp.height()/h_fov
        
        ratio = w_ratio if w_ratio >= h_ratio else h_ratio
        self.scale(ratio, ratio)
        
    def canonicalTransform(self, xform=None):
        '''return Qt's transform as a numpy array in the more common ordering'''
        
        anames = "m11 m21 m31 m12 m22 m32 m13 m23 m33".split()
        if xform is None:
            xform = self.transform()
        return np.array([getattr(xform, aa)() for aa in anames]).reshape(3,3)
    
class BugPlotScene(QGraphicsScene):
    
    items = {}
    xmin, xmax, ymin, ymax = (inf, -inf, inf, -inf)

    def __init__(self, *pargs, **kwargs):
        super().__init__(*pargs, **kwargs)

        self._model = None
        
        self._background = None
        self.setBackgroundBrush(QBrush(Qt.BrushStyle.NoBrush))
        self.setForegroundBrush(QBrush(Qt.BrushStyle.NoBrush))
        self.x, self.y = 0., 0.
        
        if len(pargs) == 1:
            sr = None
            view = pargs[0]
        else:
            sr = pargs[0]
            view = pargs[1]
        log.info(f"BugPlotScene.__init__({sr}, {view})")
        self._scope_frame = None
        self._debug_extras = []
        self._last_frame = None
        
        # add a circle around the origin
        if DRAW_ORIGIN:
            pen = QPen(Qt.GlobalColor.red)
            pen.setWidth(10)
            brush = QBrush(Qt.GlobalColor.black, Qt.BrushStyle.NoBrush)
            self.addEllipse(-25, 25, 50, 50, pen, brush)

    def updatePosition(self, xy, fov):
        log.debug(f"BugPlotScene.updatePosition frame {xy.x(), xy.y()}")

        # while self._debug_extras:
        #     oo = self._debug_extras.pop()
        #     self.removeItem(oo)
            
        center = QPointF(fov.x()+fov.width()/2, fov.y()+fov.height()/2)

        # # draw origin
        # pen = QPen(Qt.GlobalColor.red)
        # pen.setWidth(0)
        # brush = QBrush(Qt.GlobalColor.red, Qt.BrushStyle.SolidPattern)
        # ellipse_size = fov.width() * 0.0025
        # self._debug_extras.append(
        #     self.addEllipse(-ellipse_size/2, -ellipse_size/2, 
        #                     ellipse_size, ellipse_size, 
        #                     pen, brush))

        self.parent().centerOn(center)
        
        self.update()
            
        log.debug(f"BugPlotScene.updateImage ends")

    def setTableModel(self, tableModel):
        
        log.info("BugPlotScene.setTableModel() starts")
        self._model = tableModel
        tableModel.dataChanged.connect(self.dataChanged)
        
        nrows = self._model.rowCount()
        for rx in range(nrows):
            self._createPath(rx)

        log.info("BugPlotScene.setTableModel() ends")
                
    def drawDebugPath(self):
        '''draw a rectangle for scaling checks. Should be 1/2 width and 1/2 height
            of the minimum size window'''
                        
        if self._debug_path:
            self.removeItem(self._debug_path)
            self._debug_path = None
            
        rect = self.sceneRect()
        xmin, ymin, ww, hh = rect.getRect()
        log.info(f"BugPlotScene.drawDebugPath(({xmin},{ymin},{ww},{hh}))")

        pen = QPen()
        pen.setWidthF(PEN_WIDTH*5)
        pen.setStyle(Qt.PenStyle.SolidLine)
        path = QPainterPath()
        path.addRect(QRectF(0.0, 0.0, 1.0, 1.00))
        self._debug_path = self.addPath(path, pen)
        
    @pyqtSlot(QModelIndex, QModelIndex)
    def dataChanged(self, index0, index1):
        '''slot triggered when table's data changes and needs to be redrawn'''
        
        log.info(f"BugPlotScene.dataChanged({index0}, {index1})")
        row0, row1 = index0.row(), index1.row()
        for rx in range(row0, row1+1):
            self._createPath(rx)
        
        pass
            
    def _createPath(self, row):

        log.info(f"BugPlotScene._createPath({row})")
        inx = self._model.createIndex(row, 0)
        qv = self._model.data(inx, TrackRowRole)
        _, mode, _, points, sn = qv.value()  # @UnusedVariable
        # convert coordinate points from mm to um
        points = mm2um(points)
        
        pen = QPen()
        pen.setWidthF(PEN_WIDTH)
        
        npoints = points.shape[1]
        path = QPainterPath()
        if npoints > 0:
            so = []
            for iy in range(npoints):
                x, y = points[:,iy]
                so.append(f'({x:.3f},{y:.3f})')
                path.addEllipse(x-PT_RAD, y-PT_RAD, 2*PT_RAD, 2*PT_RAD)
                
#             print(f'R{sn}->' + ",".join(so))
            for iy, iz in zip(range(npoints-1),range(1,npoints)):
                x0, y0 = points[0,iy], points[1,iy]
                x1, y1 = points[0,iz], points[1,iz]
                if iy == 0:
                    path.moveTo(x0, y0)
                path.lineTo(x1, y1)

                self.xmin = min((self.xmin, x0, x1))
                self.xmax = max((self.xmax, x0, x1))
                self.ymin = min((self.ymin, y0, y1))
                self.ymax = max((self.ymax, y0, y1))

            if mode == MODE_AREA:
                path.closeSubpath()
        
            # update the list of scene items
            try:
                old_item = self.items[row]
                self.removeItem(old_item)
            except KeyError:
                pass
            pen = QPen()
            pen.setWidthF(PEN_WIDTH)
            item = self.addPath(path, pen)
            self.items[row] = item

