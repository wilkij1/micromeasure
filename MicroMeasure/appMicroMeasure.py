FORCE_DEBUG = True

import logging
log = logging.getLogger()

import numpy as np
from numpy import (array, atleast_2d, diff, hstack, inf,  # @UnusedImport
	max, min, ndarray, prod, roll, sum, zeros)  # @UnusedImport
from numpy.linalg import norm
import sys

from PyQt6.QtWidgets import (
	QApplication, 
	QLabel, 
	QMainWindow, 
	QSizePolicy, 
	)
from PyQt6.QtCore import (
	Qt, 
	QAbstractTableModel, 
	QModelIndex, 
	QObject, 
	QPoint, 
	QPointF,
	QSize, 
	QVariant, 
	pyqtSlot, 
# 	pyqtSignal, 
	)
from PyQt6.QtGui import (
	QActionGroup, 
	QFont, 
 	QPainter,  # @UnusedImport
	QPen,  # @UnusedImport
 	)
from PyQt6.QtCore import (
 	QRectF,
	)
from PyQt6.QtMultimedia import (
	QMediaCaptureSession, 
	QVideoSink, 
	) 

from bugplotgraphicsview import CAM_DISPLAY_MODE
if CAM_DISPLAY_MODE == "direct":
	from bugplotgraphicsview import BugPlotViewBackground, BugPlotViewCrosshairs
from MicroMeasure import Ui_MicroMeasure
#from microscope_interface import DebugMicroscopeInterface as MicroscopeInterface
from microscope_interface import MicroscopeInterface
from microscope_interface import (
	MODE_HW, MODE_HW_DEBUG, MODE_SW_DEBUG,)
from bugplotgraphicsview import (
	KEY_LEFT, KEY_RIGHT, 
	mode_names, 
	MODE_POINTS, MODE_LINEAR, MODE_MULTILINE, MODE_AREA, 
	)
import time

CAMERA_UPS = 10
# DEFAULT_MAGNIFICATION = 40.0
# MIN_MAGNIFICATION = 20.0
# MAX_MAGNIFICATION = 500.0

# UNITY_MAG_PX_PER_MM = 2.250  # value for 1280x1024, 15 FPS
# UNITY_MAG_PX_PER_MM = 1.280  # value for 640x480, 30 FPS
# W_CAMERA_PX = 1280
# H_CAMERA_PX = 720
# CAMERA_UPS = 15	# camera update interval
# W_CAMERA_PX = 640
# H_CAMERA_PX = 480
# CAMERA_UPS = 30	# camera update interval

# ==============================================================================
# Tracking modes
# ==============================================================================
TrackRowRole = Qt.ItemDataRole.UserRole

ST_OPEN, ST_CLOSED = range(2)

def um2mm(x):
	return np.array(x/1000).astype(float)

def mm2um(x):
	return np.array(x*1000).astype(int)

class AppWindow(QMainWindow):

	def __init__(self):

		super().__init__()

		self.microPosition = QPointF(0., 0.)
		self.deltaUpdates = np.zeros((10,)) * np.nan
		self.lastUpdate = time.time()
		
		# connect to the microscope
		log.info(f"instantiate MicroscopeInterface with 0.050 poll interval")
		self.microscope = MicroscopeInterface()
		
		# setup from Designer definition
		log.info(f"instantiate and initialize Ui_MicroMeasure")
		self.ui = Ui_MicroMeasure()
		self.ui.setupUi(self)

		if CAM_DISPLAY_MODE == "direct":
			self._bugPlotViewBackground = BugPlotViewBackground(self.ui.centralwidget)
			sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
			sizePolicy.setHorizontalStretch(0)
			sizePolicy.setVerticalStretch(4)
			sizePolicy.setHeightForWidth(self._bugPlotViewBackground.sizePolicy().hasHeightForWidth())
			self._bugPlotViewBackground.setSizePolicy(sizePolicy)
			self._bugPlotViewBackground.setMinimumSize(QSize(640, 480))
			self._bugPlotViewBackground.setToolTip("")
			self._bugPlotViewBackground.setObjectName("bugPlotViewBackground")
			self.ui.gridLayout.addWidget(self._bugPlotViewBackground, 1, 0, 1, 1)
			
			self._bugPlotViewCrosshairs= BugPlotViewCrosshairs(self.ui.centralwidget)
			sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
			sizePolicy.setHorizontalStretch(0)
			sizePolicy.setVerticalStretch(4)
			sizePolicy.setHeightForWidth(self._bugPlotViewBackground.sizePolicy().hasHeightForWidth())
			self._bugPlotViewBackground.setSizePolicy(sizePolicy)
			self._bugPlotViewBackground.setMinimumSize(QSize(640, 480))
			self._bugPlotViewBackground.setToolTip("")
			self._bugPlotViewBackground.setObjectName("bugPlotViewCrosshairs")
			self.ui.gridLayout.addWidget(self._bugPlotViewCrosshairs, 1, 0, 1, 1)
		elif CAM_DISPLAY_MODE == "drawBackground":
			# create QVideoSink to retrieve a frame and
			#  QMediaCaptureSession to link camera to sink
			vs = QVideoSink()
			vs.videoFrameChanged.connect(self.ui.bugPlotGraphicsView.video_frame_changed)
			self._videoSink = vs

		captureSession = QMediaCaptureSession()
		self._captureSession = captureSession

		captureSession.setCamera(self.microscope._camera)
		if CAM_DISPLAY_MODE == "direct":
			captureSession.setVideoOutput(self._bugPlotViewBackground)
		else:
			self.ui.bugPlotGraphicsView.setVideoSink(vs)
			captureSession.setVideoOutput(self._videoSink)

		# setup the TrackObservationsModel
		log.info(f"instantiate TrackObservationsTableModel")
		self.tableModel = TrackObservationsTableModel(mode=MODE_LINEAR)
		self.ui.tableView.setModel(self.tableModel)
		self.tableModel.view = self.ui.tableView
		self.ui.tableView.resizeColumnsToContents()
		self.ui.tableView.horizontalHeader().setStretchLastSection(True)

		# setup 
		scene = self.ui.bugPlotGraphicsView.scene()
		self._bugScene = scene
		scene.setTableModel(self.tableModel)
		
		# setup menu item handlers
		log.info(f"connecting UI slots")
		self.ui.actionNew.triggered.connect(self.actionNew)
		self.ui.actionExit.triggered.connect(self.actionExit)

		mode_group = QActionGroup(self.ui.menubar)
		mode_group.addAction(self.ui.actionPoint)
		mode_group.addAction(self.ui.actionLine)
		mode_group.addAction(self.ui.actionMultiline)
		mode_group.addAction(self.ui.actionArea)
		self.ui.actionLine.setChecked(True)
		
		self.ui.actionPoint.triggered.connect(self.actionChangeMode2Points)
		self.ui.actionLine.triggered.connect(self.actionChangeMode2Line)
		self.ui.actionMultiline.triggered.connect(self.actionChangeMode2Multiline)
		self.ui.actionArea.triggered.connect(self.actionChangeMode2Area)

		self.ui.toolButtonLarge.clicked.connect(self.actionLargeButton)
		self.ui.toolButtonSmall.clicked.connect(self.actionSmallButton)
		
		# ======================================================================
		# status bar initialization
		# ======================================================================
		log.info(f"initialize statusbar")

		font = QFont("monospace")
		font.setStyleHint(QFont.StyleHint.Courier)
		
		lbl = QLabel()
		lbl.setFont(font)
		lbl.setText(f"X +000.000, Y +000.000")
		self.ui.statusbar.addWidget(lbl)
		self.statusPosLabel = lbl
		
		lb1 = QLabel()
		lb1.setFont(font)
		lb1.setText(f" {mode_names[MODE_LINEAR]} ")
		self.ui.statusbar.addWidget(lb1)
		self.statusModeLabel = lb1

		lb2 = QLabel()
		lb2.setFont(font)
		lb2.setText("unknown")
  # lb2.setText(f"{int(self.microscope.magnification()):3d}x ")
		self.ui.statusbar.addWidget(lb2)
		self.statusMagLabel = lb2
		
		lb3 = QLabel()
		lb3.setFont(font)
		lb3.setText(f"	FPS")
		self.ui.statusbar.addWidget(lb3)
		self.statusFpsLabel = lb3
		
		lb4 = QLabel()
		lb4.setFont(font)
		lb4.setText(f"scale = (?,?)")
		self.ui.statusbar.addWidget(lb4)
		self.statusViewScaleLabel = lb4
		
		ports = self.microscope.port_list()
		if ports:
			port = ports[0]
			mode = MODE_HW_DEBUG if FORCE_DEBUG else MODE_HW
		else:
			port = None
			mode = MODE_SW_DEBUG
		log.info(f"microscope mode is {mode}")
		
		log.info(f"connect to microscope")
		self.microscope.connectToScope(port=port, mode=mode)

		log.info(f"connect scope slots")
		self.microscope.positionUpdated.connect(self.positionUpdated)
		self.microscope.largeClicked.connect(self.actionLargeButton)
		self.microscope.smallClicked.connect(self.actionSmallButton)
		
		# start microscope interface
		log.info("start microscope timers")
		self.microscope.startScope()
		
		log.info(f"AppWindow.__init__ completed")

	@pyqtSlot(QPoint)
	@pyqtSlot(QPointF)
	def positionUpdated(self, xy):
		self.microPosition = xy
		xx = um2mm(xy.x())
		yy = um2mm(xy.y())
		xlabel = f"{'+' if xx >= 0 else '-'}{np.abs(xx):07.3f}"
		ylabel = f"{'+' if yy >= 0 else '-'}{np.abs(yy):07.3f}"  
		self.statusPosLabel.setText(f"{xlabel}  {ylabel}")
		self.ui.bugPlotGraphicsView.scene().updatePosition(xy, self.FOV())
		
	@pyqtSlot()
	def imageUpdated(self, image):
		pass
	
	@pyqtSlot()
	def actionNew(self):
		# build a new table model with no entries
		self.tableModel = TrackObservationsTableModel()
		self.ui.tableView.setModel(self.tableModel)
		self.tableModel.view = self.ui.tableView
		
	@pyqtSlot()
	def actionExit(self):
		sys.exit()

	@pyqtSlot()			
	def actionChangeMode2Points(self):
		self.tableModel.change_mode(MODE_POINTS)
		self.statusModeLabel.setText(f" {mode_names[MODE_POINTS]} ")
		pass

	@pyqtSlot()			
	def actionChangeMode2Line(self):
		self.tableModel.change_mode(MODE_LINEAR)
		self.statusModeLabel.setText(f" {mode_names[MODE_LINEAR]} ")
		pass
	
	@pyqtSlot()			
	def actionChangeMode2Multiline(self):
		self.tableModel.change_mode(MODE_MULTILINE)
		self.statusModeLabel.setText(f" {mode_names[MODE_MULTILINE]} ")
		pass
	
	@pyqtSlot()			
	def actionChangeMode2Area(self):
		self.tableModel.change_mode(MODE_AREA)
		self.statusModeLabel.setText(f" {mode_names[MODE_AREA]} ")
		pass
	
	@pyqtSlot()	
	def actionLargeButton(self):
		self.tableModel.add_point(self.microPosition, KEY_LEFT)
	
	@pyqtSlot()
	def actionSmallButton(self):
		self.tableModel.add_point(self.microPosition, KEY_RIGHT)

	def debug_update(self):
		log.info(f"AppWindow.debug_update")
		log.info('update timer called')

	def FOV(self):
		'''return the field of view in um for width and height'''
		
		fov = self.microscope.cameraFOV()
		w_fov = int(fov.width())
		h_fov = int(fov.height())
		
		xx, yy = self.microPosition.x(), self.microPosition.y()
			
		rt = QRectF(np.round(xx-w_fov/2, 0), np.round(yy-h_fov/2, 0), 
					float(w_fov), float(h_fov))
				
		return rt
	
 # def magnification(self):		
 # 	return self._magnification
 #
 # def setMagnification(self, magnification):
 # 	self._magnification = magnification
 #
 # 	log.warn(f"NEED TO SET SCALING in BugPlotGraphicsView.setMagnification()")
#		 w_fov, h_fov = self.FOV()
#		 self.scale()

 # @pyqtSlot()
 # def updateScopeDisplay(self):
 #
 # 	raise NotImplementedError()
 #
 # 	log.info(f"AppWindow.updateScopeDisplay")
 # 	frame, ts_frame = self._capture.read()
 #
 # 	if frame is None:
 # 		log.debug(f"AppWindow.updateScopeDisplay received an empty frame {self._frame_count}")
 # 		return
 #
 # 	self._frame_count += 1
 #
 # 	# update the FPS display in the status bar
 # 	dt = ts_frame - self._last_ts_frame
 # 	self.statusFpsLabel.setText(f"{1/dt:4.1f} FPS ")
 # 	self._last_ts_frame = ts_frame
 #
 # 	# convert from default GBR to RGB color encoding
 # 	rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
 # 	h, w, ch = rgbImage.shape
 # 	bytesPerLine = ch * w
 # 	# convert to a QImage
 # 	im = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format.Format_RGB888)
 # 	px = QPixmap(im)
 # 	vb = self.viewBackground
 # 	vb.setPixmap(px.scaled(
 # 		vb.size(), Qt.AspectRatioMode.KeepAspectRatioByExpanding,
 # 		Qt.TransformationMode.SmoothTransformation))
 # 	vb.update()

		
# class VideoCaptureThreading(QThread):
# 	'''Read video frames asynchronously in a thread. Return the last
# 		acquired frame when the non-blocking .read() method is called.'''
#
# 	def __init__(self, src, width, height):
#
# 		log.info(f"VideoCaptureThreading.__init__()")
#
# 		self.src = src
# 		self.cap = cv2.VideoCapture(self.src)
#
# 		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width) #width)
# 		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height) #height)
# 		self.cap.set(cv2.CAP_PROP_FPS, 30)
# 		ww = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
# 		hh = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
# 		fps = self.cap.get(cv2.CAP_PROP_FPS)
# 		log.info(f"VideoCaptureThreading({ww}, {hh}, {fps})")
#
# 		self.grabbed = None
# 		self.frame = None
# 		self.timestamp = None
#
# 		self.running = False
# 		self.read_lock = threading.Lock()
#
# 		super().__init__()
#
# 	def set(self, var1, var2):
# 		self.cap.set(var1, var2)
#
# 	def run(self):
#
# 		log.info(f"VideoCaptureThreading.run() started")
# 		self._frame_cnt = 0
# 		t_last_frame = 0
# 		self.started = True
#
# 		log.info(f"VideoCaptureThreading.run() loop starting")
# 		while self.running:
# 			grabbed, frame = self.cap.read()
# 			frame = frame.copy()
#
# 			if grabbed:
# # 				log.debug(f"VideoCaptureThreading.run() grabbed a frame")
#
# 				self._frame_cnt += 1
# 				tt = time.time()
# 				dt = tt - t_last_frame
# 				t_last_frame = tt
#
# 				with self.read_lock:
# 					self.grabbed = grabbed
# 					self.frame = frame
# 					self.timestamp = tt
# 			else:
# 				log.debug(f"VideoCaptureThreading.run BAD GRAB")
#
# 		log.info(f"VideoCaptureThreading.run completed")
#
# 	def read(self):
# 		'''return the last read frame or None if it's called too soon'''
#
# 		with self.read_lock:
# 			if self.grabbed is None:
# 				return None, None
#
# 			frame = self.frame #.copy()
# 			self.grabbed = None   # set so caller can tell if a new frame has been read
# 			ts = self.timestamp
#
# 		return frame, ts
#
# 	def start(self):
#
# 		log.info(f"VideoCaptureThreading.start()")
# 		self.running = True
# 		super().start()
#
# 	def stop(self):
# 		self.running = False
# 		super().stop()
#
# 	def __exit__(self, exec_type, exc_value, traceback):
# 		self.cap.release()
		
class TrackObservationsTableModel(QAbstractTableModel):
	'''
	manage microscope measurement operation
	
	the table units displayed are in millimeters
	'''
	_headerDataNames = 'Mode Value Points Comments'.split()
	COL_MODE, COL_VALUE, COL_PX, COL_COMMENTS = range(len(_headerDataNames))
	
	def __init__(self, *pargs, mode=MODE_LINEAR, debug=FORCE_DEBUG, **kwargs):
		
		QAbstractTableModel.__init__(self, *pargs)
		
		self.mode = mode
		self.rows = []
		self.debug = FORCE_DEBUG
		if debug and False:
			self.rows = [
				TrackRow(mode=MODE_LINEAR, 
						points=[(0,0.100), (0,0.200)], comments='linear #1, really long comment here', close=True),
				TrackRow(mode=MODE_LINEAR, 
						points=[(0.140, 0.090), (-0.080, 0.050)], comments='linear #2', close=True),
				TrackRow(mode=MODE_POINTS, 
						points=[(-0.010,0.)], comments='points #1', close=True),
				TrackRow(mode=MODE_AREA,
						points=[(0,0), (0.100,0), (0.100,0.100)], comments='area #1', close=True),
				TrackRow(mode=MODE_AREA,
						points=[(-1,-1), (1,-1), (1,1), (-1,1)], comments='area #1', close=True),
			]
			
		self.rows.append(TrackRow(mode))
		
	def columnCount(self, *args, **kwargs):
		return len(self._headerDataNames)

	def rowCount(self, *args, **kwargs):
		return len(self.rows)
		
	def headerData(self, section, orientation, role, *args, **kwargs):
		
#		 log.debug(f"TOTM:headerData({section}, {orientation}, {role})")
		
		if role == Qt.ItemDataRole.DisplayRole:
			if orientation == Qt.Orientation.Horizontal:
				return QVariant(self._headerDataNames[section])
			elif orientation == Qt.Orientation.Vertical:
				return QVariant(f'R{self.rows[section].serial_number:04d}')
#		 elif role == Qt.FontRole:
#			 return QVariant()
#			 boldFont = QFont()
#			 boldFont.setBold(True)
#			 return boldFont
		else:
			return QVariant()
		
	def data(self, index, role, *args, **kwargs):
		
#		 log.debug(f"TOTM:data({role})")

		# column formatting functions
#		 def _data_serial_number(row): return f'{row.serial_number:04d}'
		def _data_mode(row):
			mode_map = {
				MODE_POINTS:'P', MODE_LINEAR: 'L', 
				MODE_MULTILINE: 'M', MODE_AREA: 'A'}
			return mode_map[row.mode]
		def _data_value(row): 
			if row.value is None:
				return ''
			else:
				return f'{row.value:.3}'
		
		def _data_pgeneric(row, px):
			try:
				return f'({row.points[0,px]:.3f},{row.points[1,px]:.3f})'
			except IndexError:
				return ''
#		 def _data_p0(row): return _data_pgeneric(row, 0)
#		 def _data_p1(row): return _data_pgeneric(row, 1)
		def _data_px(row):
			pts = [_data_pgeneric(row, px) for px in range(row.points.shape[1])]
			return ','.join(pts)
		def _data_comments(row):	return row.comments
		
		# satisfy custom query for the row data
		if index.isValid() and role == TrackRowRole:
			row = self.rows[index.row()]
			return QVariant((row.status, row.mode, row.value, 
							row.points, row.serial_number))
		
		if not index.isValid() or role != Qt.ItemDataRole.DisplayRole:
			return QVariant()

		row = self.rows[index.row()]
		hfuncs = [_data_mode, _data_value, _data_px, _data_comments]
		cx_map = {ix:ff for ix, ff in enumerate(hfuncs)}
		return QVariant(cx_map[index.column()](row))
	
	def flags(self, index):
		if index.column() == self.COL_COMMENTS:
			return (Qt.ItemFlag.ItemIsEnabled|Qt.ItemFlag.ItemIsSelectable
				|Qt.ItemFlag.ItemIsEditable)
		else:
			return Qt.ItemFlag.ItemIsEnabled
	
	def setData(self, index, value, role):
		r1, c1 = index.row(), index.column()
		if c1 != self.COL_COMMENTS:
			return False

		self.rows[r1].comments = value
		# emit signal that row is modified
		self.dataChanged.emit(self.index(r1, self.COL_COMMENTS),
							self.index(r1, self.COL_COMMENTS))
		return True
	
	def add_point(self, xy, keypress=KEY_LEFT):

		xy = array([um2mm(xy.x()), um2mm(xy.y())]).reshape((2,1))
		orow = self.rows[-1]
		nrows = len(self.rows)
		if orow.mode == MODE_POINTS:
			orow.add_point(xy)
			orow.close_row()
			row0 = self.rowCount() - 1
			self.beginInsertRows(QModelIndex(), len(self.rows), len(self.rows))
			self.rows.append(TrackRow(orow.mode))
			self.endInsertRows()
			row1 = self.rowCount() - 1
			
		elif orow.mode == MODE_LINEAR:
			orow.add_point(xy)
			row0 = self.rowCount() - 1
			if len(orow) == 2:
				# close out the row if this is the second point
				orow.close_row()
				self.beginInsertRows(QModelIndex(), len(self.rows), len(self.rows))
				self.rows.append(TrackRow(orow.mode))
				self.endInsertRows()
			row1 = self.rowCount() - 1
		elif orow.mode in [MODE_MULTILINE, MODE_AREA]:
			orow.add_point(xy)
			row0 = self.rowCount() - 1
			if keypress == KEY_RIGHT:
				orow.close_row()
				self.beginInsertRows(QModelIndex(), len(self.rows), len(self.rows))
				self.rows.append(TrackRow(orow.mode))
				self.endInsertRows()
			row1 = self.rowCount() - 1
		nrows = len(self.rows)
		inx0 = self.index(row0, self.COL_VALUE)
		inx1 = self.index(row1, self.COL_PX)
		self.dataChanged.emit(inx0, inx1)
		self.view.scrollToBottom()
		
	def pop_point(self):
		'''remove the last point added, return True if successful'''
		
		rc = False
		try:
			# try to pop a single point
			self.rows[-1].pop_point()
			rc = True
		except IndexError:
			# didn't work (already empty) so go back to the last row and open it
			try:
				self.rows.pop()
				orow = self.rows[-1]
				orow.pop_point()
				rc = True
			except IndexError:
				# all the way back at the beginning, nothing to do
				pass
			
		nrows = len(self.rows)
		self.dataChanged.emit(self.index(nrows, self.COL_MODE), 
							self.index(nrows, self.COL_COMMENTS))
		self.view.scrollToBottom()

		return rc
	
	def change_mode(self, mode):
		
		orow = self.rows[-1]
		if len(orow) > 0:
			try:
				orow.close_row()
			except TypeError:
				# couldn't close the row, so throw it away
				self.rows.pop()
		else:
			self.rows.pop()
			
		# add an open row in the new mode	
		self.rows.append(TrackRow(mode))
		self.mode = mode
		
		nrows = len(self.rows)
		self.dataChanged.emit(self.index(nrows-1, self.COL_MODE),
							self.index(nrows-1, self.COL_COMMENTS))

		
class TrackRow(QObject):
	
	_serial_number = 0
	
	def __init__(self, mode=MODE_LINEAR, points=None, 
				comments=None, close=False):
		super().__init__()
		
		self.mode = mode
		self.value = None
		self.status = ST_OPEN
		self.points = zeros((2,0))
		
		self.comments = comments
	
		self.serial_number = self.assign_serial_number()
		
		if points:
			if not isinstance(points, ndarray):
				# assume that the input is a list of xy pairs
				points = atleast_2d(points).T
			else: 
				points = atleast_2d(points)
			if points.shape[0] != 2:
				points = points.T
			npoints = points.shape[1]
			
			last_ky = KEY_RIGHT if close else KEY_LEFT
			if npoints > 1:
				for ix in range(points.shape[1]-1):
					self.add_point(points[:, ix], keypress=KEY_LEFT)
				ix += 1
			else:
				ix = 0
			self.add_point(points[:, ix], keypress=last_ky)

		if close:
			self.close_row()

		pass
	
	def __len__(self):
		return self.points.shape[1]
	
	@classmethod
	def assign_serial_number(cls):
		sn = cls._serial_number
		cls._serial_number += 1
		return sn
	
	def add_point(self, xy, keypress=False):
		xy = array(xy).reshape((2,1))
		self.points = hstack((self.points, xy))

	def close_row(self):		
		if self.mode == MODE_POINTS:
			pass
		elif self.mode in [MODE_LINEAR, MODE_MULTILINE]:
			self.value = float(sum(norm(diff(self.points, axis=1), axis=0)))
		else:
			# calculate enclosed area
			pts = self.points
			
			t1 = prod([pts[0], roll(pts[1],-1)], axis=0)
			t2 = prod([pts[1], roll(pts[0],-1)], axis=0)
			self.value = abs(sum(t1-t2))/2.0

		self.status = ST_CLOSED
	
	def pop_point(self):

		self.points = self.points[:, :-1]
		self.value = None
		if len(self):
			raise IndexError

	def path(self):		
		return self._path
	
def except_hook(cls, exception, traceback):
	sys.__excepthook__(cls, exception, traceback)
		
if __name__ == "__main__":
	sys.excepthook = except_hook
	
	logging.basicConfig(
		filename='c:/users/jeffr/OneDrive/Desktop/camera_example.log', 
		level=logging.DEBUG, filemode='w',
		format='%(asctime)s.%(msecs)03d %(levelname)7s %(message)s', datefmt='%H:%M:%S')
	
	app = QApplication(sys.argv)
	w = AppWindow()
	w.show()
	rc = app.exec()
	sys.exit(rc)
