
# name for the microscope camera so it can be found by description
SELECTED_CAM = "GENERAL WEBCAM"

# The 1080x1920 mode would be desirable but the NV12 format display has
#  problems, at least on the Alienware. The YUYV display works fine but is
#  only available up to  
# CAM_HEIGHT = 1920
# CAM_WIDTH = 1080
# CAM_FMT = "Format.Format_NV12"
# highest resolution YUYV on this camera

# CAM_HEIGHT = 720
# CAM_WIDTH = 1280
CAM_FMT = "Format.Format_YUYV"

CAM_HEIGHT = 480
CAM_WIDTH = 800

UM_PER_PX_WIDTH = 12.5      # set based on the current WxH and magnification
UM_PER_PX_HEIGHT = 12.5

DEFAULT_MAGNIFICATION = 100
# UNITY_MAG_PX_PER_MM = 2.250

from PyQt6.QtCore import (
    QObject, 
    QTimer, 
    )
from PyQt6.QtCore import (
    QSizeF,
    )
from PyQt6.QtCore import (
    QPointF, 
    pyqtSignal,
    pyqtSlot, 
    )
from PyQt6.QtMultimedia import (
    QCamera, 
    QMediaDevices,
    )

import logging
import numpy as np
from numpy import mean, pi, cos, sin, array, floor
import time

import serial.tools.list_ports;
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

def um2mm(x):
    return float(x/1000.)

def mm2um(x):
    return int(x * 1000)

# UM_PER_MM = 1000.   # microns/millimeter conversion factor

SEC_TIMER1 = 0.100
SEC_TIMER2 = 0.350
# ----------
# Debugging mode values passed to MicroscopeInterface
# DEBUG_OFF     Normal operation
# DEBUG_HW      Debugging mode using the interface box but not the microscope
# DEBUG_SW      Debugging mode done completely in SW
MODE_HW = 'hw'
MODE_HW_DEBUG = 'hw_debug'
MODE_SW_DEBUG = 'sw_debug'

DEBUG_LOG_INTERVAL = 0.500  # time between log updates when in a debug mode

#
# Register address definitions
#
# The AXIS registers should be monitored for the current position. In
#  normal operation they are a copied from the counter registers every
#  cycle. In debug mode they are copied from the DEBUG registers.
RG_X_AXIS = 0       # X axis current value
RG_Y_AXIS = 2       # Y axis current value
# The CAPTURE registers are latched from the AXIS registers on the 
#  leading edge of either footswitch being pressed
RG_X_CAPTURE = 4
RG_Y_CAPTURE = 6
# The TIMER registers keep track of time. No overflow protection so 
#  multiple reads and compare should be used to access
RG_MS_TIMER = 8
RG_SEC_TIMER = 10
RG_X_COUNT = 12     # raw count value for X axis (normally ignored)
RG_Y_COUNT = 14     # raw count value for y axis (normally ignored)
# The DEBUG registers can be written via the Modbus interface. In debug
#  mode they are looped back to the AXIS registers.
RG_X_DEBUG = 16
RG_Y_DEBUG = 18
RG_COUNT = 20

#
# Bit mapping
#
BT_FTLO = 0         # large footswitch pressed 
BT_FTSO = 1         # small footswitch pressed
BT_DEBUG_MODE = 2   # set to activate debug mode
BT_INITIALIZE = 3   # reset to initialize (clear counters, reset debug)
BT_OUTBITD2 = 4     # extra
BT_OUTBITD3 = 5     # extra
BT_OUTBITD4 = 6     # extra
BT_COUNT = 7

#---------------------------------------------------------------------------# 
# configure the client logging
#---------------------------------------------------------------------------# 
log = logging.getLogger()
log.setLevel(logging.INFO)

W_TRAVEL = 197.0
H_TRAVEL = 145.0

# camera resolution when set for 1X (can't actually be set below 20X)
MM2UM = 1000.  # microns/mm

POS_UPS = 10        # position update interval

class MicroscopeInterface(QObject):
    
    # signal definitions
    positionUpdated = pyqtSignal(QPointF)
    largeClicked = pyqtSignal() # large foot pedal button pushed
    smallClicked = pyqtSignal() # small foot pedal button pushed
        
    def __init__(self):
        '''setup the microscope for connection'''
        
        log.debug(f"MicroscopeInterface.__init__()")
        
        super().__init__()

        self.interface_mode = None
        self.connected = False
        self._pos_poll_interval = 1./POS_UPS
        self.polling_enabled = False
        self.keep_running = True
        self._posTimer = None
        
        self.X, self.Y = None, None

        self._port = None
        self._mode = None
        self._mb_client = None
        
        # debug state updates the position constantly by overwriting the
        #  position registers in the Velocio programmable controller.
        #  Running debug mode requires the interface box but does not
        #  involve the microscope 
        self._update_log_time = time.time() + DEBUG_LOG_INTERVAL
        self._theta_gain = 0.05     # 2pi radians/sec
        self._x_gain = 2000.0 
        self._y_gain = 1000.0
        self._x_epoch = 0.0
        self._y_epoch = 0.0
        self._debug_update_time = time.time()
        self._debug_epoch = time.time()

        self._w_travel, self._h_travel = W_TRAVEL, H_TRAVEL

        #
        # find and log info for all available cameras
        # Initialize "GENERAL WEBCAM" (the one from jiusion)
        #
        cam_name = SELECTED_CAM
        cams = QMediaDevices.videoInputs()
        self._camera = None
        for ii, cam_dev in enumerate(cams):
            desc = cam_dev.description()
            logging.debug(f"Camera {ii} is {desc}, id={str(cam_dev.id())}")
            logging.debug(f"    Min FPS  Max FPS    Resolution  Pixel Format")
            for cam_fmt in cam_dev.videoFormats():
                mn, mx = cam_fmt.minFrameRate(), cam_fmt.maxFrameRate()
                res = (cam_fmt.resolution().height(), cam_fmt.resolution().width())
                px_fmt = str(cam_fmt.pixelFormat())
                logging.debug(f"    {mn:7.1f}  {mx:7.1f}  {str(res):12s}  {str(px_fmt)}")
            if desc == cam_name:
                logging.info(f"found target camera {cam_name}")
                camera = QCamera(cam_dev)
                self._camera = camera
            
        if self._camera is None:
            raise ValueError(f"target camera {cam_name} was not found")

        # reduce the video output resolution
        cam_dev = self._camera.cameraDevice()
        for cam_fmt in cam_dev.videoFormats():
            res = cam_fmt.resolution()
            px_fmt = str(cam_fmt.pixelFormat())
            if (res.height() == CAM_HEIGHT and res.width() == CAM_WIDTH and CAM_FMT in px_fmt):
                self._camera.setCameraFormat(cam_fmt)
                log.info(f"setting camera resolution to {res.height()}x{res.width()}, {px_fmt}")
                break
        else:
            raise ValueError(f"could not located desired camera format")
        
        log.debug(f"MicroscopeInterface.__init__ completed")

#     def __del__(self):
#         self.wait()

    def cameraFormat(self):
        return self._camera.cameraFormat()
    
    def cameraWidth(self):
        return self.cameraFormat().resolution().width()
    
    def cameraHeight(self):
        return self.cameraFormat().resolution().height()
    
    def cameraFOV(self):
        '''return the camera's FOV, in scene units (um), based on the current magnification'''

        res = self._camera.cameraFormat().resolution()
        ww, hh = res.width(), res.height()  # width and height in pixels
        
        return QSizeF(ww * UM_PER_PX_WIDTH, hh * UM_PER_PX_HEIGHT)
    
    def FOV(self, w_camera=None, h_camera=None, unity_scale=None, 
            magnification=None):
        '''return the field of view in um for width and height'''
        
        if w_camera is None: w_camera = self._w_camera
        if h_camera is None: h_camera = self._h_camera
        if unity_scale is None: unity_scale = self._unity_scale
        if magnification is None: magnification = self._magnification

        px_per_um = um2mm(unity_scale * magnification)
        
        w_fov = int(w_camera / px_per_um)
        h_fov = int(h_camera / px_per_um)

        xx, yy = self.microPosition.x(), self.microPosition.y()
            
        rt = QRectF(np.round(xx-w_fov/2, 0), np.round(yy-h_fov/2, 0), 
                    float(w_fov), float(h_fov))
                
        return rt
    
    def read_position(self):
        return self.X, self.Y
    
    def pos_poll_interval(self):
        return self._pos_poll_interval
    
    def port_list(self):
        """return the list of possible serial ports"""

        ports = serial.tools.list_ports.comports()
        if ports:
            ports = [pp.device for pp in ports]
            print(f'available ports: {ports}')
        else:
            log.warning('no available comm ports')

        return ports

    def startScope(self):
        # start the position update timer
        timer = QTimer()
        self._posTimer = timer
        timer.timeout.connect(self._updateScope)
        timer.start(int(self._pos_poll_interval*1000))
        
        # start streaming from the camera
        self._camera.start()
        
    def stopScope(self):
        if self._posTimer:
            self._posTimer.stop()
            
        self._camera.stop()

    def connectToScope(self, port=None, mode=MODE_HW):

        if mode == MODE_SW_DEBUG and port:
            raise ValueError(f"port should not be specified for SW")
        self._port = port
        self._mode = mode

        if port:
            try:
                client = ModbusClient(method='rtu', port=self._port, timeout=3, retries=3,
                                      baudrate=9600,
                                      strict=mode != MODE_HW_DEBUG)
                client.set_debug(True)
                client.connect()
                self._mb_client = client

            except (OSError, IOError) as e:
                log.diff(f'connect failure {str(e)}')
                raise IOError(f"failed to connect to '{port}'") from None

        self.set_hw_debug_mode(mode)

        self.connected = True

    def set_hw_debug_mode(self, mode):

        if mode == MODE_HW_DEBUG:
            # start HW debug mode
            self._mb_client.write_coil(BT_DEBUG_MODE, 1, unit=1)
        elif mode == MODE_HW:
            # turn off debug mode
            self._mb_client.write_coil(BT_DEBUG_MODE, 0, unit=1)

    @pyqtSlot()
    def _updateScope(self):
        
        log.debug(f"MicroscopeInterface._updateScope")
        t0 = time.time()

        if self._mode in [MODE_HW_DEBUG, MODE_SW_DEBUG]:
            self.debug_pos_update()

        if self._mode == MODE_SW_DEBUG:
            xx, yy = self._x_debug_pos, self._y_debug_pos
            if xx != self.X or yy != self.Y:
                self.X, self.Y = xx, yy
                xy = QPointF(xx, yy)
                self.positionUpdated.emit(xy)
            log.debug(f"MicroscopeInterface._updateScope ends (1)")
            return

        rb = self._mb_client.read_coils(0, BT_COUNT, unit=1)
        rr = self._mb_client.read_holding_registers(0, RG_COUNT, unit=1)

        # check for debug mode
        if rb.bits[BT_DEBUG_MODE]:
            xx = int(self._x_debug_pos)
            yy = int(self._y_debug_pos)
            rgs = [int(xx & 0xFFFF),
                   int(xx >> 16) & 0xFFFF,
                   int(yy % 0xFFFF),
                   int(yy >> 16) & 0xFFFF]
            for ix, rg in enumerate(rgs):
                self._mb_client.write_register(RG_X_DEBUG + ix, rg, unit=1)

        f0 = ''.join(['%s' % str(bb)[0] for bb in rb.bits])
        
        ftl = bool(rb.bits[BT_FTLO])
        if ftl:
            # reset to watch for next press
            self._mb_client.write_coil(BT_FTLO, 0, unit=1)
        fts = bool(rb.bits[BT_FTSO])
        if fts:
            # reset to watch for next press
            self._mb_client.write_coil(BT_FTSO, 0, unit=1)
        ff = []
        if ftl:
            ff.append('FTL')
        if fts:
            ff.append('FTS')
        if not ff:
            ff.append('---')
        f1 = '+'.join(ff)

        r32 = [None]*int(len(rr.registers)/2)
        for ix, (r0, r1) in enumerate(zip(rr.registers[0:-1:2],rr.registers[1::2])):
            t32 = r0 + r1*65536
            if t32 >= 2**31:
                t32 = t32 - 2**32
            r32[ix] = t32
        f2 = ' '.join(['%d:%-10d' % (ix*2,rg) for ix, rg in enumerate(r32)])

        self._update_elapsed_time = time.time() - t0
        
        if fts or ftl or (t0 > self._update_log_time):
            log.info('%5.3f %8s %8s %s' 
                     % (self._update_elapsed_time, f1, f0, f2))
            self._update_log_time = t0 + 0.500
            
        xx, yy = float(r32[0]), float(r32[1])
            
        if xx != self.X or yy != self.Y:
            self.X, self.Y = xx, yy
            xy = QPointF(xx, yy)
            self.positionUpdated.emit(xy)

        if ftl:
            self.largeClicked.emit(xy)
        if fts:
            self.smallClicked.emit(xy)
        
        log.debug(f"MicroscopeInterface._updateScope ends (2)")

    def debug_pos_update(self):
        '''loop in a ellipse of x_gain x y_gain'''

        tm = time.time()

        theta = 2*pi*(tm - self._debug_epoch)*self._theta_gain
        xx = np.round(self._x_epoch + self._x_gain * np.cos(theta), 0)
        yy = np.round(self._y_epoch + self._y_gain * np.sin(theta), 0) 
        
        self._x_debug_pos, self._y_debug_pos = xx, yy
            
        self._debug_update_time = tm


DBG_R1 = 1.0
DBG_R2 = DBG_R1/6
DBG_R3 = 0.8*DBG_R2
# DBG_R3 = 0
DBG_PH1 = 0.0
DBG_W1 = 2*pi/240
DBG_PH2 = pi
DBG_W2 = DBG_W1 * DBG_R1/DBG_R2 *2


