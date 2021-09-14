#  labcams - https://jpcouto@bitbucket.org/jpcouto/labcams.git
# Copyright (C) 2020 Joao Couto - jpcouto@gmail.com
#
#  This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from .cams import *
# Allied Vision Technologies cameras
from pymba import *

def AVT_get_ids():
    with Vimba() as vimba:
        # get system object
        system = vimba.system()
        # list available cameras (after enabling discovery for GigE cameras)
        if system.GeVTLIsPresent:
            system.GeVDiscoveryAllOnce
        #time.sleep(0.01)
        camsIds = vimba.camera_ids()
        cams = [vimba.camera(id) for id in camsIds]
        camsModel = []
        for camid,cam in zip(camsIds,cams):
            try:
                cam.open()
            except:
                camsModel.append('')
                continue
            camsModel.append('{0} {1} {2}'.format(cam.info.cameraName,
                                                  cam.info.modelName,
                                                  cam.info.serialString))
    return camsIds,camsModel

class AVTCam(GenericCam):
    def __init__(self, camId = None, outQ = None,
                 exposure = None,
                 frameRate = 30., 
                 gain = 10,
                 frameTimeout = 500, #increasing the timeout stopped the frame timeout that was happening
                 nFrameBuffers = 10,
                 triggered = Event(),
                 triggerSource = 'Line1',
                 triggerMode = 'LevelHigh',
                 triggerSelector = 'FrameStart',
                 acquisitionMode = 'Continuous',
                 nTriggeredFrames = 1000,
                 recorderpar = None):
        self.drivername = 'AVT'
        super(AVTCam,self).__init__(outQ=outQ,recorderpar=recorderpar)
        if camId is None:
            display('Need to supply a camera ID.')
        self.cam_id = camId
        if not exposure is None:
            display("Setting exposure manually")
            self.exposure = exposure            
        else:
            display("Calculating exposure from frame rate") #TODO probably better to read camera max instead, after camera init
            #self.exposure = ((1000000/int(frameRate)) - 1000)/1000.
            self.exposure = (1000/int(frameRate))*0.9 #todo implement optional exposure parameter
        self.frame_rate = frameRate
        self.gain = gain
        self.frameTimeout = frameTimeout
        self.triggerSource = triggerSource
        self.triggerSelector = triggerSelector
        self.acquisitionMode = acquisitionMode
        self.nTriggeredFrames = nTriggeredFrames 
        self.nbuffers = nFrameBuffers
        self.triggerMode = triggerMode
        self.tickfreq = float(1.0)
        with Vimba() as vimba:
            system = vimba.system()
            if system.GeVTLIsPresent:
                system.GeVDiscoveryAllOnce
            time.sleep(0.01)
            self.cam = vimba.camera(camId)
            self.cam.open()
            names = self.cam.feature_names()
            # get a frame
            self.cam.feature("AcquisitionMode").value = 'SingleFrame'
            self.set_exposure(self.exposure)
            self.set_framerate(self.frame_rate)
            self.set_gain(self.gain)
            self.tickfreq = float(1.0) #float(self.cam.GevTimestampTickFrequency)          

            self.cam.TriggerSource = "Software"
            self.cam.TriggerMode = 'Off'
            self.cam.TriggerSelector = 'FrameStart'
            frame = self.cam.new_frame()
            frame.announce()
            self.cam.start_capture()
            frame.queue_for_capture()
            self.cam.AcquisitionStart()
            frame.wait_for_capture()
            self.cam.AcquisitionStop()
            self.h = frame.data.height
            self.w = frame.data.width
            #print("For camera " + self.cam_id + ": pixel format "+ self.cam.feature("PixelFormat").value, flush = True)
            self.nchan = 3 #todo implement pixelformat selection / detection
            #self.nchan = 1
            self.dtype = np.uint8
            self._init_variables(dtype = self.dtype)
            #print(str(frame.buffer_data().shape), flush = True)
            #framedata = np.ndarray(buffer = frame.buffer_data(),
            #                       dtype = self.dtype,
            #                       shape = (self.h,
            #                                self.w)).copy()
            framedata = frame.buffer_data_numpy().copy()
            print("framedata shape: {0}".format(framedata.shape), flush = True)
            self.img[:] = np.reshape(framedata,self.img.shape)[:]
            display("AVT [{1}] = Got info from camera (name: {0})".format(
                self.cam.info.cameraName,self.cam_id))
            self.cam.end_capture()
            self.cam.revoke_all_frames()
            self.cam = None
        self.triggered = triggered
        if self.triggered.is_set():
            display('AVT [{0}] - Triggered mode ON.'.format(self.cam_id))
            self.triggerSource = triggerSource
    def _init_controls(self):
        self.ctrevents = dict(
            exposure=dict(
                function = 'set_exposure',
                widget = 'float',
                variable = 'exposure',
                units = 'ms',
                type = 'float',
                min = 0.001,
                max = 100000,
                step = 10),
            gain = dict(
                function = 'set_gain',
                widget = 'float',
                variable = 'gain',
                units = 'ms',
                type = 'int',
                min = 0,
                max = 30,
                step = 1),
            framerate = dict(
                function = 'set_framerate',
                widget = 'float',
                type = 'float',
                variable = 'frame_rate',
                units = 'fps',
                min = 0.001,
                max = 1000,
                step = 1))
        
    def set_exposure(self,exposure = 30):
        '''Set the exposure time is in ms'''
        if not self.cam is None:
            display('[AVT {0}] Setting exposure to {1} ms.'.format(
                    self.cam_id, self.exposure))
            try:
                self.exposure = exposure

                self.cam.ExposureMode = "Timed" #TODO ENUM
                self.cam.ExposureTime =  int(self.exposure*1000)
                
            except VimbaException as err:
                Display("Error setting exposure on {0}. Supported range: {1}".format(self.cam_id, self.cam.feature("ExposureTime").range))
                raise VimbaException(err.error_code)

    
    def set_framerate(self,frame_rate = 30):
        '''set the frame rate of the AVT camera.''' 
        if not self.cam is None:
            display('[AVT {0}] Setting frame rate to {1} .'.format(
                    self.cam_id, self.frame_rate))

            try:
                self.frame_rate = frame_rate

                self.cam.AcquisitionFrameRateEnable = True
                self.cam.AcquisitionFrameRate = self.frame_rate
                if self.cam_is_running:
                    self.start_trigger.set()
                    self.stop_trigger.set()
            except VimbaException as err:
                display("Error setting frame rate on {0}. Supported range: {1}".format(self.cam_id, self.cam.feature("AcquisitionFrameRate").range))
                raise VimbaException(err.error_code)

    def set_gain(self,gain = 0):
        ''' Set the gain of the AVT camera'''
        self.gain = int(gain)
        if not self.cam is None:
            self.cam.GainAuto = "Off"
            self.cam.Gain = self.gain
            display('[AVT {0}] Setting camera gain to {1} .'.format(
                self.cam_id, self.gain))

    #TODO implement optional white balance setting via config file
    def set_whitebalance(self):
        if not self.cam is None:
            display('[AVT {0}] Start auto white balance .'.format(
                self.cam_id))
            self.cam.feature("BalanceWhiteAuto").value = "Once"

    #TODO support via command line option
    def list_supported_features(self):
        if not self.cam is None:
            display('Supported Features on camera ' + self.cam_id + ':\n'.join(self.cameraFeatureNames))

    #TODO support via command line option
    def list_supported_pixel_formats(self):
        if not self.cam is None:
            print("Supported pixel formats on " + self.cam_id + ':\n'.join(self.cam.feature("PixelFormat").range))


    
    def _cam_init(self):
        self.nframes.value = 0
        self.lastframeid = -1
        self.recorded_frames = []
        self.vimba = Vimba()
        self.vimba.startup()
        system = self.vimba.system()
        if system.GeVTLIsPresent:
            system.GeVDiscoveryAllOnce
            time.sleep(0.1)
        # prepare camera
        self.cam = self.vimba.camera(self.cam_id)
        self.cam.open()
        # cam.EventSelector = 'FrameTrigger'
        self.cam.EventNotification = 'On'
        self.cam.PixelFormat = 'RGB8' #todo implement pixeolformat selection / detection
        #self.cam.PixelFormat = 'Mono8'
        self.cameraFeatureNames = self.cam.feature_names()
        self.set_exposure(self.exposure)
        self.set_framerate(self.frame_rate)
        self.set_gain(self.gain)
        self.set_whitebalance()

        #print some supported stuff
        #self.list_supported_features()
        #self.list_supported_pixel_formats()

        self.cam.SyncOutSelector = 'SyncOut1'
        self.cam.SyncOutSource = 'FrameReadout'#'Exposing'
        if self.triggered.is_set():
            self.cam.TriggerSource = self.triggerSource#'Line1'#self.triggerSource
            self.cam.TriggerMode = 'On'
            #cam.TriggerOverlap = 'Off'
            self.cam.TriggerActivation = self.triggerMode #'LevelHigh'##'RisingEdge'
            self.cam.AcquisitionMode = self.acquisitionMode
            self.cam.TriggerSelector = self.triggerSelector
            if self.acquisitionMode == 'MultiFrame':
                self.cam.AcquisitionFrameCount = self.nTriggeredFrames
                self.cam.TriggerActivation = self.triggerMode #'LevelHigh'##'RisingEdge'
        else:
            display('[Cam - {0}] Using no trigger.'.format(self.cam_id))
            self.cam.AcquisitionMode = 'Continuous'
            self.cam.TriggerSource = 'Software'
            self.cam.TriggerMode = 'Off'
            self.cam.TriggerSelector = 'FrameStart'
        # create new frames for the camera
        self.frames = []
        for i in range(self.nbuffers):
            self.frames.append(self.cam.new_frame())    # creates a frame
            self.frames[i].announce()
        self.cam.start_capture()
        for f,ff in enumerate(self.frames):
            try:
                ff.queue_for_capture()
            except:
                display('Queue frame error while getting cam ready: '+ str(f))
                continue                    
        self.camera_ready.set()
        self.nframes.value = 0
        # Ready to wait for trigger
            
    def _cam_startacquisition(self):
        #self.cam.GevTimestampControlReset #--is this required anymore with the usb cams? or is it required for gige cameras still?
        self.cam.AcquisitionStart()
        if self.triggered.is_set():
            self.cam.TriggerSelector = self.triggerSelector
            self.cam.TriggerMode = 'On'
        #tstart = time.time()
        self.lastframeids = [-1 for i in self.frames]

    def _cam_loop(self):
        # run and acquire frames
        #sortedfids = np.argsort([f._frame.frameID for f in frames])
        for ibuf in range(self.nbuffers):
            f = self.frames[ibuf]
            try:
                f.wait_for_capture(timeout_ms = self.frameTimeout)
                timestamp = f.data.timestamp/self.tickfreq
                frameID = f.data.frameID
                #print('Frame id:{0}'.format(frameID))
                if not frameID in self.recorded_frames:
                    self.recorded_frames.append(frameID)
                    #frame = np.ndarray(buffer = f.buffer_data(),
                    #                    dtype = self.dtype,
                    #                    shape = (f.data.height,
                    #                            f.data.width)).copy()
                    frame = f.buffer_data_numpy().copy()

                    #display("Time {0} - {1}:".format(str(1./(time.time()-tstart)),self.nframes.value))
                    #tstart = time.time()
                    try:
                        f.queue_for_capture()
                    except:
                        display('Queue frame failed: '+ str(f))
                        return None,(None,None)
                    self.lastframeids[ibuf] = frameID
                    if not frame is None:
                        self.nframes.value = frameID
                    return frame,(frameID,timestamp)
                
            except VimbaException as err:
                display('VimbaException during camera loop: ' +  str(err))
                return None,(None,None)
            except Exception as err:
                display('Other exception during camera loop: ' + str(err))
                return None,(None,None)

    def _cam_close(self):
        self.cam.AcquisitionStop()
        display('[AVT] - Stopped acquisition.')
        # Check if all frames are done...
        for ibuf in range(self.nbuffers):
            f = self.frames[ibuf]
            try:
                f.wait_for_capture(timeout_ms = self.frameTimeout)
                timestamp = f.data.timestamp/self.tickfreq
                frameID = f.data.frameID
                frame = np.ndarray(buffer = f.buffer_data(),
                                   dtype = self.dtype,
                                   shape = (f.data.height,
                                            f.data.width)).copy()
                if self.saving.is_set():
                    self.was_saving = True
                    if not frameID in self.lastframeids :
                        self.queue.put((frame.copy(),(frameID,timestamp)))
                elif self.was_saving:
                    self.was_saving = False
                    self.queue.put(['STOP'])

                self.lastframeids[ibuf] = frameID
                self.nframes.value = frameID
                self.frame = frame
            except VimbaException as err:
                display('VimbaException at camera close: ' + str(err))
        self.cam.stop_frame_acquisition()
        self.cam.end_capture()
        try:
            self.cam.revoke_all_frames()
        except:
            display('Failed to revoke frames.')
        self.cam.close()
        display('AVT [{0}] - Close event: {1}'.format(
            self.cam_id,
            self.close_event.is_set()))
        self.vimba.shutdown()