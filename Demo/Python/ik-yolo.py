import sys
import time
import platform
import re
import cv2
import numpy as np
import mvsdk
from ultralytics import YOLO 

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QLineEdit, QCheckBox, QSlider
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap

def parse_config_file(file_path):
    params = {}
    pattern = re.compile(r'(\w+)\s*=\s*"?([\w\.\-]+)"?\s*;')
    with open(file_path, 'r', encoding='utf-8') as f:
        text = f.read()
    for match in pattern.finditer(text):
        key = match.group(1)
        val = match.group(2)
        if re.fullmatch(r'\d+', val):
            val = int(val)
        elif re.fullmatch(r'\d+\.\d+', val):
            val = float(val)
        params[key] = val
    return params

def convert_frame_to_qpixmap(frame):
    if len(frame.shape) == 2 or frame.shape[2] == 1:
        h, w = frame.shape[:2]
        bytes_per_line = w
        gray_data = frame.tobytes()
        qimg = QImage(gray_data, w, h, bytes_per_line, QImage.Format_Grayscale8)
    else:
        h, w, ch = frame.shape
        rgb = frame[..., ::-1]
        bytes_per_line = ch * w
        rgb_data = rgb.tobytes()
        qimg = QImage(rgb_data, w, h, bytes_per_line, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg)

class YOLOWorker(QThread):
    yoloResult = pyqtSignal(QPixmap)
    
    def __init__(self, yolo_model):
        super().__init__()
        self.yolo_model = yolo_model
        self.frame = None
        self.running = True

    @pyqtSlot(np.ndarray)
    def updateFrame(self, frame):
        self.frame = frame

    def run(self):
        while self.running:
            if self.frame is not None:
                frame_to_process = self.frame.copy()
                self.frame = None
                try:
                    results = self.yolo_model(frame_to_process)
                    boxes = results[0].boxes
                    for box in boxes:
                        coords = box.xyxy[0]
                        x1, y1, x2, y2 = int(coords[0].item()), int(coords[1].item()), int(coords[2].item()), int(coords[3].item())
                        cls_id = int(box.cls[0].item())
                        conf = float(box.conf[0].item())
                        label = f"{self.yolo_model.model.names[cls_id]} {conf:.2f}"
                        cv2.rectangle(frame_to_process, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(frame_to_process, label, (x1, max(y1-10, 0)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                except Exception as e:
                    print("YOLO detection error in worker:", e)
                pix = convert_frame_to_qpixmap(frame_to_process)
                self.yoloResult.emit(pix)
            self.msleep(10)

    def stop(self):
        self.running = False
        self.wait()

class CameraWorker(QThread):
    updateFrameSignal = pyqtSignal(QPixmap, QPixmap, float)
    sendFrameForYOLO = pyqtSignal(np.ndarray)
    
    def __init__(self, camera_app):
        super().__init__()
        self.camera_app = camera_app
        self.running = True
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0
        self.prev_frame = None

    def run(self):
        while self.running:
            try:
                current_time = time.time()
                self.frame_count += 1
                if (current_time - self.prev_time) >= 1.0:
                    self.fps = self.frame_count / (current_time - self.prev_time)
                    self.frame_count = 0
                    self.prev_time = current_time

                if self.camera_app.camera_type == 'webcam':
                    ret, frame_bgr = self.camera_app.cap.read()
                    if not ret:
                        continue
                    if self.camera_app.video_size is None:
                        h = int(self.camera_app.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        w = int(self.camera_app.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        self.camera_app.video_size = (w, h)
                else:
                    pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.camera_app.hCamera, 50)
                    mvsdk.CameraImageProcess(self.camera_app.hCamera, pRawData, self.camera_app.pFrameBuffer, FrameHead)
                    mvsdk.CameraReleaseImageBuffer(self.camera_app.hCamera, pRawData)

                    if platform.system() == "Windows":
                        mvsdk.CameraFlipFrameBuffer(self.camera_app.pFrameBuffer, FrameHead, 1)

                    frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.camera_app.pFrameBuffer)
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    ch = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
                    frame_bgr = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, ch))
                    if ch == 1:
                        frame_bgr = cv2.cvtColor(frame_bgr, cv2.COLOR_GRAY2BGR)
                    else:
                        frame_bgr = frame_bgr.copy()
                    if self.camera_app.video_size is None:
                        h, w = frame_bgr.shape[:2]
                        self.camera_app.video_size = (w, h)

                cv2.putText(frame_bgr, f"FPS: {self.fps:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                if self.camera_app.yolo_enabled and self.camera_app.yolo_worker is not None:
                    self.sendFrameForYOLO.emit(frame_bgr.copy())

                pix_orig = convert_frame_to_qpixmap(frame_bgr)

                if self.camera_app.checkDiff.isChecked():
                    if self.prev_frame is not None:
                        diff = cv2.absdiff(frame_bgr, self.prev_frame)
                        cv2.putText(diff, f"FPS: {self.fps:.2f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                        pix_diff = convert_frame_to_qpixmap(diff)
                    else:
                        pix_diff = QPixmap()
                    self.prev_frame = frame_bgr.copy()
                else:
                    pix_diff = QPixmap()

                if (self.camera_app.is_recording and self.camera_app.writer_orig and 
                    self.camera_app.writer_diff and self.camera_app.video_size):
                    if (frame_bgr.shape[1], frame_bgr.shape[0]) != self.camera_app.video_size:
                        frame_bgr = cv2.resize(frame_bgr, self.camera_app.video_size)
                    self.camera_app.writer_orig.write(frame_bgr)
                    if not self.camera_app.checkDiff.isChecked():
                        self.camera_app.writer_diff.write(frame_bgr)
                    else:
                        diff_frame = cv2.absdiff(frame_bgr, self.prev_frame)
                        self.camera_app.writer_diff.write(diff_frame)

                self.updateFrameSignal.emit(pix_orig, pix_diff, self.fps)

            except Exception as e:
                print("Error in CameraWorker:", e)

    def stop(self):
        self.running = False
        self.wait()

class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MvSDK + PyQt (Выбор камеры, настройка FPS, YOLO, Diff, Trigger, Recording)")
        
        # --- Переменные для камеры ---
        self.hCamera = None          
        self.cap = None              
        self.camera_type = None     
        self.capability = None
        self.monoCamera = False
        self.pFrameBuffer = None
        self.FrameBufferSize = 0

        self.devList = mvsdk.CameraEnumerateDevice()  
        self.webcamList = self.enumerate_webcams()     
        self.resList = []
        self.loadModes = [("ByModel", 0), ("ByName", 1), ("BySN", 2)]

        self.config_file_path = "settings.Config"
        self.config_params = {}
        self.load_config()

        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0
        self.video_fps = 30.0  
        self.video_size = None

        self.flash_freq = 10.0
        self.flash_period = 1.0 / self.flash_freq
        self.prev_frame = None
        self.is_recording = False
        self.writer_orig = None
        self.writer_diff = None

        self.worker = None  

        self.yolo_enabled = False
        self.yolo_model = None
        self.yolo_worker = None

        self.initUI()

    def enumerate_webcams(self):
        webcams = []
        for i in range(4):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    webcams.append(i)
                cap.release()
        print("Обнаруженные веб-камеры:", webcams)
        return webcams

    def load_config(self):
        try:
            self.config_params = parse_config_file(self.config_file_path)
            print("Config loaded:", self.config_params)
        except Exception as e:
            print("Не удалось загрузить config:", e)
            self.config_params = {}

    def initUI(self):
        self.comboCamera = QComboBox()
        for idx in self.webcamList:
            self.comboCamera.addItem(f"Webcam {idx}", {"type": "webcam", "index": idx})
        for i, dev in enumerate(self.devList):
            desc = f"{i}: {dev.GetFriendlyName()} (SN:{dev.GetSn()})"
            self.comboCamera.addItem(desc, {"type": "mvsdk", "device": dev})

        self.comboLoadMode = QComboBox()
        for (name, val) in self.loadModes:
            self.comboLoadMode.addItem(name, val)

        self.btnOpen = QPushButton("Open Camera")
        self.btnOpen.clicked.connect(self.on_open_camera)

        self.comboRes = QComboBox()
        self.btnSetRes = QPushButton("Set Resolution")
        self.btnSetRes.clicked.connect(self.on_set_resolution)

        self.btnApplyConf = QPushButton("Apply Config")
        self.btnApplyConf.clicked.connect(self.on_apply_config)

        self.editFreq = QLineEdit(str(self.flash_freq))
        self.editFreq.setFixedWidth(60)
        self.editFreq.editingFinished.connect(self.on_freq_changed)

        self.editVideoFPS = QLineEdit(str(self.video_fps))
        self.editVideoFPS.setFixedWidth(60)
        self.editVideoFPS.editingFinished.connect(self.on_video_fps_changed)

        self.checkDiff = QCheckBox("Difference Mode")
        self.checkDiff.setChecked(False)

        self.checkHardwareTrigger = QCheckBox("Hardware Trigger")
        self.checkHardwareTrigger.setChecked(False)
        self.checkHardwareTrigger.stateChanged.connect(self.on_trigger_mode_changed)

        self.btnRecord = QPushButton("Start Recording")
        self.btnRecord.clicked.connect(self.on_record_clicked)
        
        self.btnYOLO = QPushButton("Enable YOLO")
        self.btnYOLO.setCheckable(True)
        self.btnYOLO.clicked.connect(self.on_yolo_clicked)
        
        # Новые виджеты для выбора модели YOLO и использования CUDA
        self.comboYOLOModel = QComboBox()
        self.comboYOLOModel.addItem("YOLOv8n.pt")
        self.comboYOLOModel.addItem("drone_far_yolo11n.pt")
        self.checkCUDA = QCheckBox("Use CUDA")
        self.checkCUDA.setChecked(True)

        self.labelExposure = QLabel("Exposure: 30000 us")
        self.sliderExposure = QSlider(Qt.Horizontal)
        self.sliderExposure.setMinimum(100)
        self.sliderExposure.setMaximum(100000)
        self.sliderExposure.setValue(30000)
        self.sliderExposure.setTickInterval(10000)
        self.sliderExposure.valueChanged.connect(self.on_exposure_slider_changed)
        
        self.labelGain = QLabel("Gain: 1")
        self.sliderGain = QSlider(Qt.Horizontal)
        self.sliderGain.setMinimum(1)
        self.sliderGain.setMaximum(16)
        self.sliderGain.setValue(1)
        self.sliderGain.setTickInterval(1)
        self.sliderGain.valueChanged.connect(self.on_gain_slider_changed)

        self.checkStrobe = QCheckBox("Strobe Enable")
        self.checkStrobe.setChecked(False)
        self.editStrobeDelay = QLineEdit("0")
        self.editStrobeDelay.setFixedWidth(80)
        self.editStrobePulse = QLineEdit("1000")
        self.editStrobePulse.setFixedWidth(80)
        self.editStrobePolarity = QLineEdit("0")
        self.editStrobePolarity.setFixedWidth(80)
        self.btnApplyStrobe = QPushButton("Apply Strobe Settings")
        self.btnApplyStrobe.clicked.connect(self.on_apply_strobe_settings)

        self.labelOrig = QLabel("No camera")
        self.labelOrig.setScaledContents(False)  

        self.labelDiff = QLabel("Diff off")
        self.labelDiff.setScaledContents(False)
        self.labelDiff.setVisible(False)

        self.labelFps = QLabel("FPS: 0.0")

        row1 = QHBoxLayout()
        row1.addWidget(self.comboCamera)
        row1.addWidget(self.comboLoadMode)
        row1.addWidget(self.btnOpen)

        row2 = QHBoxLayout()
        row2.addWidget(self.comboRes)
        row2.addWidget(self.btnSetRes)
        row2.addWidget(self.btnApplyConf)
        row2.addWidget(QLabel("FlashFreq:"))
        row2.addWidget(self.editFreq)
        row2.addWidget(QLabel("Video FPS:"))
        row2.addWidget(self.editVideoFPS)
        row2.addWidget(self.checkDiff)
        row2.addWidget(self.checkHardwareTrigger)
        row2.addWidget(self.btnRecord)
        row2.addWidget(self.btnYOLO)
        # Новые элементы для выбора модели YOLO и режима CUDA
        row2.addWidget(QLabel("YOLO Model:"))
        row2.addWidget(self.comboYOLOModel)
        row2.addWidget(self.checkCUDA)

        rowExposure = QHBoxLayout()
        rowExposure.addWidget(self.labelExposure)
        rowExposure.addWidget(self.sliderExposure)
        
        rowGain = QHBoxLayout()
        rowGain.addWidget(self.labelGain)
        rowGain.addWidget(self.sliderGain)

        rowStrobe = QHBoxLayout()
        rowStrobe.addWidget(self.checkStrobe)
        rowStrobe.addWidget(QLabel("Delay (us):"))
        rowStrobe.addWidget(self.editStrobeDelay)
        rowStrobe.addWidget(QLabel("Pulse (us):"))
        rowStrobe.addWidget(self.editStrobePulse)
        rowStrobe.addWidget(QLabel("Polarity:"))
        rowStrobe.addWidget(self.editStrobePolarity)
        rowStrobe.addWidget(self.btnApplyStrobe)

        rowImages = QHBoxLayout()
        rowImages.addWidget(self.labelOrig)
        rowImages.addWidget(self.labelDiff)

        layout = QVBoxLayout()
        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addLayout(rowExposure)
        layout.addLayout(rowGain)
        layout.addLayout(rowStrobe)
        layout.addLayout(rowImages)
        layout.addWidget(self.labelFps)

        self.setLayout(layout)
        self.setMinimumSize(1000, 600)

    def on_exposure_slider_changed(self, value):
        self.labelExposure.setText(f"Exposure: {value} us")
        if self.camera_type == 'mvsdk' and self.hCamera:
            err = mvsdk.CameraSetExposureTime(self.hCamera, value)
            if err == 0:
                print(f"Exposure set to {value} us")
            else:
                print("Failed to set exposure, error code:", err)

    def on_gain_slider_changed(self, value):
        self.labelGain.setText(f"Gain: {value}")
        if self.camera_type == 'mvsdk' and self.hCamera:
            err = mvsdk.CameraSetAnalogGain(self.hCamera, int(value))
            if err == 0:
                print(f"Analog gain set to {value}")
            else:
                print("Failed to set analog gain, error code:", err)

    def on_video_fps_changed(self):
        txt = self.editVideoFPS.text().strip()
        try:
            val = float(txt)
            if val > 0:
                self.video_fps = val
                print(f"Video FPS set to {val}")
        except ValueError:
            print("Invalid Video FPS input")

    def on_open_camera(self):
        selected = self.comboCamera.currentData()
        if not selected:
            return
        self.close_camera()  

        if selected["type"] == "webcam":
            self.camera_type = "webcam"
            webcam_index = selected["index"]
            self.cap = cv2.VideoCapture(webcam_index)
            if not self.cap.isOpened():
                print(f"Не удалось открыть веб-камеру {webcam_index}")
                return
            ret, frame = self.cap.read()
            if ret:
                h, w = frame.shape[:2]
                self.video_size = (w, h)
            print(f"Webcam {webcam_index} opened successfully.")
        else:
            self.camera_type = "mvsdk"
            dev = selected["device"]
            loadMode = self.comboLoadMode.currentData()
            try:
                self.hCamera = mvsdk.CameraInit(dev, loadMode, -1)
            except mvsdk.CameraException as e:
                print("CameraInit failed:", e)
                self.hCamera = None
                return

            self.capability = mvsdk.CameraGetCapability(self.hCamera)
            self.monoCamera = (self.capability.sIspCapacity.bMonoSensor != 0)
            if self.monoCamera:
                mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
            else:
                mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

            mvsdk.CameraSetTriggerMode(self.hCamera, 0)
            mvsdk.CameraSetAeState(self.hCamera, 0)
            mvsdk.CameraSetExposureTime(self.hCamera, self.sliderExposure.value())
            mvsdk.CameraSetAnalogGain(self.hCamera, self.sliderGain.value())
            mvsdk.CameraPlay(self.hCamera)

            maxw = self.capability.sResolutionRange.iWidthMax
            maxh = self.capability.sResolutionRange.iHeightMax
            ch = 1 if self.monoCamera else 3
            self.FrameBufferSize = maxw * maxh * ch
            self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)
            self.comboRes.clear()
            self.resList.clear()
            count = self.capability.iImageSizeDesc
            pResDesc = self.capability.pImageSizeDesc
            for i in range(count):
                r = pResDesc[i]
                desc = r.GetDescription()
                w = r.iWidthFOV
                h = r.iHeightFOV
                text = f"{desc} ({w}x{h})"
                self.comboRes.addItem(text)
                self.resList.append(r.clone())
            print("mvsdk камера открыта успешно.")
            self.video_size = None 

        if self.worker:
            self.worker.stop()
        self.worker = CameraWorker(self)
        self.worker.updateFrameSignal.connect(self.on_update_frame)
        self.worker.sendFrameForYOLO.connect(lambda frame: 
                                             self.yolo_worker.updateFrame(frame)
                                             if self.yolo_worker is not None else None)
        self.worker.start()

    def on_set_resolution(self):
        if self.camera_type != 'mvsdk' or not self.hCamera:
            return
        idxRes = self.comboRes.currentIndex()
        if idxRes < 0 or idxRes >= len(self.resList):
            return
        newRes = self.resList[idxRes]
        err = mvsdk.CameraSetImageResolution(self.hCamera, newRes)
        if err == 0:
            print(f"Set resolution => iIndex={newRes.iIndex}, ROI=({newRes.iWidthFOV}x{newRes.iHeightFOV})")
            self.prev_frame = None
        else:
            print("CameraSetImageResolution failed:", err)

    def on_apply_config(self):
        if self.camera_type != 'mvsdk' or not self.hCamera:
            return
        cfg = self.config_params
        iIndex = cfg.get("iIndex", None)
        if iIndex is not None:
            iH = cfg.get("iHOffsetFOV", 0)
            iV = cfg.get("iVOffsetFOV", 0)
            wF = cfg.get("iWidthFOV", 640)
            hF = cfg.get("iHeightFOV", 480)
            err = mvsdk.CameraSetImageResolutionEx(
                self.hCamera, iIndex, 0, 0, iH, iV, wF, hF, 0, 0
            )
            if err == 0:
                print(f"Apply config => iIndex={iIndex}, ROI=({wF}x{hF}), offset=({iH},{iV})")
                self.prev_frame = None
            else:
                print("CameraSetImageResolutionEx failed:", err)
        exp_time = cfg.get("exp_time", None)
        if exp_time is not None:
            err = mvsdk.CameraSetExposureTime(self.hCamera, exp_time)
            if err == 0:
                print(f"Set ExposureTime={exp_time} us")
            else:
                print("CameraSetExposureTime failed:", err)
        analog_gain = cfg.get("analog_gain", None)
        if analog_gain is not None:
            mvsdk.CameraSetAnalogGain(self.hCamera, int(analog_gain))
            print(f"Set analog_gain={analog_gain}")

    def on_freq_changed(self):
        txt = self.editFreq.text().strip()
        try:
            val = float(txt)
            if val > 0:
                self.flash_freq = val
                self.flash_period = 1.0 / val
                print(f"Flash frequency set => {val} Hz")
        except ValueError:
            print("Invalid freq input")

    def on_trigger_mode_changed(self, state):
        if self.camera_type != 'mvsdk' or not self.hCamera:
            return
        if state == Qt.Checked:
            err = mvsdk.CameraSetTriggerMode(self.hCamera, 2)
            if err == 0:
                print("Hardware Trigger ON")
            else:
                print("CameraSetTriggerMode(2) failed:", err)
        else:
            err = mvsdk.CameraSetTriggerMode(self.hCamera, 0)
            if err == 0:
                print("Hardware Trigger OFF => Continuous ON")
            else:
                print("CameraSetTriggerMode(0) failed:", err)

    def on_apply_strobe_settings(self):
        if self.camera_type != 'mvsdk' or not self.hCamera:
            return
        strobe_enabled = self.checkStrobe.isChecked()
        ret_mode = mvsdk.CameraSetStrobeMode(self.hCamera, 1 if strobe_enabled else 0)
        if ret_mode != 0:
            print("CameraSetStrobeMode failed:", ret_mode)
        try:
            delay = int(self.editStrobeDelay.text())
            ret_delay = mvsdk.CameraSetStrobeDelayTime(self.hCamera, delay)
            if ret_delay != 0:
                print("CameraSetStrobeDelayTime failed:", ret_delay)
        except ValueError:
            print("Invalid strobe delay value")
        try:
            pulse = int(self.editStrobePulse.text())
            ret_pulse = mvsdk.CameraSetStrobePulseWidth(self.hCamera, pulse)
            if ret_pulse != 0:
                print("CameraSetStrobePulseWidth failed:", ret_pulse)
        except ValueError:
            print("Invalid strobe pulse value")
        try:
            polarity = int(self.editStrobePolarity.text())
            ret_pol = mvsdk.CameraSetStrobePolarity(self.hCamera, polarity)
            if ret_pol != 0:
                print("CameraSetStrobePolarity failed:", ret_pol)
        except ValueError:
            print("Invalid strobe polarity value")
        print("Strobe settings applied.")

    def on_record_clicked(self):
        if not self.is_recording:
            self.start_recording()
            self.btnRecord.setText("Stop Recording")
        else:
            self.stop_recording()
            self.btnRecord.setText("Start Recording")

    def start_recording(self):
        self.is_recording = True
        time_str = time.strftime("%Y%m%d_%H%M%S")
        filename_orig = f"orig_{time_str}.avi"
        filename_diff = f"diff_{time_str}.avi"
        print(f"Start recording to {filename_orig} and {filename_diff}")
        self.video_size = None 
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.writer_orig = cv2.VideoWriter(filename_orig, fourcc, self.video_fps, (640,480))
        self.writer_diff = cv2.VideoWriter(filename_diff, fourcc, self.video_fps, (640,480))

    def stop_recording(self):
        self.is_recording = False
        if self.writer_orig:
            self.writer_orig.release()
            self.writer_orig = None
        if self.writer_diff:
            self.writer_diff.release()
            self.writer_diff = None
        print("Recording stopped.")

    @pyqtSlot(QPixmap, QPixmap, float)
    def on_update_frame(self, pix_orig, pix_diff, fps):
        if not self.yolo_enabled:
            scaled = pix_orig.scaled(self.labelOrig.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.labelOrig.setPixmap(scaled)
        if self.checkDiff.isChecked():
            scaled_diff = pix_diff.scaled(self.labelDiff.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.labelDiff.setVisible(True)
            self.labelDiff.setPixmap(scaled_diff)
        else:
            self.labelDiff.setVisible(False)
        self.labelFps.setText(f"FPS: {fps:.2f}")

    @pyqtSlot(QPixmap)
    def on_yolo_result(self, pix):
        scaled = pix.scaled(self.labelOrig.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.labelOrig.setPixmap(scaled)

    def on_yolo_clicked(self):
        self.yolo_enabled = self.btnYOLO.isChecked()
        if self.yolo_enabled:
            self.btnYOLO.setText("Disable YOLO")
            if self.yolo_model is None:
                model_file = self.comboYOLOModel.currentText()
                device = "cuda" if self.checkCUDA.isChecked() else "cpu"
                try:
                    self.yolo_model = YOLO(model_file)
                    # Переносим модель на нужное устройство:
                    self.yolo_model.model.to(device)
                    print(f"YOLO model {model_file} loaded on device {device}.")
                except Exception as e:
                    print("Failed to load YOLO model:", e)
                    self.yolo_enabled = False
                    self.btnYOLO.setChecked(False)
                    self.btnYOLO.setText("Enable YOLO")
                    return
            if self.yolo_worker is None:
                self.yolo_worker = YOLOWorker(self.yolo_model)
                self.yolo_worker.yoloResult.connect(self.on_yolo_result)
                self.yolo_worker.start()
        else:
            self.btnYOLO.setText("Enable YOLO")
            if self.yolo_worker is not None:
                self.yolo_worker.stop()
                self.yolo_worker = None


    def close_camera(self):
        self.stop_recording()
        if self.worker:
            self.worker.stop()
            self.worker = None
        if self.yolo_worker:
            self.yolo_worker.stop()
            self.yolo_worker = None
        if self.camera_type == "mvsdk" and self.hCamera:
            mvsdk.CameraUnInit(self.hCamera)
            self.hCamera = None
        if self.camera_type == "webcam" and self.cap:
            self.cap.release()
            self.cap = None
        if self.pFrameBuffer:
            mvsdk.CameraAlignFree(self.pFrameBuffer)
            self.pFrameBuffer = None

    def closeEvent(self, event):
        self.close_camera()
        event.accept()

def main():
    app = QApplication(sys.argv)
    w = CameraApp()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
