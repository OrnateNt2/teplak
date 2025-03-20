import sys
import time
import platform
import re
import cv2
import numpy as np
import mvsdk
from ultralytics import YOLO  # добавляем YOLOv8

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QLineEdit, QCheckBox, QSlider
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
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

class CameraWorker(QThread):
    updateFrameSignal = pyqtSignal(QPixmap, QPixmap, float)
    
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

                if self.camera_app.flash_period > 0:
                    phase = (current_time % self.camera_app.flash_period) / self.camera_app.flash_period
                else:
                    phase = 0.0

                # Захват и обработка кадра
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.camera_app.hCamera, 50)
                mvsdk.CameraImageProcess(self.camera_app.hCamera, pRawData, self.camera_app.pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(self.camera_app.hCamera, pRawData)

                if platform.system() == "Windows":
                    mvsdk.CameraFlipFrameBuffer(self.camera_app.pFrameBuffer, FrameHead, 1)

                frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.camera_app.pFrameBuffer)
                frame = np.frombuffer(frame_data, dtype=np.uint8)
                ch = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, ch))

                if self.camera_app.video_size is None:
                    h, w = frame.shape[:2]
                    self.camera_app.video_size = (w, h)
                    if (self.camera_app.is_recording and 
                        self.camera_app.writer_orig and 
                        self.camera_app.writer_diff):
                        self.camera_app.writer_orig.release()
                        self.camera_app.writer_diff.release()
                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        time_str = time.strftime("%Y%m%d_%H%M%S")
                        filename_orig = f"orig_{time_str}.avi"
                        filename_diff = f"diff_{time_str}.avi"
                        print("Adjusting video size to:", self.camera_app.video_size)
                        self.camera_app.writer_orig = cv2.VideoWriter(
                            filename_orig, fourcc, self.camera_app.video_fps, self.camera_app.video_size
                        )
                        self.camera_app.writer_diff = cv2.VideoWriter(
                            filename_diff, fourcc, self.camera_app.video_fps, self.camera_app.video_size
                        )

                if ch == 1:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                else:
                    frame_bgr = frame.copy()

                cv2.putText(frame_bgr, f"FPS: {self.fps:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                cv2.putText(frame_bgr, f"Phase: {phase:.2f}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                cv2.putText(frame_bgr, f"FlashFreq: {self.camera_app.flash_freq:.2f}", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

                # Если YOLO включена, запускаем детекцию и отрисовываем боксы
                if self.camera_app.yolo_enabled and self.camera_app.yolo_model is not None:
                    try:
                        results = self.camera_app.yolo_model(frame_bgr)
                        # Предполагается, что передаётся одно изображение, поэтому берем results[0]
                        boxes = results[0].boxes
                        for box in boxes:
                            # Извлекаем координаты, класс и уверенность
                            coords = box.xyxy[0]
                            x1, y1, x2, y2 = int(coords[0].item()), int(coords[1].item()), int(coords[2].item()), int(coords[3].item())
                            cls_id = int(box.cls[0].item())
                            conf = float(box.conf[0].item())
                            # Используем имена классов из модели (обычно в self.yolo_model.model.names)
                            label = f"{self.camera_app.yolo_model.model.names[cls_id]} {conf:.2f}"
                            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0,0,255), 2)
                            cv2.putText(frame_bgr, label, (x1, max(y1-10, 0)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                    except Exception as e:
                        print("YOLO detection error:", e)

                pix_orig = convert_frame_to_qpixmap(frame_bgr)

                if self.camera_app.checkDiff.isChecked():
                    if self.prev_frame is not None:
                        diff = cv2.absdiff(frame_bgr, self.prev_frame)
                        cv2.putText(diff, f"FPS: {self.fps:.2f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                        cv2.putText(diff, f"Phase: {phase:.2f}", (10, 70),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                        cv2.putText(diff, f"FlashFreq: {self.camera_app.flash_freq:.2f}", (10, 110),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                        pix_diff = convert_frame_to_qpixmap(diff)
                        diff_frame = diff
                    else:
                        pix_diff = QPixmap()
                        diff_frame = np.zeros_like(frame_bgr)
                else:
                    pix_diff = QPixmap()
                    diff_frame = np.zeros_like(frame_bgr)

                self.prev_frame = frame_bgr.copy()

                if (self.camera_app.is_recording and self.camera_app.writer_orig and 
                    self.camera_app.writer_diff and self.camera_app.video_size):
                    if (frame_bgr.shape[1], frame_bgr.shape[0]) != self.camera_app.video_size:
                        frame_bgr = cv2.resize(frame_bgr, self.camera_app.video_size)
                    if (diff_frame.shape[1], diff_frame.shape[0]) != self.camera_app.video_size:
                        diff_frame = cv2.resize(diff_frame, self.camera_app.video_size)
                    self.camera_app.writer_orig.write(frame_bgr)
                    self.camera_app.writer_diff.write(diff_frame)

                self.updateFrameSignal.emit(pix_orig, pix_diff, self.fps)

            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed:", e)

    def stop(self):
        self.running = False
        self.wait()

class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MvSDK + PyQt (Resolution, Config, Diff, Trigger, Recording with Time)")
        
        # --- Камера/буфер/параметры ---
        self.hCamera = None
        self.capability = None
        self.monoCamera = False
        self.pFrameBuffer = None
        self.FrameBufferSize = 0

        # Списки устройств
        self.devList = mvsdk.CameraEnumerateDevice()
        self.resList = []
        self.loadModes = [("ByModel", 0), ("ByName", 1), ("BySN", 2)]

        # --- Чтение config (ROI, exp_time, etc.) ---
        self.config_file_path = "settings.Config"
        self.config_params = {}
        self.load_config()

        # --- Параметры для FPS ---
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0

        # --- Параметры фонарика ---
        self.flash_freq = 10.0
        self.flash_period = 1.0 / self.flash_freq
        self.phase = 0.0

        # --- Для Difference Mode ---
        self.prev_frame = None

        # --- Для записи видео ---
        self.is_recording = False
        self.writer_orig = None
        self.writer_diff = None
        self.video_fps = 30.0  # FPS для записи
        self.video_size = None  # (width, height) – определяется при первом кадре

        self.worker = None  # Поток для захвата кадров

        # --- Параметры YOLO ---
        self.yolo_enabled = False
        self.yolo_model = None

        self.initUI()

    def load_config(self):
        try:
            self.config_params = parse_config_file(self.config_file_path)
            print("Config loaded:", self.config_params)
        except Exception as e:
            print("Не удалось загрузить config:", e)
            self.config_params = {}

    def initUI(self):
        # === Выбор камеры ===
        self.comboCamera = QComboBox()
        for i, dev in enumerate(self.devList):
            txt = f"{i}: {dev.GetFriendlyName()} (SN:{dev.GetSn()})"
            self.comboCamera.addItem(txt)

        # === Выбор LoadMode ===
        self.comboLoadMode = QComboBox()
        for (name, val) in self.loadModes:
            self.comboLoadMode.addItem(name, val)

        # === Кнопка Open ===
        self.btnOpen = QPushButton("Open Camera")
        self.btnOpen.clicked.connect(self.on_open_camera)

        # === ComboBox разрешений + кнопка SetRes ===
        self.comboRes = QComboBox()
        self.btnSetRes = QPushButton("Set Resolution")
        self.btnSetRes.clicked.connect(self.on_set_resolution)

        # === Кнопка Apply Config ===
        self.btnApplyConf = QPushButton("Apply Config")
        self.btnApplyConf.clicked.connect(self.on_apply_config)

        # === Поле ввода частоты фонарика ===
        self.editFreq = QLineEdit(str(self.flash_freq))
        self.editFreq.setFixedWidth(60)
        self.editFreq.editingFinished.connect(self.on_freq_changed)

        # === CheckBox Diff Mode ===
        self.checkDiff = QCheckBox("Difference Mode")
        self.checkDiff.setChecked(False)

        # === CheckBox Hardware Trigger ===
        self.checkHardwareTrigger = QCheckBox("Hardware Trigger")
        self.checkHardwareTrigger.setChecked(False)
        self.checkHardwareTrigger.stateChanged.connect(self.on_trigger_mode_changed)

        # === Кнопка Record (Start/Stop) ===
        self.btnRecord = QPushButton("Start Recording")
        self.btnRecord.clicked.connect(self.on_record_clicked)
        
        # === Новая кнопка для YOLO (включения/отключения детекции) ===
        self.btnYOLO = QPushButton("Enable YOLO")
        self.btnYOLO.setCheckable(True)
        self.btnYOLO.clicked.connect(self.on_yolo_clicked)

        # === Регулировка экспозиции через слайдер ===
        self.labelExposure = QLabel("Exposure: 30000 us")
        self.sliderExposure = QSlider(Qt.Horizontal)
        self.sliderExposure.setMinimum(100)
        self.sliderExposure.setMaximum(100000)
        self.sliderExposure.setValue(30000)
        self.sliderExposure.setTickInterval(10000)
        self.sliderExposure.valueChanged.connect(self.on_exposure_slider_changed)
        
        # === Регулировка усиления (analog gain) через слайдер ===
        self.labelGain = QLabel("Gain: 1")
        self.sliderGain = QSlider(Qt.Horizontal)
        self.sliderGain.setMinimum(1)
        self.sliderGain.setMaximum(16)
        self.sliderGain.setValue(1)
        self.sliderGain.setTickInterval(1)
        self.sliderGain.valueChanged.connect(self.on_gain_slider_changed)

        # === Панель для управления стробом ===
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

        # === Лейблы для вывода (Original / Diff / FPS) ===
        self.labelOrig = QLabel("No camera")
        self.labelOrig.setScaledContents(True)

        self.labelDiff = QLabel("Diff off")
        self.labelDiff.setScaledContents(True)
        self.labelDiff.setVisible(False)

        self.labelFps = QLabel("FPS: 0.0")

        # Layouts
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
        row2.addWidget(self.checkDiff)
        row2.addWidget(self.checkHardwareTrigger)
        row2.addWidget(self.btnRecord)
        row2.addWidget(self.btnYOLO)  # добавляем кнопку YOLO

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
        if self.hCamera:
            err = mvsdk.CameraSetExposureTime(self.hCamera, value)
            if err == 0:
                print(f"Exposure set to {value} us")
            else:
                print("Failed to set exposure, error code:", err)

    def on_gain_slider_changed(self, value):
        self.labelGain.setText(f"Gain: {value}")
        if self.hCamera:
            err = mvsdk.CameraSetAnalogGain(self.hCamera, int(value))
            if err == 0:
                print(f"Analog gain set to {value}")
            else:
                print("Failed to set analog gain, error code:", err)

    def on_open_camera(self):
        idx = self.comboCamera.currentIndex()
        if idx < 0 or idx >= len(self.devList):
            return
        DevInfo = self.devList[idx]
        loadMode = self.comboLoadMode.currentData()  # 0,1,2

        self.close_camera()

        try:
            self.hCamera = mvsdk.CameraInit(DevInfo, loadMode, -1)
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

        print("Camera opened OK.")
        self.prev_frame = None
        self.video_size = None

        if self.worker:
            self.worker.stop()
        self.worker = CameraWorker(self)
        self.worker.updateFrameSignal.connect(self.on_update_frame)
        self.worker.start()

    def on_set_resolution(self):
        if not self.hCamera:
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
        if not self.hCamera:
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
        if not self.hCamera:
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
        if not self.hCamera:
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

    def on_yolo_clicked(self):
        # Переключаем режим YOLO
        self.yolo_enabled = self.btnYOLO.isChecked()
        if self.yolo_enabled:
            self.btnYOLO.setText("Disable YOLO")
            # Загружаем модель, если ещё не загружена
            if self.yolo_model is None:
                try:
                    self.yolo_model = YOLO("yolov8n.pt")  # можно выбрать другую модель
                    print("YOLOv8 model loaded.")
                except Exception as e:
                    print("Failed to load YOLO model:", e)
                    self.yolo_enabled = False
                    self.btnYOLO.setChecked(False)
                    self.btnYOLO.setText("Enable YOLO")
        else:
            self.btnYOLO.setText("Enable YOLO")

    def on_update_frame(self, pix_orig, pix_diff, fps):
        self.labelOrig.setPixmap(pix_orig)
        if self.checkDiff.isChecked():
            self.labelDiff.setVisible(True)
            self.labelDiff.setPixmap(pix_diff)
        else:
            self.labelDiff.setVisible(False)
        self.labelFps.setText(f"FPS: {fps:.2f}")

    def close_camera(self):
        self.stop_recording()
        if self.worker:
            self.worker.stop()
            self.worker = None
        if self.hCamera:
            mvsdk.CameraUnInit(self.hCamera)
            self.hCamera = None
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
