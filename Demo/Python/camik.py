import sys
import time
import platform
import re
import cv2
import numpy as np
import mvsdk

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer, Qt
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

class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MvSDK + PyQt (Resolution, Config, Diff, Trigger)")

        # --- Камера/буфер/параметры ---
        self.hCamera = None
        self.capability = None
        self.monoCamera = False
        self.pFrameBuffer = None
        self.FrameBufferSize = 0

        self.devList = mvsdk.CameraEnumerateDevice()
        self.resList = []
        self.loadModes = [("ByModel", 0), ("ByName", 1), ("BySN", 2)]

        # --- Чтение config (ROI, exp_time, etc.) ---
        self.config_file_path = "settings.Config"
        self.config_params = {}
        self.load_config()

        # --- Счётчик для FPS ---
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0

        # --- Параметры фонарика (Flash Frequency) ---
        self.flash_freq = 10.0
        self.flash_period = 1.0 / self.flash_freq
        self.phase = 0.0

        # --- Для Difference Mode ---
        self.prev_frame = None

        self.initUI()

        # Таймер для покадрового чтения
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(5)  # можно уменьшить до 1

    def load_config(self):
        """
        Парсинг settings.Config
        """
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

        # === Лейблы для вывода (Original / Diff / FPS) ===
        self.labelOrig = QLabel("No camera")
        self.labelOrig.setScaledContents(True)

        self.labelDiff = QLabel("Diff off")
        self.labelDiff.setScaledContents(True)
        self.labelDiff.setVisible(False)  # скрыт, если DiffMode off

        self.labelFps = QLabel("FPS: 0.0")

        # --- Компоновка ---
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

        rowImages = QHBoxLayout()
        rowImages.addWidget(self.labelOrig)
        rowImages.addWidget(self.labelDiff)

        layout = QVBoxLayout()
        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addLayout(rowImages)
        layout.addWidget(self.labelFps)

        self.setLayout(layout)
        self.setMinimumSize(1000, 600)

    def on_open_camera(self):
        """
        Открытие камеры по выбранному DevInfo и LoadMode
        """
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

        # Непрерывный режим
        mvsdk.CameraSetTriggerMode(self.hCamera, 0)

        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 30000)
        # ПОМЕНЯТЬ 10000 = 10мс

        # Запуск потока
        mvsdk.CameraPlay(self.hCamera)

        maxw = self.capability.sResolutionRange.iWidthMax
        maxh = self.capability.sResolutionRange.iHeightMax
        ch = 1 if self.monoCamera else 3
        self.FrameBufferSize = maxw * maxh * ch
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        # Заполняем ComboRes
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
        """
        Применяем ROI/экспозицию/гейн из settings.Config
        """
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
        """
        Включаем/выключаем аппаратный триггер: 2 => HW, 0 => Continuous
        """
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

    def update_frame(self):
        if not self.hCamera:
            return

        current_time = time.time()
        self.frame_count += 1
        if (current_time - self.prev_time) >= 1.0:
            self.fps = self.frame_count / (current_time - self.prev_time)
            self.frame_count = 0
            self.prev_time = current_time

        # Фаза фонарика
        if self.flash_period > 0:
            self.phase = (current_time % self.flash_period) / self.flash_period
        else:
            self.phase = 0.0

        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 50)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(self.pFrameBuffer, FrameHead, 1)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            ch = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, ch))

            if ch == 1:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = frame

            cv2.putText(frame_bgr, f"FPS: {self.fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(frame_bgr, f"Phase: {self.phase:.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(frame_bgr, f"FlashFreq: {self.flash_freq:.2f}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            pix_orig = convert_frame_to_qpixmap(frame_bgr)
            self.labelOrig.setPixmap(pix_orig)

            # Difference Mode
            if self.checkDiff.isChecked():
                self.labelDiff.setVisible(True)
                if self.prev_frame is not None:
                    if len(self.prev_frame.shape) == 2 or self.prev_frame.shape[2] == 1:
                        prev_bgr = cv2.cvtColor(self.prev_frame, cv2.COLOR_GRAY2BGR)
                    else:
                        prev_bgr = self.prev_frame
                    diff = cv2.absdiff(frame_bgr, prev_bgr)

                    cv2.putText(diff, f"FPS: {self.fps:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                    cv2.putText(diff, f"Phase: {self.phase:.2f}", (10, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                    cv2.putText(diff, f"FlashFreq: {self.flash_freq:.2f}", (10, 110),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

                    pix_diff = convert_frame_to_qpixmap(diff)
                    self.labelDiff.setPixmap(pix_diff)
                else:
                    self.labelDiff.setText("No prev_frame yet")
            else:
                self.labelDiff.setVisible(False)

            self.prev_frame = frame_bgr.copy()

            self.labelFps.setText(f"FPS: {self.fps:.2f}")

            # -------------------------
            #   ВЫВОД СТАТИСТИКИ КАДРОВ
            # -------------------------
            # tSdkFrameStatistic: iTotal, iCapture, iLost
            stat = mvsdk.tSdkFrameStatistic()
            err_stat = mvsdk.CameraGetFrameStatistic(self.hCamera, stat)
            if err_stat == 0:
                print(f"FrameStat => total:{stat.iTotal}, capture:{stat.iCapture}, lost:{stat.iLost}")
            else:
                print("CameraGetFrameStatistic failed:", err_stat)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed:", e)

    def close_camera(self):
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
