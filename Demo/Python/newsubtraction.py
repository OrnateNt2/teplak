import sys
import time
import platform
import re   # для простого парсинга
import numpy as np
import mvsdk

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QGroupBox, QLineEdit
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap


####################################
# Вспомогательные функции
####################################

def parse_config_file(file_path):
    """
    Простейший парсер для settings.Config.
    Ищет нужные ключи по регулярным выражениям.
    Возвращает словарь с параметрами.
    """
    params = {}
    # Пример:
    #  iIndex = 4;
    #  iHOffsetFOV = 976;
    #  exp_time = 14580.0;
    pattern = re.compile(r'(\w+)\s*=\s*"?([\w\.\-]+)"?\s*;')

    with open(file_path, 'r', encoding='utf-8') as f:
        text = f.read()

    for match in pattern.finditer(text):
        key = match.group(1)
        val = match.group(2)
        # Попробуем привести val к float или int
        if re.fullmatch(r'\d+', val):
            val = int(val)
        elif re.fullmatch(r'\d+\.\d+', val):
            val = float(val)
        params[key] = val

    return params


def convert_frame_to_qpixmap(frame):
    """
    Numpy (BGR или GRAY) -> QImage -> QPixmap
    """
    if len(frame.shape) == 2 or frame.shape[2] == 1:
        # GRAY
        h, w = frame.shape[:2]
        bytes_per_line = w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_Grayscale8)
    else:
        # BGR -> RGB
        h, w, ch = frame.shape
        rgb = frame[..., ::-1]
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg)


####################################
#   Основной класс PyQt
####################################

class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MvSDK + settings.Config Demo")

        # Камера
        self.hCamera = None
        self.capability = None
        self.monoCamera = False
        self.pFrameBuffer = None
        self.FrameBufferSize = 0

        # Списки
        self.devList = mvsdk.CameraEnumerateDevice()
        self.resList = []

        # Подгрузим конфиг (если есть)
        self.config_params = {}
        self.config_file_path = "settings.Config"  # имя файла по умолчанию
        # Попробуем считать
        self.load_config_file()

        # LoadModes (ByModel=0, ByName=1, BySN=2)
        self.loadModes = [("ByModel", 0),
                          ("ByName", 1),
                          ("BySN", 2)]

        # Для подсчёта FPS
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0

        # Интерфейс
        self.initUI()

        # Таймер
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(5)

    def load_config_file(self):
        """
        Загружаем settings.Config (если он есть),
        парсим ключи, например iHOffsetFOV=..., iIndex=..., exp_time=...
        Результат храним в self.config_params.
        """
        try:
            self.config_params = parse_config_file(self.config_file_path)
            print("Config loaded:", self.config_params)
        except Exception as e:
            print("Не удалось прочитать config:", e)
            self.config_params = {}

    def initUI(self):
        """
        Создаём элементы GUI:
        - Выбор камеры
        - Выбор loadMode (ByModel/ByName/BySN)
        - Кнопка "Open"
        - Кнопка "Apply config" (чтобы выставлять resolution / exposure из config)
        - Выбор разрешений (через ComboBox)
        - QLabel для видео + fps
        """
        # --- Выбор камеры ---
        self.comboCamera = QComboBox()
        for i, dev in enumerate(self.devList):
            text = f"{i}: {dev.GetFriendlyName()} (SN:{dev.GetSn()})"
            self.comboCamera.addItem(text)
        self.comboCamera.setCurrentIndex(0)

        # --- Выбор load mode ---
        self.comboLoadMode = QComboBox()
        for (name, val) in self.loadModes:
            self.comboLoadMode.addItem(name, val)
        # Если хотите по умолчанию BySN, ставьте setCurrentIndex(2)

        # --- Кнопки ---
        self.btnOpen = QPushButton("Open Camera")
        self.btnOpen.clicked.connect(self.on_open_camera)

        self.btnApplyConfig = QPushButton("Apply Config")
        self.btnApplyConfig.clicked.connect(self.on_apply_config)

        # --- Выбор разрешения ---
        self.comboRes = QComboBox()
        self.btnSetRes = QPushButton("Set Resolution")
        self.btnSetRes.clicked.connect(self.on_set_resolution)

        # --- Label для видео, FPS
        self.labelView = QLabel("No camera")
        self.labelView.setScaledContents(True)
        self.labelFps = QLabel("FPS: 0.0")

        # Компоновка
        row1 = QHBoxLayout()
        row1.addWidget(self.comboCamera)
        row1.addWidget(self.comboLoadMode)
        row1.addWidget(self.btnOpen)

        row2 = QHBoxLayout()
        row2.addWidget(self.comboRes)
        row2.addWidget(self.btnSetRes)
        row2.addWidget(self.btnApplyConfig)

        vlayout = QVBoxLayout()
        vlayout.addLayout(row1)
        vlayout.addLayout(row2)
        vlayout.addWidget(self.labelView)
        vlayout.addWidget(self.labelFps)

        self.setLayout(vlayout)
        self.setMinimumSize(800, 600)

    def on_open_camera(self):
        """
        Инициализация выбранной камеры по выбранному LoadMode
        """
        idx = self.comboCamera.currentIndex()
        if idx < 0 or idx >= len(self.devList):
            return
        DevInfo = self.devList[idx]

        loadMode = self.comboLoadMode.currentData()  # 0,1,2
        print("Open camera idx=", idx, "loadMode=", loadMode)
        self.close_camera()

        try:
            self.hCamera = mvsdk.CameraInit(DevInfo, loadMode, -1)
        except mvsdk.CameraException as e:
            print("CameraInit failed:", e)
            self.hCamera = None
            return

        self.capability = mvsdk.CameraGetCapability(self.hCamera)
        self.monoCamera = (self.capability.sIspCapacity.bMonoSensor != 0)

        # Настраиваем ISP формат
        if self.monoCamera:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # Режим непрерывного захвата
        #mvsdk.CameraSetTriggerMode(self.hCamera, 0)

        # Выключаем автоэкспозицию, ставим 30ms (пример)
        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 30000)

        # Запуск
        mvsdk.CameraPlay(self.hCamera)

        # Выделяем буфер
        maxw = self.capability.sResolutionRange.iWidthMax
        maxh = self.capability.sResolutionRange.iHeightMax
        depth = 1 if self.monoCamera else 3
        self.FrameBufferSize = maxw * maxh * depth
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        # Заполняем comboRes
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

    def on_set_resolution(self):
        """
        Применяем выбранный в comboRes вариант
        """
        if not self.hCamera:
            return
        idxRes = self.comboRes.currentIndex()
        if idxRes < 0 or idxRes >= len(self.resList):
            return
        newRes = self.resList[idxRes]
        err = mvsdk.CameraSetImageResolution(self.hCamera, newRes)
        if err == 0:
            print(f"Set resolution: iIndex={newRes.iIndex}, {newRes.iWidthFOV}x{newRes.iHeightFOV}")
        else:
            print("CameraSetImageResolution failed:", err)
        # Иногда нужно перезапустить поток
        # mvsdk.CameraPlay(self.hCamera)

    def on_apply_config(self):
        """
        Применяем параметры из config (пример: ROI / экспозицию).
        Допустим, у нас есть iIndex, iWidthFOV, iHeightFOV, iHOffsetFOV, iVOffsetFOV, exp_time
        """
        if not self.hCamera:
            return
        cfg = self.config_params

        iIndex = cfg.get("iIndex", None)
        if iIndex is not None:
            # У нас есть все поля, чтобы вызвать CameraSetImageResolutionEx
            # (чтобы точно задать ROI)
            iHOffsetFOV = cfg.get("iHOffsetFOV", 0)
            iVOffsetFOV = cfg.get("iVOffsetFOV", 0)
            iWidthFOV   = cfg.get("iWidthFOV", 640)
            iHeightFOV  = cfg.get("iHeightFOV", 480)
            iWidth      = cfg.get("iWidth", iWidthFOV)
            iHeight     = cfg.get("iHeight", iHeightFOV)

            # Пример вызова CameraSetImageResolutionEx
            # Аргументы:
            #  hCamera,
            #  iIndex, Mode=0, ModeSize=0,
            #  x, y, w, h,
            #  ZoomW=0, ZoomH=0
            err = mvsdk.CameraSetImageResolutionEx(
                self.hCamera,
                iIndex,     # iIndex
                0,          # Mode
                0,          # ModeSize
                iHOffsetFOV,
                iVOffsetFOV,
                iWidthFOV,
                iHeightFOV,
                0,
                0
            )
            if err == 0:
                print(f"Apply config ROI: iIndex={iIndex}, ROI=({iHOffsetFOV},{iVOffsetFOV},{iWidthFOV}x{iHeightFOV})")
            else:
                print("CameraSetImageResolutionEx failed:", err)

        # Экспозиция
        exp_time = cfg.get("exp_time", None)
        if exp_time is not None:
            # Например, 14580 => ~14.58 ms
            # SDK требует [микросекунды]
            err = mvsdk.CameraSetExposureTime(self.hCamera, exp_time)
            if err == 0:
                print(f"Set exposureTime={exp_time} us")
            else:
                print("CameraSetExposureTime failed:", err)

        # anal_gain
        analog_gain = cfg.get("analog_gain", None)
        if analog_gain is not None:
            # Если SDK поддерживает SetAnalogGain / SetAnalogGainX
            # analog_gain = 4 => ...
            # Пример (в зависимости от вашей модели):
            # mvsdk.CameraSetAnalogGainX(self.hCamera, float(analog_gain))
            # или:
            mvsdk.CameraSetAnalogGain(self.hCamera, int(analog_gain))
            print(f"Set analog gain={analog_gain}")

        # При необходимости можно вызвать mvsdk.CameraPlay(self.hCamera) заново

    def close_camera(self):
        if self.hCamera:
            mvsdk.CameraUnInit(self.hCamera)
            self.hCamera = None
        if self.pFrameBuffer:
            mvsdk.CameraAlignFree(self.pFrameBuffer)
            self.pFrameBuffer = None

    def update_frame(self):
        if not self.hCamera:
            return

        current_time = time.time()
        self.frame_count += 1
        if current_time - self.prev_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.prev_time)
            self.frame_count = 0
            self.prev_time = current_time
            self.labelFps.setText(f"FPS: {self.fps:.2f}")

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

            # Без resize, чтобы не «съедать» FPS
            pix = convert_frame_to_qpixmap(frame)
            self.labelView.setPixmap(pix)

        except mvsdk.CameraException as e:
            # -12 = CAMERA_STATUS_TIME_OUT
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed:", e)

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
