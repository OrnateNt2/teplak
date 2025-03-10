import sys
import time
import platform
import numpy as np
import mvsdk

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QGroupBox, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

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

class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MvSDK Demo: Video Output & FPS (PyQt)")

        # Камера
        self.hCamera = None
        self.monoCamera = False
        self.capability = None
        self.pFrameBuffer = None
        self.FrameBufferSize = 0

        # Списки для выбора
        self.devList = mvsdk.CameraEnumerateDevice()    # камеры
        self.resList = []                               # разрешения
        self.loadModes = [("ByModel", 0),
                          ("ByName", 1),
                          ("BySN", 2)]

        # Переменные для расчёта FPS
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0

        # Интерфейс
        self.initUI()

        # Таймер для обновления
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        # По умолчанию ставим 5 мс, чтобы максимально часто считывать кадры
        self.timer.start(5)

    def initUI(self):
        """
        Создаём элементы управления:
        - Выбор камеры
        - Выбор load mode (ByModel, ByName, BySN)
        - Кнопка "Open camera"
        - Выбор разрешения
        - Кнопка "Set resolution"
        - QLabel для вывода изображения
        - QLabel для вывода FPS
        """

        # --- Выбор камеры ---
        self.comboCamera = QComboBox()
        for i, dev in enumerate(self.devList):
            text = f"{i}: {dev.GetFriendlyName()} (SN:{dev.GetSn()})"
            self.comboCamera.addItem(text)
        # Если нет камер, comboCamera будет пустым

        # --- Выбор Load Mode (ByModel, ByName, BySN) ---
        self.comboLoadMode = QComboBox()
        for (name, val) in self.loadModes:
            self.comboLoadMode.addItem(name, val)
        self.comboLoadMode.setCurrentIndex(2)  # По умолчанию BySN (если хотим)

        # --- Кнопка "Open camera" ---
        self.btnOpen = QPushButton("Open Camera")
        self.btnOpen.clicked.connect(self.on_open_camera)

        # --- Выбор разрешения ---
        self.comboRes = QComboBox()  
        self.btnSetRes = QPushButton("Set Resolution")
        self.btnSetRes.clicked.connect(self.on_set_resolution)

        # --- Метки для изображения и FPS ---
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

        vlayout = QVBoxLayout()
        vlayout.addLayout(row1)
        vlayout.addLayout(row2)
        vlayout.addWidget(self.labelView)
        vlayout.addWidget(self.labelFps)

        self.setLayout(vlayout)
        self.setMinimumSize(800, 600)

    def on_open_camera(self):
        """
        Инициализация камеры по выбранному LoadMode + DevInfo
        """
        idx = self.comboCamera.currentIndex()
        if idx < 0 or idx >= len(self.devList):
            return
        DevInfo = self.devList[idx]

        loadMode = self.comboLoadMode.currentData()  # 0,1,2
        print(f"Opening camera idx={idx}, loadMode={loadMode}")

        # Если уже открыта — закрываем
        self.close_camera()

        # Инициализация
        try:
            self.hCamera = mvsdk.CameraInit(DevInfo, loadMode, -1)
        except mvsdk.CameraException as e:
            print("CameraInit Failed:", e)
            self.hCamera = None
            return

        # Получаем capability
        self.capability = mvsdk.CameraGetCapability(self.hCamera)
        self.monoCamera = (self.capability.sIspCapacity.bMonoSensor != 0)

        # Формат вывода
        if self.monoCamera:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # Непрерывная съемка (если нужно, можно вызвать CameraSetTriggerMode(hCamera, 0))
        # Выключим автоэкспозицию, выставим ~30 мс (пример)
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

        # Заполним comboRes (список предустановленных разрешений)
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

        print("Camera opened successfully.")

    def on_set_resolution(self):
        """
        Применяем выбранное (из comboRes) разрешение
        """
        if self.hCamera is None:
            return
        idxRes = self.comboRes.currentIndex()
        if idxRes < 0 or idxRes >= len(self.resList):
            return

        newRes = self.resList[idxRes]
        err = mvsdk.CameraSetImageResolution(self.hCamera, newRes)
        if err != 0:
            print("CameraSetImageResolution failed:", err)
        else:
            print(f"Set resolution: {newRes.iWidthFOV}x{newRes.iHeightFOV}")

    def close_camera(self):
        """
        Закрыть камеру (если открыта).
        """
        if self.hCamera:
            mvsdk.CameraUnInit(self.hCamera)
            self.hCamera = None
        if self.pFrameBuffer:
            mvsdk.CameraAlignFree(self.pFrameBuffer)
            self.pFrameBuffer = None

    def update_frame(self):
        """
        Считываем кадр, отображаем, считаем FPS
        """
        if not self.hCamera:
            return

        # Считаем FPS
        current_time = time.time()
        self.frame_count += 1
        if (current_time - self.prev_time) >= 1.0:
            self.fps = self.frame_count / (current_time - self.prev_time)
            self.frame_count = 0
            self.prev_time = current_time
            # Обновим labelFps
            self.labelFps.setText(f"FPS: {self.fps:.2f}")

        # Пробуем получить кадр
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 50)
            # Обрабатываем RAW -> pFrameBuffer
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            # Windows => переворот
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(self.pFrameBuffer, FrameHead, 1)

            # Преобразуем в numpy
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)

            ch = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, ch))

            # (Опционально) Не делаем resize, чтобы не «съедать» FPS
            # Если окно слишком большое, PyQt может тормозить
            #frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

            # Показ в labelView
            pix = convert_frame_to_qpixmap(frame)
            self.labelView.setPixmap(pix)

        except mvsdk.CameraException as e:
            # -12 = CAMERA_STATUS_TIME_OUT
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed:", e)
                # Можно убрать break, чтобы продолжить в цикле

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
