import sys
import time
import platform
import re
import cv2    # только для cv2.putText и cv2.absdiff (опционально, можно заменить на mvsdk.CameraDrawText)
import numpy as np
import mvsdk

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QGroupBox, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

############################
# Парсер для settings.Config
############################
def parse_config_file(file_path):
    """
    Простой парсер для извлечения строк "ключ = значение;"
    Возвращает словарь {key: val}
    """
    params = {}
    pattern = re.compile(r'(\w+)\s*=\s*"?([\w\.\-]+)"?\s*;')

    with open(file_path, 'r', encoding='utf-8') as f:
        text = f.read()

    for match in pattern.finditer(text):
        key = match.group(1)
        val = match.group(2)
        # Попытка конвертации
        if re.fullmatch(r'\d+', val):
            val = int(val)
        elif re.fullmatch(r'\d+\.\d+', val):
            val = float(val)
        params[key] = val
    return params

############################
# Конвертация numpy -> QPixmap
############################
def convert_frame_to_qpixmap(frame):
    """
    Numpy (BGR или GRAY) -> QImage -> QPixmap
    """
    h, w = frame.shape[:2]

    # Проверяем, монохром или цвет
    if len(frame.shape) == 2 or frame.shape[2] == 1:
        # GRAY
        # frame.shape = (h, w) или (h, w, 1)
        # Нужно bytes_per_line = w
        bytes_per_line = w
        gray_data = frame.tobytes()
        qimg = QImage(gray_data, w, h, bytes_per_line, QImage.Format_Grayscale8)
    else:
        # Цвет (BGR или RGB)
        # frame.shape = (h, w, 3)
        # Допустим, frame - BGR, делаем BGR->RGB
        rgb = frame[..., ::-1]  
        ch = 3
        bytes_per_line = ch * w
        rgb_data = rgb.tobytes()
        qimg = QImage(rgb_data, w, h, bytes_per_line, QImage.Format_RGB888)

    return QPixmap.fromImage(qimg)


############################
# Основной класс
############################
class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MvSDK + PyQt Demo (Video + Diff)")

        # Камера и буферы
        self.hCamera = None
        self.capability = None
        self.monoCamera = False
        self.pFrameBuffer = None
        self.FrameBufferSize = 0

        # Списки
        self.devList = mvsdk.CameraEnumerateDevice()
        self.resList = []
        self.loadModes = [("ByModel", 0), ("ByName", 1), ("BySN", 2)]

        # config
        self.config_file_path = "settings.Config"
        self.config_params = {}
        self.load_config()

        # Параметры FPS
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0

        # Параметры фонарика
        self.flash_freq = 10.0     # Гц, по умолчанию
        self.flash_period = 1.0 / self.flash_freq
        self.phase = 0.0

        # Режим разности
        self.prev_frame = None

        # Интерфейс
        self.initUI()

        # Таймер
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(5)

    def load_config(self):
        """
        Пытаемся прочитать settings.Config
        """
        try:
            self.config_params = parse_config_file(self.config_file_path)
            print("Config loaded:", self.config_params)
        except Exception as e:
            print("Не удалось загрузить config:", e)
            self.config_params = {}

    def initUI(self):
        """
        Создание интерфейса:
         - ComboBox для выбора камеры
         - ComboBox для выбора loadMode
         - Кнопка "Open"
         - ComboBox для разрешения + кнопка "Set Res"
         - Кнопка "Apply Config"
         - QLineEdit для ввода частоты фонарика
         - CheckBox для Diff Mode
         - Label для исходного кадра
         - Label для разности
         - Label для FPS
        """

        # 1) Выбор камеры
        self.comboCamera = QComboBox()
        for i, dev in enumerate(self.devList):
            txt = f"{i}: {dev.GetFriendlyName()} (SN:{dev.GetSn()})"
            self.comboCamera.addItem(txt)

        # 2) Выбор loadMode
        self.comboLoadMode = QComboBox()
        for (name, val) in self.loadModes:
            self.comboLoadMode.addItem(name, val)

        # 3) Кнопка Open
        self.btnOpen = QPushButton("Open Camera")
        self.btnOpen.clicked.connect(self.on_open_camera)

        # 4) Выбор разрешения
        self.comboRes = QComboBox()
        self.btnSetRes = QPushButton("Set Resolution")
        self.btnSetRes.clicked.connect(self.on_set_resolution)

        # 5) ApplyConfig
        self.btnApplyConf = QPushButton("Apply Config")
        self.btnApplyConf.clicked.connect(self.on_apply_config)

        # 6) Частота фонарика
        self.editFreq = QLineEdit("10.0")
        self.editFreq.setFixedWidth(60)
        self.editFreq.editingFinished.connect(self.on_freq_changed)

        # 7) CheckBox "Difference"
        self.checkDiff = QCheckBox("Difference Mode")
        self.checkDiff.setChecked(False)

        # Основные лейблы (Original / Diff)
        self.labelOrig = QLabel("No camera")
        self.labelOrig.setScaledContents(True)

        self.labelDiff = QLabel("Diff off")
        self.labelDiff.setScaledContents(True)
        self.labelDiff.setVisible(False)  # по умолчанию скрыт, если checkDiff не активен

        # FPS
        self.labelFps = QLabel("FPS: 0.0")

        # Компоновка
        row1 = QHBoxLayout()
        row1.addWidget(self.comboCamera)
        row1.addWidget(self.comboLoadMode)
        row1.addWidget(self.btnOpen)

        row2 = QHBoxLayout()
        row2.addWidget(self.comboRes)
        row2.addWidget(self.btnSetRes)
        row2.addWidget(self.btnApplyConf)
        row2.addWidget(QLabel("Flash Freq:"))
        row2.addWidget(self.editFreq)
        row2.addWidget(self.checkDiff)

        rowImages = QHBoxLayout()
        rowImages.addWidget(self.labelOrig)
        rowImages.addWidget(self.labelDiff)

        vlayout = QVBoxLayout()
        vlayout.addLayout(row1)
        vlayout.addLayout(row2)
        vlayout.addLayout(rowImages)
        vlayout.addWidget(self.labelFps)

        self.setLayout(vlayout)
        self.setMinimumSize(1000, 600)

    def on_open_camera(self):
        """
        Инициализация камеры (заданной в comboCamera) с loadMode
        """
        idx = self.comboCamera.currentIndex()
        if idx < 0 or idx >= len(self.devList):
            return
        DevInfo = self.devList[idx]
        loadMode = self.comboLoadMode.currentData()  # 0,1,2

        # Закроем, если была открыта
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

        # Непрерывная съемка, ручная экспозиция (30ms)
        #mvsdk.CameraSetTriggerMode(self.hCamera, 0)
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

        # Заполним comboRes
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
        """
        Применяем выбранный из comboRes вариант (CameraSetImageResolution)
        """
        if not self.hCamera:
            return
        idxRes = self.comboRes.currentIndex()
        if idxRes < 0 or idxRes >= len(self.resList):
            return
        newRes = self.resList[idxRes]
        err = mvsdk.CameraSetImageResolution(self.hCamera, newRes)
        if err == 0:
            print(f"Set resolution => iIndex={newRes.iIndex}, ROI=({newRes.iWidthFOV}x{newRes.iHeightFOV})")
            # Сброс prev_frame
            self.prev_frame = None
        else:
            print("CameraSetImageResolution failed:", err)

    def on_apply_config(self):
        """
        Применяем ROI, exp_time, analog_gain из settings.Config
        (примерно как в предыдущем коде).
        """
        if not self.hCamera:
            return
        cfg = self.config_params

        iIndex = cfg.get("iIndex", None)
        if iIndex is not None:
            iHOffsetFOV = cfg.get("iHOffsetFOV", 0)
            iVOffsetFOV = cfg.get("iVOffsetFOV", 0)
            iWidthFOV   = cfg.get("iWidthFOV", 640)
            iHeightFOV  = cfg.get("iHeightFOV", 480)
            # iWidth / iHeight для выхода, часто совпадают
            iWidth  = cfg.get("iWidth", iWidthFOV)
            iHeight = cfg.get("iHeight", iHeightFOV)

            err = mvsdk.CameraSetImageResolutionEx(
                self.hCamera,
                iIndex,  # iIndex
                0,       # Mode
                0,       # ModeSize
                iHOffsetFOV,
                iVOffsetFOV,
                iWidthFOV,
                iHeightFOV,
                0,
                0
            )
            if err == 0:
                print(f"Apply config => iIndex={iIndex}, ROI=({iWidthFOV}x{iHeightFOV}), offset=({iHOffsetFOV},{iVOffsetFOV})")
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
        """
        Пользователь завершил ввод "Flash Freq"
        """
        txt = self.editFreq.text().strip()
        try:
            val = float(txt)
            if val > 0:
                self.flash_freq = val
                self.flash_period = 1.0 / val
                print(f"Flash frequency set to {val} Hz, T={self.flash_period}")
        except ValueError:
            print("Invalid freq input")

    def close_camera(self):
        if self.hCamera:
            mvsdk.CameraUnInit(self.hCamera)
            self.hCamera = None
        if self.pFrameBuffer:
            mvsdk.CameraAlignFree(self.pFrameBuffer)
            self.pFrameBuffer = None

    def update_frame(self):
        """
        Считываем кадр и показываем.
        Если Difference Mode включён, формируем diff с prev_frame.
        Рассчитываем FPS и фазу мерцания (flash_freq).
        """
        if not self.hCamera:
            return

        # FPS
        current_time = time.time()
        self.frame_count += 1
        if (current_time - self.prev_time) >= 1.0:
            self.fps = self.frame_count / (current_time - self.prev_time)
            self.frame_count = 0
            self.prev_time = current_time

        # Фаза
        if self.flash_period > 0:
            self.phase = ((current_time) % self.flash_period) / self.flash_period
        else:
            self.phase = 0.0

        try:
            # Захват
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 50)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            # Windows => переворот
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(self.pFrameBuffer, FrameHead, 1)

            # В numpy
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)

            ch = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, ch))

            # Пример: добавим текст (FPS, Phase, Freq) прямо на кадр. 
            # Для этого используем cv2.putText (требует frame BGR/GRAY).
            # Mono => у нас shape=(h,w,1). cv2.putText требует (h,w,3) BGR. 
            # Упростим: если mono, конвертируем в BGR:
            if ch == 1:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            cv2.putText(frame, f"FPS: {self.fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Phase: {self.phase:.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"FlashFreq: {self.flash_freq:.2f}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Преобразуем обратно в нужный формат (если monoCamera), 
            # но можно оставить 3-канальный BGR - для отображения QPixmap нормально
            final_frame = frame

            # Вывод
            pix_orig = convert_frame_to_qpixmap(final_frame)
            self.labelOrig.setPixmap(pix_orig)

            # Режим разности
            if self.checkDiff.isChecked():
                self.labelDiff.setVisible(True)
                if self.prev_frame is not None:
                    # Считаем разницу
                    # У нас prev_frame мог быть другого размера, 
                    # но обычно нет - ROI не меняем на ходу
                    # Убедимся, что prev_frame тоже 3-channel
                    if len(self.prev_frame.shape) == 2:
                        prev_bgr = cv2.cvtColor(self.prev_frame, cv2.COLOR_GRAY2BGR)
                    elif self.prev_frame.shape[2] == 1:
                        prev_bgr = cv2.cvtColor(self.prev_frame, cv2.COLOR_GRAY2BGR)
                    else:
                        prev_bgr = self.prev_frame

                    diff = cv2.absdiff(frame, prev_bgr)
                    cv2.putText(diff, f"FPS: {self.fps:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(diff, f"Phase: {self.phase:.2f}", (10, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(diff, f"FlashFreq: {self.flash_freq:.2f}", (10, 110),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    pix_diff = convert_frame_to_qpixmap(diff)
                    self.labelDiff.setPixmap(pix_diff)
                else:
                    self.labelDiff.setText("No prev_frame yet")
            else:
                self.labelDiff.setVisible(False)

            # Сохраняем текущий кадр как prev_frame (у нас BGR 3-канальный)
            self.prev_frame = frame.copy()  # BGR

            # FPS label
            self.labelFps.setText(f"FPS: {self.fps:.2f}")

        except mvsdk.CameraException as e:
            # -12 = CAMERA_STATUS_TIME_OUT
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
