# coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import tkinter as tk
from tkinter import ttk

def apply_trigger_settings(hCamera, trig_count, trig_delay, trig_interval):
    """ Применяет настройки триггера к камере """
    try:
        count_val = int(trig_count.get())
        delay_val = int(trig_delay.get())
        interval_val = int(trig_interval.get())
    except Exception as e:
        print("Ошибка ввода значений триггера:", e)
        return

    mvsdk.CameraSetTriggerCount(hCamera, count_val)
    mvsdk.CameraSetTriggerDelayTime(hCamera, delay_val)
    mvsdk.CameraSetExtTrigJitterTime(hCamera, interval_val)
    print("Параметры триггера применены:")
    print(f"Trigger Count = {count_val}, Delay = {delay_val} us, Interval = {interval_val} us")

def show_trigger_settings_window(hCamera):
    """ Окно настроек триггера """
    settings_window = tk.Tk()
    settings_window.title("Настройки триггера")

    # Поля для ввода параметров
    trig_count = tk.StringVar(value="1")
    trig_delay = tk.StringVar(value="0")
    trig_interval = tk.StringVar(value="10000")

    ttk.Label(settings_window, text="Trigger Count:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
    count_entry = ttk.Entry(settings_window, textvariable=trig_count, width=10)
    count_entry.grid(row=0, column=1, padx=5, pady=5)

    ttk.Label(settings_window, text="Delay (us):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
    delay_entry = ttk.Entry(settings_window, textvariable=trig_delay, width=10)
    delay_entry.grid(row=1, column=1, padx=5, pady=5)

    ttk.Label(settings_window, text="Interval (us):").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
    interval_entry = ttk.Entry(settings_window, textvariable=trig_interval, width=10)
    interval_entry.grid(row=2, column=1, padx=5, pady=5)

    apply_button = ttk.Button(settings_window, text="Применить", 
                              command=lambda: apply_trigger_settings(hCamera, trig_count, trig_delay, trig_interval))
    apply_button.grid(row=3, column=0, columnspan=2, padx=5, pady=10)

    settings_window.mainloop()

def main_loop():
    # Перечисление камер
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    print(DevInfo)

    # Инициализация камеры
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return

    # Получение характеристик камеры
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Выбор режима работы камеры
    print("\nВыберите режим работы камеры:")
    print("1 - Непрерывный захват (без триггера)")
    print("2 - Внешний триггер (с настройкой параметров)")
    mode = input("Введите номер режима (1 или 2): ")

    if mode == "1":
        print("Выбран режим непрерывного захвата.")
        mvsdk.CameraSetTriggerMode(hCamera, 0)  # Непрерывный захват
    elif mode == "2":
        print("Выбран режим внешнего триггера.")
        mvsdk.CameraSetTriggerMode(hCamera, 1)  # Внешний триггер
        show_trigger_settings_window(hCamera)
    else:
        print("Некорректный выбор. Выход.")
        mvsdk.CameraUnInit(hCamera)
        return

    # Настройка экспозиции (например, ручной режим с 30 мс)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Запуск захвата изображений
    mvsdk.CameraPlay(hCamera)

    # Выделение буфера для RGB (используем максимальное разрешение)
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    print("Нажмите 'q' для выхода.")
    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth))
            else:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 3))
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Press q to end", frame)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

    # Завершение работы
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

main()
