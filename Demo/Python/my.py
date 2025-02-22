# coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import tkinter as tk
from tkinter import ttk

def apply_trigger_settings(hCamera, trig_count_var, trig_delay_var):
    """
    Применяет настройки внешнего триггера:
      - Trigger Count: количество кадров на один импульс
      - Trigger Delay (us): задержка в микросекундах перед захватом кадра
    """
    try:
        count_val = int(trig_count_var.get())
        delay_val = int(trig_delay_var.get())
    except Exception as e:
        print("Ошибка ввода значений триггера:", e)
        return

    ret1 = mvsdk.CameraSetTriggerCount(hCamera, count_val)
    ret2 = mvsdk.CameraSetTriggerDelayTime(hCamera, delay_val)
    if ret1 != 0 or ret2 != 0:
        print("Ошибка установки параметров триггера (ret1={}, ret2={})".format(ret1, ret2))
    else:
        print("Параметры триггера успешно установлены:")
        print(f"  Trigger Count = {count_val}, Trigger Delay = {delay_val} us")

def show_trigger_settings_window(hCamera):
    """ Открывает окно для ручной настройки параметров внешнего триггера """
    win = tk.Tk()
    win.title("Настройки внешнего триггера (hardware)")

    trig_count_var = tk.StringVar(value="2")
    trig_delay_var = tk.StringVar(value="12500")  # Пример для лазера 20 Гц

    ttk.Label(win, text="Trigger Count:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
    ttk.Entry(win, textvariable=trig_count_var, width=10).grid(row=0, column=1, padx=5, pady=5)

    ttk.Label(win, text="Trigger Delay (us):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
    ttk.Entry(win, textvariable=trig_delay_var, width=10).grid(row=1, column=1, padx=5, pady=5)

    ttk.Button(win, text="Применить", 
               command=lambda: apply_trigger_settings(hCamera, trig_count_var, trig_delay_var)
              ).grid(row=2, column=0, columnspan=2, padx=5, pady=10)

    win.mainloop()

def main_loop():
    # Перечисляем камеры
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    idx = 0 if nDev == 1 else int(input("Select camera (index): "))
    DevInfo = DevList[idx]
    print("Выбрана камера:")
    print(DevInfo)

    # Инициализация по имени (ByName)
    cameraName = DevInfo.GetFriendlyName()
    try:
        hCamera = mvsdk.CameraInitEx2(cameraName)
    except mvsdk.CameraException as e:
        print("CameraInitEx2 Failed({}): {}".format(e.error_code, e.message))
        return

    # Получаем характеристики камеры
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Выбор режима работы
    print("\nВыберите режим работы камеры:")
    print("1 - Непрерывный захват (без внешнего триггера)")
    print("2 - Внешний аппаратный триггер (ByName, hardware trigger)")
    mode = input("Введите номер режима (1 или 2): ")

    if mode == "1":
        print("Режим непрерывного захвата выбран.")
        mvsdk.CameraSetTriggerMode(hCamera, 0)
    elif mode == "2":
        print("Режим внешнего аппаратного триггера выбран.")
        # Устанавливаем режим внешнего триггера
        ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
        if ret != 0:
            print("Ошибка установки режима триггера:", ret)
            mvsdk.CameraUnInit(hCamera)
            return
        # Устанавливаем тип внешнего триггера – попробуйте EXT_TRIG_HIGH_LEVEL или EXT_TRIG_LOW_LEVEL,
        # в зависимости от уровня вашего импульса.
        ret = mvsdk.CameraSetExtTrigSignalType(hCamera, mvsdk.EXT_TRIG_HIGH_LEVEL)
        if ret != 0:
            print("Ошибка установки типа внешнего триггера:", ret)
        # Устанавливаем тип срабатывания затвора (если требуется)
        ret = mvsdk.CameraSetExtTrigShutterType(hCamera, mvsdk.EXT_TRIG_HIGH_LEVEL)
        if ret != 0:
            print("Ошибка установки типа срабатывания затвора:", ret)
        # Можно установить задержку внешнего триггера, если требуется (например, 0)
        ret = mvsdk.CameraSetExtTrigDelayTime(hCamera, 0)
        if ret != 0:
            print("Ошибка установки внешней задержки триггера:", ret)
        # Открываем окно для ручной настройки Trigger Count и Trigger Delay
        show_trigger_settings_window(hCamera)
    else:
        print("Некорректный выбор режима. Выход.")
        mvsdk.CameraUnInit(hCamera)
        return

    # Настройка экспозиции (ручной режим, 30 мс)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Запускаем захват изображений
    mvsdk.CameraPlay(hCamera)

    # Вычисляем размер буфера для изображения
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    print("Нажмите 'q' для выхода.")
    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        try:
            # Увеличиваем timeout до 2000 мс
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
            frame = cv2.resize(frame, (640,480), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Press q to end", frame)
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
            else:
                print("Time out, повторяем попытку...")

    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
