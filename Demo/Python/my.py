# coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import time

def input_int(prompt, default):
    try:
        s = input(f"{prompt} (по умолчанию {default}): ")
        return int(s) if s.strip() != "" else default
    except Exception:
        return default

def main_loop():
    # Перечисляем камеры
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("Камера не найдена!")
        return

    print("Список камер:")
    for i, DevInfo in enumerate(DevList):
        print(f"{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}")
    idx = 0 if nDev == 1 else int(input("Выберите камеру (индекс): "))
    DevInfo = DevList[idx]
    print("Выбрана камера:")
    print(DevInfo)

    # Инициализация по имени (ByName)
    cameraName = DevInfo.GetFriendlyName()
    try:
        hCamera = mvsdk.CameraInitEx2(cameraName)
    except mvsdk.CameraException as e:
        print(f"CameraInitEx2 Failed ({e.error_code}): {e.message}")
        return

    # Получаем характеристики камеры и устанавливаем формат ISP
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Выбор режима работы
    print("\nВыберите режим работы камеры:")
    print("1 - Непрерывный захват (без внешнего триггера)")
    print("2 - Аппаратный внешний триггер")
    mode = input("Введите номер режима (1 или 2): ")
    if mode == "1":
        print("Режим непрерывного захвата выбран.")
        mvsdk.CameraSetTriggerMode(hCamera, 0)
    elif mode == "2":
        print("Режим аппаратного внешнего триггера выбран.")
        ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
        if ret != 0:
            print(f"Ошибка установки режима внешнего триггера: {ret}")
            mvsdk.CameraUnInit(hCamera)
            return

        # Запрашиваем тип внешнего триггера
        print("Тип внешнего триггера:")
        print("  0 - LEADING_EDGE")
        print("  1 - TRAILING_EDGE")
        print("  2 - HIGH_LEVEL (по умолчанию)")
        print("  3 - LOW_LEVEL")
        print("  4 - DOUBLE_EDGE")
        extTrigType = input("Введите тип внешнего триггера (0-4, по умолчанию 2): ")
        extTrigType = int(extTrigType) if extTrigType.strip() != "" else 2
        ret = mvsdk.CameraSetExtTrigSignalType(hCamera, extTrigType)
        if ret != 0:
            print(f"Ошибка установки типа внешнего триггера: {ret}")
        ret = mvsdk.CameraSetExtTrigShutterType(hCamera, extTrigType)
        if ret != 0:
            print(f"Ошибка установки типа срабатывания затвора: {ret}")

        # Устанавливаем внешнюю задержку триггера
        extTrigDelay = input_int("Введите внешнюю задержку триггера (us)", 0)
        ret = mvsdk.CameraSetExtTrigDelayTime(hCamera, extTrigDelay)
        if ret != 0:
            print(f"Ошибка установки внешней задержки триггера: {ret}")

        # Устанавливаем интервал внешнего триггера (если требуется)
        intervalTime = input_int("Введите interval time (us) (0 по умолчанию)", 0)
        ret = mvsdk.CameraSetExtTrigIntervalTime(hCamera, intervalTime)
        if ret != 0:
            print(f"Ошибка установки interval time: {ret}")

        # Устанавливаем время подавления дребезга
        jitterTime = input_int("Введите jitter time (us) (0 по умолчанию)", 0)
        ret = mvsdk.CameraSetExtTrigJitterTime(hCamera, jitterTime)
        if ret != 0:
            print(f"Ошибка установки jitter time: {ret}")

        # Запрашиваем параметры захвата: Trigger Count и Trigger Delay (кадровый)
        trig_count = input_int("Введите Trigger Count (рекомендуется 2): ", 2)
        trig_delay = input_int("Введите Trigger Delay (us) (например, 12500): ", 12500)
        ret = mvsdk.CameraSetTriggerCount(hCamera, trig_count)
        if ret != 0:
            print(f"Ошибка установки Trigger Count: {ret}")
        ret = mvsdk.CameraSetTriggerDelayTime(hCamera, trig_delay)
        if ret != 0:
            print(f"Ошибка установки Trigger Delay: {ret}")
        print("Параметры внешнего триггера установлены:")
        print(f"  Trigger Count = {trig_count}, Trigger Delay = {trig_delay} us")
    else:
        print("Некорректный выбор режима. Выход.")
        mvsdk.CameraUnInit(hCamera)
        return

    # Настройка экспозиции: ручной режим, 30 мс
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Запускаем захват изображений
    mvsdk.CameraPlay(hCamera)

    # Выделяем буфер для изображения
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    print("Нажмите 'q' для выхода.")
    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        try:
            # Используем timeout 2000 мс – для внешнего триггера может потребоваться больше времени
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
                print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
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
