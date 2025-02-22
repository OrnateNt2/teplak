# coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import time

def main_loop():
    # Перечисляем камеры
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("Камера не найдена!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    idx = 0 if nDev == 1 else int(input("Выберите камеру (индекс): "))
    DevInfo = DevList[idx]
    print("Выбрана камера:")
    print(DevInfo)

    # Инициализация по имени (ByName)
    cameraName = DevInfo.GetFriendlyName()
    try:
        hCamera = mvsdk.CameraInitEx2(cameraName)
    except mvsdk.CameraException as e:
        print("CameraInitEx2 Failed ({}): {}".format(e.error_code, e.message))
        return

    # Получаем характеристики камеры и задаем формат ISP
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Выбор режима работы камеры
    print("\nВыберите режим работы камеры:")
    print("1 - Непрерывный захват (без внешнего триггера)")
    print("2 - Аппаратный внешний триггер (hardware trigger)")
    mode = input("Введите номер режима (1 или 2): ")

    if mode == "1":
        print("Режим непрерывного захвата выбран.")
        mvsdk.CameraSetTriggerMode(hCamera, 0)  # Непрерывный режим
    elif mode == "2":
        print("Режим аппаратного внешнего триггера выбран.")
        # Устанавливаем режим внешнего триггера
        ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
        if ret != 0:
            print("Ошибка установки режима внешнего триггера: {}".format(ret))
            mvsdk.CameraUnInit(hCamera)
            return

        # Устанавливаем тип внешнего триггера и тип срабатывания затвора.
        # Если ваш генератор выдает высокий уровень при импульсе, используйте EXT_TRIG_HIGH_LEVEL;
        # иначе, если уровень низкий – EXT_TRIG_LOW_LEVEL.
        ret = mvsdk.CameraSetExtTrigSignalType(hCamera, mvsdk.EXT_TRIG_HIGH_LEVEL)
        if ret != 0:
            print("Ошибка установки типа внешнего триггера: {}".format(ret))
        ret = mvsdk.CameraSetExtTrigShutterType(hCamera, mvsdk.EXT_TRIG_HIGH_LEVEL)
        if ret != 0:
            print("Ошибка установки типа срабатывания затвора: {}".format(ret))
        # Устанавливаем внешнюю задержку триггера (0 мкс по умолчанию)
        ret = mvsdk.CameraSetExtTrigDelayTime(hCamera, 0)
        if ret != 0:
            print("Ошибка установки внешней задержки триггера: {}".format(ret))
            
        # Запрашиваем параметры триггера с консоли.
        # Рекомендуется использовать Trigger Count = 2, если внешний импульс подается с частотой лазера.
        try:
            trig_count = int(input("Введите Trigger Count (рекомендуется 2): "))
            trig_delay = int(input("Введите Trigger Delay (us) (например, 12500): "))
        except Exception as e:
            print("Неверный ввод параметров: ", e)
            mvsdk.CameraUnInit(hCamera)
            return

        ret = mvsdk.CameraSetTriggerCount(hCamera, trig_count)
        if ret != 0:
            print("Ошибка установки Trigger Count: {}".format(ret))
        ret = mvsdk.CameraSetTriggerDelayTime(hCamera, trig_delay)
        if ret != 0:
            print("Ошибка установки Trigger Delay: {}".format(ret))
        print("Параметры внешнего триггера установлены:")
        print("  Trigger Count = {}, Trigger Delay = {} us".format(trig_count, trig_delay))
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
            # При аппаратном триггере увеличиваем timeout до 2000 мс
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
