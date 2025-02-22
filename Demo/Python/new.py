import cv2
import numpy as np
import mvsdk
import platform
import time

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

    # Режим внешнего триггера (для Trigger Count = 2)
    mvsdk.CameraSetTriggerMode(hCamera, 1)

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
            # Увеличиваем timeout до 2000 мс, чтобы дать время на поступление внешнего триггера
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
        except mvsdk.CameraException as e:
            if e.error_code == mvsdk.CAMERA_STATUS_TIME_OUT:
                print("Time out, повторяем попытку...")
                continue
            else:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
                continue

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

    # Завершение работы
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

main()
