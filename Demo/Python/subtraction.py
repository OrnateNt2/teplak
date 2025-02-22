#coding=utf-8
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
        print("Не найдена ни одна камера!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Выберите камеру: "))
    DevInfo = DevList[i]
    print(DevInfo)

    # Открытие камеры
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return

    # Получение характеристик камеры
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    # Настройка формата вывода изображения
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Режим непрерывной съемки
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    # Ручная экспозиция, время экспозиции 30 мс
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Запуск захвата
    mvsdk.CameraPlay(hCamera)

    # Расчет размера буфера RGB
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    # Запрос частоты мерцания фонарика (в Гц)
    try:
        flash_freq = float(input("Введите частоту мерцания фонарика (Hz): "))
    except Exception:
        flash_freq = 1.0
    flash_period = 1.0 / flash_freq if flash_freq > 0 else 0

    # Переменные для расчета FPS
    prev_time = time.time()
    frame_count = 0
    fps = 0.0

    prev_frame = None  # предыдущий кадр для вычисления разницы

    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        current_time = time.time()
        frame_count += 1
        elapsed = current_time - prev_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            prev_time = current_time

        # Вычисление фазы мерцания фонарика (значение от 0 до 1)
        phase = (current_time % flash_period) / flash_period if flash_period > 0 else 0.0

        try:
            # Захват кадра с камеры
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # Для Windows переворачиваем изображение
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            # Преобразование данных в формат OpenCV
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

            # Вычисление разницы между текущим и предыдущим кадром
            if prev_frame is not None:
                diff = cv2.absdiff(frame, prev_frame)
                # Наложение информации на изображение разницы (по желанию)
                cv2.putText(diff, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(diff, f"Phase: {phase:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(diff, f"Flash Freq: {flash_freq:.2f} Hz", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Разница", diff)

            # Наложение информации на оригинальное изображение
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Phase: {phase:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Flash Freq: {flash_freq:.2f} Hz", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Оригинал", frame)

            prev_frame = frame.copy()  # сохраняем текущий кадр для следующей итерации

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

    # Завершение работы камеры и освобождение ресурсов
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
