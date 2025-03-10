#coding=utf-8
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
        print("Не найдена ни одна камера!")
        return

    # Выводим список найденных камер
    for i, DevInfo in enumerate(DevList):
        print("{}: {}  SN:{}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetSn()))
    # Выбираем камеру
    i = 0 if nDev == 1 else int(input("Выберите камеру (по индексу): "))
    DevInfo = DevList[i]

    print("Выбранная камера:")
    print("  FriendlyName:", DevInfo.GetFriendlyName())
    print("  PortType:", DevInfo.GetPortType())
    print("  SN:", DevInfo.GetSn())

    # Попытка инициализировать BySN
    # Обычно: emParamLoadMode = 2 => BySN (проверьте вашу документацию!)
    emParamLoadMode = 2  # BySN
    try:
        hCamera = mvsdk.CameraInit(DevInfo, emParamLoadMode, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return

    # Получаем характеристики камеры
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    # Настраиваем формат (черно-белый или BGR)
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Убираем триггерный режим, ставим непрерывную съемку
    # (если SDK требует явно что-то выключить, убираем CameraSetTriggerMode)
    #mvsdk.CameraSetTriggerMode(hCamera, 0)  # Если нужно, используйте 0 = continuous

    # Выключаем автоэкспозицию, ставим вручную 30 мс (пример)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30_000)  # 30 ms

    # Запускаем камеру
    mvsdk.CameraPlay(hCamera)

    # Расчет максимального буфера
    FrameBufferSize = (cap.sResolutionRange.iWidthMax *
                       cap.sResolutionRange.iHeightMax *
                       (1 if monoCamera else 3))
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    # Спросим у пользователя частоту фонаря
    try:
        flash_freq = float(input("Введите частоту мерцания фонаря (Hz): "))
    except ValueError:
        flash_freq = 1.0

    flash_period = 1.0 / flash_freq if flash_freq > 0 else 0

    # Переменные для измерения FPS
    prev_time = time.time()
    frame_count = 0
    fps = 0.0

    prev_frame = None

    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        current_time = time.time()
        frame_count += 1
        elapsed = current_time - prev_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            prev_time = current_time

        # Фаза мерцания фонарика (в диапазоне [0..1])
        phase = (current_time % flash_period) / flash_period if flash_period > 0 else 0.0

        # Пытаемся получить кадр
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            # Обработка RAW → pFrameBuffer (BGR8 / MONO8)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            # Освобождаем
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # Windows: переворачиваем буфер
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            # Преобразуем в numpy
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            ch = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, ch))

            # Можно изменить размер, если хотим фиксированный (например, 640x480)
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

            # Считаем разницу с предыдущим кадром (по желанию)
            if prev_frame is not None:
                diff = cv2.absdiff(frame, prev_frame)

                # Выводим текст
                cv2.putText(diff, f"FPS: {fps:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(diff, f"Phase: {phase:.2f}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(diff, f"Flash Freq: {flash_freq:.2f} Hz", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow("Разница", diff)

            # На оригинальном кадре тоже можно вывести
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Phase: {phase:.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Flash Freq: {flash_freq:.2f} Hz", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Оригинал", frame)
            prev_frame = frame.copy()

        except mvsdk.CameraException as e:
            # CAMERA_STATUS_TIME_OUT = -12 (превышено время ожидания кадра)
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
                break

    # Освобождаем ресурсы
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
