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

    # --------------------------------------------------------------------------------
    # ВЫБОР РАЗРЕШЕНИЯ
    # --------------------------------------------------------------------------------
    # Печатаем все предустановленные разрешения (ROI), которые есть в pImageSizeDesc
    pResDesc = cap.pImageSizeDesc
    count = cap.iImageSizeDesc

    print("\nДоступные предустановки разрешения:")
    resolutions = []
    for idx in range(count):
        res = pResDesc[idx]
        desc = res.GetDescription()
        w = res.iWidthFOV
        h = res.iHeightFOV
        print(f"{idx}: {desc} ({w}x{h})")
        resolutions.append(res)  # сохраняем структуру

    if count > 1:
        idx_choice = int(input("Выберите разрешение (по индексу): "))
    else:
        idx_choice = 0

    if idx_choice < 0 or idx_choice >= count:
        idx_choice = 0  # подстраховка

    selected_res = resolutions[idx_choice]
    err = mvsdk.CameraSetImageResolution(hCamera, selected_res)
    if err != 0:
        print(f"Ошибка установки разрешения: код {err}")

    # --------------------------------------------------------------------------------
    # Настройка формата вывода изображения (MOНО или BGR)
    # --------------------------------------------------------------------------------
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # --------------------------------------------------------------------------------
    # Настройка экспозиции (убираем триггер, оставляем непрерывный режим)
    # --------------------------------------------------------------------------------
    # Режим непрерывной съемки — строку mvsdk.CameraSetTriggerMode(hCamera, 0) УДАЛЯЕМ
    mvsdk.CameraSetAeState(hCamera, 0)              # Выключаем автоэкспозицию
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000) # 30 мс экспозиции

    # Запуск захвата
    mvsdk.CameraPlay(hCamera)

    # --------------------------------------------------------------------------------
    # Расчет максимального размера кадра под буфер
    # --------------------------------------------------------------------------------
    FrameBufferSize = (cap.sResolutionRange.iWidthMax *
                       cap.sResolutionRange.iHeightMax *
                       (1 if monoCamera else 3))
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    # --------------------------------------------------------------------------------
    # Запрос частоты мерцания фонарика
    # --------------------------------------------------------------------------------
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

    while True:
        # Прерывание по кнопке 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        current_time = time.time()
        frame_count += 1
        elapsed = current_time - prev_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            prev_time = current_time

        # Фаза мерцания фонарика (0..1)
        phase = (current_time % flash_period) / flash_period if flash_period > 0 else 0.0

        try:
            # Захват кадра с камеры
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # Для Windows переворачиваем изображение
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            # Преобразование в numpy
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            channel = 1 if (FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8) else 3
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, channel))

            # Для наглядности приводим к 640x480 (если нужно)
            # Можно убрать, если хотим исходный размер
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

            # Вычисление разницы с предыдущим кадром
            if prev_frame is not None:
                diff = cv2.absdiff(frame, prev_frame)
                # Отображаем информацию на "разнице"
                cv2.putText(diff, f"FPS: {fps:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(diff, f"Phase: {phase:.2f}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(diff, f"Flash Freq: {flash_freq:.2f} Hz", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Разница", diff)

            # Отображаем информацию на оригинальном кадре
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Phase: {phase:.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Flash Freq: {flash_freq:.2f} Hz", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Оригинал", frame)

            prev_frame = frame.copy()

        except mvsdk.CameraException as e:
            # Код -12 = CAMERA_STATUS_TIME_OUT (превышено время ожидания кадра)
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
                break

    # Завершение работы с камерой
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)


def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
