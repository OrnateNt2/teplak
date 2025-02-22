import mvsdk
import cv2
import numpy as np
import ctypes
import time

def main():
    # Меню выбора настроек камеры в консоли
    print("Выберите режим работы камеры:")
    print("1. Внешний триггер (кадр по фронту меандра)")
    print("2. Автоматический захват (без внешнего триггера)")
    mode_choice = input("Введите номер режима (1 или 2): ")
    
    if mode_choice not in ['1', '2']:
        print("Неверный выбор режима. Выход.")
        return

    # Инициализация SDK (0 – китайский, 1 – английский)
    ret = mvsdk.CameraSdkInit(0)
    if ret != 0:
        print("Ошибка инициализации SDK:", ret)
        return

    # Перечисление подключённых устройств
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        print("Камера не найдена!")
        return

    # Инициализация первой найденной камеры
    try:
        hCamera = mvsdk.CameraInit(dev_list[0])
    except Exception as e:
        print("Не удалось инициализировать камеру:", e)
        return

    # Установка режима захвата камеры в зависимости от выбора пользователя
    if mode_choice == '1':
        # Внешний триггер: при поступлении внешнего сигнала камера делает кадр
        ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
        trigger_mode_text = "Внешний триггер"
    else:
        # Автоматический режим захвата: камера самостоятельно захватывает кадры
        ret = mvsdk.CameraSetTriggerMode(hCamera, 0)
        trigger_mode_text = "Автоматический захват"
    
    if ret != 0:
        print("Ошибка установки режима триггера:", ret)
        mvsdk.CameraUnInit(hCamera)
        return

    # Запуск захвата изображений
    mvsdk.CameraPlay(hCamera)
    print("Начало захвата. Для выхода нажмите 'q' в окне видео.")

    # Переменные для вычисления FPS
    frame_count = 0
    start_time = time.time()
    fps = 0

    while True:
        try:
            # Получение кадра с таймаутом 1000 мс
            buffer, frameInfo = mvsdk.CameraGetImageBuffer(hCamera, 1000)
        except Exception as e:
            print("Ошибка получения кадра:", e)
            break

        size = frameInfo.uBytes
        # Преобразование буфера в numpy-массив
        image_buffer = (ctypes.c_ubyte * size).from_address(buffer)
        arr = np.ctypeslib.as_array(image_buffer)

        # Определение формата изображения по количеству байт
        if size == frameInfo.iWidth * frameInfo.iHeight:
            # Изображение в градациях серого
            img = arr.reshape((frameInfo.iHeight, frameInfo.iWidth))
        elif size == frameInfo.iWidth * frameInfo.iHeight * 3:
            # Цветное изображение в формате RGB
            img = arr.reshape((frameInfo.iHeight, frameInfo.iWidth, 3))
            # Преобразование из RGB в BGR для корректного отображения в OpenCV
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            print("Неизвестный формат изображения")
            mvsdk.CameraReleaseImageBuffer(hCamera, buffer)
            continue

        # Освобождение буфера
        mvsdk.CameraReleaseImageBuffer(hCamera, buffer)

        # Вычисление FPS
        frame_count += 1
        elapsed_time = time.time() - start_time
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            frame_count = 0
            start_time = time.time()

        # Формирование строки с важной информацией
        overlay_text = f"FPS: {fps:.2f} | Res: {frameInfo.iWidth}x{frameInfo.iHeight} | Mode: {trigger_mode_text}"
        # Вывод текста в левом верхнем углу кадра
        cv2.putText(img, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 0), 2, cv2.LINE_AA)

        # Отображение кадра
        cv2.imshow("Камера (тригер)", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    mvsdk.CameraUnInit(hCamera)

if __name__ == '__main__':
    main()
