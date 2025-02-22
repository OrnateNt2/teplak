import mvsdk
import cv2
import numpy as np
import ctypes

def main():
    # Инициализация SDK (0 – китайский язык; можно поставить 1 – английский)
    ret = mvsdk.CameraSdkInit(0)
    if ret != 0:
        print("Ошибка инициализации SDK:", ret)
        return

    # Перечисляем подключённые устройства
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        print("Камера не найдена!")
        return

    # Инициализируем первую найденную камеру
    try:
        hCamera = mvsdk.CameraInit(dev_list[0])
    except Exception as e:
        print("Не удалось инициализировать камеру:", e)
        return

    # Настраиваем режим триггера: 
    # Предполагается, что значение 1 включает внешний режим триггера
    ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
    if ret != 0:
        print("Ошибка установки режима триггера:", ret)
        mvsdk.CameraUnInit(hCamera)
        return

    # Если требуется, можно настроить задержку триггера (в микросекундах)
    # mvsdk.CameraSetTriggerDelayTime(hCamera, 0)

    # Запускаем захват изображений
    mvsdk.CameraPlay(hCamera)
    
    print("Начало захвата. Для выхода нажмите 'q'")
    while True:
        try:
            # Ожидаем поступления кадра (таймаут 1000 мс)
            buffer, frameInfo = mvsdk.CameraGetImageBuffer(hCamera, 1000)
        except Exception as e:
            print("Ошибка получения кадра:", e)
            break

        size = frameInfo.uBytes
        # Преобразуем буфер в numpy-массив
        image_buffer = (ctypes.c_ubyte * size).from_address(buffer)
        arr = np.ctypeslib.as_array(image_buffer)

        # Определяем формат изображения по размеру буфера
        if size == frameInfo.iWidth * frameInfo.iHeight:
            # Изображение в градациях серого
            img = arr.reshape((frameInfo.iHeight, frameInfo.iWidth))
        elif size == frameInfo.iWidth * frameInfo.iHeight * 3:
            # Цветное изображение (предполагается формат RGB)
            img = arr.reshape((frameInfo.iHeight, frameInfo.iWidth, 3))
            # Перевод из RGB в BGR для корректного отображения OpenCV
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            print("Неизвестный формат изображения")
            mvsdk.CameraReleaseImageBuffer(hCamera, buffer)
            continue

        # Отображаем изображение
        cv2.imshow("Камера (тригер)", img)
        # Освобождаем буфер
        mvsdk.CameraReleaseImageBuffer(hCamera, buffer)

        # Выход по нажатию 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    mvsdk.CameraUnInit(hCamera)

if __name__ == '__main__':
    main()
