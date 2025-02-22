import mvsdk
import cv2
import numpy as np
import ctypes

def main():
    # Инициализация SDK (0 – для китайского языка, можно поставить 1 для английского)
    ret = mvsdk.CameraSdkInit(0)
    if ret != 0:
        print("Ошибка инициализации SDK:", ret)
        return

    # Перечисление подключённых устройств
    DevList = mvsdk.CameraEnumerateDevice()
    if not DevList:
        print("Камера не найдена!")
        return

    # Инициализация первой найденной камеры
    try:
        hCamera = mvsdk.CameraInit(DevList[0])
    except Exception as e:
        print("Не удалось инициализировать камеру:", e)
        return

    # Запуск захвата изображений
    mvsdk.CameraPlay(hCamera)
    
    # Небольшая задержка, чтобы камера успела начать захват
    cv2.waitKey(100)

    try:
        # Получение буфера с изображением (таймаут 1000 мс)
        buffer, frameInfo = mvsdk.CameraGetImageBuffer(hCamera, 1000)
    except Exception as e:
        print("Ошибка получения буфера изображения:", e)
        mvsdk.CameraUnInit(hCamera)
        return

    # Определение размера изображения по количеству байт
    size = frameInfo.uBytes  # общее количество байт в изображении

    # Создаем numpy массив, указывая адрес буфера и размер
    image_buffer = (ctypes.c_ubyte * size).from_address(buffer)
    arr = np.ctypeslib.as_array(image_buffer)

    # Определяем, цветное ли изображение (предполагается RGB, 3 канала) или градаций серого
    if size == frameInfo.iWidth * frameInfo.iHeight:
        # Изображение в градациях серого
        img = arr.reshape((frameInfo.iHeight, frameInfo.iWidth))
    elif size == frameInfo.iWidth * frameInfo.iHeight * 3:
        # Цветное изображение в формате RGB
        img = arr.reshape((frameInfo.iHeight, frameInfo.iWidth, 3))
        # Преобразуем RGB в BGR для корректного отображения в OpenCV
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    else:
        print("Неизвестный формат изображения")
        mvsdk.CameraReleaseImageBuffer(hCamera, buffer)
        mvsdk.CameraUnInit(hCamera)
        return

    # Отображение изображения в окне OpenCV
    cv2.imshow("Изображение с камеры", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Освобождаем буфер и закрываем камеру
    mvsdk.CameraReleaseImageBuffer(hCamera, buffer)
    mvsdk.CameraUnInit(hCamera)

if __name__ == '__main__':
    main()
