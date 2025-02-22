import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk, ImageDraw
import threading
import time
import mvsdk
import ctypes
import numpy as np

# Глобальные переменные для передачи кадра и FPS
current_image = None
fps = 0
running = True

# Глобальная переменная для дескриптора камеры
hCamera = None

# Функция захвата кадров в отдельном потоке
def capture_loop():
    global current_image, fps, running, hCamera
    frame_count = 0
    start_time = time.time()
    while running:
        try:
            # Получаем кадр с таймаутом 1000 мс
            buffer, frameInfo = mvsdk.CameraGetImageBuffer(hCamera, 1000)
        except Exception as e:
            print("Ошибка получения кадра:", e)
            continue

        size = frameInfo.uBytes
        # Преобразуем буфер в numpy-массив
        image_buffer = (ctypes.c_ubyte * size).from_address(buffer)
        arr = np.ctypeslib.as_array(image_buffer)
        # Определяем формат изображения
        if size == frameInfo.iWidth * frameInfo.iHeight:
            # Изображение в градациях серого
            img_np = arr.reshape((frameInfo.iHeight, frameInfo.iWidth))
            mode = "L"
        elif size == frameInfo.iWidth * frameInfo.iHeight * 3:
            # Цветное изображение (предполагается формат RGB)
            img_np = arr.reshape((frameInfo.iHeight, frameInfo.iWidth, 3))
            mode = "RGB"
        else:
            mvsdk.CameraReleaseImageBuffer(hCamera, buffer)
            continue

        # Освобождаем буфер
        mvsdk.CameraReleaseImageBuffer(hCamera, buffer)

        # Конвертируем массив в PIL Image
        try:
            pil_img = Image.fromarray(img_np, mode=mode)
        except Exception as e:
            print("Ошибка преобразования изображения:", e)
            continue

        # Подсчёт FPS
        frame_count += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            start_time = time.time()

        # Сохраняем последний кадр (копия для избежания конфликтов)
        current_image = pil_img.copy()

# Функция обновления изображения в tkinter (вызывается через after)
def update_image():
    global current_image, fps
    if current_image is not None:
        # Копируем изображение и накладываем текст
        img_with_overlay = current_image.copy()
        draw = ImageDraw.Draw(img_with_overlay)
        # Получаем значения из полей управления
        try:
            trig_count = trigger_count_var.get()
            trig_delay = trigger_delay_var.get()
            trig_interval = trigger_interval_var.get()
        except Exception:
            trig_count = trig_delay = trig_interval = "N/A"
        overlay_text = f"FPS: {fps:.2f}\nRes: {img_with_overlay.width}x{img_with_overlay.height}\n" \
                       f"Trigger Count: {trig_count}\nDelay: {trig_delay} us\nInterval: {trig_interval} us"
        # Рисуем текст в левом верхнем углу
        draw.text((10, 10), overlay_text, fill="green")
        # Преобразуем в ImageTk для отображения в Label
        imgtk = ImageTk.PhotoImage(img_with_overlay)
        video_label.imgtk = imgtk
        video_label.config(image=imgtk)
    root.after(30, update_image)

# Функция применения настроек триггера
def apply_trigger_settings():
    global hCamera
    try:
        count_val = int(trigger_count_var.get())
        delay_val = int(trigger_delay_var.get())
        interval_val = int(trigger_interval_var.get())
    except Exception as e:
        print("Неверный формат входных данных:", e)
        return

    ret1 = mvsdk.CameraSetTriggerCount(hCamera, count_val)
    ret2 = mvsdk.CameraSetTriggerDelayTime(hCamera, delay_val)
    # Используем CameraSetExtTrigJitterTime для установки интервала между срабатываниями триггера
    ret3 = mvsdk.CameraSetExtTrigJitterTime(hCamera, interval_val)

    if ret1 != 0 or ret2 != 0 or ret3 != 0:
        print("Ошибка установки параметров триггера (ret1={}, ret2={}, ret3={})".format(ret1, ret2, ret3))
    else:
        print("Параметры триггера успешно установлены")

# Инициализация камеры и SDK
def init_camera():
    global hCamera
    ret = mvsdk.CameraSdkInit(0)
    if ret != 0:
        print("Ошибка инициализации SDK:", ret)
        exit(1)
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        print("Камера не найдена!")
        exit(1)
    try:
        hCamera = mvsdk.CameraInit(dev_list[0])
    except Exception as e:
        print("Не удалось инициализировать камеру:", e)
        exit(1)
    # Устанавливаем режим внешнего триггера
    ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
    if ret != 0:
        print("Ошибка установки внешнего триггера:", ret)
        mvsdk.CameraUnInit(hCamera)
        exit(1)
    # Запускаем захват изображений
    mvsdk.CameraPlay(hCamera)

# Завершение работы: остановка захвата и освобождение камеры
def on_closing():
    global running, hCamera
    running = False
    time.sleep(0.2)  # небольшая задержка, чтобы поток захвата завершился
    mvsdk.CameraUnInit(hCamera)
    root.destroy()

# Основная программа
init_camera()

# Создаём главное окно tkinter
root = tk.Tk()
root.title("Настройка триггера камеры")

# Фрейм для элементов управления
control_frame = ttk.Frame(root, padding="10")
control_frame.pack(side=tk.TOP, fill=tk.X)

ttk.Label(control_frame, text="Trigger Count:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
trigger_count_var = tk.StringVar(value="1")
trigger_count_entry = ttk.Entry(control_frame, textvariable=trigger_count_var, width=10)
trigger_count_entry.grid(row=0, column=1, padx=5, pady=5)

ttk.Label(control_frame, text="Delay (us):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
trigger_delay_var = tk.StringVar(value="0")
trigger_delay_entry = ttk.Entry(control_frame, textvariable=trigger_delay_var, width=10)
trigger_delay_entry.grid(row=0, column=3, padx=5, pady=5)

ttk.Label(control_frame, text="Interval (us):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
trigger_interval_var = tk.StringVar(value="50000")
trigger_interval_entry = ttk.Entry(control_frame, textvariable=trigger_interval_var, width=10)
trigger_interval_entry.grid(row=0, column=5, padx=5, pady=5)

apply_button = ttk.Button(control_frame, text="Применить", command=apply_trigger_settings)
apply_button.grid(row=0, column=6, padx=5, pady=5)

# Метка для отображения видео
video_label = ttk.Label(root)
video_label.pack()

# Запускаем поток захвата кадров
capture_thread = threading.Thread(target=capture_loop, daemon=True)
capture_thread.start()

# Запускаем цикл обновления изображения
root.after(30, update_image)

# Обработка закрытия окна
root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
