import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk, ImageDraw
import threading
import time
import mvsdk
import ctypes
import numpy as np

current_image = None
fps = 0
running = True

hCamera = None

def capture_loop():
    global current_image, fps, running, hCamera
    frame_count = 0
    start_time = time.time()
    while running:
        try:
            buffer, frameInfo = mvsdk.CameraGetImageBuffer(hCamera, 1000)
        except Exception as e:
            print("Ошибка получения кадра:", e)
            continue

        size = frameInfo.uBytes
        image_buffer = (ctypes.c_ubyte * size).from_address(buffer)
        arr = np.ctypeslib.as_array(image_buffer)
        # Определяем формат изображения
        if size == frameInfo.iWidth * frameInfo.iHeight:
            img_np = arr.reshape((frameInfo.iHeight, frameInfo.iWidth))
            mode = "L"
        elif size == frameInfo.iWidth * frameInfo.iHeight * 3:
            img_np = arr.reshape((frameInfo.iHeight, frameInfo.iWidth, 3))
            mode = "RGB"
        else:
            mvsdk.CameraReleaseImageBuffer(hCamera, buffer)
            continue

        mvsdk.CameraReleaseImageBuffer(hCamera, buffer)

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

        current_image = pil_img.copy()

# Функция обновления изображения в tkinter
def update_image():
    global current_image, fps
    if current_image is not None:
        img_with_overlay = current_image.copy()
        draw = ImageDraw.Draw(img_with_overlay)
        try:
            trig_count = trigger_count_var.get()
            trig_delay = trigger_delay_var.get()
            trig_interval = trigger_interval_var.get()
            laser_freq = laser_freq_var.get()
            camera_freq = camera_freq_var.get()
        except Exception:
            trig_count = trig_delay = trig_interval = laser_freq = camera_freq = "N/A"
        overlay_text = (f"FPS: {fps:.2f}\n"
                        f"Res: {img_with_overlay.width}x{img_with_overlay.height}\n"
                        f"Trigger Count: {trig_count}\n"
                        f"Delay: {trig_delay} us\n"
                        f"Interval: {trig_interval} us\n"
                        f"Laser f: {laser_freq} Hz\n"
                        f"Camera f: {camera_freq} Hz")
        draw.text((10, 10), overlay_text, fill="green")
        imgtk = ImageTk.PhotoImage(img_with_overlay)
        video_label.imgtk = imgtk
        video_label.config(image=imgtk)
    root.after(30, update_image)

# Функция применения настроек триггера вручную
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
    ret3 = mvsdk.CameraSetExtTrigJitterTime(hCamera, interval_val)

    if ret1 != 0 or ret2 != 0 or ret3 != 0:
        print(f"Ошибка установки параметров триггера (ret1={ret1}, ret2={ret2}, ret3={ret3})")
    else:
        print("Параметры триггера успешно установлены")

# Функция расчёта параметров триггера по заданным частотам
def calculate_parameters():
    try:
        laser_freq = float(laser_freq_var.get())
        camera_freq = float(camera_freq_var.get())
    except Exception as e:
        print("Ошибка ввода частот:", e)
        return

    if laser_freq <= 0:
        print("Частота лазера должна быть больше нуля.")
        return

    # Предполагаем, что для получения двух кадров на период лазера требуется:
    # Trigger Count = 2
    # Период лазера T = 1 / laser_freq (сек)
    # Первый кадр: задержка = T/4, второй: с интервалом T/2 относительно первого
    T = 1.0 / laser_freq  # период в секундах
    computed_delay = int((T / 4) * 1e6)      # в микросекундах
    computed_interval = int((T / 2) * 1e6)     # в микросекундах
    computed_trigger_count = 2

    trigger_count_var.set(str(computed_trigger_count))
    trigger_delay_var.set(str(computed_delay))
    trigger_interval_var.set(str(computed_interval))
    print("Вычисленные параметры:")
    print(f"Trigger Count = {computed_trigger_count}, Delay = {computed_delay} us, Interval = {computed_interval} us")
    # Можно также проверить, соответствует ли частота камеры требуемой (ожидается f_camera = 2*laser_freq)
    expected_camera_freq = 2 * laser_freq
    print(f"Ожидаемая частота камеры: {expected_camera_freq} Гц (введено {camera_freq} Гц)")

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
    ret = mvsdk.CameraSetTriggerMode(hCamera, 1)
    if ret != 0:
        print("Ошибка установки внешнего триггера:", ret)
        mvsdk.CameraUnInit(hCamera)
        exit(1)
    mvsdk.CameraPlay(hCamera)

# Завершение работы
def on_closing():
    global running, hCamera
    running = False
    time.sleep(0.2)
    mvsdk.CameraUnInit(hCamera)
    root.destroy()

# Основная программа
init_camera()

# Создаём главное окно tkinter
root = tk.Tk()
root.title("Настройка триггера камеры")

# Фрейм для параметров частот
freq_frame = ttk.Frame(root, padding="10")
freq_frame.pack(side=tk.TOP, fill=tk.X)

ttk.Label(freq_frame, text="Laser Frequency (Hz):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
laser_freq_var = tk.StringVar(value="20")
laser_freq_entry = ttk.Entry(freq_frame, textvariable=laser_freq_var, width=10)
laser_freq_entry.grid(row=0, column=1, padx=5, pady=5)

ttk.Label(freq_frame, text="Camera Frequency (Hz):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
camera_freq_var = tk.StringVar(value="40")
camera_freq_entry = ttk.Entry(freq_frame, textvariable=camera_freq_var, width=10)
camera_freq_entry.grid(row=0, column=3, padx=5, pady=5)

calc_button = ttk.Button(freq_frame, text="Рассчитать параметры", command=calculate_parameters)
calc_button.grid(row=0, column=4, padx=5, pady=5)

# Фрейм для ручного ввода параметров триггера
control_frame = ttk.Frame(root, padding="10")
control_frame.pack(side=tk.TOP, fill=tk.X)

ttk.Label(control_frame, text="Trigger Count:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
trigger_count_var = tk.StringVar(value="2")
trigger_count_entry = ttk.Entry(control_frame, textvariable=trigger_count_var, width=10)
trigger_count_entry.grid(row=0, column=1, padx=5, pady=5)

ttk.Label(control_frame, text="Delay (us):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
trigger_delay_var = tk.StringVar(value="12500")
trigger_delay_entry = ttk.Entry(control_frame, textvariable=trigger_delay_var, width=10)
trigger_delay_entry.grid(row=0, column=3, padx=5, pady=5)

ttk.Label(control_frame, text="Interval (us):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
trigger_interval_var = tk.StringVar(value="25000")
trigger_interval_entry = ttk.Entry(control_frame, textvariable=trigger_interval_var, width=10)
trigger_interval_entry.grid(row=0, column=5, padx=5, pady=5)

apply_button = ttk.Button(control_frame, text="Применить", command=apply_trigger_settings)
apply_button.grid(row=0, column=6, padx=5, pady=5)

# Метка для отображения видео
video_label = ttk.Label(root)
video_label.pack()

# Запуск потока захвата кадров
capture_thread = threading.Thread(target=capture_loop, daemon=True)
capture_thread.start()

root.after(30, update_image)
root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
