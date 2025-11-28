import pygame
import tkinter as tk
from tkinter import ttk, scrolledtext
import paho.mqtt.client as mqtt
import json
import time
import math
from collections import deque 
import threading 

# --- IMPORTY DO WYKRESÓW (MATPLOTLIB) ---
import matplotlib
matplotlib.use("TkAgg") 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# --- KONFIGURACJA SIECI ---
BROKER_ADDRESS = "192.168.1.1"   # <-- Zmień na IP
BROKER_PORT = 1883

# Tematy MQTT
TOPIC_SET_VELOCITY = "odrive/set_velocity"
TOPIC_FEEDBACK     = "odrive/feedback"
TOPIC_CMD          = "odrive/cmd"

# --- KONFIGURACJA STEROWANIA ---
ABSOLUTE_MAX_LIMIT = 30.0  
JOYSTICK_DEADZONE = 0.2    

# --- KONFIGURACJA KOŁA ---
WHEEL_DIAMETER_CM = 45.0
GEAR_RATIO = 100.8
WHEEL_CIRCUMFERENCE_M = (WHEEL_DIAMETER_CM / 100.0) * math.pi
DISTANCE_PER_MOTOR_REV = WHEEL_CIRCUMFERENCE_M / GEAR_RATIO

# --- KLASA LAG ESTYMATORA ---
class LatencyEstimator:
    def __init__(self, maxlen=50):
        self.history = deque(maxlen=maxlen)
    
    def push_target(self, target_vel):
        self.history.append((time.time(), target_vel))
        
    def estimate_lag(self, current_measured_vel):
        if abs(current_measured_vel) < 0.5:
            return None
        best_time = None
        min_diff = float('inf')
        for t_stamp, t_vel in reversed(self.history):
            diff = abs(t_vel - current_measured_vel)
            if diff < min_diff:
                min_diff = diff
                best_time = t_stamp
            if diff > min_diff + 2.0: 
                break
        if best_time:
            lag_seconds = time.time() - best_time
            return lag_seconds * 1000.0 
        return None

# --- Inicjalizacja ---
pygame.init()
pygame.joystick.init()
latency_estimator = LatencyEstimator(maxlen=100) 

# --- Zmienne globalne ---
joysticks = []
current_speed_limit = 10.0 
measured_velocity = 0.0
measured_position = 0.0
start_position_offset = 0.0
last_feedback_time = 0.0
client = None 

# Licznik do optymalizacji wykresu
plot_refresh_counter = 0 
PLOT_SKIP_FRAMES = 4  # Odświeżaj wykres co 4 klatki (czyli co 200ms zamiast 50ms)

# Zmienne sterowania
key_throttle = 0.0     
key_max_limit = 10.0   

# --- DANE WYKRESU ---
plot_data_x = deque(maxlen=200)      
plot_data_target = deque(maxlen=200) 
plot_data_meas = deque(maxlen=200)   
start_time = time.time()

# --- GUI Colors ---
BG_COLOR = "#1e1e1e"
FG_COLOR = "#ffffff"
BTN_RESET_COLOR = "#cc3333"
BTN_CMD_COLOR = "#0055aa"
BTN_FULL_START_COLOR = "#228822"
BTN_REBOOT_COLOR = "#aa0000" 

# --- TKINTER SETUP ---
root = tk.Tk()
root.title("ODrive Control Optimized")
root.configure(bg=BG_COLOR)
root.attributes('-fullscreen', True) 

def end_fullscreen(event=None):
    root.attributes('-fullscreen', False)
    root.geometry("1200x800")

# --- LAYOUT ---
main_frame = tk.Frame(root, bg=BG_COLOR)
main_frame.pack(fill="both", expand=True)

left_frame = tk.Frame(main_frame, bg=BG_COLOR, width=500)
left_frame.pack(side="left", fill="y", padx=10, pady=10)
left_frame.pack_propagate(False)

center_frame = tk.Frame(main_frame, bg=BG_COLOR)
center_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

right_frame = tk.Frame(main_frame, bg=BG_COLOR, width=400)
right_frame.pack(side="left", fill="y", padx=10, pady=10)
right_frame.pack_propagate(False)

# ==========================================
# 1. LEWY PANEL
# ==========================================
joy_container = tk.Frame(left_frame, bg=BG_COLOR)
joy_container.pack(side="top", fill="x")
joystick_list_frame = tk.Frame(joy_container, bg=BG_COLOR)

def reset_joysticks():
    global joysticks
    for widget in joystick_list_frame.winfo_children():
        widget.destroy()
    pygame.joystick.quit()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    joysticks = [pygame.joystick.Joystick(i) for i in range(count)]
    if not joysticks:
        tk.Label(joystick_list_frame, text="BRAK JOYSTICKA (Użyj Klawiatury)", bg=BG_COLOR, fg="yellow").pack()
    for i, joystick in enumerate(joysticks):
        joystick.init()
        frame = tk.LabelFrame(joystick_list_frame, text=f"Joy {i}: {joystick.get_name()[:15]}", bg=BG_COLOR, fg=FG_COLOR)
        frame.pack(fill="x", padx=5, pady=5)
        for axis in range(joystick.get_numaxes()):
            tk.Label(frame, text=f"Axis {axis}", bg=BG_COLOR, fg="#aaa").pack()

tk.Button(joy_container, text="⟳ RESET JOYSTICK", command=reset_joysticks, bg=BTN_RESET_COLOR, fg="white", font=("Arial",10,"bold")).pack(fill="x", pady=(0,10))
joystick_list_frame.pack(fill="both", expand=True)

# Instrukcja
instr_frame = tk.LabelFrame(left_frame, text="Sterowanie Klawiaturą", bg=BG_COLOR, fg="#ccc")
instr_frame.pack(fill="x", pady=10)
tk.Label(instr_frame, text="[W] - Przód   [S] - Tył", bg=BG_COLOR, fg="white", font=("Arial", 10, "bold")).pack()
tk.Label(instr_frame, text="[R] - Max++   [F] - Max--", bg=BG_COLOR, fg="white", font=("Arial", 10, "bold")).pack()
tk.Label(instr_frame, text="[ESC] - Okno", bg=BG_COLOR, fg="#888").pack()

# --- WYKRES PRĘDKOŚCI ---
plot_frame = tk.LabelFrame(left_frame, text="Wykres Prędkości (Target vs Measured)", bg=BG_COLOR, fg=FG_COLOR)
plot_frame.pack(side="bottom", fill="both", expand=True, pady=(20, 0))

# Optymalizacja wykresu: Ustawiamy tło i parametry raz
fig = Figure(figsize=(4, 3), dpi=100, facecolor=BG_COLOR)
ax = fig.add_subplot(111)
ax.set_facecolor(BG_COLOR)
ax.tick_params(colors='white')
for spine in ax.spines.values(): spine.set_color('white')
ax.grid(True, color='#444', linestyle='--')

line_target, = ax.plot([], [], color='#00ffff', linewidth=2, label='Target', linestyle='--') 
line_meas, = ax.plot([], [], color='#00ff00', linewidth=2, label='Measured') 
ax.legend(loc="upper right", facecolor=BG_COLOR, labelcolor='white', framealpha=0.5)

canvas_plot = FigureCanvasTkAgg(fig, master=plot_frame)
canvas_plot.get_tk_widget().pack(fill="both", expand=True)

def update_plot():
    # Sprawdzamy czy mamy dane
    if len(plot_data_x) > 1:
        line_target.set_data(plot_data_x, plot_data_target)
        line_meas.set_data(plot_data_x, plot_data_meas)
        
        ax.set_xlim(min(plot_data_x), max(plot_data_x) + 0.1)
        limit = ABSOLUTE_MAX_LIMIT * 1.1
        ax.set_ylim(-limit, limit)
        
        # draw_idle jest lżejsze niż draw, ale nadal kosztowne
        canvas_plot.draw_idle()

# ==========================================
# 2. ŚRODKOWY PANEL
# ==========================================
gauge_frame = tk.LabelFrame(center_frame, text="Wskaźniki", bg=BG_COLOR, fg=FG_COLOR)
gauge_frame.pack(pady=10, fill="both")
gauge_canvas = tk.Canvas(gauge_frame, width=400, height=300, bg=BG_COLOR, highlightthickness=0)
gauge_canvas.pack(pady=10)
target_label = tk.Label(gauge_frame, text="Target: 0.0 RPS", bg=BG_COLOR, fg=FG_COLOR, font=("Arial", 24, "bold"))
target_label.pack()

feedback_frame = tk.LabelFrame(center_frame, text="Feedback & Diagnostyka", bg=BG_COLOR, fg="#00ff00")
feedback_frame.pack(pady=20, fill="x", padx=10)

lbl_meas_vel = tk.Label(feedback_frame, text="RPS: 0.00", bg=BG_COLOR, fg="#00ff00", font=("Consolas", 18))
lbl_meas_vel.pack(anchor="w", padx=10, pady=2)

speed_container = tk.Frame(feedback_frame, bg=BG_COLOR)
speed_container.pack(anchor="w", padx=10, pady=2)
lbl_speed_kmh = tk.Label(speed_container, text="0.0 km/h", bg=BG_COLOR, fg="#ff00ff", font=("Consolas", 20, "bold"))
lbl_speed_kmh.pack(side="left", padx=(0, 20))
lbl_speed_ms = tk.Label(speed_container, text="0.00 m/s", bg=BG_COLOR, fg="#ff88ff", font=("Consolas", 14))
lbl_speed_ms.pack(side="left")

lbl_meas_pos = tk.Label(feedback_frame, text="Pozycja: 0.00 obr", bg=BG_COLOR, fg="#00ccff", font=("Consolas", 18))
lbl_meas_pos.pack(anchor="w", padx=10, pady=5)

def reset_trip():
    global start_position_offset
    start_position_offset = measured_position

dist_container = tk.Frame(feedback_frame, bg=BG_COLOR)
dist_container.pack(anchor="w", padx=10, pady=5)
lbl_distance = tk.Label(dist_container, text="Dystans: 0.00 m", bg=BG_COLOR, fg="#ffffff", font=("Consolas", 18))
lbl_distance.pack(side="left")
btn_reset_trip = tk.Button(dist_container, text="[RESET]", command=reset_trip, bg="#444", fg="white", font=("Arial", 8))
btn_reset_trip.pack(side="left", padx=10)

tk.Frame(feedback_frame, height=1, bg="#444").pack(fill="x", padx=5, pady=5)
lbl_packet_age = tk.Label(feedback_frame, text="Sieć (Packet Age): -- ms", bg=BG_COLOR, fg="#aaaaaa", font=("Consolas", 11))
lbl_packet_age.pack(anchor="w", padx=10)
lbl_system_lag = tk.Label(feedback_frame, text="Lag: -- ms", bg=BG_COLOR, fg="#aaaaaa", font=("Consolas", 16, "bold"))
lbl_system_lag.pack(anchor="w", padx=10, pady=(5,10))

# ==========================================
# 3. PRAWY PANEL
# ==========================================
mqtt_status_var = tk.StringVar(value="MQTT: Rozłączono")
tk.Label(right_frame, textvariable=mqtt_status_var, bg=BG_COLOR, fg=FG_COLOR, font=("Arial", 12)).pack(anchor="w")

mqtt_console = scrolledtext.ScrolledText(right_frame, bg="#222222", fg="#00ff00", height=20, font=("Consolas", 10))
mqtt_console.pack(fill="both", expand=True, pady=5)

def log_mqtt(msg):
    mqtt_console.insert(tk.END, msg + "\n")
    mqtt_console.see(tk.END)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        mqtt_status_var.set("MQTT: POŁĄCZONO")
        client.subscribe(TOPIC_FEEDBACK)
        log_mqtt(">> Połączono.")

def on_message(client, userdata, msg):
    global measured_velocity, measured_position, last_feedback_time
    last_feedback_time = time.time()
    try:
        payload = json.loads(msg.payload.decode())
        measured_velocity = payload.get("v_meas", 0.0)
        measured_position = payload.get("p_meas", 0.0)
        
        trip_turns = measured_position - start_position_offset
        trip_distance_m = trip_turns * DISTANCE_PER_MOTOR_REV
        
        speed_ms = measured_velocity * DISTANCE_PER_MOTOR_REV
        speed_kmh = speed_ms * 3.6
        
        lbl_meas_vel.config(text=f"RPS: {measured_velocity:.2f}")
        lbl_speed_kmh.config(text=f"{speed_kmh:.1f} km/h")
        lbl_speed_ms.config(text=f"{speed_ms:.2f} m/s")
        
        lbl_meas_pos.config(text=f"Pozycja: {measured_position:.2f} obr")
        lbl_distance.config(text=f"Dystans: {trip_distance_m:.2f} m")
        
        lag = latency_estimator.estimate_lag(measured_velocity)
        if lag is not None:
            lbl_system_lag.config(text=f"Lag: {int(lag)} ms")
            if lag < 300: lbl_system_lag.config(fg="#00ff00")
            elif lag < 600: lbl_system_lag.config(fg="orange")
            else: lbl_system_lag.config(fg="red")
        else:
            if abs(measured_velocity) < 0.5: lbl_system_lag.config(text="Lag: (Stop)", fg="#555")
    except: pass

def init_mqtt():
    global client
    log_mqtt("--- Inicjalizacja MQTT ---")
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER_ADDRESS, BROKER_PORT)
        client.loop_start()
    except Exception as e:
        mqtt_status_var.set("MQTT: Błąd")
        log_mqtt(f"Błąd: {e}")

# --- KOMENDY ---
cmd_frame = tk.LabelFrame(right_frame, text="Polecenia ODrive", bg=BG_COLOR, fg=FG_COLOR)
cmd_frame.pack(fill="x", pady=20, side="bottom")

def send_cmd(cmd):
    if client and client.is_connected():
        client.publish(TOPIC_CMD, cmd)
        log_mqtt(f">> CMD: {cmd}")

btn_full_start = None

def run_full_start_sequence():
    global btn_full_start
    log_mqtt("\n=== FULL START SEQUENCE ===")
    send_cmd("calibrate")
    
    def countdown():
        btn_full_start.config(state="disabled", bg="#555555")
        for i in range(10, 0, -1):
            btn_full_start.config(text=f"CZEKAJ: {i}s...")
            time.sleep(1)
        
        btn_full_start.config(text="KONFIGURACJA...")
        send_cmd("closed_loop")
        time.sleep(0.1)
        send_cmd("set_vel_mode")
        time.sleep(0.1)
        send_cmd("set_ramp_mode")
        
        log_mqtt("=== FULL START ZAKOŃCZONY ===\n")
        btn_full_start.config(text="★ FULL START (AUTO) ★", state="normal", bg=BTN_FULL_START_COLOR)

    threading.Thread(target=countdown, daemon=True).start()

btn_full_start = tk.Button(cmd_frame, text="★ FULL START (AUTO) ★", command=run_full_start_sequence, 
                           bg=BTN_FULL_START_COLOR, fg="white", height=2, font=("Arial", 11, "bold"))
btn_full_start.pack(fill="x", padx=10, pady=(10, 20))

tk.Button(cmd_frame, text="1. KALIBRACJA", command=lambda: send_cmd("calibrate"), bg="#AA8800", fg="white", height=1).pack(fill="x", padx=10, pady=2)
tk.Button(cmd_frame, text="2. CLOSED LOOP", command=lambda: send_cmd("closed_loop"), bg="#006600", fg="white", height=1).pack(fill="x", padx=10, pady=2)
tk.Button(cmd_frame, text="3. TRYB VELOCITY", command=lambda: send_cmd("set_vel_mode"), bg="#004488", fg="white", height=1).pack(fill="x", padx=10, pady=2)
tk.Button(cmd_frame, text="4. RAMP MODE", command=lambda: send_cmd("set_ramp_mode"), bg="#550088", fg="white", height=1).pack(fill="x", padx=10, pady=2)
tk.Button(cmd_frame, text="⚠ REBOOT ODRIVE", command=lambda: send_cmd("reboot_odrive"), bg=BTN_REBOOT_COLOR, fg="white", height=1, font=("Arial", 10, "bold")).pack(fill="x", padx=10, pady=(10, 2))

# ==========================================
# 4. LOGIKA GŁÓWNA (ZOPTYMALIZOWANA PĘTLA)
# ==========================================

def draw_gauge(canvas, val, max_v):
    canvas.delete("all")
    cx, cy, r = 200, 150, 130
    canvas.create_arc(cx-r, cy-r, cx+r, cy+r, start=0, extent=180, style="arc", outline="#333", width=25)
    limit_angle = (current_speed_limit / max_v) * 180
    canvas.create_arc(cx-r, cy-r, cx+r, cy+r, start=180, extent=-limit_angle, style="arc", outline="#665500", width=25)
    val_clamped = max(-max_v, min(max_v, val))
    draw_angle = (val_clamped / max_v) * 90 
    color = "#00ff00" if val >= 0 else "#ff5500"
    canvas.create_arc(cx-r, cy-r, cx+r, cy+r, start=90, extent=-draw_angle, style="arc", outline=color, width=25)
    canvas.create_text(cx, cy-20, text=f"{val:.1f}", fill="white", font=("Arial", 36, "bold"))
    canvas.create_text(cx, cy+25, text=f"Max Limit: {current_speed_limit:.1f} RPS", fill="#888", font=("Arial", 12))

def calculate_speed():
    global current_speed_limit, key_max_limit
    joy_val = 0.0
    joy_active = False
    if joysticks:
        try:
            joy = joysticks[0]
            if joy.get_numaxes() > 3:
                axis3 = joy.get_axis(3)
                current_speed_limit = ((1.0 - axis3) / 2.0) * ABSOLUTE_MAX_LIMIT
            else:
                current_speed_limit = key_max_limit
            axis1 = -joy.get_axis(1)
            if abs(axis1) > JOYSTICK_DEADZONE:
                joy_val = axis1 * current_speed_limit
                joy_active = True
        except: pass
    else:
        current_speed_limit = key_max_limit
    if joy_active: return joy_val
    return key_throttle * current_speed_limit

def on_key_press(event):
    global key_throttle, key_max_limit
    k = event.keysym.lower()
    if k == 'w': key_throttle = 1.0
    elif k == 's': key_throttle = -1.0
    elif k == 'r': key_max_limit = min(ABSOLUTE_MAX_LIMIT, key_max_limit + 1.0)
    elif k == 'f': key_max_limit = max(1.0, key_max_limit - 1.0)
    elif k == 'escape': end_fullscreen()

def on_key_release(event):
    global key_throttle
    k = event.keysym.lower()
    if k in ['w', 's']: key_throttle = 0.0

def main_loop():
    global plot_refresh_counter
    pygame.event.pump()
    target_rps = calculate_speed()
    
    # 1. Komunikacja i dane (Zawsze)
    if client and client.is_connected():
        client.publish(TOPIC_SET_VELOCITY, json.dumps({"velocity": round(target_rps, 3)}))
        latency_estimator.push_target(target_rps)
    
    # 2. Aktualizacja GUI (Zegary, Tekst) - Co klatkę
    target_label.config(text=f"Target: {target_rps:.2f} RPS")
    draw_gauge(gauge_canvas, target_rps, ABSOLUTE_MAX_LIMIT)
    
    if last_feedback_time > 0:
        diff_ms = (time.time() - last_feedback_time) * 1000.0
        lbl_packet_age.config(text=f"Sieć (Packet Age): {int(diff_ms)} ms")
        lbl_packet_age.config(fg="#88ff88" if diff_ms < 200 else "red")
    else:
        lbl_packet_age.config(text="Sieć: Brak danych", fg="grey")

    # 3. Aktualizacja Wykresu - TYLKO CO N-TĄ KLATKĘ (FIX NA LAGI)
    current_time = time.time() - start_time
    plot_data_x.append(current_time)
    plot_data_target.append(target_rps)
    plot_data_meas.append(measured_velocity)

    if plot_refresh_counter % PLOT_SKIP_FRAMES == 0:
        update_plot()
    
    plot_refresh_counter += 1
    root.after(50, main_loop)

# START
reset_joysticks()
init_mqtt()
root.bind("<KeyPress>", on_key_press)
root.bind("<KeyRelease>", on_key_release)
main_loop()
root.mainloop()