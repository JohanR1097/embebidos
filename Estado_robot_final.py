# === CÓDIGO 1 REFRACTORIZADO ===
import os
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"

import threading
import queue
import random
import cv2
import numpy as np
from picamera2 import Picamera2
import paho.mqtt.client as mqtt
import json
from rich import print
from rich.console import Console
from scipy.optimize import linear_sum_assignment
import time

# Consola enriquecida
output = Console()
print_lock = threading.Lock()

# --- COLAS DE DATOS ---
cola_mensajes = queue.Queue()
cola_frames = queue.Queue(maxsize=1)

# --- CONFIGURACION MQTT ---
direccion_broker = "172.20.10.2"
puerto_mqtt = 1883
tema_posicion = "camara/localizacion"
tema_destino = "camara/objetivo"

cliente = mqtt.Client()
cliente.connect(direccion_broker, puerto_mqtt)
cliente.loop_start()

# --- CONFIGURACION DE LA CAMARA ---
camera = Picamera2()
cam_config = camera.create_preview_configuration(main={"format": 'RGB888', "size": (320, 440)})
camera.configure(cam_config)
camera.start()

hsv_inferior = np.array([-10, 205, 205])
hsv_superior = np.array([10, 305, 305])
muestra_color = False
seleccion_manual = False

filas, columnas = 8, 6
margen = 0

# Coordenadas objetivo aleatorias
while True:
    fila_objetivo = random.randint(0, filas - 1)
    col_objetivo = random.randint(0, columnas - 1)
    if (fila_objetivo, col_objetivo) != (0, 0):
        break

with print_lock:
    print(f"Objetivo inicial: ({fila_objetivo+1},{col_objetivo+1})")

ultima_posicion = None
robots_detectados = []
formacion_actual = []
nombre_formacion = ""

# === FUNCIONES ===

def manejador_mqtt():
    global ultima_posicion
    while True:
        mensaje = cola_mensajes.get()
        if mensaje == "salir":
            break
        elif mensaje == "listo":
            if ultima_posicion:
                cliente.publish(tema_posicion, f"{ultima_posicion[0]+1},{ultima_posicion[1]+1}")
            cliente.publish(tema_destino, f"{fila_objetivo+1},{col_objetivo+1}")
            with print_lock:
                print("[MQTT] Coordenadas enviadas por 'listo'.")

def callback_raton(evento, x, y, flags, param):
    global hsv_inferior, hsv_superior, muestra_color, seleccion_manual, fila_objetivo, col_objetivo
    if evento == cv2.EVENT_LBUTTONDOWN:
        if muestra_color:
            pixel = hsv_img[y, x]
            hsv_inferior = np.array([pixel[0] - 10, pixel[1] - 50, pixel[2] - 50])
            hsv_superior = np.array([pixel[0] + 10, pixel[1] + 50, pixel[2] + 50])
            with print_lock:
                print(f"[COLOR] Nuevo rango HSV: {hsv_inferior} - {hsv_superior}")
            muestra_color = False
        elif seleccion_manual:
            col = x // (frame_actual.shape[1] // columnas)
            row = y // (frame_actual.shape[0] // filas)
            fila_objetivo, col_objetivo = row, col
            with print_lock:
                print(f"[OBJETIVO] Nuevo objetivo manual: ({fila_objetivo+1},{col_objetivo+1})")
            seleccion_manual = False

def definir_formacion(tipo):
    if tipo == "vertical":
        return [(f, columnas // 2) for f in range(2, 7)]
    elif tipo == "horizontal":
        return [(filas // 2, c) for c in range(1, 6)]
    elif tipo == "cruz":
        return [(4, 3), (3, 3), (5, 3), (4, 2), (4, 4)]
    elif tipo == "cuadrado":
        return [(3,2), (3,3), (4,2), (4,3), (4,4)]
    return []

def seleccionar_formacion():
    global formacion_actual, nombre_formacion
    with print_lock:
        print("
[FORMA] Elegir formacion:")
        print("1. Vertical
2. Horizontal
3. Cruz
4. Cuadrado
5. Cancelar")
    opcion = input("Opcion: ").strip()
    opciones = {"1": "vertical", "2": "horizontal", "3": "cruz", "4": "cuadrado"}
    nombre_formacion = opciones.get(opcion, "")
    formacion_actual = definir_formacion(nombre_formacion) if nombre_formacion else []
    if not formacion_actual:
        with print_lock:
            print("[FORMA] Cancelado o invalido.")

def emparejar(actuales, objetivos):
    costo = np.zeros((len(actuales), len(objetivos)))
    for i, (xa, ya) in enumerate(actuales):
        for j, (xo, yo) in enumerate(objetivos):
            costo[i, j] = abs(xa - xo) + abs(ya - yo)
    filas_res, columnas_res = linear_sum_assignment(costo)
    return list(zip([actuales[i] for i in filas_res], [objetivos[j] for j in columnas_res]))

def procesamiento_video():
    global hsv_img, ultima_posicion, frame_actual
    while True:
        frame_actual = camera.capture_array()
        alto, ancho, _ = frame_actual.shape
        ancho_celda = (ancho - 2 * margen) // columnas
        alto_celda = (alto - 2 * margen) // filas

        hsv_img = cv2.cvtColor(frame_actual, cv2.COLOR_RGB2HSV)
        mascara = cv2.inRange(hsv_img, hsv_inferior, hsv_superior)
        iluminacion = np.zeros((filas, columnas), dtype=int)
        robots_detectados.clear()

        for i in range(filas):
            for j in range(columnas):
                x1, y1 = j * ancho_celda, i * alto_celda
                x2, y2 = x1 + ancho_celda, y1 + alto_celda
                celda = mascara[y1:y2, x1:x2]
                iluminacion[i, j] = cv2.countNonZero(celda)
                if iluminacion[i, j] / (ancho_celda * alto_celda) >= 0.99:
                    robots_detectados.append((i+1, j+1))
                cv2.rectangle(frame_actual, (x1, y1), (x2, y2), (255, 255, 255), 1)

        for i, j in formacion_actual:
            x1 = (j-1) * ancho_celda
            y1 = (i-1) * alto_celda
            cv2.rectangle(frame_actual, (x1, y1), (x1 + ancho_celda, y1 + alto_celda), (0, 0, 255), 2)

        x1 = col_objetivo * ancho_celda
        y1 = fila_objetivo * alto_celda
        cv2.rectangle(frame_actual, (x1, y1), (x1 + ancho_celda, y1 + alto_celda), (255, 0, 255), 3)
        cv2.putText(frame_actual, "Objetivo", (x1+5, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        ultima_posicion = None
        for i in range(filas):
            for j in range(columnas):
                if iluminacion[i, j] > 150:
                    ultima_posicion = (i, j)
                    cv2.putText(frame_actual, f"({i+1},{j+1})", (j*ancho_celda+5, i*alto_celda+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        try:
            cola_frames.put_nowait((frame_actual.copy(), mascara.copy()))
        except queue.Full:
            pass

# === HILOS INICIALES ===
thread_camara = threading.Thread(target=procesamiento_video)
thread_mqtt = threading.Thread(target=manejador_mqtt)
thread_camara.start()
thread_mqtt.start()

cv2.namedWindow("Monitor")
cv2.setMouseCallback("Monitor", callback_raton)

# === BUCLE PRINCIPAL ===
while True:
    if not cola_frames.empty():
        frame, mascara = cola_frames.get()
        cv2.imshow("Monitor", frame)
        cv2.imshow("Mascara", mascara)
    tecla = cv2.waitKey(1) & 0xFF
    if tecla == ord('q'):
        cola_mensajes.put("salir")
        break
    elif tecla == ord('c'):
        muestra_color = True
    elif tecla == ord('p'):
        if ultima_posicion:
            cliente.publish(tema_posicion, f"{ultima_posicion[0]+1},{ultima_posicion[1]+1}")
        cliente.publish(tema_destino, f"{fila_objetivo+1},{col_objetivo+1}")
    elif tecla == ord('o'):
        seleccion_manual = True
    elif tecla == ord('m'):
        seleccionar_formacion()
    elif tecla == ord('f'):
        if not formacion_actual:
            print("[ERROR] No hay formacion seleccionada.")
        elif not robots_detectados:
            print("[ERROR] No se detectaron robots.")
        else:
            pares = emparejar(robots_detectados, formacion_actual)
            for idx, (origen, destino) in enumerate(pares):
                cliente.publish(tema_posicion, f"{origen[0]},{origen[1]}")
                cliente.publish(tema_destino, f"{destino[0]},{destino[1]}")
                print(f"[ASIGNACION] Robot {idx}: {origen} -> {destino}")

cv2.destroyAllWindows()
camera.stop()
cliente.loop_stop()
cliente.disconnect()
thread_camara.join()
thread_mqtt.join()
