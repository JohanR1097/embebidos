import network
import json
from time import sleep
from umqtt.robust import MQTTClient
from machine import Pin
from neopixel import NeoPixel

# Configuración WiFi
SSID = "iPhone de Johan"
PWD = "Johan2024"

# Configuración del Broker MQTT (IP de la Raspberry Pi)
BROKER = "172.20.10.2"
CLIENT_ID = "ESP32_Matriz"
TOPIC_SUB = b"test/topic"      # Topic para recibir comandos de la RPi
TOPIC_PUB = b"rpi/topic"       # Topic donde el ESP32 puede enviar datos

# Configuración de la Matriz LED
LED_PIN = 5  # Cambia esto según tu conexión física al ESP32
MATRIX_WIDTH = 8
MATRIX_HEIGHT = 8
NUM_PIXELS = MATRIX_WIDTH * MATRIX_HEIGHT
np = NeoPixel(Pin(LED_PIN), NUM_PIXELS)

# Intensidad por defecto (0.3 equivale a un 30% del brillo máximo)
default_intensity = 0.03

# Colores predefinidos
base_colors = {
    "rojo": (255, 0, 0),
    "verde": (0, 255, 0),
    "azul": (0, 0, 255),
    "amarillo": (255, 255, 0),
    "cyan": (0, 255, 255),
    "magenta": (255, 0, 255),
    "blanco": (255, 255, 255),
    "apagado": (0, 0, 0)
}

# Conectar a la red WiFi
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PWD)
    while not wlan.isconnected():
        sleep(1)
    print("Conexión WiFi establecida:", wlan.ifconfig())

# Función para aplicar la intensidad a un color
def apply_intensity(color, intensity):
    return tuple(int(c * intensity) for c in color)

# Función para llenar la matriz con un color específico
def fill_matrix(color):
    for i in range(NUM_PIXELS):
        np[i] = color
    np.write()

# Función que maneja los mensajes recibidos
def message(topic, msg):
    global default_intensity
    print("Mensaje recibido en el topic:", topic)
    print("Contenido:", msg.decode())
    
    try:
        data = json.loads(msg.decode())
        
        # Obtener el color y la intensidad del mensaje
        color_name = data.get("color", "apagado").lower()
        intensity = data.get("intensidad", default_intensity)

        if intensity < 0 or intensity > 1:
            print("Valor de intensidad inválido. Debe estar entre 0 y 1.")
            return

        if color_name in base_colors:
            color = apply_intensity(base_colors[color_name], intensity)
            fill_matrix(color)
            print(f"Matriz encendida en color: {color_name} con intensidad: {intensity}")
        else:
            print("Color no reconocido.")
    except ValueError:
        print("Error al interpretar el mensaje JSON.")

# Configuración del cliente MQTT
client = MQTTClient(CLIENT_ID, BROKER)
client.set_callback(message)

# Conectar a WiFi y al broker MQTT
wifi_connect()
client.connect()
client.subscribe(TOPIC_SUB)
print(f"Suscrito al topic: {TOPIC_SUB.decode()}")

# Loop principal para recibir mensajes
try:
    while True:
        client.wait_msg()  # Espera por nuevos mensajes
except KeyboardInterrupt:
    client.disconnect()
    print("Desconectado del broker.")