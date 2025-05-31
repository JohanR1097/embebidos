# === CÓDIGO 2 REFRACTORIZADO ===
import paho.mqtt.client as mqtt
import json
from rich import print
from rich.console import Console
from rich.table import Table
from datetime import datetime

# Consola para salida bonita
monitor_console = Console()

def conexion_exitosa(conexion, datos, banderas, resultado):
    conexion.subscribe("robot/estado")
    print("[SUSCRIPCION] Canal 'robot/estado' activado")

def procesar_mensaje(conexion, datos, mensaje):
    try:
        contenido = json.loads(mensaje.payload.decode())
        hora_actual = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        tabla_estado = Table(title=f"[bold blue]Estado del Robot - {hora_actual}[/bold blue]")
        tabla_estado.add_column("Parámetro", style="cyan", no_wrap=True)
        tabla_estado.add_column("Lectura", style="magenta")

        for clave, valor in contenido.items():
            tabla_estado.add_row(str(clave), str(valor))

        monitor_console.print(tabla_estado)

        with open("bitacora_estado.txt", "a") as archivo:
            archivo.write(f"{hora_actual} - {json.dumps(contenido)}\n")

    except Exception as error:
        print(f"[ERROR] Formato de mensaje incorrecto: {error}")

cliente_monitor = mqtt.Client()
cliente_monitor.on_connect = conexion_exitosa
cliente_monitor.on_message = procesar_mensaje
cliente_monitor.connect("172.20.10.2", 1883)

print("[MONITOR] Esperando información de los robots...")
cliente_monitor.loop_forever()
