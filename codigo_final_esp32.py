
from MPU6050 import MPU6050 as SensorInercial
from motores import Motores as SistemaMotores
from sensores import SensoresLaser as Distanciometros
from pid_controller import PIDController as ControladorPID
from matriz import MatrizLED as PantallaLED
import time
import machine
from umqtt.simple import MQTTClient
import network
import _thread
import ujson

WIFI_SSID = "iPhone de Johan"
WIFI_PASS = "Johan2024"

# MQTT
MQTT_BROKER = "172.20.10.2"
MQTT_PORT = 1883
MQTT_TOPIC_POSICION = b"camara/localizacion"
MQTT_TOPIC_OBJETIVO = b"camara/objetivo"
ID_ROBOT = "robot1"
MQTT_TOPIC_LED = b"robot/%s/led_matrix" % ID_ROBOT.encode()

# Estados globales
posicion_actual = None
coordenada_destino = None
flag_posicion_recibida = False
flag_objetivo_recibido = False

# Parámetros
vel_ini, vel_fin, vel_giro = 410, 380, 750
kp_control = 100
angulo_giro_fijo = 90

# Mapa
tamano_celda = 70
desfase_x = 80
desfase_y = 70

# Inicialización
matriz_led = PantallaLED()
matriz_led.encender_azul()
mpu = SensorInercial()
mpu.calibrarGiroscopio()
motores = SistemaMotores()
sensores = Distanciometros()
control_pid = ControladorPID(kp_control)

orientacion_actual = "NORTE"

def conectar_wifi():
    red = network.WLAN(network.STA_IF)
    red.active(True)
    red.connect(WIFI_SSID, WIFI_PASS)
    print("[WIFI] Conectando...")
    while not red.isconnected():
        time.sleep(1)
    print("[WIFI] Conectado:", red.ifconfig())

def procesar_mqtt(topico, mensaje):
    global posicion_actual, coordenada_destino
    global flag_posicion_recibida, flag_objetivo_recibido

    if topico == MQTT_TOPIC_POSICION:
        try:
            x, y = map(int, mensaje.decode().split(","))
            posicion_actual = [x, y]
            flag_posicion_recibida = True
            print(f"[MQTT] Posición recibida: {posicion_actual}")
        except Exception as e:
            print(f"[ERROR] Posición malformada: {e}")

    elif topico == MQTT_TOPIC_OBJETIVO:
        try:
            x, y = map(int, mensaje.decode().split(","))
            coordenada_destino = [x, y]
            flag_objetivo_recibido = True
            print(f"[MQTT] Objetivo recibido: {coordenada_destino}")
        except Exception as e:
            print(f"[ERROR] Objetivo malformado: {e}")

    elif topico == MQTT_TOPIC_LED:
        try:
            datos = ujson.loads(mensaje.decode())
            leds = datos.get("leds", [])
            color = datos.get("color", [255,255,255])
            print("[MQTT] Patrón LED recibido")
            matriz_led.mostrar_patron(leds, color)
        except Exception as e:
            print("[ERROR] LED Matrix:", e)

def iniciar_mqtt():
    cliente = MQTTClient("ESP32", MQTT_BROKER, MQTT_PORT)
    cliente.set_callback(procesar_mqtt)
    cliente.connect()
    print("[MQTT] Conectado al broker")
    cliente.subscribe(MQTT_TOPIC_POSICION)
    cliente.subscribe(MQTT_TOPIC_OBJETIVO)
    cliente.subscribe(MQTT_TOPIC_LED)
    print("[MQTT] Subscripciones completadas")
    return cliente

def enviar_estado():
    global orientacion_actual, sensores, mqtt_cliente
    while True:
        try:
            distancia, _ = sensores.leer_distancias()
            estado = "moviendose" if motores.en_movimiento() else "detenido"
            reporte = ujson.dumps({
                "angle": orientacion_actual,
                "status": estado,
                "distance": distancia
            })
            mqtt_cliente.publish(b"robot/estado", reporte)
            print("[ESTADO] Publicado:", reporte)
            time.sleep(1)
        except Exception as e:
            print("[ESTADO] Error:", e)
            time.sleep(2)

def girar(direccion):
    global yaw_ref
    yaw_ref = mpu.actualizarYaw()
    if direccion == "derecha":
        motores.girar_derecha(vel_giro)
    else:
        motores.girar_izquierda(vel_giro)

    print(f"[GIRO] Inicio: {yaw_ref:.1f}°")
    while True:
        yaw_actual = mpu.actualizarYaw()
        diferencia = yaw_actual - yaw_ref
        if direccion == "derecha" and diferencia <= -85:
            motores.detener_motores()
            break
        elif direccion == "izquierda" and diferencia >= 85:
            motores.detener_motores()
            break
        time.sleep_ms(10)
    time.sleep_ms(200)
    print(f"[GIRO] Finalizado. Yaw: {mpu.actualizarYaw():.1f}°")

def actualizar_direccion(giro):
    global orientacion_actual
    secuencia = ["NORTE", "ESTE", "SUR", "OESTE"]
    idx = secuencia.index(orientacion_actual)
    if giro == "derecha":
        orientacion_actual = secuencia[(idx + 1) % 4]
    elif giro == "izquierda":
        orientacion_actual = secuencia[(idx - 1) % 4]
    elif giro == "180":
        orientacion_actual = secuencia[(idx + 2) % 4]
    print("[DIR] Actualizada a:", orientacion_actual)

def alinear_con(direccion_destino):
    if orientacion_actual == direccion_destino:
        return
    mapa = {
        ("NORTE", "ESTE"): "derecha",
        ("ESTE", "SUR"): "derecha",
        ("SUR", "OESTE"): "derecha",
        ("OESTE", "NORTE"): "derecha",
        ("NORTE", "OESTE"): "izquierda",
        ("OESTE", "SUR"): "izquierda",
        ("SUR", "ESTE"): "izquierda",
        ("ESTE", "NORTE"): "izquierda",
    }
    if (orientacion_actual, direccion_destino) in mapa:
        giro = mapa[(orientacion_actual, direccion_destino)]
        girar(giro)
        actualizar_direccion(giro)
    else:
        girar("derecha"); actualizar_direccion("derecha")
        girar("derecha"); actualizar_direccion("derecha")

def avanzar_hacia(destino):
    global yaw_ref, posicion_actual
    yaw_ref = mpu.actualizarYaw()
    print(f"[MOVER] Desde {posicion_actual} hacia {destino}")
    x_dest, y_dest = destino
    x_act, y_act = posicion_actual

    # Movimiento vertical (Y)
    if y_dest != y_act:
        delta_y = y_dest - y_act
        while orientacion_actual != ("NORTE" if delta_y > 0 else "SUR"):
            girar("derecha")
            actualizar_direccion("derecha")
        distancia_mm = abs(delta_y) * tamano_celda
        motores.mover_adelante(vel_ini)
        d_inicial = sensores.leer_distancias()[0]
        d_meta = d_inicial - distancia_mm
        while True:
            actual = sensores.leer_distancias()[0]
            error = yaw_ref - mpu.actualizarYaw()
            ajuste = control_pid.calcular_ajuste(error)
            vi = max(min(vel_fin - ajuste, 900), 300)
            vd = max(min(vel_fin + ajuste, 900), 300)
            motores.mover_ruedas(vi, vd)
            if abs(actual - d_meta) <= 10:
                break
            time.sleep_ms(20)
        motores.detener_motores()
        y_act = y_dest

    # Movimiento horizontal (X)
    if x_dest != x_act:
        delta_x = x_dest - x_act
        while orientacion_actual != ("ESTE" if delta_x > 0 else "OESTE"):
            girar("derecha")
            actualizar_direccion("derecha")
        distancia_mm = abs(delta_x) * tamano_celda
        motores.mover_adelante(vel_ini)
        d_inicial = sensores.leer_distancias()[0]
        d_meta = d_inicial - distancia_mm
        while True:
            actual = sensores.leer_distancias()[0]
            error = yaw_ref - mpu.actualizarYaw()
            ajuste = control_pid.calcular_ajuste(error)
            vi = max(min(vel_fin - ajuste, 900), 300)
            vd = max(min(vel_fin + ajuste, 900), 300)
            motores.mover_ruedas(vi, vd)
            if abs(actual - d_meta) <= 10:
                break
            time.sleep_ms(20)
        motores.detener_motores()
        x_act = x_dest

    posicion_actual = [x_act, y_act]
    print(f"[MOVER] Nueva posición: {posicion_actual}")

# === BUCLE PRINCIPAL ===
try:
    print("[INICIO] Inicializando...")
    yaw_ref = mpu.actualizarYaw()
    conectar_wifi()
    mqtt_cliente = iniciar_mqtt()
    _thread.start_new_thread(enviar_estado, ())

    while True:
        mqtt_cliente.check_msg()
        if not (flag_posicion_recibida and flag_objetivo_recibido):
            print("[ESPERA] Esperando coordenadas...")
            time.sleep(0.1)
            continue
        avanzar_hacia(coordenada_destino)
        flag_posicion_recibida = False
        flag_objetivo_recibido = False
        time.sleep(5)

except KeyboardInterrupt:
    print("[INTERRUPCION] Deteniendo robot...")
    motores.detener_motores()
except Exception as err:
    print(f"[ERROR CRITICO] {err}")
    machine.reset()
