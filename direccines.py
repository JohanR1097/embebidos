from machine import Pin, SoftI2C

# Configura I2C (ajusta los pines SDA y SCL según tu hardware)
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=400000)  # Pines comunes en ESP32

print("Escaneando bus I2C...")
devices = i2c.scan()

if len(devices) == 0:
    print("No se encontraron dispositivos I2C!")
else:
    print("Dispositivos I2C encontrados en las direcciones:")
    for device in devices:
        print(hex(device))