import time
import serial
import json
import paho.mqtt.client as mqtt
import Adafruit_DHT
import spidev
import RPi.GPIO as GPIO

# --- Configuración MQTT ---
BROKER = "mqtt.beebotte.com"
PORT = 1883
TOKEN = "tu_token"
CHANNEL = "Greenhouse"

# Recursos
RESOURCES = {
    "Soil1": "FC28_1",
    "Soil2": "FC28_2",
    "AirQuality": "MQ135",
    "Rain": "FC37",
    "CapSoil1": "Capacitivo_1",
    "CapSoil2": "Capacitivo_2",
    "Light": "KY018",
    "Temp": "DHT22_Temperatura",
    "Hum": "DHT22_Humedad",
    "Ultra1": "HC_SR04_1",
    "Ultra2": "HC_SR04_2",
    "AutoMode": "AutoMode",
    "Relay1": "Relay_Bomba",
    "Relay2": "Relay_Sirena",
    "Relay3": "Relay_LED",
    "Relay4": "Relay_Ventilador",
    "Relay5": "Relay_Calefactor",
    "Relay6": "Relay_Nebulizador",
    "Relay7": "Relay_Lluvia",
    "Relay8": "Relay_BombaTanque"
}

# --- Configuración LM393 para LDR ---
LDR_PIN = 18  # Usamos GPIO18 para la señal digital del sensor
GPIO.setup(LDR_PIN, GPIO.IN)

def read_adc(channel):
    adc = spi.xfer2([1, (8+channel)<<4, 0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

# --- DHT22 ---
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4  # GPIO4

# --- Serial Arduino ---
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# --- GPIO ---
GPIO.setmode(GPIO.BCM)

# Relés asignados
RELAY_PINS = {
    "Relay1": 5,
    "Relay2": 6,
    "Relay3": 13,
    "Relay4": 19,
    "Relay5": 26,
    "Relay6": 16,
    "Relay7": 20,
    "Relay8": 21
}

for pin in RELAY_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)  # Relés desactivados inicialmente

relay_states = {r: False for r in RELAY_PINS}

# Ultrasonidos
ULTRA1_TRIG, ULTRA1_ECHO = 17, 27
ULTRA2_TRIG, ULTRA2_ECHO = 22, 23
GPIO.setup(ULTRA1_TRIG, GPIO.OUT)
GPIO.setup(ULTRA1_ECHO, GPIO.IN)
GPIO.setup(ULTRA2_TRIG, GPIO.OUT)
GPIO.setup(ULTRA2_ECHO, GPIO.IN)

def medir_distancia(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()
    while GPIO.input(echo) == 1:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2
    return round(distance, 2)

# --- Control de relés ---
def set_relay(name, state):
    """Cambia el estado de un relé y lo sincroniza en la nube si hay cambio"""
    pin = RELAY_PINS[name]
    if relay_states[name] != state:
        GPIO.output(pin, GPIO.LOW if state else GPIO.HIGH)
        relay_states[name] = state
        print(f"{name} -> {'ON' if state else 'OFF'}")

        # Publicar cambio a la nube
        relay_payload = {
            "resource": RESOURCES[name],
            "data": 1 if state else 0
        }
        client.publish(f"{CHANNEL}/control", json.dumps(relay_payload))

# --- MQTT ---
AutoMode = True  # modo automático por defecto

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker MQTT:", rc)
    client.subscribe(f"{CHANNEL}/control")

def on_message(client, userdata, msg):
    global AutoMode
    try:
        data = json.loads(msg.payload.decode())
        resource = data.get("resource")
        value = data.get("data")
        if resource == RESOURCES["AutoMode"]:
            AutoMode = bool(value)
            print("Modo automático:", AutoMode)
        elif resource in RELAY_PINS:  # Control manual solo si AutoMode = False
            if not AutoMode:
                set_relay(resource, bool(value))
    except Exception as e:
        print("Error MQTT:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("token:%s" % TOKEN)
client.connect(BROKER, PORT, 60)
client.loop_start()

# --- Umbrales configurables ---
SOIL_LOW = 40
SOIL_HIGH = 90
AIR_BAD = 300
LIGHT_LOW = 0
TEMP_HIGH = 30
TEMP_LOW = 15
HUM_LOW = 40
HUM_HIGH = 80
RAIN_WET = 500
ULTRA_FILL = 180
ULTRA_STOP = 50

# --- Bucle principal ---
try:
    while True:
        payload = {}

        # Sensores Raspberry
        ldr_value = GPIO.input(LDR_PIN)  # 0 o 1, dependiendo de si hay luz o no
        humedad, temperatura = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
        payload[RESOURCES["Light"]] = ldr_value
        if humedad is not None and temperatura is not None:
            payload[RESOURCES["Temp"]] = round(temperatura, 2)
            payload[RESOURCES["Hum"]] = round(humedad, 2)

        payload[RESOURCES["Ultra1"]] = medir_distancia(ULTRA1_TRIG, ULTRA1_ECHO)
        payload[RESOURCES["Ultra2"]] = medir_distancia(ULTRA2_TRIG, ULTRA2_ECHO)

        # Datos Arduino
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode("utf-8").strip()
                data = json.loads(line)
                for key in ["Soil1", "Soil2", "AirQuality", "Rain", "CapSoil1", "CapSoil2"]:
                    payload[RESOURCES[key]] = data.get(key, None)
            except Exception as e:
                print("Error leyendo Arduino:", e)

        # --- Lógica automática ---
        if AutoMode:
            soil = (payload.get(RESOURCES["Soil1"], 100) +
                    payload.get(RESOURCES["Soil2"], 100) +
                    payload.get(RESOURCES["CapSoil1"], 100) +
                    payload.get(RESOURCES["CapSoil2"], 100)) / 4
            if soil < SOIL_LOW:
                set_relay("Relay1", True)
            elif soil > SOIL_HIGH:
                set_relay("Relay1", False)

            air = payload.get(RESOURCES["AirQuality"], 0)
            set_relay("Relay2", air > AIR_BAD)

            rain = payload.get(RESOURCES["Rain"], 0)
            set_relay("Relay7", rain > RAIN_WET)

            set_relay("Relay3", ldr_value == LIGHT_LOW)

            if temperatura is not None:
                if temperatura > TEMP_HIGH or (humedad and humedad > HUM_HIGH):
                    set_relay("Relay4", True)
                else:
                    set_relay("Relay4", False)

                set_relay("Relay5", temperatura < TEMP_LOW)

            if humedad is not None:
                set_relay("Relay6", humedad < HUM_LOW)

            if (payload[RESOURCES["Ultra1"]] > ULTRA_FILL or 
                payload[RESOURCES["Ultra2"]] > ULTRA_FILL):
                set_relay("Relay8", True)
            if (payload[RESOURCES["Ultra1"]] < ULTRA_STOP or 
                payload[RESOURCES["Ultra2"]] < ULTRA_STOP):
                set_relay("Relay8", False)

        # Enviar lecturas de sensores
        if payload:
            client.publish(f"{CHANNEL}/data", str({"data": payload}))
            print("Enviado:", payload)

        time.sleep(2)

except KeyboardInterrupt:
    print("Programa detenido")
    spi.close()
    ser.close()
    client.loop_stop()
    client.disconnect()
    GPIO.cleanup()
