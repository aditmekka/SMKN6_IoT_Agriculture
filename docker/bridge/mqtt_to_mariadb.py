import os
import json
import time
import socket
import paho.mqtt.client as mqtt
import mysql.connector
import sys

# --- Real-time log flushing ---
sys.stdout.reconfigure(line_buffering=True)

# --- Environment variables ---
BROKER = os.getenv("MQTT_BROKER", "mosquitto")
TOPIC = os.getenv("MQTT_TOPIC", "esp32/data")

DB_HOST = os.getenv("DB_HOST", "mariadb")
DB_USER = os.getenv("DB_USER", "iotuser")
DB_PASS = os.getenv("DB_PASSWORD", "iotpass")
DB_NAME = os.getenv("DB_NAME", "iot_data")


# --- Database connection ---
def connect_db():
    while True:
        try:
            db = mysql.connector.connect(
                host=DB_HOST,
                user=DB_USER,
                password=DB_PASS,
                database=DB_NAME
            )
            print("✅ Connected to MariaDB")
            return db
        except Exception as e:
            print("⏳ Waiting for MariaDB:", e)
            time.sleep(3)


db = connect_db()
cursor = db.cursor()

# --- Auto-create table if not exists ---
cursor.execute("""
CREATE TABLE IF NOT EXISTS sensor_readings (
  id INT AUTO_INCREMENT PRIMARY KEY,
  device_id VARCHAR(50),
  temperature FLOAT,
  humidity FLOAT,
  pressure FLOAT,
  altitude FLOAT,
  soilMoisture1 FLOAT,
  soilMoisture2 FLOAT,
  soilMoisture3 FLOAT,
  soilMoisture4 FLOAT,
  soilMoisture5 FLOAT,
  airQuality FLOAT,
  rainLevel FLOAT,
  timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
)
""")
db.commit()


# --- MQTT callback ---
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print("📩 Received:", data)

        cursor.execute("""
            INSERT INTO sensor_readings (
                device_id, temperature, humidity, pressure, altitude,
                soilMoisture1, soilMoisture2, soilMoisture3, soilMoisture4, soilMoisture5,
                airQuality, rainLevel
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
        """, (
            data.get("device_id"),
            data.get("temperature"),
            data.get("humidity"),
            data.get("pressure"),
            data.get("altitude"),
            data.get("soilMoisture1"),
            data.get("soilMoisture2"),
            data.get("soilMoisture3"),
            data.get("soilMoisture4"),
            data.get("soilMoisture5"),
            data.get("airQuality"),
            data.get("rainLevel"),
        ))

        db.commit()

    except Exception as e:
        print("❌ Error inserting:", e)


# --- Wait for MQTT broker ---
def wait_for_mqtt():
    while True:
        try:
            s = socket.create_connection((BROKER, 1883), timeout=3)
            s.close()
            print("✅ MQTT broker is ready")
            return
        except Exception as e:
            print("⏳ Waiting for MQTT broker:", e)
            time.sleep(3)


# --- Main ---
wait_for_mqtt()

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPIC)

print(f"🚀 Listening for messages on {TOPIC} ...")
client.loop_forever()
