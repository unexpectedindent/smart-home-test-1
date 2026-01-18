# !/usr/bin/env python3
import serial
import paho.mqtt.client as mqtt
import time
import threading
import os
import json
from typing import Optional


class SerialMQTTBridge:
    def __init__(self):
        self.serial_timeout = 1
        self.reconnect_delay = 2
        self.heartbeat_interval = 60
        self.serial_port = os.getenv('SERIAL_PORT', '/dev/ttyUSB0')
        self.baud_rate = int(os.getenv('BAUD_RATE', '115200'))
        self.mqtt_broker = os.getenv('MQTT_BROKER', 'mosquitto')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))

        self.command_topic = 'home/test/relay/set'
        self.state_topic = 'home/test/relay/state'

        self.ser: Optional[serial.Serial] = None
        self.mqtt_client = mqtt.Client()
        self.running = True

        self.setup_mqtt()
        self.heartbeat_topic = 'home/test/relay/heartbeat'
        self.last_heartbeat = 0

    def setup_mqtt(self):
        """Настройка MQTT клиента"""
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback при подключении к MQTT"""
        if rc == 0:
            print("Connected to MQTT broker successfully")
            client.subscribe(self.command_topic)
        else:
            print(f"MQTT connection failed with code: {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback при отключении от MQTT"""
        print(f"MQTT disconnected (code: {rc})")
        if rc != 0:
            print("Unexpected disconnection, attempting reconnect...")
            self.connect_mqtt()

    def on_mqtt_message(self, client, userdata, msg):
        """Обработка входящих MQTT сообщений"""
        try:
            payload = msg.payload.decode().strip()
            print(f"MQTT → Serial: '{payload}'")

            try:
                data = json.loads(payload)
                if 'command' in data:
                    self.write_to_serial(data['command'])
                return
            except json.JSONDecodeError:
                # Если не JSON, обрабатываем как plain text
                pass

            self.write_to_serial(payload)
        except Exception as e:
            print(f"Error processing MQTT message: {e}")

    def connect_serial(self) -> bool:
        """Подключение к Serial порту"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.serial_timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                write_timeout=self.reconnect_delay,
                rtscts=False,
                dsrdtr=False
            )
            time.sleep(self.reconnect_delay)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"Connected to serial: {self.serial_port}")
            return True
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            return False

    def write_to_serial(self, data: str):
        """Запись данных в Serial порт"""
        if not self.ser or not self.ser.is_open:
            print("Serial port not available")
            return

        try:
            # Добавляем символ новой строки если его нет
            if not data.endswith('\n'):
                data += '\n'
            self.ser.write(data.encode())
        except serial.SerialException as e:
            print(f"Serial write error: {e}")
            self.connect_serial()

    def read_serial_loop(self):
        """Бесконечный цикл чтения из Serial порта"""
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(self.reconnect_delay)
                    continue

                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode().strip()
                    if line:
                        print(f"Serial: {line}")
                        self.process_serial_data(line)

            except serial.SerialException as e:
                print(f"Serial read error: {e}. Reconnecting...")
                time.sleep(self.reconnect_delay)
                self.connect_serial()
            except UnicodeDecodeError:
                print("Serial decode error - ignoring line")
            except Exception as e:
                print(f"Unexpected error in serial loop: {e}")
                time.sleep(self.serial_timeout)

    def process_serial_data(self, data: str):
        """Обработка данных из Serial порта"""
        try:
            data_clean = data.strip()
            if not data_clean:
                return

            print(f"Serial: {data_clean}")
            self.mqtt_client.publish("home/test/relay/debug", data_clean)
            data_upper = data_clean.upper()

            # Проверка состояний
            if any(pattern in data_upper for pattern in ["RELAY: ON", "RELAY_ON", "STATE: ON"]):
                self.mqtt_client.publish(self.state_topic, "ON", retain=True)
            elif any(pattern in data_upper for pattern in ["RELAY: OFF", "RELAY_OFF", "STATE: OFF"]):
                self.mqtt_client.publish(self.state_topic, "OFF", retain=True)
            # Можно добавить обработку других сообщений

            # Публикация сырых данных для отладки
            self.mqtt_client.publish("home/test/relay/debug", data_clean)

        except Exception as e:
            print(f"Error processing serial data: {e}")
        # Можно добавить обработку других сообщений

    def connect_mqtt(self):
        """Подключение к MQTT брокеру с повторными попытками"""
        while self.running:
            try:
                self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
                return True
            except Exception as e:
                print(f"MQTT connection failed: {e}. Retrying in 5 seconds...")
                time.sleep(5)
        return False

    def start(self):
        """Запуск моста"""
        try:
            print(f"Starting Serial-MQTT Bridge")
            print(f"Serial: {self.serial_port}@{self.baud_rate}")
            print(f"MQTT: {self.mqtt_broker}:{self.mqtt_port}")

            # Подключение к Serial
            if not self.connect_serial():
                print("Failed to connect to serial port")
                return

            # Подключение к MQTT
            if not self.connect_mqtt():
                print("Failed to connect to MQTT broker")
                return

            # Запуск MQTT в отдельном потоке
            mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
            mqtt_thread.daemon = True
            mqtt_thread.start()

            print("Serial-MQTT bridge started successfully")
            heartbeat_thread = threading.Thread(target=self.heartbeat_loop)
            heartbeat_thread.daemon = True
            heartbeat_thread.start()
            # Основной цикл чтения Serial
            try:
                self.read_serial_loop()
            except KeyboardInterrupt:
                print("Bridge stopped by user")
            finally:
                self.stop()
        except Exception as e:
            print(f"Fatal error: {e}")
            self.stop()

    def is_serial_connected(self):
        return self.ser and self.ser.is_open

    def is_mqtt_connected(self):
        return self.mqtt_client.is_connected()

    def heartbeat_loop(self):
        """Отправка heartbeat сообщений"""
        while self.running:
            try:
                self.mqtt_client.publish(self.heartbeat_topic, "alive", retain=False)
                time.sleep(self.heartbeat_interval)
            except Exception as e:
                print(f"Heartbeat error: {e}")

    def stop(self):
        """Остановка моста"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.mqtt_client.disconnect()
        print("Bridge stopped")


def main():
    bridge = SerialMQTTBridge()
    bridge.start()


if __name__ == "__main__":
    main()