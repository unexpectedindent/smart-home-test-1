#define RELAY_PIN 5
#define LED_PIN 2

void setup() {
  Serial.begin(115200);
    
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("ESP8266 Relay Controller Ready");
  Serial.println("Commands: ON, OFF, TOGGLE, STATUS");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    processCommand(command);
  }
}

void processCommand(String cmd) {
  cmd.toUpperCase();

  if (cmd == "ON") {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    Serial.println("RELAY: ON");
  }
  else if (cmd == "OFF") {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("RELAY: OFF");
  }
  else if (cmd == "TOGGLE") {
    digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.println("RELAY: TOGGLED");
  }
  else if (cmd == "STATUS") {
    Serial.println(digitalRead(RELAY_PIN) ? "RELAY: ON" : "RELAY: OFF");
  }
  else {
    Serial.println("UNKNOWN COMMAND");
  }
}
