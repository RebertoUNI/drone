#include "interfacciaWeb.h"
#include <WebServer.h>
#include <WiFi.h>

const char *AP_SSID = "Drone_Test";
const char *AP_PASS = "DaniGay7";

#define RXD2 16
#define TXD2 17
#define NUCLEO_BAUD 115200

// Spegnimento automatico: l'STM32 spegne a 8000 ms.
// L'ESP32 e' un backup ridondante, scatta poco dopo.
#define ESP_MOTOR_TIMEOUT_MS 8500

// Limiti trim motori (devono coincidere con TRIM_MIN / TRIM_MAX dell'STM32)
#define TRIM_MIN 0.80f
#define TRIM_MAX 1.20f

WebServer server(80);

// Variabili globali per memorizzare telemetria
String current_pitch = "0.00";
String current_roll = "0.00";
String current_yaw_rate = "0.00";
String current_pid_pitch_p = "1.00";
String current_pid_pitch_i = "0.00";
String current_pid_pitch_d = "0.00";
String current_pid_roll_p = "1.00";
String current_pid_roll_i = "0.00";
String current_pid_roll_d = "0.00";
String current_pid_yaw_p = ""; // vuoti finche' l'STM32 non risponde
String current_pid_yaw_i = "";
String current_pid_yaw_d = "";
String current_trim[4] = {"", "", "", ""}; // vuoti finche' l'STM32 non risponde
String current_motors = "0";               // stato motori riportato dall'STM32

// Stato lato ESP32 per il watchdog di backup
bool esp_motors_on = false;
unsigned long esp_motor_start_ms = 0;

String telemetryField(const String &payload, const char *name) {
  String prefix = String(name) + ":";
  int start = payload.indexOf(prefix);
  if (start < 0) {
    return "";
  }

  start += prefix.length();
  int end = payload.indexOf(',', start);
  return payload.substring(start, end < 0 ? payload.length() : end);
}

// Invia lo stop di emergenza all'STM32 (ripetuto: un byte corrotto non deve
// poter impedire lo spegnimento)
void sendKillToStm() {
  for (int i = 0; i < 3; i++) {
    Serial2.print("CMD:e\n");
  }
  Serial2.print("THR:0\n");
  esp_motors_on = false;
  current_motors = "0";
}

void handleRoot() { server.send(200, "text/html", index_html); }

void handleCommand() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    if (action == "t") {
      Serial2.print("CMD:t\n");
      esp_motors_on = true;
      esp_motor_start_ms = millis();
    } else if (action == "s" || action == "e") {
      sendKillToStm();
    } else {
      Serial2.print("CMD:" + action + "\n");
    }
  }
  server.send(200, "text/plain", "OK");
}

// Emergency stop: spegne SEMPRE, a prescindere dallo stato
void handleEstop() {
  sendKillToStm();
  server.send(200, "text/plain", "OK");
}

void handleThrottle() {
  if (server.hasArg("val")) {
    String val = server.arg("val");
    Serial2.print("THR:" + val + "\n");
  }
  server.send(200, "text/plain", "OK");
}

void handlePID() {
  if (server.hasArg("pitch_p") && server.hasArg("pitch_i") &&
      server.hasArg("pitch_d") && server.hasArg("roll_p") &&
      server.hasArg("roll_i") && server.hasArg("roll_d")) {

    String pp = server.arg("pitch_p");
    String ip = server.arg("pitch_i");
    String dp = server.arg("pitch_d");
    String pr = server.arg("roll_p");
    String ir = server.arg("roll_i");
    String dr = server.arg("roll_d");
    // Invia stringa completa: PID:pitchP,pitchI,pitchD,rollP,rollI,rollD
    Serial2.printf("PID:%s,%s,%s,%s,%s,%s\n", pp.c_str(), ip.c_str(),
                   dp.c_str(), pr.c_str(), ir.c_str(), dr.c_str());
  }
  server.send(200, "text/plain", "OK");
}

// PID Yaw: /pidyaw?yaw_p=1.000&yaw_i=0.000&yaw_d=0.000
void handlePIDYaw() {
  if (server.hasArg("yaw_p") && server.hasArg("yaw_i") &&
      server.hasArg("yaw_d")) {
    String yp = server.arg("yaw_p");
    String yi = server.arg("yaw_i");
    String yd = server.arg("yaw_d");
    // Invia: YAW:P,I,D
    Serial2.printf("YAW:%s,%s,%s\n", yp.c_str(), yi.c_str(), yd.c_str());
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Parametri mancanti");
}

// Moltiplicatori motori: /trim?m1=1.000&m2=1.000&m3=1.000&m4=1.000
// M1 Front Left, M2 Front Right, M3 Back Right, M4 Back Left
void handleTrim() {
  if (server.hasArg("m1") && server.hasArg("m2") && server.hasArg("m3") &&
      server.hasArg("m4")) {
    float k[4];
    const char *names[4] = {"m1", "m2", "m3", "m4"};
    for (int i = 0; i < 4; i++) {
      k[i] = server.arg(names[i]).toFloat();
      if (k[i] < TRIM_MIN || k[i] > TRIM_MAX) {
        server.send(400, "text/plain", "Trim fuori range");
        return;
      }
    }
    Serial2.printf("TRIM:%.3f,%.3f,%.3f,%.3f\n", k[0], k[1], k[2], k[3]);
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Parametri mancanti");
}

// Direzione: /dir?p=50&r=-30&y=1
// p: -100..+100 (+ avanti)  | r: -100..+100 (+ destra)   [joystick analogico]
// y: -1, 0, +1 (+ orario)                                [pulsanti yaw]
// La pagina lo rinvia ogni 100 ms finche' il joystick/tasto e' attivo;
// l'STM32 riporta il drone in piano se non riceve DIR per 400 ms.
void handleDir() {
  if (server.hasArg("p") && server.hasArg("r") && server.hasArg("y")) {
    int p = constrain(server.arg("p").toInt(), -100, 100);
    int r = constrain(server.arg("r").toInt(), -100, 100);
    int y = constrain(server.arg("y").toInt(), -1, 1);
    Serial2.printf("DIR:%d,%d,%d\n", p, r, y);
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Parametri mancanti");
}

// Endpoint JSON con angoli, parametri PID, trim e stato motori.
void handleTelemetry() {
  String json = "{";
  json += "\"pitch\":\"" + current_pitch + "\",";
  json += "\"roll\":\"" + current_roll + "\",";
  json += "\"yaw_rate\":\"" + current_yaw_rate + "\",";
  json += "\"pid_pitch_p\":\"" + current_pid_pitch_p + "\",";
  json += "\"pid_pitch_i\":\"" + current_pid_pitch_i + "\",";
  json += "\"pid_pitch_d\":\"" + current_pid_pitch_d + "\",";
  json += "\"pid_roll_p\":\"" + current_pid_roll_p + "\",";
  json += "\"pid_roll_i\":\"" + current_pid_roll_i + "\",";
  json += "\"pid_roll_d\":\"" + current_pid_roll_d + "\",";
  json += "\"pid_yaw_p\":\"" + current_pid_yaw_p + "\",";
  json += "\"pid_yaw_i\":\"" + current_pid_yaw_i + "\",";
  json += "\"pid_yaw_d\":\"" + current_pid_yaw_d + "\",";
  json += "\"trim_m1\":\"" + current_trim[0] + "\",";
  json += "\"trim_m2\":\"" + current_trim[1] + "\",";
  json += "\"trim_m3\":\"" + current_trim[2] + "\",";
  json += "\"trim_m4\":\"" + current_trim[3] + "\",";
  json += "\"motors\":\"" + current_motors + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  // La riga di telemetria e' lunga (~260 caratteri): buffer RX piu' grande
  // del default (256) per non perdere byte mentre gira il web server.
  // Va chiamato PRIMA di Serial2.begin().
  Serial2.setRxBufferSize(1024);
  Serial2.begin(NUCLEO_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(20); // readStringUntil non deve bloccare il web server

  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress IP = WiFi.softAPIP();

  Serial.print("AP Pronto. Connettiti a IP: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/estop", HTTP_GET, handleEstop);
  server.on("/throttle", HTTP_GET, handleThrottle);
  server.on("/pid", HTTP_GET, handlePID);
  server.on("/pidyaw", HTTP_GET, handlePIDYaw);
  server.on("/trim", HTTP_GET, handleTrim);
  server.on("/dir", HTTP_GET, handleDir);
  server.on("/telemetry", HTTP_GET, handleTelemetry);

  server.begin();
}

void loop() {
  server.handleClient();

  // Watchdog di backup: se i motori risultano accesi da troppo tempo, spegni
  if (esp_motors_on &&
      (millis() - esp_motor_start_ms >= ESP_MOTOR_TIMEOUT_MS)) {
    sendKillToStm();
  }

  // Acquisizione asincrona della stringa telemetrica da STM32
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim();
    if (incoming.startsWith("ANG:")) {
      String payload = incoming.substring(4);
      int commaIndex1 = payload.indexOf(',');
      if (commaIndex1 > 0) {
        current_pitch = payload.substring(0, commaIndex1);
        int commaIndex2 = payload.indexOf(',', commaIndex1 + 1);
        if (commaIndex2 > 0) {
          current_roll = payload.substring(commaIndex1 + 1, commaIndex2);
        } else {
          current_roll = payload.substring(commaIndex1 + 1);
        }

        String value = telemetryField(payload, "yaw_rate");
        if (value.length() > 0)
          current_yaw_rate = value;

        value = telemetryField(payload, "pid_pitch_p");
        if (value.length() > 0)
          current_pid_pitch_p = value;
        value = telemetryField(payload, "pid_pitch_i");
        if (value.length() > 0)
          current_pid_pitch_i = value;
        value = telemetryField(payload, "pid_pitch_d");
        if (value.length() > 0)
          current_pid_pitch_d = value;
        value = telemetryField(payload, "pid_roll_p");
        if (value.length() > 0)
          current_pid_roll_p = value;
        value = telemetryField(payload, "pid_roll_i");
        if (value.length() > 0)
          current_pid_roll_i = value;
        value = telemetryField(payload, "pid_roll_d");
        if (value.length() > 0)
          current_pid_roll_d = value;
        value = telemetryField(payload, "pid_yaw_p");
        if (value.length() > 0)
          current_pid_yaw_p = value;
        value = telemetryField(payload, "pid_yaw_i");
        if (value.length() > 0)
          current_pid_yaw_i = value;
        value = telemetryField(payload, "pid_yaw_d");
        if (value.length() > 0)
          current_pid_yaw_d = value;

        const char *trimNames[4] = {"trim1", "trim2", "trim3", "trim4"};
        for (int i = 0; i < 4; i++) {
          value = telemetryField(payload, trimNames[i]);
          if (value.length() > 0)
            current_trim[i] = value;
        }

        value = telemetryField(payload, "motors");
        if (value.length() > 0) {
          current_motors = value;
          // Se l'STM32 dice che i motori sono spenti, allinea lo stato ESP32
          // (ignora telemetria vecchia nei primi 1.5 s dopo lo start)
          if (value == "0" && millis() - esp_motor_start_ms > 1500)
            esp_motors_on = false;
        }
      }
    }
  }
}
