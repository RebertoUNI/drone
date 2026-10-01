#include "interfacciaWeb.h"
#include <WebServer.h>
#include <WiFi.h>

const char *AP_SSID = "Drone_Test";
const char *AP_PASS = "DaniGay7";

#define RXD2 16
#define TXD2 17
#define NUCLEO_BAUD 115200

WebServer server(80);

// Variabili globali per memorizzare telemetria
String current_pitch = "0.00";
String current_roll = "0.00";
String current_pid_pitch_p = "1.00";
String current_pid_pitch_i = "0.00";
String current_pid_pitch_d = "0.00";
String current_pid_roll_p = "1.00";
String current_pid_roll_i = "0.00";
String current_pid_roll_d = "0.00";
String current_takeoff_duration = "5000";
String current_takeoff_gain[4] = {"1.000", "1.000", "1.000", "1.000"};

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

void handleRoot() { server.send(200, "text/html", index_html); }

void handleCommand() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    Serial2.print("CMD:" + action + "\n");
  }
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

void handleTakeoff() {
  if (server.hasArg("duration") && server.hasArg("g1") &&
      server.hasArg("g2") && server.hasArg("g3") && server.hasArg("g4")) {
    String duration = server.arg("duration");
    String g1 = server.arg("g1");
    String g2 = server.arg("g2");
    String g3 = server.arg("g3");
    String g4 = server.arg("g4");
    Serial2.printf("TAKEOFF:%s,%s,%s,%s,%s\n", duration.c_str(), g1.c_str(),
                   g2.c_str(), g3.c_str(), g4.c_str());
  }
  server.send(200, "text/plain", "OK");
}

// Endpoint JSON con angoli e parametri PID e compensazione allo stacco.
void handleTelemetry() {
  String json =
      "{\"pitch\":\"" + current_pitch + "\", \"roll\":\"" + current_roll +
      "\", \"pid_pitch_p\":\"" + current_pid_pitch_p +
      "\", \"pid_pitch_i\":\"" + current_pid_pitch_i +
      "\", \"pid_pitch_d\":\"" + current_pid_pitch_d + "\", \"pid_roll_p\":\"" +
      current_pid_roll_p + "\", \"pid_roll_i\":\"" + current_pid_roll_i +
      "\", \"pid_roll_d\":\"" + current_pid_roll_d +
      "\", \"takeoff_duration\":\"" + current_takeoff_duration +
      "\", \"takeoff_g1\":\"" + current_takeoff_gain[0] +
      "\", \"takeoff_g2\":\"" + current_takeoff_gain[1] +
      "\", \"takeoff_g3\":\"" + current_takeoff_gain[2] +
      "\", \"takeoff_g4\":\"" + current_takeoff_gain[3] + "\"}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(NUCLEO_BAUD, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress IP = WiFi.softAPIP();

  Serial.print("AP Pronto. Connettiti a IP: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/throttle", HTTP_GET, handleThrottle);
  server.on("/pid", HTTP_GET, handlePID);
  server.on("/takeoff", HTTP_GET, handleTakeoff);
  server.on("/telemetry", HTTP_GET, handleTelemetry);

  server.begin();
}

void loop() {
  server.handleClient();

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

        String value = telemetryField(payload, "pid_pitch_p");
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
        value = telemetryField(payload, "takeoff_duration");
        if (value.length() > 0)
          current_takeoff_duration = value;
        value = telemetryField(payload, "takeoff_g1");
        if (value.length() > 0)
          current_takeoff_gain[0] = value;
        value = telemetryField(payload, "takeoff_g2");
        if (value.length() > 0)
          current_takeoff_gain[1] = value;
        value = telemetryField(payload, "takeoff_g3");
        if (value.length() > 0)
          current_takeoff_gain[2] = value;
        value = telemetryField(payload, "takeoff_g4");
        if (value.length() > 0)
          current_takeoff_gain[3] = value;
      }
    }
  }
}