#include <Arduino.h>
#include "LEDshift.h"

LEDshift matrix(17500000L, 2, 8, 0, false);

void setup() {
  Serial.begin(115200);

  matrix.Begin(LS_10bitsDeep);
  // matrix.setClock(10000000L);
  int32_t time2 = matrix.SyncBuffers();
  Serial.printf("Tiempo de sincronizado: %i us\n", time2);
  //matrix.PrintBuffer();

  // matrix.setOutput(0, 1.0);
  matrix.setOutput(8, 1.0);
}


uint32_t last = millis(), time = millis();
float val = 0, inc = 0.01;

void loop() {
  matrix.loop();
  static char cmd[64] = {};
  static int index = 0;

  while (Serial.available() > 0) {
    int32_t out = 0, count;
    float val = 0;
    char c = Serial.read();
    if (c == '\n') {
      if (strcmp(cmd, "out") == 0) {
        Serial.println("Mostrando valores de salida");
        for (int out = 0; out < 16; out++) {
          Serial.printf(" Salida %d: %0.3f\n", out, matrix.getOutput(out));
        }
      }
      else if (strcmp(cmd, "print") == 0) {
        Serial.println("Mostrando buffer de salida");
        matrix.PrintBuffer();
      }
      else if (strcmp(cmd, "clear") == 0) {
        Serial.println("Poniendo salidas a 0");
        for (int x = 0; x < 16; x++)
          matrix.setOutput(x, 0.0);
      }
      else if (strcmp(cmd, "fill") == 0) {
        Serial.println("Poniendo salidas a 100%");
        for (int x = 0; x < 16; x++)
          matrix.setOutput(x, 1.0);
      }
      else if (strncmp(cmd, "fill", 4) == 0) {
        count = sscanf(cmd, "fill %f", &val);
        if (count == 1) {
          Serial.printf("Poniendo salidas a %0.3f", val);
          for (int x = 0; x < 16; x++)
            matrix.setOutput(x, val);
        }
      }
      else if (strcmp(cmd, "sync") == 0) {
        uint32_t time = matrix.SyncBuffers();
        Serial.printf("Tiempo de sincronizacion: %i us\n", time);
      }
      else {
        count = sscanf(cmd, "%d = %f", &out, &val);
        if (count == 2) {
          Serial.printf("Set salida %d: %0.3f\n", out, val);
          matrix.setOutput(out, val);
        }
        else if (count == 1)
          Serial.printf("Salida %d: %0.3f\n", out, matrix.getOutput(out));
        else
          Serial.printf("Error! %s\n", cmd);
        index = 0;
      }
      memset(cmd, 0, sizeof(cmd));

    }
    else
      cmd[index++] = c;
  }

  if (millis() - time > 10) {
    time = millis();

    matrix.setOutput(0, val);
    if (val >= 1.0)
      inc = -0.01;
    else if (val <= 0)
      inc = 0.01;
    val += inc;
  }

}
