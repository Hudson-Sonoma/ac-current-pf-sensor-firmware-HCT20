#include <Arduino.h>
#include "unity_config_hw_test.hpp"

void unityOutputStart() {
    Serial.begin(115200);
}

void unityOutputChar(char c) {
    Serial.printf("%c", c);
}

void unityOutputFlush() {
    Serial.flush();
}

void unityOutputComplete() {}