#include "Arm.h"
#include "Fields.h"

using namespace Arms;

#define ENABLE_RESET 1
#define ARM_INDEX 0

Arm arm;

Fields::DepthSensor xSensor;
Fields::DepthSensor zSensor;

Position lastPosition;

#ifdef ENABLE_RESET
bool isReset;
int resetLight = 12;
int resetButton = 13;
#endif


void setup() {
    Serial.begin(9600);

    pinMode(resetButton, INPUT);
    pinMode(resetLight, OUTPUT);

    // Left arm
    if (ARM_INDEX == 0) {
        arm = {
            ServoInfo{ makeServo(3), 175.0 / 180.0, 0, 90 }, 0,
            ServoInfo{ makeServo(5), 170.0 / 180.0, 0, 50 }, 11.7,
            ServoInfo{ makeServo(6), 250.0 / 180.0, 0, 180 }, 20.0,
        };

    // Right arm
    } else {
        arm = {
            ServoInfo{ makeServo(3), 1.0, 0, 180 }, 0,
            ServoInfo{ makeServo(5), 1.0, 0, 130 }, 11.7,
            ServoInfo{ makeServo(6), 1.0, 0, 180 }, 20.0,
        };

    }

    xSensor = Fields::makeSensor(4, 11);
    zSensor = Fields::makeSensor(8, 9);

    toggleReset();
}

void toggleReset() {
    Serial.println("Reset!");
    isReset = !isReset;
    digitalWrite(resetLight, isReset ? HIGH : LOW);

    if (isReset) {
        reset(arm);
    }
}

bool findPosition(Position* pos) {
    long xValue = Fields::readSensor(xSensor);
    long zValue = Fields::readSensor(zSensor);

    // Ignore invalid values
    if (xValue > 20.0 || zValue > 20.0) {
        return false;
    }

    pos->x = xValue;
    pos->z = zValue;

    return true;
}

void debugServo(ServoInfo servo) {
    float angle = map(Fields::readSensor(xSensor), 0.0, 40.0, 0.0, 180.0);
    Arms::writeServo(arm.upper, angle);

    Serial.println(angle);
}

void loop() {

#ifdef ENABLE_RESET
    if (digitalRead(resetButton) == HIGH) {
        toggleReset();
        delay(500);
    }

    if (isReset) {
        return;
    }
#endif

    Position pos = {0, 0, 0};

    if (findPosition(&pos)) {
        if (random(3) == 1) {
            pos.y = 20;
        };

        Serial.print(pos.x);
        Serial.print(";");
        Serial.print(pos.y);
        Serial.print(";");
        Serial.println(pos.z);

        if (!moveArm(arm, pos, 0)) {
            Serial.println("Fail");
        }

        lastPosition = pos;

    } else {
        reset(arm);
    }

    delay(600);
}
