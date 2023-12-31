dovediff
-------

A relatively simple triple joint inverse kinematics solver for the Arduino.

## Usage

`dovediff` contains an example of use. In essence, you need to find your servos' approximate range of motion, and let the program know using a `ServoInfo` type.  After each servo for an arm is configured, you can create an `Arm` that combines them all. Make sure you know which pin is hooked up to which arm!

```cpp
#include "Arm.h"
using namespace Arms;

arm = {
    // Servo on pin 3 (base):
    //   - Turns 180 degrees when instructed for 175.0 (ratio: 175.0 / 180.0)
    //   - Has a 0 degree offset
    //   - Should be limited to only turn 90*ratio degrees
    //   - Is attached to an arm with 0 length
    ServoInfo{ makeServo(3), 175.0 / 180.0, 0, 90 }, 0,

    ServoInfo{ makeServo(5), 170.0 / 180.0, 0, 50 }, 11.7,
    ServoInfo{ makeServo(6), 250.0 / 180.0, 0, 180 }, 20.0,
};

void loop() {
    Position pos = {rand(0, 30), 0, rand(0, 30)};

    if (moveArm(arm, &pos)) {
        delay(600);        

    } else {
        Serial.print("Could not move to ");
        Serial.print(pos.x);
        Serial.print(";");
        Serial.print(pos.y);
        Serial.print(";");
        Serial.println(pos.z);
    }
}
```

The project also includes a simple utility to get a position from 2 proximity sensors, `Fields`:

```cpp
#include "Fields.h"
using namespace Fields;

Fields::DepthSensor xSensor = makeSensor(4, 11);
Fields::DepthSensor ySensor = makeSensor(8, 9);

void setup() {
    xSensor = makeSensor(4, 11);
    ySensor = makeSensor(8, 9);
}

void loop() {
    Serial.print("x: ");
    Serial.println(readSensor(xSensor));

    Serial.print("y: ");
    Serial.println(readSensor(ySensor));
}
```

## Limitations

This project was made on and for low-accuracy equipment in a very limited timespan. The math is only accurate enough for the purposes we needed it for, and your mileage may vary.  The biggest issue we had was that the Y axis was seemingly flipped. Since we could temporary fix this problem well enough for what we needed, we didn't (yet) figure out the problem.
