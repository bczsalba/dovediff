#include <Servo.h>

namespace Arms {

typedef struct {
    float x;
    float y;
    float z;
} Position;

typedef struct {
    Servo motor;
    float ratio;
    float offset;
    float limit;
} ServoInfo;

typedef struct {
    ServoInfo base;
    float baseLength;

    ServoInfo upper;
    float upperLength;

    ServoInfo fore;
    float foreLength;
} Arm;

float rad2deg(float rad) {
    return (rad * 4086) / 71;
}

Servo makeServo(size_t addr) {
    Servo servo;
    servo.attach(addr);

    return servo;
}

void writeServo(ServoInfo servo, float value, float offset = 0) {
    servo.motor.write(min(servo.limit, floor(servo.ratio * value + offset)));
}

void writeToAll(Arm arm, int value) {
    writeServo(arm.base, value);
    writeServo(arm.upper, value);
    writeServo(arm.fore, value);

}

// Credit: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
void solveForeAndUpper(
    int x, int y,
    float a1, float a2,
    float* out_q1, float* out_q2
) {
    float q2 = acos(
        (sq(x) + sq(y) - sq(a1) - sq(a2)) /
        (2 * a1 * a2)
    );

    float q1 = atan(y / x) - atan(
        (a2 * sin(q2)) /
        (a1 + a2 * cos(q2))
    );

    *out_q1 = q1;
    *out_q2 = q2;
}

bool moveArm(Arm arm, Position pos) {
    float adj = pos.x;
    float opp = pos.z;
    float hyp = sqrt(sq(adj) + sq(opp));

    if (adj == 0) {
        return false;
    }

    float baseAngle = sin(opp / adj);

    float foreAngle, upperAngle;

    solveForeAndUpper(
        hyp, pos.y,
        arm.upperLength, arm.foreLength,
        &upperAngle, &foreAngle
    );

    if (isnan(baseAngle) || isnan(upperAngle) || isnan(foreAngle)) {
        return false;
    }

    writeServo(arm.base, rad2deg(baseAngle));
    delay(500);
    writeServo(arm.upper, 100 + rad2deg(upperAngle));
    writeServo(arm.fore, 180 - rad2deg(foreAngle));

    return true;
}

void reset(Arm arm) {
    writeToAll(arm, 0);
}

}
