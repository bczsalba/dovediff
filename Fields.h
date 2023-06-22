namespace Fields {

typedef struct {
    int trig;
    int echo;
} DepthSensor;

int maxDistance = 25;

DepthSensor makeSensor(int trig, int echo) {
    DepthSensor sensor = {trig, echo};

    pinMode(sensor.echo, INPUT);
    pinMode(sensor.trig, OUTPUT);
    return sensor;
}

long readSensor(DepthSensor sensor){
    digitalWrite(sensor.trig, LOW);
    delayMicroseconds(2);

    digitalWrite(sensor.trig, HIGH);
    delayMicroseconds(5);

    digitalWrite(sensor.trig, LOW);
    long duration = pulseIn(sensor.echo, HIGH);;

    return duration / 29 / 2;
}

}
