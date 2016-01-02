#include <Arduino.h>

class DCMotor {
private:
	int pin0, pin1;

public:
	DCMotor(int p0, int p1) : pin0(p0), pin1(p1) {
		pinMode(pin0, OUTPUT);
		pinMode(pin1, OUTPUT);
	}

	void drive(int level0, int level1) {
		analogWrite(pin0, level0);
		analogWrite(pin1, level1);
	}
};

DCMotor *m1 = NULL, *m2 = NULL;

void setup() {
	m1 = new DCMotor(5,6);
	m2 = new DCMotor(9,10);
	
	m1->drive(0, 255);
	m2->drive(0, 255);
	
	delay(250);
}

void loop() {
	m1->drive(0, 255);
	m2->drive(0, 255);
	delay(1000);

	m1->drive(0, 0);
	m2->drive(0, 0);
	delay(400);

	m1->drive(0, 255);
	m2->drive(255, 0);
	delay(1000);

	m1->drive(0, 0);
	m2->drive(0, 0);
	delay(400);
	
	m1->drive(255, 0);
	m2->drive(255, 0);
	delay(1000);
	
	m1->drive(0, 0);
	m2->drive(0, 0);
	delay(400);
}
