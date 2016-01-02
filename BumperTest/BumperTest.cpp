#include <Arduino.h>

int pin0 = A0;
int pin1 = A1;

#define LEDPIN 7

void setup() {
	pinMode(pin0, INPUT_PULLUP);
	pinMode(pin1, INPUT_PULLUP);
	Serial.begin(9600);

	pinMode(LEDPIN, OUTPUT);
}

void showPin(int pin, int currentState, int prevState) {
	char buf[80];

	sprintf(buf, "PIN %d: %s => %s", pin, 
		prevState == HIGH ? "HIGH" : "LOW",
		currentState == HIGH ? "HIGH" : "LOW");
#if 0
	sprintf(buf, "PIN %d: %d => %d", pin, 
		prevState, currentState);
#endif

	Serial.println(buf);
}

void loop() {
	static int pin0state = HIGH;
	static int pin1state = HIGH;
	int p0, p1;

#if 0
	p0 = analogRead(pin0);
	p1 = analogRead(pin1);

	p0 = (p0 < 256) ? LOW : HIGH;
	p1 = (p1 < 256) ? LOW : HIGH;
#else
	p0 = digitalRead(pin0);
	p1 = digitalRead(pin1);
#endif

	if (p0 != pin0state) {
		showPin(pin0, p0, pin0state);
		pin0state = p0;
	}

	if (p1 != pin1state) {
		showPin(pin1, p1, pin1state);
		pin1state = p1;
	}

	digitalWrite(LEDPIN, !pin0state | !pin1state);
}
