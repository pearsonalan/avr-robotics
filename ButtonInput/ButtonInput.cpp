#include <Arduino.h>
#include <Servo.h>

Servo servo;

void setup() {
	char buf[80];

	servo.attach(3);
	pinMode(4, INPUT_PULLUP);
	pinMode(13, OUTPUT);
	Serial.begin(9600);

	servo.write(100);

	sprintf(buf, "Size of int is %d, short is %d, long is %d, ptr is %d",
		sizeof(int), sizeof(short), sizeof(long), sizeof(void*));
	Serial.println(buf);
}

void loop() {
	static long count = 0;
	static int buttonState = HIGH;
	static int up = 0, down = 0;
	char buf[80];
	int s;

	s = digitalRead(4);
	if (s != buttonState) {
		buttonState = s;
		if (buttonState == HIGH) {
			servo.write(55);
			up++;
		} else {
			servo.write(145);
			down++;
		}
	}

	digitalWrite(13, buttonState ? HIGH : LOW);

	if (++count == 50000) {
		sprintf(buf, "Button = %s. Up = %d, Down = %d",
			buttonState ? "HIGH" : "LOW", up, down);
		Serial.println(buf);
		count = 0;
	}
}
