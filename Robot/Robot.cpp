#include <Arduino.h>
#include <Servo.h>

#define PIN9  9
#define PIN10 10


//
// Class definitions
//

// forward definition of Robot class
class Robot;

// class to manage a button which sends a reset signal to the robot when pressed
class ResetButton {
private:
	// which pin the reset button is connected to
	int pin;

	// the current state (HIGH/LOW) of the button input (HIGH = unpressed, LOW = pressed)
	int status;

public:
	ResetButton(int pin);
	void Check(Robot* r);
};


// class to manage switches which represent touch sensors on the robot.
// these switches are connected to ground and use an internal pull-up resisistor.
class TouchSensor {
private:
	// which pin the reset button is connected to
	int pin;

	// the current state (HIGH/LOW) of the button input (HIGH = unpressed, LOW = pressed)
	int status;

	// the last time the sensor was activated
	unsigned long lastActivation;

public:
	TouchSensor(int pin);
	void Check(unsigned long time, Robot* r);
};


// class to control the steering mechanism of the robot
class SteeringControl {
private:
	// which pin the steering is attached to
	int pin;

	// ideally, 0 is full left, 90 is straight ahead and 180 is full right, but
	// it is typically a bit off.  This adjustment factor compensates for that
	int adjustment;
	
	// the servo class to drive the steering servo motor
	Servo servo;


public:

	SteeringControl(int pin);

	// turn the steering wheel to the given heading
	// 0 = straight ahead
	void SetDirection(int heading);
};


// class to represent a single DC motor controlled with an H-Bridge
// The DC Motor must be connected with PWM output pins
class DCMotor {
private:
	int pin0, pin1;

public:
	DCMotor(int p0, int p1) : pin0(p0), pin1(p1) {
		pinMode(pin0, OUTPUT);
		pinMode(pin1, OUTPUT);
	}

	void Drive(int level0, int level1) {
		analogWrite(pin0, level0);
		analogWrite(pin1, level1);
	}
};

// class to control the drive wheels of the robot
class DriveControl {
private:
	DCMotor left;
	DCMotor right;

public:
	DriveControl() : left(PIN9,PIN10), right(PIN5,PIN6) {}

	// stop both wheels
	void Stop();

	// drive both wheels forward at the given rate
	void Forward(int rate);

	// drive both wheels in reverse at the given rate
	void Reverse(int rate);

	// execute a left turn by turning the left wheel forward at rate,
	// and holding the right wheel still
	void Left(int rate);

	// execute a right turn by turning the right wheel forward at rate,
	// and holding the left wheel still
	void Right(int rate);

	// pivot the robot left by turning the left wheel forward full
	// and the right wheel reverse full
	void PivotLeft();

	// pivot the robot right by turning the right wheel forward full
	// and the left wheel reverse full
	void PivotRight();
};


// class to synchronize the control of the steering and drive systems to
// perform complex maneuvers

#define HELM_IDLE        0
#define HELM_FORWARD     1
#define HELM_REVERSE     2
#define HELM_LEFT        3
#define HELM_RIGHT       4
#define HELM_PIVOT_LEFT  5
#define HELM_PIVOT_RIGHT 6

class Helmsman {
private:
	Robot&           robot;
	SteeringControl& steering;
	DriveControl&    drive;
	int              state;
	unsigned long    startTime;
	unsigned long	 maneuverTime;

public:
	Helmsman(Robot& robot, SteeringControl& steering, DriveControl& drive) :
		robot(robot), steering(steering), drive(drive), state(HELM_IDLE), startTime(0) {}

	// halt whatever maneuver is being performed
	void Stop();

	// perform a left turn
	void TurnLeft(unsigned long currentTime, unsigned long maneuverTime);

	// peform a right turn
	void TurnRight(unsigned long currentTime, unsigned long maneuverTime);

	// perform a left pivot
	void PivotLeft(unsigned long currentTime, unsigned long maneuverTime);

	// peform a right pivot
	void PivotRight(unsigned long currentTime, unsigned long maneuverTime);

	// move forward for a bit
	void MoveForward(unsigned long currentTime, unsigned long maneuverTime);

	// move in reverse for a bit
	void MoveReverse(unsigned long currentTime, unsigned long maneuverTime);

	// update state
	void Tick(unsigned long time);
};


#define LED_OFF 0
#define LED_ON  1

#define LED_FLASH_SLOW  2
#define LED_FLASH_FAST  3
#define LED_SHORT_ON    4

// driver for LED to show state
class StatusLED {
private:
	int pin;
	int offTime, onTime;
	unsigned long lastUpdateTime;
	int state, setting;

public:
	StatusLED(int pin) : pin(pin), offTime(0), onTime(0), lastUpdateTime(0), state(LED_OFF), setting(LED_OFF) {
		pinMode(this->pin, OUTPUT);
		digitalWrite(this->pin, LOW);
	}

	void Tick(unsigned long time);
	void Set(int setting);
};

void StatusLED::Set(int setting) {
	this->setting = setting;
	this->lastUpdateTime = 0;
	
	switch (setting) {
	case LED_OFF:
		digitalWrite(this->pin, LOW);
		this->state = LED_OFF;
		break;

	case LED_ON:
	case LED_SHORT_ON:
		digitalWrite(this->pin, HIGH);
		this->state = LED_ON;
		this->lastUpdateTime = micros();
		this->onTime = 400;
		break;

	case LED_FLASH_SLOW:
		digitalWrite(this->pin, HIGH);
		this->state = LED_ON;
		this->lastUpdateTime = micros();
		this->onTime = 600;
		this->offTime = 200;
		break;

	case LED_FLASH_FAST:
		digitalWrite(this->pin, HIGH);
		this->state = LED_ON;
		this->lastUpdateTime = micros();
		this->onTime = 100;
		this->offTime = 100;
		break;
	}
}

void StatusLED::Tick(unsigned long time) {
	switch (setting) {
	case LED_SHORT_ON:
		if (this->state == LED_ON) {
			if (time > this->lastUpdateTime + ((unsigned long)this->onTime * 1000L)) {
				digitalWrite(this->pin, LOW);
				this->state = LED_OFF;
			}
		}
		break;

	case LED_FLASH_SLOW:
	case LED_FLASH_FAST:
		if (this->state == LED_OFF) {
			if (time > this->lastUpdateTime + ((unsigned long)this->offTime * 1000L)) {
				digitalWrite(this->pin, HIGH);
				this->state = LED_ON;
				this->lastUpdateTime = micros();
			}
		} else {
			if (time > this->lastUpdateTime + ((unsigned long)this->onTime * 1000L)) {
				digitalWrite(this->pin, LOW);
				this->state = LED_OFF;
				this->lastUpdateTime = micros();
			}
		}
		break;
	}
}


//
// Robot
//

// PIN assignments
#define RESET_BUTTON_PIN    A2
#define STEERING_PIN        3
#define LED_PIN             13

// Robot States
#define STATE_INIT			0		// Initial state. do nothing, wait for a reset
#define STATE_RESET			1		// Performing a reset. This is the state held while reset button is down
#define STATE_RUN			2		// Normal run state
#define STATE_BACKWARD      3
#define STATE_IDLE			4
#define STATE_FWD_LEFT		5
#define STATE_FWD_RIGHT		6
#define STATE_LEFT			7
#define STATE_RIGHT			8
#define STATE_BACK_LEFT		9
#define STATE_BACK_RIGHT	10
#define STATE_FORWARD       11


// Robot class is implemented as a state machine
class Robot {
private:
	ResetButton		resetButton;
	SteeringControl	steering;
	DriveControl    drive;
	Helmsman        helm;
	StatusLED       led;
	StatusLED       bumperLED;
	TouchSensor     leftBumperSensor;
	TouchSensor     rightBumperSensor;
	TouchSensor     leftTouchSensor;
	TouchSensor     rightTouchSensor;

	int				state;

	unsigned long   currentTime;

public:
	Robot();

	// enter the reset state
	void Reset();

	// enter the Run state
	void Run();

	// called by the Helmsman to signal that the last maneuver is complete
	void HelmFinished(int move);

	// called by touch sensors to indicate that they have been activated
	void SensorActivated(TouchSensor* which);

	// called repeatedly in the loop()
	void Tick(unsigned long time);

private:
	// internal state methods
	void doReset();
	void doRun();
	void forward(unsigned long time) ;
	void backward(unsigned long time) ;
	void turnLeft() ;
	void turnRight() ;
};


//
// ResetButton methods
//

ResetButton::ResetButton(int pin) : pin(pin), status(HIGH) {
	pinMode(this->pin, INPUT_PULLUP);
}

// check for the status of the reset button
void ResetButton::Check(Robot *robot) {
	int buttonStatus;

	buttonStatus = digitalRead(this->pin);

	// TODO: we could debounce here by wating some time for the button to settle down
	// before firing the Reset signal to the robot
	
	if (buttonStatus != this->status) {
		// status has changed
		this->status = buttonStatus;

		if (this->status == HIGH) {
			// button has been released. tell the robot to enter the
			// run state
			robot->Run();
		} else {
			// button is being pressed.  tell the robot to enter the
			// reset state
			robot->Reset();
		}
	}
}


//
// TouchSensor methods
//

TouchSensor::TouchSensor(int pin) : pin(pin), status(HIGH), lastActivation(0) {
	pinMode(this->pin, INPUT_PULLUP);
}

// check for the status of the sensor
void TouchSensor::Check(unsigned long time, Robot *robot) {
	int buttonStatus;

	buttonStatus = digitalRead(this->pin);

	// TODO: we could debounce here by wating some time for the button to settle down
	// before firing the Reset signal to the robot
	
	if (buttonStatus != this->status) {
		// status has changed
		this->status = buttonStatus;

		if (this->status == LOW) {
			// sensor is being pressed. see if we signaled the robot recently (in the last 10ms)
			// also the check for time < lastActivation takes care of clock wrap-around which 
			// happens every 70 hours on the arduino
			if (time - this->lastActivation > 10000 || time < this->lastActivation) {
				// signal the robot
				robot->SensorActivated(this);
				this->lastActivation = time;
			}
		}
	}
}


//
// SteeringControl methods
//

// TODO: dont hardcode adjustment. perhaps a calibration system
//    but that would need more buttons and be complicated

SteeringControl::SteeringControl(int pin) : 
	pin(pin),
	adjustment(10)
{
	this->servo.attach(this->pin);
}


void SteeringControl::SetDirection(int heading) {
	this->servo.write(heading + 90 + this->adjustment);
}


//
// DriveControl methods
//

// stop both wheels
void DriveControl::Stop() {
	this->left.Drive(0, 0);
	this->right.Drive(0, 0);
}


// drive both wheels forward at the given rate
void DriveControl::Forward(int rate) {
	this->left.Drive(rate, 0);
	this->right.Drive(rate, 0);
}

// drive both wheels in reverse at the given rate
void DriveControl::Reverse(int rate) {
	this->left.Drive(0, rate);
	this->right.Drive(0, rate);
}

// execute a left turn by turning the left wheel forward at rate,
// and holding the right wheel still
void DriveControl::Left(int rate) {
	this->left.Drive(rate, 0);
	this->right.Drive(0, 0);
}

// execute a right turn by turning the right wheel forward at rate,
// and holding the left wheel still
void DriveControl::Right(int rate) {
	this->left.Drive(0, 0);
	this->right.Drive(rate, 0);
}

// pivot the robot left by turning the left wheel forward full
// and the right wheel reverse full
void DriveControl::PivotLeft() {
	this->left.Drive(255, 0);
	this->right.Drive(0, 255);
}

// pivot the robot right by turning the right wheel forward full
// and the left wheel reverse full
void DriveControl::PivotRight() {
	this->left.Drive(0, 255);
	this->right.Drive(255, 0);
}


// 
// Helmsman methods
//

// stop the robot
void Helmsman::Stop() {
	this->state = HELM_IDLE;
	this->startTime = 0;
	this->maneuverTime = 0;
	this->steering.SetDirection(0);
	this->drive.Stop();
}

// perform a left turn
void Helmsman::TurnLeft(unsigned long currentTime, unsigned long maneuverTime) {
	this->state = HELM_LEFT;
	this->startTime = currentTime;
	this->maneuverTime = maneuverTime * 1000;
	this->steering.SetDirection(-60);
	this->drive.Left(255);
}

// peform a right turn
void Helmsman::TurnRight(unsigned long currentTime, unsigned long maneuverTime) {
	this->state = HELM_RIGHT;
	this->startTime = currentTime;
	this->maneuverTime = maneuverTime * 1000;
	this->steering.SetDirection(60);
	this->drive.Right(255);
}

// perform a left pivot
void Helmsman::PivotLeft(unsigned long currentTime, unsigned long maneuverTime) {
	this->state = HELM_PIVOT_LEFT;
	this->startTime = currentTime;
	this->maneuverTime = maneuverTime * 1000;
	this->steering.SetDirection(-90);
	this->drive.PivotLeft();
}

// peform a right turn
void Helmsman::PivotRight(unsigned long currentTime, unsigned long maneuverTime) {
	this->state = HELM_PIVOT_RIGHT;
	this->startTime = currentTime;
	this->maneuverTime = maneuverTime * 1000;
	this->steering.SetDirection(90);
	this->drive.PivotRight();
}

// move forward for a time
void Helmsman::MoveForward(unsigned long currentTime, unsigned long time) {
	this->state = HELM_FORWARD;
	this->startTime = currentTime;
	this->maneuverTime = time * 1000;
	this->steering.SetDirection(0);
	this->drive.Forward(255);
}

// move in reverse for a time
void Helmsman::MoveReverse(unsigned long currentTime, unsigned long time) {
	char buf[120];

	this->state = HELM_REVERSE;
	this->startTime = currentTime;
	this->maneuverTime = time * 1000;
	this->steering.SetDirection(0);
	this->drive.Reverse(255);

	sprintf(buf, "Starting REVERSE at %ld, maneuverTime = %ld\n", startTime, maneuverTime);
	Serial.println(buf);
}

// update state
void Helmsman::Tick(unsigned long time) {
	char buf[120];
	int move = this->state;

	switch (this->state) {
	case HELM_IDLE:
		break;

	case HELM_FORWARD:
	case HELM_REVERSE:
	case HELM_LEFT:
	case HELM_RIGHT:
	case HELM_PIVOT_LEFT:
	case HELM_PIVOT_RIGHT:
		if (time - this->startTime >= this->maneuverTime) {
			sprintf(buf, "Maneuver %d done. Time = %ld, start = %ld, maneuverTime = %ld\n", move, time, startTime, maneuverTime);
			Serial.println(buf);

			this->state = HELM_IDLE;
			this->robot.HelmFinished(move);
		}
		break;
	}
}


//
// Robot methods
//

Robot::Robot() :
	resetButton(RESET_BUTTON_PIN),
	steering(STEERING_PIN),
	drive(),
	helm(*this, steering, drive),
	led(LED_PIN),
	bumperLED(7),
	leftBumperSensor(A5),
	rightBumperSensor(A4),
	leftTouchSensor(A0),
	rightTouchSensor(A1),
	state(STATE_INIT),
	currentTime(0)
{
	this->led.Set(LED_FLASH_SLOW);
	this->bumperLED.Set(LED_OFF);
}

const static char* ResetStateMessage = "RESET";
const static char* RunStateMessage = "RUN";
const static char* BackwardMessage = "BACKWARD";
const static char* ForwardMessage = "FORWARD";
const static char* TurnLeftMessage = "TURN LEFT";
const static char* TurnRightMessage = "TURN RIGHT";

// enter the reset state
void Robot::Reset() {
	Serial.println(ResetStateMessage);
	this->led.Set(LED_OFF);
	this->state = STATE_RESET;
}

// enter the Run state
void Robot::Run() {
	Serial.println(RunStateMessage);
	this->led.Set(LED_OFF);
	this->state = STATE_RUN;
}

// receive a notification from a sensor
void Robot::SensorActivated(TouchSensor* which) {
	// print a message indicating which touch sensor was activated
	Serial.print("sensor activated: #");
	Serial.println((long)(void *)which);

	// set the bumper LED on for a short time
	this->bumperLED.Set(LED_SHORT_ON);

	if (which == &leftBumperSensor) {
		// left REAR bumper sensor activated... 
		this->forward(800);
		this->state = STATE_FWD_RIGHT;
	} else if (which == &rightBumperSensor) {
		this->forward(800);
		this->state = STATE_FWD_LEFT;
	} else if (which == &leftTouchSensor) {
		this->backward(800);
		this->state = STATE_BACK_RIGHT;
	} else if (which == &rightTouchSensor) {
		this->backward(800);
		this->state = STATE_BACK_LEFT;
	}
}

// perform the operations in the Reset state
void Robot::doReset() {
	this->steering.SetDirection(0);
	this->led.Set(LED_OFF);
}

// perform the operations in the Run state
void Robot::doRun() {
	// start by driving forward for a while
	this->forward(15000);
}

void Robot::backward(unsigned long time) {
	Serial.println(BackwardMessage);
	this->state = STATE_BACKWARD;
	this->helm.MoveReverse(currentTime, time);
	this->led.Set(LED_ON);
}

void Robot::forward(unsigned long time) {
	Serial.println(ForwardMessage);
	this->state = STATE_FORWARD;
	this->helm.MoveForward(currentTime, time);
	this->led.Set(LED_ON);
}

void Robot::turnLeft() {
	Serial.println(TurnLeftMessage);
	this->state = STATE_LEFT;
	this->helm.TurnLeft(currentTime, 800);
	this->led.Set(LED_FLASH_FAST);
}

void Robot::turnRight() {
	Serial.println(TurnRightMessage);
	this->state = STATE_RIGHT;
	this->helm.TurnRight(currentTime, 800);
	this->led.Set(LED_FLASH_FAST);
}

// called by the Helmsman when a maneuver is complete
void Robot::HelmFinished(int move) {
	Serial.print("HelmFinished state=");
	Serial.print(this->state);
	Serial.print(", move=");
	Serial.println(move);

	switch (this->state) {
	case STATE_FORWARD:
		// forward movement done.. sit idle
		this->helm.Stop();
		this->state = STATE_IDLE;
		break;

	case STATE_BACKWARD:
		// backward movement done.. sit idle
		this->helm.Stop();
		this->state = STATE_IDLE;
		break;

	case STATE_FWD_LEFT:
		// forward done.. now turn left
		this->turnLeft();
		break;

	case STATE_FWD_RIGHT:
		// forward done.. now turn right
		this->turnRight();
		break;
		
	case STATE_BACK_LEFT:
		// back done.. now turn left
		this->turnLeft();
		break;

	case STATE_BACK_RIGHT:
		// forward done.. now turn right
		this->turnRight();
		break;
		
	case STATE_LEFT:
	case STATE_RIGHT:
		this->forward(15000);
		break;
	}
}

// perform the main state machine operation.
void Robot::Tick(unsigned long time) {
	this->currentTime = time;

	// check inputs which may change state
	this->resetButton.Check(this);
	this->leftBumperSensor.Check(time, this);
	this->rightBumperSensor.Check(time, this);
	this->leftTouchSensor.Check(time, this);
	this->rightTouchSensor.Check(time, this);

	// let the helm update its state
	this->helm.Tick(time);

	// let the status LED update its state
	this->led.Tick(time);
	this->bumperLED.Tick(time);

	// execute operations for the current state
	switch (this->state) {
	case STATE_INIT:
		// do nothing in this state
		break;

	case STATE_RESET:
		// keep the robot reset
		this->doReset();
		break;

	case STATE_RUN:
		this->doRun();
		break;
	}
}

// the singleton robot object.  allocated in setup()
Robot *robot = NULL;


//
// Main methods
//

void setup() {
	Serial.begin(9600);
	robot = new Robot();
}

void loop() {
	robot->Tick(micros());
}
