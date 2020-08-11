#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>
#include <i2cEncoderLibV2.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <AccelStepper.h>


#define EN_PIN    9  // Nano v3:	16 Mega:	38	//enable (CFG6)
#define DIR_PIN   12  //			19			55	//direction
#define STEP_PIN  11  //			18			54	//step
#define CS_PIN    10  //			17			64	//chip select

#define STALL_VALUE 12 // [-64..63]

bool dir = true;

bool vsense;

uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

uint32_t MOTOR_STEPS = 200;
uint32_t MICROSTEPS = 32;
uint32_t PULLEY_TEETH = 20;
float BELT_PITCH = 2.0;
float steps_mm = (MOTOR_STEPS * MICROSTEPS)/(PULLEY_TEETH * BELT_PITCH);

float distance = 0.0;
float startPosition = 100.0;
float seekSpeed = 70.0;
float maxDistance = 800.0;

float endPosition = 200.0 + startPosition;
float desiredPosition = 0.0;
float desiredTime = 30.0;      //Time in Minutes
float accDistance = 0.0;

bool homed = false;
bool seeked = false;
bool ended = false;

unsigned int last_time = 0;
unsigned int last_bounce = 0;
unsigned int lastQuery = 0;
int queryTime = 20000;

int currentStatus = 100;
int stallThreshold = 30;
int stallCounter = 20;

// float maxSpeed = endPosition / (desiredTime * 60.0);

float maxSpeed = 300.0;         // in mm/s
float acceleration = 50.0;   // in mm/s^2

float reqSpeed = maxSpeed;
float speed = 0.0;
float zeroThresh = 0.1;

float timebase = 1000000.0;

bool high = false;

int state = 3;          // 0 - none  1 - seeking  2 - running  3 - menu  4 - paused
int menuState = 0;      // 0 - status  1 - startPos  2 - endPos  3 - duration  4 - start  5 - cancel/stop
int lastMenuState = 0;
String menuLabels[] = {"HOME", "SPOS", "EPOS", "TIME", " GO ", "STOP", "SPED", "DIST", "STEPS"};
bool inProp = false;
bool firstInProp = true;
bool mult = false;
int multNum = 5;

int stateMin = 0;
int stateMax = 0;

int menuSizes[] = {8};
float minVals[] = {0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,2};
float maxVals[] = {1.0,maxDistance,maxDistance,999.9,1.0,1.0,50.0,300.0,256.0};

float val = 0;
float lastVal = 0;


TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
i2cEncoderLibV2 Encoder(0x11); /* A0 and A4 are soldered */
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();



void setup() {
	Serial.begin(9600);

    // uint32_t tot_steps = MOTOR_STEPS * MICROSTEPS;
    // int pulley_len = PULLEY_TEETH * BELT_PITCH;
    // steps_mm = tot_steps/pulley_len;

	// while(!Serial);
	Serial.println("Start...");
	SPI.begin();
    Wire.begin();
	pinMode(MISO, INPUT_PULLUP);

    Encoder.reset();
    Encoder.begin(
    i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE
    | i2cEncoderLibV2::DIRE_RIGHT | i2cEncoderLibV2::IPUP_ENABLE
    | i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);

    Encoder.writeCounter((int32_t) 0); /* Reset the counter value */
    Encoder.writeMax((int32_t) 5); /* Set the maximum threshold*/
    Encoder.writeMin((int32_t) 0); /* Set the minimum threshold */
    Encoder.writeStep((int32_t) 1); /* Set the step to 1*/
    Encoder.writeInterruptConfig(0xff); /* Enable all the interrupt */
    Encoder.writeAntibouncingPeriod(10); /* Set an anti-bouncing of 200ms */
    Encoder.writeDoublePushPeriod(50); /*Set a period for the double push of 500ms */


    alpha4.begin(0x70);  // pass in the address
    alpha4.clear();
    alpha4.writeDisplay();



	driver.begin(); 			// Initiate pins and registeries
	driver.rms_current(900); 	// Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
	driver.stealthChop(1); 	// Enable extremely quiet stepping

    driver.push();
    driver.toff(3);
    driver.tbl(1);
    driver.hysteresis_start(4);
    driver.hysteresis_end(-2);
    driver.diag1_stall(1);
    driver.diag1_active_high(1);
    driver.coolstep_min_speed(0xFFFFF); // 20bit max
    driver.THIGH(0);
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.sg_stall_value(STALL_VALUE);
    // driver.microsteps(MICROSTEPS);
    setMicrosteps(seekSpeed);

    //TMC2130 outputs on (LOW active)
    digitalWrite(EN_PIN, LOW);

    stepper.setMaxSpeed(maxSpeed * steps_mm);
    stepper.setAcceleration(acceleration * steps_mm);
}

void loop() {
    unsigned long ms = micros();
    float deltaTime = ms - last_time;
    last_time = ms;

    if (deltaTime < 0) {
        deltaTime = 0;
    }

    if (Encoder.updateStatus()) {
        if (Encoder.readStatus(i2cEncoderLibV2::PUSHP)) {
            Serial.println("Pressed");
            inProp = !inProp;
            firstInProp = true;
        }
        if (Encoder.readStatus(i2cEncoderLibV2::PUSHD)) {
            mult = !mult;
            if (mult) {
                Encoder.writeStep((int32_t) multNum);
            }
            else {
                Encoder.writeStep((int32_t) 1);
            }
        }
    }
    // Second menu layer
    if (inProp) {
        if (firstInProp) {
            int input = 0;
            float set_Speed = seekSpeed * steps_mm;
            switch (menuState) {
                case 0:
                    input = 0;
                    home();
                    // inProp = !inProp;
                    state = 0;
                    break;
                case 1:
                    input = startPosition;
                    desiredPosition = input*steps_mm;
                    stepper.moveTo(desiredPosition);
                    stepper.setMaxSpeed(set_Speed);
                    homed = false;
                    state = 1;
                    break;
                case 2:
                    input = endPosition;
                    desiredPosition = input*steps_mm;
                    stepper.moveTo(desiredPosition);
                    stepper.setMaxSpeed(set_Speed);
                    homed = false;
                    state = 1;
                    break;
                case 3:
                    input = desiredTime;
                    break;
                case 4:
                    input = 0;
                    state = 5;
                    stepper.moveTo(accDistance * steps_mm);
                    stepper.setSpeed(speed * steps_mm);

                    break;
                case 5:
                    input = 0;
                    state = 0;
                    break;
                case 6:
                    input = speed;
                    break;
                case 7:
                    input = accDistance;
                    break;
                case 8:
                    input = MICROSTEPS;
                    break;
            }
            stateMax = maxVals[menuState];
            stateMin = minVals[menuState];
            setupEncoder(input);
            firstInProp = false;
            writeNumber(input);
            alpha4.writeDisplay();
        }
        else {
            if (ms > lastQuery + queryTime) {
                lastVal = val;
                val = (float)Encoder.readCounterInt();
                if (val != lastVal) {
                    writeNumber(val);
                    alpha4.writeDisplay();
                
                    switch (menuState) {
                        case 0:
                            break;
                        case 1:
                            startPosition = val;
                            desiredPosition = startPosition;
                            stepper.moveTo(desiredPosition * steps_mm);
                            // stepper.setMaxSpeed(seekSpeed * steps_mm);
                            break;
                        case 2:
                            endPosition = val;
                            desiredPosition = endPosition;
                            stepper.moveTo(desiredPosition * steps_mm);
                            break;
                        case 3:
                            desiredTime = val;
                            break;
                        case 4:
                            break;
                        case 5:
                            break;
                        case 6:
                            speed = val;
                            break;
                        case 7:
                            accDistance = val;
                            break;
                        case 8:
                            MICROSTEPS = val;
                            setSteps(MICROSTEPS);
                            break;
                    }
                }
                lastQuery = ms;
            }
        }
    }
    else {
        if (firstInProp) {
            Encoder.writeCounter((int32_t) menuState); /* Reset the counter value */
            Encoder.writeMax((int32_t) 8); /* Set the maximum threshold*/
            Encoder.writeMin((int32_t) 0); /* Set the minimum threshold */
            Encoder.writeStep((int32_t) 1); /* Set the step to 1*/
            firstInProp = false;
            writeWord(menuLabels[menuState]);
            alpha4.writeDisplay();
        }
        lastMenuState = menuState;
        menuState = Encoder.readCounterByte();
        if (menuState != lastMenuState) {
            writeWord(menuLabels[menuState]);
            alpha4.writeDisplay();
        }
    }


    if (state == 1) {
        stepper.run();
    }

    else if (state == 2) {
        if (!homed) {
            home();
            seeked = false;
            ended = false;
        }
        else if (!seeked) {
            stepper.moveTo(startPosition * steps_mm);
            stepper.setMaxSpeed(seekSpeed * steps_mm);
            if (stepper.currentPosition() == startPosition * steps_mm) {
                seeked = true;
                float speed = abs(startPosition - endPosition) / (desiredTime * 60.0); 
                setMicrosteps (speed);
                stepper.setCurrentPosition(startPosition * steps_mm);
                stepper.moveTo(endPosition * steps_mm);
                float steps = abs(startPosition - endPosition) * steps_mm / (desiredTime * 60.0);
                stepper.setSpeed(steps);
            }
            stepper.run();
        }
        else if (seeked && !ended) {
            stepper.runSpeedToPosition();
            if (stepper.currentPosition() == endPosition * steps_mm) {
                ended = true;
                homed = false;
                seeked = false;
                state = 0;
            }
        }
    }

    else if (state == 5) {
        if (stepper.currentPosition() / steps_mm == accDistance) {
            Serial.println(ms - last_bounce);
            Serial.println(speed * steps_mm);
            last_bounce = ms;
            accDistance = -accDistance;
            stepper.moveTo(accDistance * steps_mm);
            stepper.setSpeed(speed * steps_mm);
        }
        stepper.runSpeedToPosition();
    }

    
}

void setMicrosteps(float speed) {
    int micros = 256;
    if (speed < 1.5) {
        micros = 16;
    }
    else if (speed < 3.0) {
        micros = 32;
    }
    else if (speed < 7.0) {
        micros = 16;
    }
    else if (speed < 15.0) {
        micros = 8;
    }
    else if (speed < 35.0) {
        micros = 8;
    }
    else {
        micros = 4;
    }

    if (micros != MICROSTEPS) {
        MICROSTEPS = micros;
        driver.microsteps(MICROSTEPS);
        steps_mm = (MOTOR_STEPS * MICROSTEPS)/(PULLEY_TEETH * BELT_PITCH);
    }
}

void setSteps(int steps) {
    int currPos = stepper.currentPosition();
    float currPosMM = currPos / steps_mm;
    if (steps == 2 || steps == 4 || steps == 8 || steps == 16 || steps == 32 || steps == 64 || steps == 128 || steps == 256){
        MICROSTEPS = steps;
        driver.microsteps(MICROSTEPS);
        steps_mm = (MOTOR_STEPS * MICROSTEPS)/(PULLEY_TEETH * BELT_PITCH);
    }
    float newPos = currPosMM * steps_mm;
    stepper.setCurrentPosition(newPos);
}

void setupEncoder (int input) {
    Encoder.writeCounter((int32_t) input); /* Reset the counter value */
    Encoder.writeMax((int32_t) stateMax); /* Set the maximum threshold*/
    Encoder.writeMin((int32_t) stateMin); /* Set the minimum threshold */
    Encoder.writeStep((int32_t) 1); /* Set the step to 1*/
}

void writeWord (String word) {
    if (word.length() > 4) {
        word = word.substring(0,3);
    }
    alpha4.writeDigitAscii(0, word[0]);
    alpha4.writeDigitAscii(1, word[1]);
    alpha4.writeDigitAscii(2, word[2]);
    alpha4.writeDigitAscii(3, word[3]);
}

void writeNumber(float score)
{
    // Serial.println(score);
    String stringScore = String((int)score);
    for (int j = 0; j <= (4 - stringScore.length()); j++)
    {
        String temp = "0";
        temp += stringScore;
        stringScore = temp;
    }
    if (score < 10) {
        stringScore = "0" + stringScore;
    }

    // Serial.println(stringScore);
    alpha4.writeDigitAscii(0, stringScore[0]);
    alpha4.writeDigitAscii(1, stringScore[1]);
    alpha4.writeDigitAscii(2, stringScore[2]);
    alpha4.writeDigitAscii(3, stringScore[3]);

}

void home() {
    setMicrosteps (seekSpeed);
    currentStatus = 100;
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(seekSpeed * steps_mm);
    stepper.moveTo(-2.0 * maxDistance * steps_mm);
    int counter = 0;
    while (currentStatus > stallThreshold && counter < stallCounter) {
        uint32_t drv_status = driver.DRV_STATUS();
        currentStatus = (drv_status & SG_RESULT_bm);
        if (currentStatus < stallThreshold) {
            counter++;
        }
        Serial.println(currentStatus);
        stepper.run();
    }
    stepper.setCurrentPosition(0);
    homed = true;
    Serial.println("homed");
}