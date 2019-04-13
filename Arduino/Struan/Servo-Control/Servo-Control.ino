#include <Servo.h>

#define SERVO_X_PIN 5
#define SERVO_Y_PIN 6
#define SERVO_Z_PIN 9
#define SERVO_X_START_ANGLE 90
#define SERVO_Y_START_ANGLE 90
#define SERVO_Z_START_ANGLE 90

Servo servoX, servoY, servoZ;

int16_t servoXAngle{SERVO_X_START_ANGLE};
int16_t servoYAngle{SERVO_Y_START_ANGLE};
int16_t servoZAngle{SERVO_Z_START_ANGLE};

void setup()
{
	Serial.begin(9600);

	servoX.attach(SERVO_X_PIN);
	servoY.attach(SERVO_Y_PIN);
	servoZ.attach(SERVO_Z_PIN);
	
	delay(100);

	servoX.write(servoXAngle);
	servoY.write(servoYAngle);
	servoZ.write(servoZAngle);

	delay(100);

	Serial.println("Program Begin");
}

void loop()
{
	if(Serial.available() > 0)
	{
		/*for(uint8_t inputError = 0x0; inputError <= 0x9; inputError++)
		{
			if(Serial.read() == 0xAA)
			{
				if(Serial.read() == 0x55)
				{
					if(Serial.read() == 0x58)
					{*/
						servoXAngle = Serial.read();
						Serial.println("HERE");
						Serial.println(servoXAngle);
						servoXAngle = map(servoXAngle,0,180,0,180);
						Serial.println(servoXAngle);
					/*}
					if(Serial.read() == 0x59)
					{
						servoYAngle = Serial.read();
					}
					if(Serial.read() == 0x5A)
					{
						servoZAngle = Serial.read();
					}
				}
			}
		}*/
	}
	
	servoX.write(servoXAngle);
	servoY.write(servoYAngle);
	servoZ.write(servoZAngle);
	
	delay(1);
}
