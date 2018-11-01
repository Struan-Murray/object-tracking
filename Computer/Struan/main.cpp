//stty -F /dev/tty.usbmodem641 sane raw pass8 -echo -hupcl clocal 9600

#include <iostream> // Basic IO
#include <fstream>  // File IO
#include <chrono>   // Provides cross-platform timing
#include <thread>   // Provides control of thread features (ie XSLEEP)


#define PORT_REP "/dev/ttyACM0" // Address of microcontroller
#define ACCESS1 0xAA            // Microcontroller's 1st verification byte
#define ACCESS2 0x55            // Microcontroller's 2nd verification byte
#define SLEEP_TIME 4000         // Wait time (ms) for microcontroller to wake-up & reset

void printError();

using namespace std;

uint16_t x{0x0}, y{0x0}, z{0x0};
uint8_t xByte{0x0}, yByte{0x0}, zByte{0x0};
//uint8_t a1{ACCESS1}, a2{ACCESS2}, vX{0x58},vY{0x59},vZ{0x5A};

int main()
{
	cout << "PROGRAM START" << endl << endl;
	fstream arduinoPort;
	arduinoPort.open(PORT_REP);

	if(arduinoPort.is_open())
	{
		// Not a busy XSLEEP, will move aside for other threads. XCROSS_PLATFORM.
		std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
		cout << "Port " << PORT_REP <<" Opened" << endl << endl;
	}
	else
	{
		printError();
		cout << "STRUAN: Microcontroller is either: not connected, locked by another program, or on another port (Change PORT_REP). Currently: " << PORT_REP <<"\n";
		return -1;
	}

	for(;;)
	{
		if(arduinoPort.is_open())
		{
			cout << "X: ";        	//Request X positioning from user. XUSER_INPUT
			cin >> x;
			cout << "Y: ";        	//Request Y positioning from user. XUSER_INPUT
			cin >> y;
			cout << "Z: ";        	//Request Z positioning from user. XUSER_INPUT
			cin >> z;

			xByte = x;
			yByte = y;
			zByte = z;

			cout << "8: " << xByte << " 16: " << x << endl;

			arduinoPort << xByte; 	// Sending data to Arduino. XTOMICROCONTROLLER
			//fprintf(arduinoPort,"%c%c%c%c%c%c%c%c",a1,a2,vX,xS,vY,yS,vZ,zS);
		}
		else
		{
			cout << "Port " << PORT_REP << " Closed" << endl;
			perror("Program failure information");
			break;
		}
	}

	arduinoPort.close();
	return 0;
}

void printError()
{
	cout << "Port " << PORT_REP << " did not open" << endl << endl;
	perror("\aPROGRAM FAILURE\n\nProgram failure information is as follows:\nCOMPUTER");
}
