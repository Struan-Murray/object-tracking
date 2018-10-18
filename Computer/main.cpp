//stty -F /dev/tty.usbmodem641 sane raw pass8 -echo -hupcl clocal 9600

#include <iostream>
#include <fstream>
#include <chrono> // Provides cross-platform timing
#include <thread> // Provides control of thread features (ie sleep)


#define PORT_REP "/dev/ttyACM0"
#define ACCESS1 0xAA
#define ACCESS2 0x55
#define SLEEP_TIME 4000

using namespace std;



uint16_t x{0x0}, y{0x0}, z{0x0};
uint8_t xS{0x0}, yS{0x0}, zS{0x0};
//uint8_t a1{ACCESS1}, a2{ACCESS2}, vX{0x58},vY{0x59},vZ{0x5A};

int main()
{
	cout << "PROGRAM START" << endl << endl;
	fstream arduinoPort;
	arduinoPort.open(PORT_REP);

	// Not a busy sleep, will move aside for other threads. Cross platform.
	std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));

	if(arduinoPort.is_open())
	{
		cout << "Port " << PORT_REP <<" Opened" << endl << endl;
	}
	else
	{
		cout << "Port " << PORT_REP << " did not open" << endl << endl;
		perror("");
		return -1;
	}

	for(;;)
	{
		if(arduinoPort.is_open())
		{
			cout << "X: ";
			cin >> x;
			cout << "Y: ";
			cin >> y;
			cout << "Z: ";
			cin >> z;
			xS = x;
			yS = y;
			zS = z;
			cout << "8: " << xS << " 16: " << x << endl;
			arduinoPort << x;
			//fprintf(arduinoPort,"%c%c%c%c%c%c%c%c",a1,a2,vX,xS,vY,yS,vZ,zS);
		}
		else
		{
			cout << "Port Closed" << endl;
			break;
		}
		
	}

	
	arduinoPort.close();
	return 0;
}
