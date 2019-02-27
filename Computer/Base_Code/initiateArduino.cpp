#include "initiateArduino.h"
#include <opencv2/videoio.hpp>
#include <iostream>

int initiateArduino()
{
	if (false /*arduino.isConnected()/**/)
	{
		std::cout << "Arduino Connection Established\n\n";
	}
	else
	{
		perror("STRUAN: Arduino not connected. Camera movement is disabled. Port name is likely incorrect, or Arduino is not connected.\nCOMPUTER");
		std::cout << "\n";
	}
	return 0;
}
