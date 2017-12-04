
#ifndef TEENSY_H
#define TEENSY_H

class Teensy
{
protected:
	bool handshake 			= false;
	bool command_arrived 	= true;

	Teensy() {}

public:
	bool notWaiting() {
		if (handshake) command_arrived = true;

		if (command_arrived) return true;
		else return false;
	}
};

#endif