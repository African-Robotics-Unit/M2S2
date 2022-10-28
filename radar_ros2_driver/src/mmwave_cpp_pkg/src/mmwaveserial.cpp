#include <CppLinuxSerial/SerialPort.hpp>
#include <iostream>

using namespace mn::CppLinuxSerial;

int main() {

	SerialPort serialPort("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);

	serialPort.SetTimeout(500); 
	serialPort.Open();

	// Write command
	serialPort.Write("dfeDataOutputMode 1\n");

	// Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
	std::string readData;
	serialPort.Read(readData);
	std::cout<<"Response: \n";
	std::cout<<readData;

	// Close the serial port
	serialPort.Close();

	return 0;
}

//g++ mmwaveserial.cpp -lCppLinuxSerial -o test_serial