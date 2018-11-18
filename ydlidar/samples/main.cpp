
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
//#include <unistd.h>

using namespace std;
using namespace ydlidar;
CYdLidar drv;
static bool running = false;

static void Stop(int signo)
{

    printf("Received exit signal\n");
    running = true;

}

int main(int argc, char * argv[])
{

    bool showHelp = argc > 1 && !strcmp(argv[1], "--help");
    printf(" YDLIDAR C++ TEST\n");
    //   if (argc<4 || showHelp )
       //{
    //     
       //		printf("Usage: %s <serial_port> <baudrate> <intensities>\n\n",argv[0]);
    //           printf("Example:%s /dev/ttyUSB0 115200 0\n\n",argv[0]);
       //		if (!showHelp)
       //		{				
    //               return -1;
       //		}
    //           else
    //               return 0;
    //   }

    const std::string port = "\\\\.\\com4";
    const int baud = 115200;
    const int intensities = 0;

    //signal(SIGINT, Stop);
    //signal(SIGTERM, Stop);

    drv.setSerialPort(port);
    drv.setSerialBaudrate(baud);
    drv.setIntensities(intensities);

    drv.initialize();
    while (!running) {
        bool hardError;
        LaserScan scan;

        if (drv.doProcessSimple(scan, hardError)) {
            fprintf(stderr, "Scan received: %u ranges\n", (unsigned int)scan.ranges.size());

        }
        //usleep(50*1000);


    }
    drv.turnOff();
    drv.disconnecting();

    return 0;


}
