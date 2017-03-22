#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string>
#include <string.h>
#include <fstream>
#include <sys/ioctl.h>
#include <stdint.h>
#include <vector>

#include "misc.h"
#include "HexUtils.h"
using namespace std;

string version="0.1";

enum BLCommand {
    BLCommand_Start				= 's',
    BLCommand_GetDevID			= 'i',
    BLCommand_GetBLVersion		= 'v',
    BLCommand_SetRWAddress		= 'a',
    BLCommand_Write				= 'w',
    BLCommand_Read				= 'r',
    BLCommand_SetUsrMainAddress	= 'm',
    BLCommand_ErasePage			= 'e',
    BLCommand_Exit				= 'x',
    BLCommand_Any					= '*',
    BLCommand_Unknown				= '?',
    BLCommand_Timeout				= '$',
    BLCommand_Unexpected			= '!',
};



string blDevIdToString[]
{
    "Nordic nRF24LU1(+)",
    "Nordic nRF24LE1-24",
    "Nordic nRF24LE1-32",
    "Nordic nRF24LE1-48"
};


enum ResetPin
{
    DISABLED,
    DTR,
    RTS,
};

enum Command
{
    C_CHECK,
    C_FLASH,
};

enum BLCommandReturnType
{
    BLCommandReturnType_OK,
    BLCommandReturnType_BadCharacter,
    BLCommandReturnType_Timeout,
    BLCommandReturnType_SerialPortFailure,
};


string hexfile = "";
string port = "/dev/ttyUSB0";
speed_t baudrate = B38400;
ResetPin resetpin = DTR;
ResetPin blenabled = DISABLED;
Command command = C_CHECK;


void usage() {
    cout << "Usage : nrf24lx1Flash -f [hexfile] -p port -b baudrate -r reset -b BL_enabled -c command\n";
    cout << "-f [hexfile], path to hex file\n";
    cout << "-p port, port device\n\t-default /dev/ttyUSB0 \n";
    cout << "-b baudrate, UART speed\n\t-default 38400 \n";
    cout << "-r reset, reset pin DTR or RTS\n\t-default DTR\n";
    cout << "-bl BL_enabled, DISABLED or DTR or RTS\n\t-default DISBALED";
    cout << "-c command, CHECK, FLASH\n\t-default CHECK\n";
    cout << "\nVersion " << version << "\n";
}

bool processCommandLine(int argc, char **argv) {
    int i;
    for(i=1;i<argc;i++) {
        if(strcmp(argv[i],"-f")==0) {
            if( i + 1 == argc ) {
                cout << "Error: hex file not specified.\n";
                usage();
            } else {
                hexfile = argv[i+1];
            }
        }
        if(strcmp(argv[i],"-p")==0) {
            if( i + 1 == argc ) {
                cout << "Error: port not specified.\n";
            } else {
                port = argv[i+1];
            }
        }
        if(strcmp(argv[i],"-b")==0) {
            if( i + 1 == argc ) {
                cout << "Error: baudrate not specified.\n";
            } else {
                if(strcmp(argv[i+1],"50")==0) { baudrate = B50; continue;};
                if(strcmp(argv[i+1],"75")==0) { baudrate = B75; continue;};
                if(strcmp(argv[i+1],"110")==0) { baudrate = B110; continue;};
                if(strcmp(argv[i+1],"134")==0) { baudrate = B134; continue;};
                if(strcmp(argv[i+1],"150")==0) { baudrate = B150; continue;};
                if(strcmp(argv[i+1],"200")==0) { baudrate = B200; continue;};
                if(strcmp(argv[i+1],"300")==0) { baudrate = B300; continue;};
                if(strcmp(argv[i+1],"600")==0) { baudrate = B600; continue;};
                if(strcmp(argv[i+1],"1200")==0) { baudrate = B1200; continue;};
                if(strcmp(argv[i+1],"1800")==0) { baudrate = B1800; continue;};
                if(strcmp(argv[i+1],"2400")==0) { baudrate = B2400; continue;};
                if(strcmp(argv[i+1],"4800")==0) { baudrate = B4800; continue;};
                if(strcmp(argv[i+1],"9600")==0) { baudrate = B9600; continue;};
                if(strcmp(argv[i+1],"19200")==0) { baudrate = B19200; continue;};
                if(strcmp(argv[i+1],"38400")==0) { baudrate = B38400; continue;};
                if(strcmp(argv[i+1],"57600")==0) { baudrate = B57600; continue;};
                if(strcmp(argv[i+1],"115200")==0) { baudrate = B115200; continue;};
                if(strcmp(argv[i+1],"230400")==0) { baudrate = B230400; continue;};
                if(strcmp(argv[i+1],"460800")==0) { baudrate = B460800; continue;};
                if(strcmp(argv[i+1],"500000")==0) { baudrate = B500000; continue;};
                if(strcmp(argv[i+1],"576000")==0) { baudrate = B576000; continue;};
                if(strcmp(argv[i+1],"921600")==0) { baudrate = B921600; continue;};
                if(strcmp(argv[i+1],"1000000")==0) { baudrate = B1000000; continue;};
                if(strcmp(argv[i+1],"1152000")==0) { baudrate = B1152000; continue;};
                if(strcmp(argv[i+1],"1500000")==0) { baudrate = B1500000; continue;};
                if(strcmp(argv[i+1],"2000000")==0) { baudrate = B2000000; continue;};
                if(strcmp(argv[i+1],"2500000")==0) { baudrate = B2500000; continue;};
                if(strcmp(argv[i+1],"3000000")==0) { baudrate = B3000000; continue;};
                if(strcmp(argv[i+1],"3500000")==0) { baudrate = B3500000; continue;};
                if(strcmp(argv[i+1],"4000000")==0) { baudrate = B4000000; continue;};
                cout << "Errror: invalid baudrate " << argv[i+1] << "\n";
                return false;
            }
        }
        if(strcmp(argv[i],"-r")==0) {
            if( i + 1 == argc ) {
                cout << "Error: reset not specified.\n";
                return false;
            } else {
                if(strcmp(argv[i+1],"DTR")==0) { resetpin = DTR; continue; };
                if(strcmp(argv[i+1],"RTS")==0) { resetpin = RTS; continue; };
                cout << "Errror: invalid reset specified " << argv[i+1] << "\n";
                return false;
            }
        }
        if(strcmp(argv[i],"-bl")==0) {
            if( i + 1 == argc ) {
                cout << "Error: reset not specified.\n";
                return false;
            } else {

                if(strcmp(argv[i+1],"DISABLED")==0) { blenabled = DISABLED; continue; };
                if(strcmp(argv[i+1],"DTR")==0) { blenabled = DTR; continue; };
                if(strcmp(argv[i+1],"RTS")==0) { blenabled = RTS; continue; };
                cout << "Errror: invalid blenabled specified " << argv[i+1] << "\n";
                return false;
            }
        }
        if(strcmp(argv[i],"-c")==0) {
            if( i + 1 == argc ) {
                cout << "Error: command not specified.\n";
                return false;
            } else {

                if(strcmp(argv[i+1],"CHECK")==0) { command = C_CHECK; continue; };
                if(strcmp(argv[i+1],"FLASH")==0) { command = C_FLASH; continue; };
                cout << "Errror: invalid command specified " << argv[i+1] << "\n";
                return false;
            }
        }
        if(strcmp(argv[i],"-h")==0 | strcmp(argv[i],"--help")==0 ) {
            usage();
            exit(0);
        }
    }
    if(hexfile.compare("")==0 && command == C_FLASH) {
        cout<< "Error: hexfile not specified\n";
        return 0;
    }
    return true;
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                cout <<  "error %d from tcgetattr" <<  errno;
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                cout << "error from tcsetattr:" << errno;
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                cout << "error from tggetattr: " << errno;
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                cout << "error setting term attributes:"<<errno;
}

int progSerPort;

bool OpenSerialPort(speed_t baudRate)
{
    string s = port;
    strreplace(s,"/dev/", "");
    string lockFileName = "/tmp/LCK.." + s;

    try
    {
        ifstream *lockFile = new ifstream(lockFileName.c_str());
    }
    catch (...)
    {
        cout << "Error: Lock file " + lockFileName + " exists!  It is likely that another serial terminal has this port open!\r\n";
        return false;
    }

    progSerPort = open(port.c_str(),O_RDWR | O_NOCTTY | O_SYNC );

    set_interface_attribs (progSerPort, B38400, 0);
    set_blocking (progSerPort, 0);
    return true;
}

void RtsSet(int fd, bool value) {
    int sercmd = TIOCM_RTS;
    if(value) {
        ioctl(fd, TIOCMBIS, &sercmd); // Set the RTS pin.
    } else {
        ioctl(fd, TIOCMBIC, &sercmd); // Reset the RTS pin.
    }
}

void DtrSet(int fd, bool value) {
    int sercmd = TIOCM_DTR;
    if(value) {
        ioctl(fd, TIOCMBIS, &sercmd); // Set the DTR pin.
    } else {
        ioctl(fd, TIOCMBIC, &sercmd); // Reset the DTR pin.
    }
}

void ResetDevice()
{
    //Enable the reset signal, which will put the chip in reset (the reset signal must be either RTS or DTR)
    if (resetpin == RTS)
    {

        RtsSet(progSerPort,true);
    }
    else
    {
        DtrSet(progSerPort,true);
    }

    //Wait 100 ms for the UART driver to have time to process any serial data the chip was sending prior to reset
    sleep_ms(100);

    //If the bootloader enable signal is enabled (set to RTS or DTR), then enable the signal here before we come out of reset
    if (blenabled == RTS)
    {
        RtsSet(progSerPort,true);
        sleep_ms(1);
    }
    else if (blenabled == DTR)
    {
        DtrSet(progSerPort,true);

        //Sleep for 1 ms to give the enable pin plenty of time to activate
        sleep_ms(1);
    }

    //Disble the reset signal, which returns the device to the normal state
    if (resetpin == RTS)
    {
        RtsSet(progSerPort,false);
    }
    else
    {
        DtrSet(progSerPort,false);
    }

    //Sleep for 1 ms to give the device time to power up
    sleep_ms(1);

    //If the bootloader enable signal is enabled (set to RTS or DTR), then disable the signal now that the device is powered up
    if (blenabled == RTS)
    {
        RtsSet(progSerPort,false);

        //Sleep for 1 ms to give the enable pin plenty of time to activate
        sleep_ms(1);
    }
    else if (blenabled == DTR)
    {
        DtrSet(progSerPort,false);

        //Sleep for 1 ms to give the enable pin plenty of time to activate
        sleep_ms(1);
    }
}


BLCommandReturnType WriteSerialByte(unsigned char buffer)
{
    try
    {
        write(progSerPort,&buffer,1);
        fsync(progSerPort);
        return BLCommandReturnType_OK;
    }
    catch(...)
    {
        return BLCommandReturnType_SerialPortFailure;
    }
}

BLCommandReturnType ReadSerialDataByte(unsigned char *buffer)
{
  fd_set set;
  struct timeval timeout;
  int rv;

  FD_ZERO(&set); /* clear the set */
  FD_SET(progSerPort, &set); /* add our file descriptor to the set */

  timeout.tv_sec = 0;
  timeout.tv_usec = 2000000;

  rv = select(progSerPort + 1, &set, NULL, NULL, &timeout);
  if(rv == -1)
    return BLCommandReturnType_SerialPortFailure; /* an error accured */
  else if(rv == 0)
    return BLCommandReturnType_Timeout; /* a timeout occured */
  else
    read( progSerPort, (void *)buffer,1 ); /* there was data to read */

    return BLCommandReturnType_OK;
}



BLCommandReturnType ReadSerialCommandByte(BLCommand command)
{
    unsigned char buffer = 0;
    BLCommandReturnType blCommandReturnVal;

    blCommandReturnVal = ReadSerialDataByte(&buffer);

    if (blCommandReturnVal == BLCommandReturnType_OK)
    {
        if ((command == BLCommand_Any) || (buffer == (char)command))
        {
            return BLCommandReturnType_OK;
        }
        else
        {
            return BLCommandReturnType_BadCharacter;
        }
    }
    else
    {
        return blCommandReturnVal;
    }
}



BLCommandReturnType ExecuteCommandStartBootloader()
{
    if (!OpenSerialPort(baudrate))
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    //reset the chip
    ResetDevice();

    //send the start byte
    if (WriteSerialByte((unsigned char)BLCommand_Start) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    //wait for the return character and return it
    return ReadSerialCommandByte(BLCommand_Start);
}

BLCommandReturnType ExecuteCommandGetDevID(uint16_t *buffer)
{
    unsigned char tempByte = 0;

    if (WriteSerialByte((unsigned char )BLCommand_GetDevID) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (ReadSerialDataByte(&tempByte) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_Timeout;
    }

    *buffer = tempByte;

    if (ReadSerialDataByte(&tempByte) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_Timeout;
    }

    (*buffer) += ((uint16_t)tempByte) << 8;

    return ReadSerialCommandByte(BLCommand_GetDevID);
}


BLCommandReturnType ExecuteCommandGetBLVersion(unsigned char *buffer)
{
    if (WriteSerialByte((unsigned char)BLCommand_GetBLVersion) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (ReadSerialDataByte(buffer) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_Timeout;
    }

    return ReadSerialCommandByte(BLCommand_GetBLVersion);
}

BLCommandReturnType ExecuteCommandExitBootloader()
{
    BLCommandReturnType retVal = BLCommandReturnType_Timeout;

    if (WriteSerialByte((unsigned char)BLCommand_Exit) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    retVal = ReadSerialCommandByte(BLCommand_Exit);

    return retVal;
}


string ConvertBLDevIDToString(uint16_t blDevId)
{
    string retVal;

    if(blDevId >= sizeof(&blDevIdToString))
    {
        retVal = "[Invalid Device ID]";
    }
    else
    {
        retVal = blDevIdToString[blDevId];
    }

    retVal += " (ID: " + ToString(blDevId) + ")\n";

    return retVal;
}


bool CheckUC()
{
    string s = port;
    BLCommandReturnType blCommandReturnVal;


    if (resetpin == blenabled)
    {
        cout << "Reset and bootloader enable pins cannot be the same!\n";
        return false;
    }

    //begin start bootloader
    blCommandReturnVal = ExecuteCommandStartBootloader();

    if (blCommandReturnVal == BLCommandReturnType_SerialPortFailure)
    {
        cout << "Error: Could not open COM port!\n";
        return false;
    }
    else if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout << "Error: Device not connected!\n";
        return false;
    }
    //end start	bootloader

    //begin device id check
    uint16_t devId = 0;

    blCommandReturnVal = ExecuteCommandGetDevID(&devId);

    if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout << "Lost communications with device!\n";
        return false;
    }
    //end device id check

    //begin bootloader version check
    unsigned char blVersion = 0;

    blCommandReturnVal = ExecuteCommandGetBLVersion(&blVersion);

    if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout << "Lost communications with device!\n";
        return false;
    }
    //end device id check

    //begin exit bootloader only
    blCommandReturnVal = ExecuteCommandExitBootloader();

    if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout<< "Lost communications with device!\n";
        return false;
    }
    //end exit bootloader

    //If we have made it this far, then everything went to plan, so update the microcontroller type and bootloader version textboxes
    cout << ConvertBLDevIDToString(devId);
    cout << "Device found!\n";

    return true;
}

BLCommandReturnType ExecuteCommandErasePage(uint8_t pageNum)
{
    if (WriteSerialByte((uint8_t)BLCommand_ErasePage) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (WriteSerialByte(pageNum) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    return ReadSerialCommandByte(BLCommand_ErasePage);
}

BLCommandReturnType ExecuteCommandSetRWAddress(uint16_t address)
{
    if (WriteSerialByte((uint8_t)BLCommand_SetRWAddress) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (WriteSerialByte((uint8_t)address & 0xFF) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (WriteSerialByte((uint8_t)(address >> 8)) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    return ReadSerialCommandByte(BLCommand_SetRWAddress);
}


BLCommandReturnType ExecuteCommandWrite(uint8_t buffer[], uint16_t startIndex, uint16_t len)
{
    uint numWritten = 0;

    while(numWritten < len)
    {
        uint numToWrite;

        //The write operation only supports writing up to 256 bytes per write
        if ((len - numWritten) > 256)
        {
            numToWrite = 256;
        }
        else
        {
            numToWrite = len - numWritten;
        }

        //Send write command
        if (WriteSerialByte((uint8_t)BLCommand_Write) != BLCommandReturnType_OK)
        {
            return BLCommandReturnType_SerialPortFailure;
        }

        //Send number of bytes to be written (0 here indicates we want to write 256 bytes, so casting numToWrite to a byte does the trick of getting rid of the MSb)
        if (WriteSerialByte((uint8_t)numToWrite) != BLCommandReturnType_OK)
        {
            return BLCommandReturnType_SerialPortFailure;
        }

        //Send the bytes to be written one by one
        for(unsigned int x = 0; x < numToWrite; x++)
        {
            if (WriteSerialByte(buffer[startIndex + numWritten]) != BLCommandReturnType_OK)
            {
                return BLCommandReturnType_SerialPortFailure;
            }

            numWritten++;
        }

        if (ReadSerialCommandByte(BLCommand_Write) != BLCommandReturnType_OK)
        {
            return BLCommandReturnType_Timeout;
        }
    }

    return BLCommandReturnType_OK;
}

BLCommandReturnType ExecuteCommandSetUsrMainAddress(uint16_t address)
{
    if (WriteSerialByte((uint8_t)BLCommand_SetUsrMainAddress) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (WriteSerialByte((uint8_t)address & 0xFF) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    if (WriteSerialByte((uint8_t)address >> 8) != BLCommandReturnType_OK)
    {
        return BLCommandReturnType_SerialPortFailure;
    }

    return ReadSerialCommandByte(BLCommand_SetUsrMainAddress);
}

BLCommandReturnType ExecuteCommandRead(uint8_t buffer[], uint16_t startIndex, uint16_t len)
{
    unsigned int numRead = 0;

    while(numRead < len)
    {
        unsigned int numToRead;

        //The read operation only supports reading up to 256 bytes per read
        if ((len - numRead) > 256)
        {
            numToRead = 256;
        }
        else
        {
            numToRead = len - numRead;
        }

        //Send read command
        if (WriteSerialByte((uint8_t)BLCommand_Read) != BLCommandReturnType_OK)
        {
            return BLCommandReturnType_SerialPortFailure;
        }

        //Send number of bytes to be read (0 here indicates we want to read 256 bytes, so casting numToRead to a byte does the trick of getting rid of the MSb)
        if (WriteSerialByte((uint8_t)numToRead) != BLCommandReturnType_OK)
        {
            return BLCommandReturnType_SerialPortFailure;
        }

        //Read in the bytes from the micro one by one
        for(unsigned int x = 0; x < numToRead; x++)
        {
            if (ReadSerialDataByte(&buffer[startIndex + numRead]) != BLCommandReturnType_OK)
            {
                return BLCommandReturnType_SerialPortFailure;
            }

            numRead++;
        }

        if (ReadSerialCommandByte(BLCommand_Read) != BLCommandReturnType_OK)
        {
            return BLCommandReturnType_Timeout;
        }
    }

    return BLCommandReturnType_OK;
}



bool ProgramUC()
{
    string debugInfo = "";
    vector<HexRecord> *hexRecords;
    BLCommandReturnType blCommandReturnVal;
    uint8_t readData[512] ;
    bool pageErased[31];
    bool dataInPage[31];
    uint8_t ucFlash [0x4000];
    uint8_t jumpInstruction[] = {0x02, 0x3E, 0x00};

    ucFlash[0] = jumpInstruction[0];
    ucFlash[1] = jumpInstruction[1];
    ucFlash[2] = jumpInstruction[2];

    #if DEBUG_MSG_BOX_ENABLED
    outputString = "";
    #endif


    cout << "Initiating program/verify sequence...\r\n";

    if (resetpin == blenabled)
    {
        cout << "Reset and bootloader enable pins cannot be the same!";
        return false;
    }

    cout << "Parsing hex file...\r\n";

    //begin parse hex file
    hexRecords = HexUtils::ParseHexFile(hexfile, debugInfo);

    //If hexRecords is null, then the file could not be found
    if (hexRecords == NULL)
    {
        cout << "Could not open hex file!";
        return false;
    }

    for (unsigned int  x = 0; x < sizeof(pageErased); x++)
    {
        pageErased[x] = false;
        dataInPage[x] = false;
    }

    for (unsigned int x = 0; x < sizeof(ucFlash); x++)
    {
        ucFlash[x] = 0xFF;
    }

    //Loop through each of the individual hex records to build the complete flash image
    for (unsigned int x = 0; x < hexRecords->size(); x++)
    {
        HexRecord curHexRecord = (HexRecord)hexRecords->at((int)x);

        //If an address is found that is in the bootloader's address space, then throw an error
        if ((curHexRecord.address + curHexRecord.len) >= 0x3E00)
        {
            cout << "Attempting to program address in bootloader's address space (0x3E00 through 0x3FFF)!";
            return false;
        }

        //Loop through the data in the hex record to build the flash image
        for (unsigned int y = 0; y < curHexRecord.len; y++)
        {
            uint8_t curPage = (uint8_t)((curHexRecord.address + y) / 512);

            uint8_t data = curHexRecord.data[y];
            int addr = curHexRecord.address;
            ucFlash[ addr + y] = data;

            if (!dataInPage[curPage] && (ucFlash[curHexRecord.address + y] != 0xFF))
                dataInPage[curPage] = true;
        }
    }
    //end parse hex file

    cout << "Parsing hex file complete!\r\n";
    cout << "Programming device flash...\r\n";

    //begin start bootloader
    blCommandReturnVal = ExecuteCommandStartBootloader();

    if (blCommandReturnVal == BLCommandReturnType_SerialPortFailure)
    {
        cout << "Could not open COM port!";
        return false;
    }
    else if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout << "Device not connected!\n";
        return false;
    }
    //end start	bootloader

    //begin device id check
    uint16_t devId = 0;

    blCommandReturnVal = ExecuteCommandGetDevID(&devId);

    if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout << "Lost communications with device!";
        return false;
    }
    //end device id check

    //begin bootloader version check
    uint8_t blVersion = 0;

    blCommandReturnVal = ExecuteCommandGetBLVersion(&blVersion);

    if (blCommandReturnVal != BLCommandReturnType_OK)
    {
        cout << "Lost communications with device!";
        return false;
    }
    //end device id check

    cout << ConvertBLDevIDToString(devId);

    //begin write hex file data
    for (uint8_t curPage = 0; curPage < 31; curPage++)
    {
        unsigned int writeStartAddress = 0;
        unsigned int writeEndAddress = 0;
        unsigned int curPageStartAddress = (unsigned int )curPage * 512;
        unsigned int curPageEndAddress = curPageStartAddress + 512 - 1;
        unsigned int curRWAddress = 0;

        //If there is data in the page, then we will do some writing
        if (dataInPage[curPage])
        {
            //Page 0 is a special case because it contains the jump to bootloader code
            if (curPage == 0)
            {
                //Erase page 0 here
                if (ExecuteCommandErasePage(0) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Mark page erased
                pageErased[0] = true;

                //Set the RW address so we can write the jump instruction
                if (ExecuteCommandSetRWAddress(0) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Update our shadow copy of the chip's RW address
                curRWAddress = 0;

                //Write the jump instruction
                if (ExecuteCommandWrite(jumpInstruction, 0, 3) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Update our shadow copy of the chip's RW address
                curRWAddress += 3;

                //Set the user main address that lived in the jump instruction of the user's hex file (bytes 1 (MSB) and 2 (LSB))
                if (ExecuteCommandSetUsrMainAddress((uint16_t)(ucFlash[1] << 8) + ucFlash[2]) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Since we've already written the first 3 bytes in the page, update the start address for the rest of the page accordingly
                writeStartAddress = 3;
            }
            else
            {
                //Erase the page if it hasn't been erased
                if (!pageErased[curPage])
                {
                    if (ExecuteCommandErasePage(curPage) != BLCommandReturnType_OK)
                    {
                        cout << "Lost communications with device!";
                        return false;
                    }

                    pageErased[curPage] = true;
                }

                //We haven't written any data, so the start address is just the first address of the page
                writeStartAddress = curPageStartAddress;
            }

            //Loop through this page
            while (writeStartAddress <= curPageEndAddress)
            {
                //Check to see if the current byte is 0xFF or not (FLASH erases to 0xFF, so we do not have to write that value)
                if (ucFlash[writeStartAddress] != 0xFF)
                {
                    uint endFFhCount = 0; //Counter for number of 0xFF values seen after the last non-0xFF value

                    //We have found a non-0xFF value, so now we will try to find where we will end this write operation
                    writeEndAddress = writeStartAddress;

                    //curViewingAddress is the address we're inspecting to see if we want to write the value or not
                    uint curViewingAddress = writeEndAddress + 1;

                    //Check if curViewingAddress is within the current page boundary
                    while (curViewingAddress <= curPageEndAddress)
                    {
                        //We're in the current page boundary, so check if the next value in the flash image is non-0xFF
                        if (ucFlash[writeEndAddress + 1] != 0xFF)
                        {
                            //The next value is non-0xFF, so set writeEndAddress to curViewingAddress and zero out endFFhCount
                            writeEndAddress = curViewingAddress;
                            endFFhCount = 0;
                        }
                        else
                        {
                            //The next value is 0xFF, so increment endFFhCount
                            endFFhCount++;

                            //We have to write at least 8 0xFF values before it makes sense to break a single write into two
                            if (endFFhCount >= 8)
                            {
                                //We are on our 8th 0xFF, so we will break this write
                                break;
                            }
                        }

                        //curViewingAddress is always one location ahead of writeEndAddress plus endFFhCount
                        curViewingAddress = writeEndAddress + endFFhCount + 1;
                    }

                    //Update the chip's RW address if it isn't equal to writeStartAddress
                    if (curRWAddress != writeStartAddress)
                    {
                        if (ExecuteCommandSetRWAddress((uint16_t)writeStartAddress) != BLCommandReturnType_OK)
                        {
                            cout << "Lost communications with device!";
                            return false;
                        }
                    }

                    //Update our shadow copy of the chip's RW address
                    curRWAddress = writeStartAddress;

                    //Write the curent block to the chip
                    uint numBytesToWrite = writeEndAddress - writeStartAddress + 1;

                    if (ExecuteCommandWrite(ucFlash, (uint16_t)curRWAddress, (uint16_t)numBytesToWrite) != BLCommandReturnType_OK)
                    {
                        cout << "Lost communications with device!";
                        return false;
                    }

                    //Update shadow copy of chip's RW address
                    curRWAddress += numBytesToWrite;

                    //Update writeStartAddress
                    writeStartAddress = writeEndAddress + 1;
                }
                else
                {
                    //The value was 0xFF, so look at the next value to try to start the write
                    writeStartAddress++;
                }
            }
        }
        else
        {
            //There is no data in the page, so erase it if it hasn't already been erased
            if (!pageErased[curPage])
            {
                //Erase current page
                if (ExecuteCommandErasePage(curPage) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Mark page erased
                pageErased[curPage] = true;
            }
        }
        //Update the progress bar
        //cout << (((curPage + 1) * 100) / 32) << "% complete\n";
    }
    //end write hex file data

    cout << "Programming device flash complete!\r\n";
    cout << "Verifying device flash...\r\n";

    //begin verify hex file data
    for (uint8_t curPage = 0; curPage < 31; curPage++)
    {
        uint readStartAddress = 0;
        uint readEndAddress = 0;
        uint curPageStartAddress = (uint)curPage * 512;
        uint curPageEndAddress = curPageStartAddress + 512 - 1;
        uint curRWAddress = 0;
        uint userMainJumpInstLoc = 0x3E09;

        //If there is data in the page, then we will do some writing
        if (dataInPage[curPage])
        {
            //Page 0 is a special case because it contains the jump to user code
            if (curPage == 0)
            {
                //Set the RW address so we can read the jump to user main instruction
                if (ExecuteCommandSetRWAddress((uint16_t)userMainJumpInstLoc) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                curRWAddress = userMainJumpInstLoc;

                //Read the jump to user main instruction from the chip
                if (ExecuteCommandRead(readData, 0, 3) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Update our shadow copy of the chip's RW address
                curRWAddress += 3;

                //Verify jump to user main instruction
                if ((readData[0] != 0x02) || (readData[1] != ucFlash[1]) || (readData[2] != ucFlash[2]))
                {
                    cout << "Programming error in jump to user main instruction starting at 0x" + ToHexString(userMainJumpInstLoc) + ": expected [0x02 0x" + ToHexString(ucFlash[1]) + " 0x" + ToHexString(ucFlash[2]) + "], zzbut received [0x" + ToHexString(readData[0]) + " 0x" + ToHexString(readData[1])+ " 0x" + ToHexString(readData[2]) + "]!";
                    return false;
                }

                //Set the RW address so we can read the jump to bootloader instruction
                if (ExecuteCommandSetRWAddress(0) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Update our shadow copy of the chip's RW address
                curRWAddress = 0;

                //Read the jump instruction
                if (ExecuteCommandRead(readData, 0, 3) != BLCommandReturnType_OK)
                {
                    cout << "Lost communications with device!";
                    return false;
                }

                //Update our shadow copy of the chip's RW address
                curRWAddress += 3;

                //Verify jump to bootloader instruction
                if ((readData[0] != jumpInstruction[0]) || (readData[1] != jumpInstruction[1]) || (readData[2] != jumpInstruction[2]))
                {
                    cout << "Programming error in jump to bootloader instruction starting at 0x0000: expected [0x" + ToHexString(jumpInstruction[0]) + " 0x" + ToHexString(jumpInstruction[1]) + " 0x" + ToHexString(jumpInstruction[2]) + "], xxbut received [0x" + ToHexString(readData[0]) + " 0x" + ToHexString(readData[1]) + " 0x" + ToHexString(readData[2]) + "]!";
                    return false;
                }

                //Since we've already read the first 3 bytes in the page, update the start address for the rest of the page accordingly
                readStartAddress = 3;
            }
            else
            {
                //We haven't read any data, so the start address is just the first address of the page
                readStartAddress = curPageStartAddress;
            }

            //Loop through this page
            while (readStartAddress <= curPageEndAddress)
            {
                //Check to see if the current byte is 0xFF or not (FLASH erases to 0xFF, so we do not have to read that value since it automatically matches)
                if (ucFlash[readStartAddress] != 0xFF)
                {
                    uint endFFhCount = 0; //Counter for number of 0xFF values seen after the last non-0xFF value

                    //We have found a non-0xFF value, so now we will try to find where we will end this read operation
                    readEndAddress = readStartAddress;

                    //curViewingAddress is the address we're inspecting to see if we want to read the value or not
                    uint curViewingAddress = readEndAddress + 1;

                    //Check if curViewingAddress is within the current page boundary
                    while (curViewingAddress <= curPageEndAddress)
                    {
                        //We're in the current page boundary, so check if the next value in the flash image is non-0xFF
                        if (ucFlash[readEndAddress + 1] != 0xFF)
                        {
                            //The next value is non-0xFF, so set readEndAddress to curViewingAddress and zero out endFFhCount
                            readEndAddress = curViewingAddress;
                            endFFhCount = 0;
                        }
                        else
                        {
                            //The next value is 0xFF, so increment endFFhCount
                            endFFhCount++;

                            //We have to read at least 8 0xFF values before it makes sense to break a single read into two
                            if (endFFhCount >= 8)
                            {
                                //We are on our 8th 0xFF, so we will break this read
                                break;
                            }
                        }

                        //curViewingAddress is always one location ahead of readEndAddress plus endFFhCount
                        curViewingAddress = readEndAddress + endFFhCount + 1;
                    }

                    //Update the chip's RW address if it isn't equal to readStartAddress
                    if (curRWAddress != readStartAddress)
                    {
                        if (ExecuteCommandSetRWAddress((uint16_t)(readStartAddress)) != BLCommandReturnType_OK)
                        {
                            cout << "Lost communications with device!";
                            return false;
                        }
                    }

                    //Update our shadow copy of the chip's RW address
                    curRWAddress = readStartAddress;

                    //Read the curent block to the chip
                    uint numBytesToRead = readEndAddress - readStartAddress + 1;


                    if (ExecuteCommandRead(readData, readStartAddress, (uint16_t)numBytesToRead) != BLCommandReturnType_OK)
                    {
                        cout << "Lost communications with device!";
                        return false;
                    }

                    //Verify read data
                    for (unsigned int x = 0; x < numBytesToRead; x++)
                    {
                        //Check the current byte against the flash image
                        if (readData[readStartAddress + x] != ucFlash[curRWAddress])
                        {
                            cout << "Programming error at address 0x";
                            cout << ToHexString(curRWAddress);
                            cout << ": expected 0x";
                            cout << ToHexString(ucFlash[curRWAddress]);
                            cout << " but received 0x";
                            cout << ToHexString(readData[x])+ "!";
                            return false;
                        }

                        //Increment shadow copy of chip's RW address
                        curRWAddress++;
                    }

                    //Update readStartAddress
                    readStartAddress = readEndAddress + 1;
                }
                else
                {
                    //The value was 0xFF, so look at the next value to try to start the write
                    readStartAddress++;
                }
            }
        }
        //Update the progress bar
        //cout << (((curPage + 1) * 100) / 32) << "% complete\n";
    }
    //end verify hex file data


    //begin exit bootloader
    if (ExecuteCommandExitBootloader() != BLCommandReturnType_OK)
    {
        cout << "Lost communications with device!";
        return false;
    }
    //end exit bootloader

    cout << "Verifying device flash complete!\r\n";
    cout << "Program/verify sequence completed successfully!\n";

    return true;
}



int main(int argc, char **argv)
{
    if(argc==1) usage();
    if(!processCommandLine(argc, argv)) exit(0);
    if(command==C_CHECK) {
        CheckUC();
    } else
    {
        ProgramUC();
    }

    return 0;
}

