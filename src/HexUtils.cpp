#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdint.h>
using namespace std;
#include "HexUtils.h"
#include "misc.h"

HexUtils::HexUtils()
{
    //ctor
}

HexUtils::~HexUtils ()
{
    //dtor
}

vector<HexRecord> *HexUtils::ParseHexFile(string fileName, string &debugInfo)
{
    ifstream *streamReader;
    uint numHexRecords = 0;
    HexRecord curHexRecord;
    vector<HexRecord> *hexRecords = new vector<HexRecord>();

    try
    {
        streamReader = new ifstream(fileName.c_str());
    }
    catch (...)
    {
        return NULL;
    }

    debugInfo = "";

    if (!streamReader->is_open()) return NULL;

    while (!streamReader->eof())
    {
        char line[4096];
        debugInfo += ToString(numHexRecords + 1) + ") ";
        line[0]=0;
        streamReader->getline(line,4096);
        if(streamReader->eof() && line[0]==0)
            break;
        if(strcmp(line,":00000001FF")==0) {
            int y;
            y++;
        }
        curHexRecord.rawStr =  line;
        debugInfo += curHexRecord.rawStr;
        debugInfo += "\r\n";

        if (curHexRecord.rawStr[0] != ':')
        {
            debugInfo += "\tInvalid Record\r\n";
        }
        else
        {
            debugInfo += "\tValid Record\r\n";
        }

        curHexRecord.len = ParseHex<int>(curHexRecord.rawStr.substr(1, 2));
        debugInfo += "\tLength: 0x" + ToHexString(curHexRecord.len) + "\r\n";

        curHexRecord.address = ParseHex<int>(curHexRecord.rawStr.substr(3, 4));
        debugInfo += "\tAddress: 0x" + ToHexString(curHexRecord.address) + "\r\n";

        curHexRecord.type = ParseHex<int>(curHexRecord.rawStr.substr(7, 2));

        debugInfo += "\tType: 0x" + ToHexString(curHexRecord.type);

        switch (curHexRecord.type)
        {
            case 0:
                debugInfo += " (Data record)\r\n";
                break;
            case 1:
                debugInfo += " (End of file record)\r\n";
                break;
            case 2:
                debugInfo += " (Extended segment address record)\r\n";
                break;
            case 3:
                debugInfo += " (Start segment address record)\r\n";
                break;
            case 4:
                debugInfo += " (Extended linear address record)\r\n";
                break;
            case 5:
                debugInfo += " (Start linear address record)\r\n";
                break;
            default:
                debugInfo += " (Unknown record type)\r\n";
                break;
        }

        if (curHexRecord.len > 0)
        {
            curHexRecord.data = new uint8_t[curHexRecord.len];
            debugInfo += "\tData: 0x";

            for (uint x = 0; x < curHexRecord.len; x++)
            {
                string s = curHexRecord.rawStr.substr(9 + (x * 2), 2);
                uint8_t t = ParseHex<uint8_t>(s);
                curHexRecord.data[x]=t;

                if (curHexRecord.data[x] < 0x10)
                    debugInfo += "0";

                debugInfo += ToHexString(curHexRecord.data[x]);
            }

            debugInfo += "\r\n";
        }
        else
        {
            debugInfo += "\tNo data\r\n";
        }

        curHexRecord.checkSum = ParseHex<int>(curHexRecord.rawStr.substr(9 + (curHexRecord.len * 2), 2));
        debugInfo += "\tRecord checksum: 0x" + ToHexString(curHexRecord.checkSum) + "\r\n";

        curHexRecord.calcSum = 0;

        for (uint x = 1; x < 9 + (curHexRecord.len * 2); x += 2)
        {
            curHexRecord.calcSum += ParseHex<int>(curHexRecord.rawStr.substr(x, 2));
        }

        curHexRecord.calcSum = -curHexRecord.calcSum;
        curHexRecord.calcSum &= 0xFF;

        debugInfo += "\tCalculated checksum: 0x" + ToHexString(curHexRecord.calcSum) + "\r\n";

        if (curHexRecord.calcSum == curHexRecord.checkSum)
        {
            debugInfo += "\tChecksums match\r\n\r\n";
        }
        else
        {
            debugInfo += "\tChecksums do not match\r\n\r\n";
            return NULL;
        }

        hexRecords->push_back(curHexRecord);
        numHexRecords++;
    }

    streamReader->close();

    return hexRecords;
}


