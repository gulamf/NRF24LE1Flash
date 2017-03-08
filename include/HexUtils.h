#ifndef HEXUTILS_H
#define HEXUTILS_H

using namespace std;
struct HexRecord
{
    string rawStr;
    uint16_t len;
    uint16_t address;
    uint16_t type;
    uint8_t *data;
    uint16_t checkSum;
    uint16_t calcSum;
};


class HexUtils
{
    public:
        HexUtils();
        virtual ~HexUtils();
        static vector<HexRecord> *ParseHexFile(string fileName,  string &debugInfo);

    protected:
    private:
};

#endif // HEXUTILS_H
