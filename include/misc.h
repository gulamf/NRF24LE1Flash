#ifndef MISC_H_INCLUDED
#define MISC_H_INCLUDED

#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

template<typename T>
std::string ToString(const T& v)
{
    std::ostringstream ss;
    ss << v;
    return ss.str();
}

template<typename T>
T FromString(const std::string& str)
{
    std::istringstream ss(str);
    T ret;
    ss >> ret;
    return ret;
}

template<typename T>
T ParseHex(const std::string& str) {
    T value = 0;
    for(unsigned int i=0;i<str.length();i++) {
        value = value << 4;
        if(str[i]>='0' && str[i]<='9') {
            int v = str[i]-'0';
            value += v;
        }
        else if(str[i]>='A' && str[i]<='F') {
            int v = str[i]-'A' + 10;
            value += v;
        }
    }
    return value;
}

std::string ToHexString(unsigned int value);


std::string strreplace(std::string &s,
                      const std::string &toReplace,
                      const std::string &replaceWith);

#if _POSIX_C_SOURCE >= 199309L
#include <time.h>   // for nanosleep
#else
#include <unistd.h> // for usleep
#endif

void sleep_ms(int milliseconds);

#endif // MISC_H_INCLUDED
