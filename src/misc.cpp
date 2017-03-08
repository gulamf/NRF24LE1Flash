#include "misc.h"

std::string ToHexString(unsigned int value) {
    char str[8];
    sprintf(str,"%X",value);
    return std::string(str);
}

std::string strreplace(std::string &s,
                      const std::string &toReplace,
                      const std::string &replaceWith)
{
    return(s.replace(s.find(toReplace), toReplace.length(), replaceWith));
}

void sleep_ms(int milliseconds)
{
#if _POSIX_C_SOURCE >= 199309L
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
#else
    usleep(milliseconds * 1000);
#endif
}
