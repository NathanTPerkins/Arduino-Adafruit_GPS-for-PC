#include "Adafruit_GPS.h"

char *Adafruit_GPS::build(char *_nmea, const char *_source, const char *_sentence, char ref, bool noCRLF){
    snprintf(_nmea, 2, "$");
    return _nmea;
}

void Adafruit_GPS::addChecksum(char *_buf){}