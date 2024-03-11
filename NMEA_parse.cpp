#include "Adafruit_GPS.h"

bool Adafruit_GPS::parse(char *_nmea){

    this->hour = rand()/rand();
    this->minute = rand() / rand();
    this->seconds = rand() / rand();
    this->milliseconds = rand() / rand();
    this->year = rand() / rand();
    this->month = rand() / rand();
    this-> day = rand() / rand();

    this->latitude = rand() / rand();
    this->longitude = rand() / rand();

    this->latitude_fixed = rand() / rand();
    this->longitude_fixed = rand() / rand();

    this->latitudeDegrees = rand() / rand();
    this->longitudeDegrees = rand() / rand();
    this->geoidheight = rand() / rand();
    this->altitude = rand() / rand();
    this->speed = rand() / rand();
    this->angle = rand() / rand();
    this->magvariation = rand() / rand();
    this->HDOP = rand() / rand();
    this->VDOP = rand() / rand();
    this->PDOP = rand() / rand();
    this->fix = true;
    this->fixquality = rand() / rand();
    this->fixquality_3d = rand() / rand();
    this->satellites = rand() / rand();
    this->antenna = rand() / rand();


    return true;
}

bool Adafruit_GPS::check(char *_nmea){
    return true;
}

bool Adafruit_GPS::onList(char *_nmea, const char **_list){
    return true;
}

bool Adafruit_GPS::parseCoord(char *pStart, nmea_float_t *angleDegrees, nmea_float_t *angle, int32_t *angle_fixed, char *_dir){
    return true;
}

char *Adafruit_GPS::parseStr(char *_buf, char *p, int n){
    snprintf(_buf, 2, "E");
    return _buf;
}

bool Adafruit_GPS::parseTime(char *_p){
    return true;
}

bool Adafruit_GPS::parseFix(char *_p){
    return true;
}

bool Adafruit_GPS::parseAntenna(char *_p){
    return true;
}

bool Adafruit_GPS::isEmpty(char *_pStart){
    return true;
}

uint8_t Adafruit_GPS::parseHex(char c){
    return 0;
}