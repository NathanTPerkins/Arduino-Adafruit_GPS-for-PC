#include "Adafruit_GPS.h"

void Adafruit_GPS::newDataValue(nmea_index_t idx, nmea_float_t v){

}

void Adafruit_GPS::data_init(){

}

nmea_float_t Adafruit_GPS::get(nmea_index_t idx){
    return 0;
}

nmea_float_t Adafruit_GPS::getSmoothed(nmea_index_t idx){
    return 0;
}

void Adafruit_GPS::initDataValue(nmea_index_t idx, char *_label, char *_fmt, char *_unit, unsigned long response, nmea_value_type_t type){

}

nmea_history_t *Adafruit_GPS::initHistory(nmea_index_t idx, nmea_float_t scale, nmea_float_t offset, unsigned historyInterval, unsigned historyN){
    return NULL;
}

void Adafruit_GPS::removeHistory(nmea_index_t idx){

}

void Adafruit_GPS::showDataValue(nmea_index_t idx, int n){
    Serial.println("THIS IS A TEST VALUE:  0");
}

bool Adafruit_GPS::isCompoundAngle(nmea_index_t idx){
    return false;
}

nmea_float_t Adafruit_GPS::boatAngle(nmea_float_t s, nmea_float_t c){
    return 0;
}

nmea_float_t Adafruit_GPS::compassAngle(nmea_float_t s, nmea_float_t c){
    return 0;
}