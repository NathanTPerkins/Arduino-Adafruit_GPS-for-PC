#include "Adafruit_GPS.h"

Adafruit_GPS::Adafruit_GPS(const char * filename){
    if(filename == NULL){
        this->random_values = true;
        srand(time(NULL));
        return;
    }
    this->random_values = false;
    
    #ifdef USING_CSV
    this->_filename = new char[strlen(filename) + 1];
    strncpy(this->_filename, filename, strlen(filename) + 1);
    this->sensor_data = new csv_parser::parser(filename, 10);
    this->file_index = 0;
    #else
    srand(time(NULL));
    #endif
    
    this->_running = false;
}

Adafruit_GPS::Adafruit_GPS(HardwareSerial* _serial, const char * filename){
    if(filename == NULL){
        this->random_values = true;
        srand(time(NULL));
        return;
    }
    this->random_values = false;
    
    #ifdef USING_CSV
    this->_filename = new char[strlen(filename) + 1];
    strncpy(this->_filename, filename, strlen(filename) + 1);
    this->sensor_data = new csv_parser::parser(filename, 10);
    this->file_index = 0;
    #else
    srand(time(NULL));
    #endif
    
    this->_running = false;
}

#ifdef USE_SW_SERIAL
Adafruit_GPS::Adafruit_GPS(SoftwareSerial* _serial, const char * filename){
    if(filename == NULL){
        this->random_values = true;
        srand(time(NULL));
        return;
    }
    this->random_values = false;
    
    #ifdef USING_CSV
    this->_filename = new char[strlen(filename) + 1];
    strncpy(this->_filename, filename, strlen(filename) + 1);
    this->sensor_data = new csv_parser::parser(filename, 10);
    this->file_index = 0;
    #else
    srand(time(NULL));
    #endif
    
    this->_running = false;
}
#endif

Adafruit_GPS::~Adafruit_GPS(){
    if(this->_filename != nullptr){
        delete this->_filename;
        this->_filename = nullptr;
    }
    #ifdef USING_CSV
    if(this->sensor_data != nullptr){
        delete this->sensor_data;
        this->sensor_data = nullptr;
    }
    #endif
    this->_running = false;
    this->paused = false;
}

bool Adafruit_GPS::begin(uint32_t baud_rate){
    this->_running = true;
    return true;
}

void Adafruit_GPS::common_init(){
    this->paused = false;
    this->recvdflag = false;
    hour = minute = seconds = year = month = day = fixquality = fixquality_3d = satellites = antenna = 0; // uint8_t
    lat = lon = mag = 0;          // char
    fix = false;                  // bool
    milliseconds = 0;             // uint16_t
    latitude = longitude = geoidheight = altitude = speed = angle = magvariation = HDOP = VDOP = PDOP = 0.0; 
}

size_t Adafruit_GPS::available(){
    #ifdef USING_CSV
    if(this->sensor_data != nullptr){
        return this->sensor_data->getSize();
    } 
    #endif
    return 1;
}

size_t Adafruit_GPS::write(uint8_t c){
    return 1;
}

char Adafruit_GPS::read(){
    return 'E';
}

void Adafruit_GPS::sendCommand(const char *_cmd){

}

bool Adafruit_GPS::newNMEAreceived(){
    return true;
}

void Adafruit_GPS::pause(bool p){
    this->paused = p;
}

char *Adafruit_GPS::lastNMEA(){
    recvdflag = false;
    return "TEST";
}

bool Adafruit_GPS::waitForSentence(const char *_wait, uint8_t max, bool usingInteruppts){
    return true;
}

bool Adafruit_GPS::LOCUS_StartLogger(){
    recvdflag = false;
    return true;
}

bool Adafruit_GPS::LOCUS_StopLogger(){
    recvdflag = false;
    return true;
}

bool Adafruit_GPS::LOCUS_ReadStatus(){
    return true;
}

bool Adafruit_GPS::standby(){
    return true;
}

bool Adafruit_GPS::wakeup(){
    return true;
}

nmea_float_t Adafruit_GPS::secondsSinceFix(){
    return millis();
}

nmea_float_t Adafruit_GPS::secondsSinceTime(){
    return millis();
}

nmea_float_t Adafruit_GPS::secondsSinceDate(){
    return millis();
}

void Adafruit_GPS::resetSentTime() {
    this->sentTime = millis();
}