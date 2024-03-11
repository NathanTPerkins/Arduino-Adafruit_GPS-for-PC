#ifndef ADAFRUIT_GPS_H

#ifndef NMEA_EXTRAS 
#ifndef ARDUINO_ARCH_AVR
#define NMEA_EXTENSIONS 
#endif
#else
#if (NMEA_EXTRAS > 0)
#define NMEA_EXTENSIONS
#endif
#endif
#define GPS_DEFAULT_I2C_ADDR  0x10 
#define GPS_MAX_I2C_TRANSFER  32 
#define GPS_MAX_SPI_TRANSFER  100                     
#define MAXLINELENGTH 120 
#define NMEA_MAX_SENTENCE_ID 20 
#define NMEA_MAX_SOURCE_ID 3 

#include "Arduino.h"
#ifdef USE_SW_SERIAL
#include <SoftwareSerial.h>
#endif
#include "Adafruit_PMTK.h"
#include "NMEA_data.h"

#ifdef USING_CSV
#include <CSVParser.h>
#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2
#define GYRO_X 3
#define GYRO_Y 4
#define GYRO_Z 5
#define TEMP 6
#endif

typedef enum {
  NMEA_BAD = 0,
  NMEA_HAS_DOLLAR = 1, 
  NMEA_HAS_CHECKSUM = 2,   
  NMEA_HAS_NAME = 4,       
  NMEA_HAS_SOURCE = 10,    
  NMEA_HAS_SENTENCE = 20,  
  NMEA_HAS_SENTENCE_P = 40 
} nmea_check_t;

class Adafruit_GPS /*: public Print*/ {
public:
    // Adafruit_GPS.cpp
    bool begin(uint32_t baud_or_i2caddr);

    #ifdef USE_SW_SERIAL
    Adafruit_GPS(SoftwareSerial *ser, const char *); 
    #endif
    Adafruit_GPS(HardwareSerial *ser, const char *); 
    Adafruit_GPS(const char *); 
    void common_init(void);
    virtual ~Adafruit_GPS();

    size_t available(void);
    size_t write(uint8_t);
    char read(void);
    void sendCommand(const char *);
    bool newNMEAreceived();
    void pause(bool b);
    char *lastNMEA(void);
    bool waitForSentence(const char *wait, uint8_t max = MAXWAITSENTENCE,
                        bool usingInterrupts = false);
    bool LOCUS_StartLogger(void);
    bool LOCUS_StopLogger(void);
    bool LOCUS_ReadStatus(void);
    bool standby(void);
    bool wakeup(void);
    nmea_float_t secondsSinceFix();
    nmea_float_t secondsSinceTime();
    nmea_float_t secondsSinceDate();
    void resetSentTime();

    // NMEA_parse.cpp
    bool parse(char *);
    bool check(char *nmea);
    bool onList(char *nmea, const char **list);
    uint8_t parseHex(char c);

    // NMEA_build.cpp
    #ifdef NMEA_EXTENSIONS
    char *build(char *nmea, const char *thisSource, const char *thisSentence,
                char ref = 'R', bool noCRLF = false);
    #endif
    void addChecksum(char *buff);

    // NMEA_data.cpp
    void newDataValue(nmea_index_t tag, nmea_float_t v);
    #ifdef NMEA_EXTENSIONS
    nmea_float_t get(nmea_index_t idx);
    nmea_float_t getSmoothed(nmea_index_t idx);
    void initDataValue(nmea_index_t idx, char *label = NULL, char *fmt = NULL, char *unit = NULL, unsigned long response = 0, nmea_value_type_t type = NMEA_SIMPLE_FLOAT);
    nmea_history_t *initHistory(nmea_index_t idx, nmea_float_t scale = 10.0, nmea_float_t offset = 0.0, unsigned historyInterval = 20, unsigned historyN = 192);
    void removeHistory(nmea_index_t idx);
    void showDataValue(nmea_index_t idx, int n = 7);
    bool isCompoundAngle(nmea_index_t idx);
    #endif
    nmea_float_t boatAngle(nmea_float_t s, nmea_float_t c);
    nmea_float_t compassAngle(nmea_float_t s, nmea_float_t c);

    int thisCheck = 0;
    char thisSource[NMEA_MAX_SOURCE_ID] = {0};
    char thisSentence[NMEA_MAX_SENTENCE_ID] = {0};
    char lastSource[NMEA_MAX_SOURCE_ID] = {0};
    char lastSentence[NMEA_MAX_SENTENCE_ID] = {0};

    uint8_t hour;          ///< GMT hours
    uint8_t minute;        ///< GMT minutes
    uint8_t seconds;       ///< GMT seconds
    uint16_t milliseconds; ///< GMT milliseconds
    uint8_t year;          ///< GMT year
    uint8_t month;         ///< GMT month
    uint8_t day;           ///< GMT day

    nmea_float_t latitude;  ///< Floating point latitude value in degrees/minutes
                            ///< as received from the GPS (DDMM.MMMM)
    nmea_float_t longitude; ///< Floating point longitude value in degrees/minutes
                            ///< as received from the GPS (DDDMM.MMMM)

    /** Fixed point latitude and longitude value with degrees stored in units of
    1/10000000 of a degree. See pull #13 for more details:
    https://github.com/adafruit/Adafruit-GPS-Library/pull/13 */
    int32_t latitude_fixed;  ///< Fixed point latitude in decimal degrees.
                            ///< Divide by 10000000.0 to get a double.
    int32_t longitude_fixed; ///< Fixed point longitude in decimal degrees
                            ///< Divide by 10000000.0 to get a double.

    nmea_float_t latitudeDegrees;  ///< Latitude in decimal degrees
    nmea_float_t longitudeDegrees; ///< Longitude in decimal degrees
    nmea_float_t geoidheight;      ///< Diff between geoid height and WGS84 height
    nmea_float_t altitude;         ///< Altitude in meters above MSL
    nmea_float_t speed;            ///< Current speed over ground in knots
    nmea_float_t angle;            ///< Course in degrees from true north
    nmea_float_t magvariation; ///< Magnetic variation in degrees (vs. true north)
    nmea_float_t HDOP; ///< Horizontal Dilution of Precision - relative accuracy
                        ///< of horizontal position
    nmea_float_t VDOP; ///< Vertical Dilution of Precision - relative accuracy
                        ///< of vertical position
    nmea_float_t PDOP; ///< Position Dilution of Precision - Complex maths derives
                        ///< a simple, single number for each kind of DOP
    char lat = 'X';    ///< N/S
    char lon = 'X';    ///< E/W
    char mag = 'X';    ///< Magnetic variation direction
    bool fix;          ///< Have a fix?
    uint8_t fixquality;    ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
    uint8_t fixquality_3d; ///< 3D fix quality (1, 3, 3 = Nofix, 2D fix, 3D fix)
    uint8_t satellites;    ///< Number of satellites in use
    uint8_t antenna;       ///< Antenna that is used (from PGTOP)

    uint16_t LOCUS_serial;  ///< Log serial number
    uint16_t LOCUS_records; ///< Log number of data record
    uint8_t LOCUS_type;     ///< Log type, 0: Overlap, 1: FullStop
    uint8_t LOCUS_mode;     ///< Logging mode, 0x08 interval logger
    uint8_t LOCUS_config;   ///< Contents of configuration
    uint8_t LOCUS_interval; ///< Interval setting
    uint8_t LOCUS_distance; ///< Distance setting
    uint8_t LOCUS_speed;    ///< Speed setting
    uint8_t LOCUS_status;   ///< 0: Logging, 1: Stop logging
    uint8_t LOCUS_percent;  ///< Log life used percentage

    #ifdef NMEA_EXTENSIONS
    // NMEA additional public variables
    nmea_datavalue_t
        val[NMEA_MAX_INDEX]; ///< an array of data value structs, val[0] = most
                            ///< recent HDOP so that ockam indexing works
    nmea_float_t depthToKeel =
        2.4; ///< depth from surface to bottom of keel in metres
    nmea_float_t depthToTransducer =
        0.0; ///< depth of transducer below the surface in metres

    char toID[NMEA_MAX_WP_ID] = {
        0}; ///< id of waypoint going to on this segment of the route
    char fromID[NMEA_MAX_WP_ID] = {
        0}; ///< id of waypoint coming from on this segment of the route

    char txtTXT[63] = {0}; ///< text content from most recent TXT sentence
    int txtTot = 0;        ///< total TXT sentences in group
    int txtID = 0;         ///< id of the text message
    int txtN = 0;          ///< the TXT sentence number
    #endif                   // NMEA_EXTENSIONS

    private:

    bool _running;
    bool random_values;
    char *_filename;
    int file_index;

    #ifdef USING_CSV
    csv_parser::parser* sensor_data;
    #endif

    // NMEA_data.cpp
    void data_init();
    // NMEA_parse.cpp
    const char *tokenOnList(char *token, const char **list);
    bool parseCoord(char *p, nmea_float_t *angleDegrees = NULL,
                    nmea_float_t *angle = NULL, int32_t *angle_fixed = NULL,
                    char *dir = NULL);
    char *parseStr(char *buff, char *p, int n);
    bool parseTime(char *);
    bool parseFix(char *);
    bool parseAntenna(char *);
    bool isEmpty(char *pStart);

    const char *sources[7] = {"II", "WI", "GP", "PG","GN", "P",  "ZZZ"};
    #ifdef NMEA_EXTENSIONS
    const char *sentences_parsed[21] = {"GGA", "GLL", "GSA", "RMC", "DBT", "HDM", "HDT", "MDA", "MTW", "MWV", "RMB", "TOP", "TXT", "VHW", "VLW", "VPW", "VWR", "WCV", "XTE", "ZZZ"}; ///< parseable sentence ids
    const char *sentences_known[15] = {"APB", "DPT", "GSV", "HDG", "MWD", "ROT", "RPM", "RSA", "VDR", "VTG", "ZDA", "ZZZ"}; ///< known, but not parseable
    #else
    const char *sentences_parsed[6] = {"GGA", "GLL", "GSA", "RMC", "TOP", "ZZZ"};
    const char *sentences_known[4] = {"DBT", "HDM", "HDT", "ZZZ"};
    #endif

    // Make all of these times far in the past by setting them near the middle of
    // the millis() range. Timing assumes that sentences are parsed promptly.
    uint32_t lastUpdate =
        2000000000L; ///< millis() when last full sentence successfully parsed
    uint32_t lastFix = 2000000000L;  ///< millis() when last fix received
    uint32_t lastTime = 2000000000L; ///< millis() when last time received
    uint32_t lastDate = 2000000000L; ///< millis() when last date received
    uint32_t recvdTime =
        2000000000L; ///< millis() when last full sentence received
    uint32_t sentTime = 2000000000L; ///< millis() when first character of last
                                    ///< full sentence received
    bool paused;

    uint8_t parseResponse(char *response);
    #ifdef USE_SW_SERIAL
    SoftwareSerial *gpsSwSerial;
    #endif
    bool noComms = false;
    HardwareSerial *gpsHwSerial;
    // Stream *gpsStream;
    // TwoWire *gpsI2C;
    // SPIClass *gpsSPI;
    int8_t gpsSPI_cs = -1;
    // SPISettings gpsSPI_settings =
    //     SPISettings(1000000, MSBFIRST, SPI_MODE0); // default
    char _spibuffer[GPS_MAX_SPI_TRANSFER]; // for when we write data, we need to
                                            // read it too!
    uint8_t _i2caddr;
    char _i2cbuffer[GPS_MAX_I2C_TRANSFER];
    int8_t _buff_max = -1, _buff_idx = 0;
    char last_char = 0;

    volatile char line1[MAXLINELENGTH]; ///< We double buffer: read one line in
                                        ///< and leave one for the main program
    volatile char line2[MAXLINELENGTH]; ///< Second buffer
    volatile uint8_t lineidx = 0; ///< our index into filling the current line
    volatile char *currentline;   ///< Pointer to current line buffer
    volatile char *lastline;      ///< Pointer to previous line buffer
    volatile bool recvdflag;      ///< Received flag
    volatile bool inStandbyMode;  ///< In standby flag
};


#endif