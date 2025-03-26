#include <RTClib.h>

#define AUTO_SET_RTC 0

/* 
    I^2C Pins
    +------------------+-------+-------+
    | Board            | SDA   | SCL   |
    +------------------+-------+-------+
    | Arduino Uno      | A4    | A5    | 
    | Arduino Mega     | D20   | D21   | 
    | Raspberry Pico 2 | GPIO4 | GPIO5 | 
    +------------------+-------+-------+
*/

RTC_DS3231 rtc;
String command;
char delim = ' ';

/* Time Functions */
void setDateTime(int year, int month, int day, int hr, int min, int sec);
int32_t getUnixTimestamp();

/* Command Functions */
String getStrSegmentByDelim(String originalStr, char delim, int order);

/* Print Functions */
void printTimestamp();
void printDateTime();
void helpMenu();

void setup(){
    Serial.begin(115200);
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        while (1);
    }

    if(AUTO_SET_RTC == 1) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    helpMenu();
}

void loop(){
    if(Serial.available()){
        command = Serial.readStringUntil('\n');
        command.trim();
        Serial.print("\nCommand: ");
        Serial.println(command);
        if(command.equals("help") || command.equals("h")) helpMenu();
        else if(command.startsWith("s")){
            String date = getStrSegmentByDelim(command, delim, 1);
            String time = getStrSegmentByDelim(command, delim, 2);
            int year = getStrSegmentByDelim(date, '/', 0).toInt();
            int month = getStrSegmentByDelim(date, '/', 1).toInt();
            int day = getStrSegmentByDelim(date, '/', 2).toInt();
            int hour = getStrSegmentByDelim(time, ':', 0).toInt();
            int minute = getStrSegmentByDelim(time, ':', 1).toInt();
            int second = getStrSegmentByDelim(time, ':', 2).toInt();
            setDateTime(year, month, day, hour, minute, second);
            printDateTime();
        }
        else if(command.equals("pdt")) printDateTime();
        else if(command.equals("pts")) printTimestamp();
        else if(command.equals("gt")){
            Serial.print("Temperature: ");
            Serial.print(rtc.getTemperature());
            Serial.println(" C");
        }
        else {
            Serial.print("Unknown command: ");
            Serial.print(command);
        }
    }
    delay(1000);
}

/*
    TIME FUNCTIONS
*/

void setDateTime(int year, int month, int day, int hr, int min, int sec){
    rtc.adjust(DateTime(year, month, day, hr, min, sec));
}

int32_t getUnixTimestamp(){ return rtc.now().unixtime(); }

/*
  COMMAND FUNCTIONS
*/

String getStrSegmentByDelim(String originalStr, char delim, int order){
    int delimCount = 0;
    int startIdx = 0;
    int endIdx = 0;
    for(unsigned int i = 0; i < originalStr.length(); i++){
      if(originalStr[i] == delim){
        ++delimCount;
        if(order < delimCount) break;
        startIdx = i+1;
        endIdx = i+1;
      } 
      else if(order == delimCount && originalStr[i] != delim) endIdx++;
    }
    if(startIdx < endIdx) return(originalStr.substring(startIdx, endIdx+1));
    else return "";
  }

/*
  PRINT FUNCTIONS
*/

void printTimestamp(){
    int32_t uts = getUnixTimestamp();
    Serial.print("Unix Timestamp [s]: ");
    Serial.println(uts);
}

void printDateTime(){
    DateTime now = rtc.now();
    Serial.print("Date & Time: ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.println(now.second(), DEC);
}

void helpMenu(){
    Serial.println("\nThe following commands are supported:");
    Serial.println("\t- \"s\" yyyy/mm/dd hh:mm:ss       : set date and time"); 
    Serial.println("\t- \"pdt\"                         : print date and time");
    Serial.println("\t- \"pts\"                         : print unix timestamp");
    Serial.println("\t- \"gt\"                          : print temperature");
    Serial.println("\t- \"help\" or \"h\"                 : bring up this menu");
}