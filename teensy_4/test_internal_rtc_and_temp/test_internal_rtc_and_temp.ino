#include <TimeLib.h>
#include <InternalTemperature.h>

String command;
char delim = ' ';

/* Time Functions */
time_t getTeensy3Time();
int32_t getUnixTimestamp();
void setDateTime(int year, int month, int day, int hr, int min, int sec);

/* Command Functions */
String getStrSegmentByDelim(String originalStr, char delim, int order);

/* Print Functions */
void printTimestamp();
void printDateTime();
void helpMenu();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000 );
  setSyncProvider(getTeensy3Time);
  helpMenu();
}

void loop() {
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
        else if(command.equals("ct")){
            Serial.print("CPU Temp: ");
            Serial.print(InternalTemperature.readTemperatureC());
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

time_t getTeensy3Time() { return Teensy3Clock.get(); }
int32_t getUnixTimestamp() { return (int32_t)now(); }

void setDateTime(int year, int month, int day, int hr, int min, int sec){
   setTime(hr, min, sec, day, month, year);
}

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
    Serial.print("DateTime: ");
    Serial.print(year());
    Serial.print('/');
    Serial.print(month());
    Serial.print('/');
    Serial.print(day());
    Serial.print(" ");
    Serial.print(hour());
    Serial.print(':');
    Serial.print(minute());
    Serial.print(':');
    Serial.println(second());
}

void helpMenu(){
    Serial.println("\nThe following commands are supported:");
    Serial.println("\t- \"s\" yyyy/mm/dd hh:mm:ss       : set date and time"); 
    Serial.println("\t- \"pdt\"                         : print date and time");
    Serial.println("\t- \"pts\"                         : print unix timestamp");
    Serial.println("\t- \"ct\"                          : print processor temperature");
    Serial.println("\t- \"help\" or \"h\"                 : bring up this menu");
}