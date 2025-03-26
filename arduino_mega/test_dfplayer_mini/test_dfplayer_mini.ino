#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>

SoftwareSerial softSerial(/*RX*/10, /*TX*/11);
#define FPSerial softSerial

#define MP3_FOLDER "MP3"

#define MIN_VOLUME 0
#define MAX_VOLUME 30

DFRobotDFPlayerMini myDFPlayer;
String command;

String getStrSegmentByDelim(String originalStr, char delim, int order);
void helpMenu();
void printDetail(uint8_t type, int value);

void setup()
{
  FPSerial.begin(9600);
  Serial.begin(115200);

  Serial.println("Testing DFRobot DFPlayer Mini");
  Serial.println("Initializing DFPlayer ... (May take 3~5 seconds)");
  
  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) { 
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0);
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(20); 
}

void loop()
{
  if(Serial.available()){
    command = Serial.readStringUntil('\n');
    if(command.equals("help") || command.equals("h")) helpMenu();
    else if(command.startsWith("p")){
      int fileNum = getStrSegmentByDelim(command, ' ', 1).toInt();
      if(fileNum == 0) fileNum = 1;                                  // Defaults to playing the first track
      myDFPlayer.play(fileNum);
      Serial.print("Playing ");
      Serial.print(fileNum);
      Serial.println(".mp3");
    }
    else if(command.equals("|>")) myDFPlayer.start();
    else if(command.equals("||")) myDFPlayer.pause();
    else if(command.equals("<<")) myDFPlayer.previous();
    else if(command.equals(">>")) myDFPlayer.next();
    else if(command.startsWith("v")){
      int val = getStrSegmentByDelim(command, ' ', 1).toInt();
      if(val > MAX_VOLUME) val = MAX_VOLUME;
      else if(val < MIN_VOLUME) val = MIN_VOLUME;
      myDFPlayer.volume(val);
      Serial.print("Set volume to ");
      Serial.println(val);
    }
    else if(command.equals("gv")){
      Serial.print("Volume: ");
      Serial.println(myDFPlayer.readVolume());
    }
    else if(command.equals("gm")){
      Serial.print("Current mp3: ");
      Serial.print(myDFPlayer.readCurrentFileNumber());
      Serial.println(".mp3");
    }
    else{
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }
  if (myDFPlayer.available()) printDetail(myDFPlayer.readType(), myDFPlayer.read());
  delay(500);
}

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

void helpMenu(){
  Serial.println("The following commands are supported:");
  Serial.println("\t- \"p\" num               : play specified mp3 file (default is 1)");
  Serial.println("\t- \"|>\"                  : continue mp3 playback");
  Serial.println("\t- \"||\"                  : pause mp3 playback");
  Serial.println("\t- \"<<\"                  : go to previous mp3");
  Serial.println("\t- \">>\"                  : go to next mp3");
  Serial.println("\t- \"v\" num               : set the volume (integer)");
  Serial.println("\t- \"gv\"                  : get the current volume");
  Serial.println("\t- \"gm\"                  : get the current mp3 file number");
  Serial.println("\t- \"help\" or \"h\"         : bring up this menu");
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

