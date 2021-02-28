

// final sketch PresSens GPS
// 18.11.2020

/*
Pressure and Movement Sensor 

30 bar Pressure Sensor          BlueRobotics MS5837-30BA
9 axis IMU                      SPARKFUN IMU Breakout - MPU 9250
GPS NEO 6MV2                    

Louis Rautmann
University of Rostock

*/


// Bibliotheken
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
//#include <SdFat.h>
#include <SD.h>
#include <SPI.h>
#include "MPU9250.h"
#include "MS5837.h"

// Blue-Robotics Pressure Sensor
MS5837 Psensor;

// IMU Setup
// creating MPU object
MPU9250 IMU(Wire,0x68);
int status;


// Grundeinstellungen Sensorik
// genutzt falls keine Config vorhanden

// IMU settings:
int AccCurrent    = 8;      // Accelerometer Range
int GyrCurrent    = 500;    // Gyroscope Range
int DLPFCurrent   = 20;     // Digital Low Pass Filter Bandwidth
int SRDCurrent    = 19;     // Sample Rate Devider

// Pressure settings
int FluidDensity  = 1025; //[kg/mÂ³]


// SD Setup

//char* DataFileName = "datalog.txt";
String SaveGPSTime;
String SaveGPSData;
String SaveGPSDate;
File myFile;
const int chipSelect = BUILTIN_SDCARD;


// CONFIG

String S_Configdate;
String ConfigDate;
String String_ConfigDate = "last saved: ";
String Acc_configStr = "Accelerometer Sensitivity: ";
String Gyr_configStr = "Gyroscope Sensitivity: ";
String IMU_DLPF_configStr ="IMU Digital Low Pass Filter Bandwith: ";
String IMU_SRD_configStr = "IMU Sample Rate Devider: ";
String FluidDensity_configStr = "Fluid Density [kg/m^3]: ";
String GPSOutrun_Str = "GPS Outrun Time [s]: ";
char inputChar;
char inputString [1280];
String InputString;
String x;

//for reading of config

int stepToLoad = 1;
int endOfLineNumber = 0;
int stepCount = 0;
int stringIndex = 0;
int Sindex ;
int i;

// GPS Setup

int GPS_RX    = 0;
int GPS_TX    = 1;
int GPS_Baud  = 9600;
int GPS_Power = 32;
int gpsCount = 0;
int gpsOutrun = 30;             // seconds
bool gpsEndPermission = false;
int EndPressure = 1085;         // millibar -> Grenze zum sicherstellen des Abtauchens
elapsedMillis GPStrytime;


// creating TinyGPSPlus Object
TinyGPSPlus gps;
//creating Software Serial port for GPS
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);



// Status LEDs
bool GPSLED_Permission = true;
int GPS_LED = 5;
int IMU_LED = 7;
int Pressure_LED = 9;
int LED_OFF = 20;
unsigned long int LEDcounter = 0;


// STARTUP

int startupTime = 10; // seconds


// LOOP
// default

bool automode = true; 
bool accessmainmenu = false;
bool GPSpermission = true;
bool Headerpermission = true;

elapsedMillis timerOutput;
int LogOut = 1;     // Messfrequenz in seconds per measurement
int freq   = 2;     // Frequenz 2 Hz
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long periodtry = 500;
// MENU

char openmenu;
char menu = 'm';
char ans;


// RTC

tmElements_t tm;
tmElements_t tmMan;
bool TimeManual = false;
int t_num;
String TimeStr;
String TimeStrMan;
int Year, Month, Day, Hour, Minute, Second ;
int t_numMan;




void setup() {


  Serial.begin(115200);
  //while(!Serial);               // comment out if standalone (no-pc)
  
  pinMode(GPS_LED,OUTPUT);
  pinMode(IMU_LED,OUTPUT);
  pinMode(GPS_Power,OUTPUT);
  pinMode(Pressure_LED,OUTPUT);


  SDSetup();                        // Initialisiere SD-Karte
  ReadConfig();                     // Lese Config falls vorhanden
  startup();                        // Start-Up Countdown zur UnterdrÃ¼ckung der automatischen Messung

  digitalWrite(GPS_Power,HIGH);     // GPS Stromversorgung An (Ã¼ber MOSFET)
  gpsSerial.begin(GPS_Baud);        // Serielle Verbindung mit GPS Modul
  GPStrytime = 0;
  getGPS_data();                    // GPS Daten Encoden
    do {getGPS_data();              // GPS Daten beziehen bis ein zulÃ¤ssige Ortsangabe vorhanden ist
    if(GPStrytime/1000 % 2 == 0){
      digitalWrite(GPS_LED,HIGH);
    }else{digitalWrite(GPS_LED,LOW);}
    if (GPStrytime>gpsOutrun*1000){        // 20sec  probieren, sonst abbrechen
      Serial.println("");
      Serial.println("GPS Time Outrun");
      Serial.println("");
       break;
       
    }
    //gpsCount=gpsCount+1;
    
    }while(!gps.location.isValid());
    gpsCount = 0;
      
  GPS_data();                       // Rohdatenbehandlung GPS
  GPS_save();                      // GPS Daten in "datalog.txt" sichern
  digitalWrite(GPS_Power,LOW);      // Stromversorgung CGPS Modul AUS
  IMU_setup();                      // IMU initialisieren
  Psensor_setup();                  // Drucksensor initialisieren
  timerOutput = 0;
  startMillis = millis();
  //Write_Header();
  // Schreibe neuen Header in "datalog.txt"
}

void loop() {
  currentMillis = millis();
    if (automode == false && accessmainmenu == true){
      showmainmenu();               // HauptmenÃ¼               
    }else if (automode == true && accessmainmenu == false){
      
      if (currentMillis - startMillis >= periodtry){//timerOutput >= LogOut * (1000*2)
         
        if (GPSpermission == true ){ 
          
          digitalWrite(GPS_Power,HIGH);
          GPStrytime = 0;
          getGPS_data();
            do {getGPS_data();
              if(GPStrytime/1000 % 2 == 0){
                    digitalWrite(GPS_LED,HIGH);
              }else{digitalWrite(GPS_LED,LOW);}
              if (GPStrytime>gpsOutrun*1000){        // 20sec  probieren, sonst abbrechen
                    Serial.println("");
                    Serial.println("GPS Time Outrun");
                    Serial.println("");
                    break;
       
              }
            }while(!gps.time.isUpdated());
          GPS_data();
          GPS_save();
          digitalWrite(GPS_Power,LOW);
          GPSpermission = false;
          if(gpsEndPermission == true){
          gpsEndPermission = false;
          //mwhile(1);
          }
        }

        if (Headerpermission == true){
          Write_Header();
          Headerpermission = false;
          }
        
        data2serial();              // serieller Output
        data2log();                 // IMU & Druckdaten in "datalog.txt"
        gps_end();                  // GPS Messung beim Auftauchen des Sensors

        LEDcounter = LEDcounter +1;
        if(LEDcounter>LED_OFF){
          digitalWrite(GPS_LED,LOW);
          digitalWrite(IMU_LED,LOW);
          digitalWrite(Pressure_LED, LOW);
        }
        
       // timerOutput = timerOutput - 500;    //LogOut*(1000*2)
       timerOutput = 0;
    
       startMillis = currentMillis;
      }

    }

    // Automatische Messung durch String 'menu' unterbrechen

    openmenu = Serial.read(); 
//    openmenu.trim();

    if (openmenu == menu){
      automode = false;
      accessmainmenu = true;
    }


}


void Psensor_setup(){
  Wire.begin();
  digitalWrite(Pressure_LED,HIGH);
  while(!Psensor.init()){
    Serial.println("Initialization of Pressure Sensor failed");
    Serial.println("try recycling power");
    while(1) {digitalWrite(Pressure_LED,HIGH);
              delay(1000);
              digitalWrite(Pressure_LED,LOW);
              delay(1000);};
  }
  
  Psensor.setModel(MS5837::MS5837_30BA);
  Psensor.setFluidDensity(FluidDensity);
  
}



void startup(){
  Serial.println("================ Pressure & Movement Sensor ================");
  
  timesetup();
  DigitalClockDisplay();
  Serial.print("automatic measurement starts in ");
  Serial.print(startupTime);
  Serial.println(" seconds");
  Serial.println("to avoid press any key");
  Serial.println("****** you can abort automatic measurement any time by sending 'm' ******");
  while(Serial.available()<=0){
    for (int countdown = 0; countdown <= startupTime; countdown++){
      if(Serial.available()>0){
        Serial.read();
      automode = false;
      accessmainmenu = true;
        break;
      }
      Serial.print(startupTime - countdown);
      Serial.print("\t");
      delay(1000);
    }
    if(Serial.available() <= 0){
      Serial.println();
      Serial.println("starting automatic measurement mode");
      break;
    }
  }
}

void SDSetup() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void IMU_setup(){
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {digitalWrite(IMU_LED,HIGH);
              delay(1000);
              digitalWrite(IMU_LED,LOW);
              delay(1000);}
  }
  // Das wollen wir spÃ¤ter aus der CONFIG Lesen===============================================================
  
  setAcc();
  setGyr();
  setDLPF();
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(SRDCurrent);
    if (status > 0){
      digitalWrite(IMU_LED,HIGH);
      delay(1000);
    }
  }
//============================================================================================================


void getGPS_data(){
  while (gpsSerial.available() > 0)

  // get GPS data
    if (gps.encode(gpsSerial.read()));
  
}
void GPS_data(){
  //serielle Verbindung am Start?
  
     if (gps.date.isValid())
  {
    SaveGPSDate = "";
    SaveGPSDate += gps.date.day();
    SaveGPSDate += ".";
    SaveGPSDate += gps.date.month();
    SaveGPSDate += ".";
    SaveGPSDate += gps.date.year();
   
  } else {
    
    Serial.println("no date");
    
  }
    if (gps.time.isValid())
  {
    //SaveGPSTime += " ; ";
    SaveGPSTime ="";
    SaveGPSTime += gps.time.hour();
    SaveGPSTime += ":";
    if (gps.time.minute() < 10)
      SaveGPSTime += "0";
    SaveGPSTime += gps.time.minute();
    SaveGPSTime += ":";
    if (gps.time.second() < 10)
      SaveGPSTime += "0";
    SaveGPSTime += gps.time.second();
    
  }else {
    Serial.println("no time");
  }
if (gps.location.isValid())
  {
    SaveGPSData = "";
    //SaveGPSData += "Lat; ";
    
    SaveGPSData += gps.location.lat();
    //SaveGPSData += "; Long; ";
    SaveGPSData += " ; ";
    SaveGPSData += gps.location.lng();
    //SaveGPSData += "; Alt: ";
    SaveGPSData += " ; ";
    SaveGPSData += gps.altitude.meters();
     if (TimeManual == false && gps.time.isValid() ){
      tm.Year   = CalendarYrToTm(gps.date.year());
      tm.Month  = gps.date.month();
      tm.Day    = gps.date.day();
      tm.Hour    = gps.time.hour();
      tm.Minute  = gps.time.minute();
      tm.Second  = gps.time.second();
      t_num      = makeTime(tm);
      setTime(t_num);
      Teensy3Clock.set(t_num);
      Serial.println();
      Serial.print("RTC was set by GPS: ");
      DigitalClockDisplay();
    }
  
  }else {
    Serial.println("no location");
    GPSLED_Permission = false;
  }
    //delay(10000);
}


void gps_end(){
  if (Psensor.pressure()>EndPressure && gpsEndPermission == false){
    gpsEndPermission = true;
  }
  if (gpsEndPermission == true && Psensor.pressure()<EndPressure){ // Sensor wieder aufgetaucht
    GPSpermission = true;
  }
}



void timesetup() {
  setSyncProvider(getTeensy3Time);
  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
}
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
void DigitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void getTime(){
  TimeStr = "";
  TimeStr += day();
  TimeStr += ".";
  TimeStr += month();
  TimeStr += ".";
  TimeStr += year();
  TimeStr += ";";
  TimeStr += hour();
  TimeStr += ":";
  if (minute() < 10)
      TimeStr += "0";
  TimeStr += minute();
  TimeStr += ":";
  if (second() < 10)
      TimeStr += "0";
  
  TimeStr += second();
  //only for 2Hz freq
  if(LEDcounter%2==0){
    TimeStr +=".0";
    }else{TimeStr +=".5";}
  
  
}

void showmainmenu(){
  Serial.read();
  Serial.println();
  Serial.println(F("================ Pressure & Movement Sensor ================"));
  Serial.println(F("             =========== MAIN MENU ==========="));
  Serial.println("[1] start automatic measurement mode");
  Serial.println("[2] get GPS position");
  Serial.println("[3] read from SD");
  Serial.println("[4] settings");
  while(!Serial.available());{
    mainmenu(Serial.read());
  }
}

void mainmenu(char i){
  switch(i){
    case '1':
      automode         = true;
      accessmainmenu   = false;
      GPSpermission    = true;
      Headerpermission = true;
      break;
    case'2':
      digitalWrite(GPS_Power,HIGH);
      delay(4000);
      getGPS_data();
      GPStrytime = 0;
        do {getGPS_data();
          if(GPStrytime/1000 % 2 == 0){
            digitalWrite(GPS_LED,HIGH);
            }else{digitalWrite(GPS_LED,LOW);}
           if (GPStrytime>gpsOutrun*1000){        // 20sec  probieren, sonst abbrechen
             Serial.println("");
             Serial.println("GPS Time Outrun");
             Serial.println("");
             break;
       
    }}while(!gps.time.isUpdated());
      GPS_data();
      GPS_save();
      digitalWrite(GPS_Power,LOW);
      digitalWrite(GPS_LED,LOW);
      showmainmenu();
      break;
    case'3':
      Serial.println("====== read from SD card ======");
      Serial.println("[1] read config.txt");
      Serial.println("[2] read datalog.txt");
      Serial.println("[3] back to main menu");
      while(!Serial.available());{
        readSD(Serial.read());
      }
      break;
    case '4':
      Serial.println("====== SETTINGS ======");
      Serial.println("[1] GPS settings");
      Serial.println("[2] Pressure settings");
      Serial.println("[3] IMU settings");
      Serial.println("[4] Clock settings");
      Serial.println("[5] back to main menu");
      while(!Serial.available());{
        settings(Serial.read());
      }
      
  }
}

void readSD(char j){
  // hier SDcard read -> 1 read config 2 read datalog 3 backtomainmenu
  switch(j){
    case '1':
      readconfig();
      break;
    case '2':
      readdatalog();
      break;
    case '3':
      showmainmenu();
      break;
  }
}

void settings(char k){
  // hier settings
  switch(k) {
    case '1':
      Serial.println("====== GPS Settings ======");
      Serial.println("[1] set GPS Outrun Time");
      Serial.println("[2] back");
      Serial.println("[3] back to main menu");
      while(!Serial.available());{
        settingsGPS(Serial.read());
      }
      break;
    case '2':
      Serial.println("====== Pressure Settings ======");
      Serial.println("[1] set fluid density");
      Serial.println("[2] back");
      Serial.println("[3] back to main menu");
      while(!Serial.available());{
        settingsPressure(Serial.read());
      }
      break;
    case '3':
      Serial.println("===== IMU Settings ======");
      Serial.println("[1] Accelerometer Settings");
      Serial.println("[2] Gyroscope Settings");
      Serial.println("[3] DLPF and SRD settings");
      Serial.println("[4] back");
      Serial.println("[5] back to main menu");
      while(!Serial.available());{
        settingsIMU(Serial.read());
      }
      break;
    case '4':
      Serial.println("====== Clock Settings ======");
      Serial.println("[1] set by GPS");
      Serial.println("[2] set manually");
      Serial.println("[3] back");
      Serial.println("[4] back to main menu");
      while(!Serial.available());{
        settingsClock(Serial.read());
      }
      break;
    case'5':
      showmainmenu();
      break;
  }
}

void settingsClock(char r){
  switch(r){
    case'1':
    TimeManual = false;
      getGPS_data();
      GPStrytime = 0;
        do {getGPS_data();
          if(GPStrytime/1000 % 2 == 0){
            digitalWrite(GPS_LED,HIGH);
          }else{digitalWrite(GPS_LED,LOW);}
          if (GPStrytime>gpsOutrun*1000){        // 20sec  probieren, sonst abbrechen
            Serial.println("");
            Serial.println("GPS Time Outrun");
            Serial.println("");
            break;
       
          }
        }while(!gps.location.isValid());
      GPS_data();
  
      break;
    case'2':
      TimeManual = true;
      Serial.read(); // delete buffer
      Serial.print("Current Time: ");
      DigitalClockDisplay();
      Serial.println("set RTC in following format: dd/MM/yyyy hh:mm:ss");
      while(!Serial.available())
        delay(100);
        TimeStrMan = Serial.readString();
        sscanf(TimeStrMan.c_str(), "%d/%d/%d %d:%d:%d", &Day, &Month, &Year, &Hour, &Minute, &Second);tmMan.Year    = CalendarYrToTm(Year);
        tmMan.Month   = Month;
        tmMan.Day     = Day;
        tmMan.Hour    = Hour;
        tmMan.Minute  = Minute;
        tmMan.Second  = Second;
        t_numMan      = makeTime(tmMan);
        setTime(t_numMan);
        Teensy3Clock.set(t_numMan);
        delay(10);
        Serial.read();
        delay(10);
        Serial.println("RTC was set manually to: ");
        DigitalClockDisplay();
      break;
    case'3':
      mainmenu('4');
      break;
    case'4':
      showmainmenu();
      break;
  }
}

void settingsIMU(char n){
  switch(n) {
    case'1':
      Serial.println("====== Accelerometer Settings ======");
      Serial.println("[1] set Accelerometer Sensitivity");
      Serial.println("[2] back");
      Serial.println("[3] back to main menu");
      while(!Serial.available());{
        settingsAcc(Serial.read());
      }
      break;
    case'2':
      Serial.println("====== Gyroscope Settings ======");
      Serial.println("[1] set Gyroscope Sensitivity");
      Serial.println("[2] back");
      Serial.println("[3] back to main menu");
      while(!Serial.available());{
        settingsGyr(Serial.read());  
      }
      break;
    case'3':
      Serial.println("====== DLPF and SRD Settings ======");
      Serial.println("DLPF - Digital Low Pass Filter Bandwith");
      Serial.println("SRD  - Sample Rate Divider");
      Serial.println("Data Output Rate = 1000 / (1 + SRD)");
      Serial.println("Note:  In order to prevent aliasing, the data should be sampled at twice the frequency of the DLPF bandwidth or higher.");
      Serial.println("Example: DLPF Bandwith = 41 Hz -> Output Rate >= 82 Hz -> SRD = max 11");
      Serial.println("[1] set DLPF");
      Serial.println("[2] set SRD");
      Serial.println("[3] back");
      Serial.println("[4] back to main menu");
      while(!Serial.available());{
        settingsBW(Serial.read());
      }
      break;
    case'4':
      mainmenu('4');
      break;
    case'5':
      showmainmenu();
      break;
  }
}

void settingsBW(char q){
  switch(q){
    case'1':
      Serial.println("====== set Digital Low Pass Filter Bandwith ======");
      Serial.print("Current: ");
      Serial.print(DLPFCurrent);
      Serial.println(" Hz");
      Serial.println("choose Range [5; 10; 20; 41; 92; 184]");
      while(!Serial.available());{
        DLPFCurrent = Serial.parseInt();
      }
      setDLPF();
      
      Serial.print("DLPF Bandwidth changed to: ");
      Serial.print(DLPFCurrent);
      Serial.println(" Hz");
      Serial.println();
      Serial.println("save to CONFIG? [y/n]");
      while(!Serial.available());{
         ans = Serial.read();
      }
      if (ans == 'y'){
        WriteConfig();
        Serial.println("SAVED TO CONFIG ");
      }else {
        Serial.println("not saved");
      }
      settingsIMU('3');
      break;
    case '2':
      Serial.println("====== set Sample Rate Divider (SRD) ======");
      Serial.print("Current: ");
      Serial.println(SRDCurrent);
      Serial.println("standard = 19 for 50 Hz Sample Rate");
      while(!Serial.available());{
        SRDCurrent = Serial.parseInt();
      }
      IMU.setSrd(SRDCurrent);
      Serial.print("SRD value changed to: ");
      Serial.println(SRDCurrent);
      Serial.println();
      Serial.println("save to CONFIG? [y/n]");
      while(!Serial.available());{
         ans = Serial.read();
      }
      if (ans == 'y'){
        WriteConfig();
        Serial.println("SAVED TO CONFIG ");
      }else {
        Serial.println("not saved");
      }
      settingsIMU('3');
      break;
    case'3':
      settings('3');
      break;
    case'4':
      showmainmenu();
      break;
  }
}

void settingsGyr(char p){
  switch(p){
    case'1':
       Serial.println("====== set Gyr. Sensitivity ======");
      Serial.print("Current: ");
      Serial.print(GyrCurrent);
      Serial.println(" deg/s");
      Serial.println("choose Range [250; 500; 1000; 2000]");
      while(!Serial.available());{
        GyrCurrent = Serial.parseInt();
      }
      setGyr();
      Serial.print("Gyroscope Range changed to: ");
      Serial.print(GyrCurrent);
      Serial.println(" deg/s");
      Serial.println();
      Serial.println("save to CONFIG? [y/n]");
      while(!Serial.available());{
         ans = Serial.read();
      }
      if (ans == 'y'){
        WriteConfig();
        Serial.println("SAVED TO CONFIG ");
      }else {
        Serial.println("not saved");
      }

      settingsIMU('2');
      break;
    case'2':
      settings('3');
      break;
    case'3':
      showmainmenu();
      break;
  }
}

void settingsAcc(char o){
  switch(o) {
    case'1':
      Serial.println("====== set Acc. Range ======");
      Serial.print("Current: ");
      Serial.print(AccCurrent);
      Serial.println(" G");
      Serial.println("choose Range [2; 4; 8; 16]");
      while(!Serial.available());{
        AccCurrent = Serial.parseInt();
      }
      setAcc();
      
      
      Serial.print("Accelerometer range changed to: ");
      Serial.print(AccCurrent);
      Serial.println(" G");
      Serial.println();
      Serial.println("save to CONFIG? [y/n]");
      while(!Serial.available());{
         ans = Serial.read();
      }
      if (ans == 'y'){
        WriteConfig();
        Serial.println("SAVED TO CONFIG");
      }else {
        Serial.println("not saved");
      }
      settingsIMU('1');
      break;
    case'2':
      settings('3');
      break;
    case'3':
      showmainmenu();
      break;
  }
  
}

void settingsPressure(char m){
  switch (m) {
    case '1':
      Serial.println("Set Fluid Density");
      Serial.print("Current: ");
      Serial.print(FluidDensity);
      Serial.println(" kg/m^3");
      while(!Serial.available());{
        FluidDensity = Serial.parseInt();
      }
      Psensor.setFluidDensity(FluidDensity);

      Serial.print("Fluid Density changed to: ");
      Serial.print(FluidDensity);
      Serial.println(" kg/m^3");
      Serial.println();
      Serial.println("save to CONFIG? [y/n]");
      while(!Serial.available());{
         ans = Serial.read();
      }
      if (ans == 'y'){
        WriteConfig();
        Serial.println("SAVED TO CONFIG");
      }else {
        Serial.println("not saved");
      }
      
      break;
    case '2':
      mainmenu('4');
      break;
    case '3':
      showmainmenu();
      break;
  }
}

void settingsGPS(char l){
  switch(l) {
    case '1':
      Serial.println();
      Serial.println("set GPS Outrun Time");
      Serial.print("Current: ");
      Serial.print(gpsOutrun);
      Serial.println(" sec");
      Serial.println();
      Serial.println("Note: GPS Cold Start is 27 s ");
      Serial.println();
      while(!Serial.available());{
        gpsOutrun = Serial.parseInt();
      }
      Serial.println();
      Serial.print("GPS Outrun Time was set to: ");
      Serial.print(gpsOutrun);
      Serial.println(" sec");
      Serial.println("save to CONFIG? [y/n]");
      while(!Serial.available());{
         ans = Serial.read();
      }
      if (ans == 'y'){
        WriteConfig();
        Serial.println("SAVED TO CONFIG");
      }else {
        Serial.println("not saved");
      }
      break;
    case '2':
      mainmenu('4'); 
      break;
    case '3':
      showmainmenu();
      break;
  }
}

void readconfig(){
  myFile = SD.open("config.txt",FILE_READ);
  if(myFile){
    Serial.println();
    Serial.println("========================================================");
    Serial.println("===================Config.TXT==========================");
    Serial.println("========================================================");
    Serial.println();  
    while(myFile.available()){
      Serial.write(myFile.read());
    }
    myFile.close();
    Serial.println("========================================================");
    Serial.println("=================== END OF FILE=========================");
    Serial.println("========================================================");
    Serial.println();
  }else {
    Serial.println("error openening config");
  }
}

void readdatalog(){
  myFile = SD.open("datalog.txt",FILE_READ);
  if(myFile){
    Serial.println();
    Serial.println("========================================================");
    Serial.println("===================DATALOG.TXT==========================");
    Serial.println("========================================================");
    Serial.println();
    while(myFile.available()){
      Serial.write(myFile.read());
    }
    myFile.close();
    Serial.println("========================================================");
    Serial.println("=================== END OF FILE=========================");
    Serial.println("========================================================");
    Serial.println();
  }else {
    Serial.println("error openening datalog");
  }
}

void data2serial(){
    getTime();
  Serial.print(TimeStr);
  Serial.print("\t");
  Psensor.read();
  Serial.print("Pressure: ");
  Serial.print(Psensor.pressure());
  Serial.print(" mbar \t");
  Serial.print("Temp: ");
  Serial.print(Psensor.temperature());
  Serial.print(" deg C \t");
  // als erstes Timestamp + Druck

   IMU.readSensor();
  Serial.print("AccX: ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
    Serial.print("; AccY: ");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
    Serial.print("; AccZ: ");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");

  Serial.print("; GyroX: ");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
    Serial.print("; GyroY: ");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
    Serial.print("; GyroZ: ");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");

  Serial.print("; MagnX: ");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
    Serial.print("; MagnY: ");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
    Serial.print("; MagnZ: ");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.println("\t");
  }

void data2log(){
  myFile = SD.open("datalog.txt", FILE_WRITE);
  if(myFile) {
    //Serial.println("datalog.txt found");
    //Build GPS Time String
    getTime();
        myFile.print(TimeStr);
    myFile.print(" ; ");
        myFile.print(Psensor.pressure());
    myFile.print(" ; ");
        myFile.print(Psensor.temperature());
    myFile.print(" ; ");        
        myFile.print(IMU.getAccelX_mss(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getAccelY_mss(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getAccelZ_mss(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getGyroX_rads(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getGyroY_rads(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getGyroZ_rads(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getMagX_uT(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getMagY_uT(),6);
    myFile.print(" ; ");
        myFile.print(IMU.getMagZ_uT(),6);
    myFile.println(" ; ");
    myFile.close();
    //Serial.println("data saved");
  } else {
    Serial.print("error opening datalog:");
//    Serial.println(DataFileName);
    }

  }


void Write_Header(){
   Serial.println();
   Serial.print("Initializing ");
   //Serial.print(DataFileName);
   Serial.println("... writing header");
    myFile = SD.open("datalog.txt", FILE_WRITE);
    if(myFile){
   myFile.println("//HEADER START//");
   myFile.println("Pressure and IMU Sensor");
   myFile.print("Datetime; Pressure[bar]; Temperature[deg]; ");
   myFile.print("AccX[m/s/s]; AccY[m/s/s]; AccZ[m/s/s]; ");
   myFile.print("GyroX[rad/s]; GyroY[rad/s]; GyroZ[rad/s]; ");
   myFile.println("MagnX[uT]; MagnY[uT]; MagnZ[uT]");
   myFile.println("[DATA]");
   myFile.close();
   
  Serial.println("header made");
  
    }else {
    Serial.print("error opening ");
    //Serial.println(DataFileName);
  }
}
void GPS_save(){

  
 myFile = SD.open("datalog.txt", FILE_WRITE);
//Serial.println(logFile);
  if (myFile) {
    Serial.println("datalog.txt found");
    //Build GPS Time String
   
       //save GPS String
      // myFile.println("testing 1, 2, 3.");
    myFile.println("[GPS]");
    myFile.print(SaveGPSDate);
    myFile.print(" ; ");
    myFile.print(SaveGPSTime);
    myFile.print(" ; ");
    myFile.println(SaveGPSData);
    myFile.close();
    Serial.println(SaveGPSDate);
    Serial.println(SaveGPSTime);
    Serial.println(SaveGPSData);
    digitalWrite(GPS_LED,HIGH);
    if (GPSLED_Permission == false){
       digitalWrite(GPS_LED,LOW);
    }
     
    SaveGPSDate   = "";
    SaveGPSTime   = "";
    SaveGPSData   = "";
    Serial.println("GPS Data saved");
    
  } else {
    Serial.print("error opening datalog:");
//    Serial.println(DataFileName);
    }

}

// Config
void WriteConfig(){
  File ConfigFile = SD.open("config.txt", FILE_WRITE);
  if (ConfigFile){
    ConfigDate += day();
    ConfigDate += ".";
    ConfigDate += month();
    ConfigDate += ".";
    ConfigDate += year();
  } else {
    Serial.println("error openening config.txt");
  }
  ConfigFile.seek(0); // start of File
  ConfigFile.println("//CONFIG START //");
  ConfigFile.print(String_ConfigDate);
  ConfigFile.println(ConfigDate);
  ConfigFile.println();
  ConfigFile.print(Acc_configStr);
  ConfigFile.println(AccCurrent);
  ConfigFile.print(Gyr_configStr);
  ConfigFile.println(GyrCurrent);
  ConfigFile.print(IMU_DLPF_configStr);
  ConfigFile.println(DLPFCurrent);
  ConfigFile.print(IMU_SRD_configStr);
  ConfigFile.println(SRDCurrent);
  ConfigFile.print(FluidDensity_configStr);
  ConfigFile.println(FluidDensity);
  ConfigFile.print(GPSOutrun_Str);
  ConfigFile.println(gpsOutrun);
  ConfigFile.println("// END OF CONFIG //;");
  ConfigFile.close();
  ConfigDate = "";
  Serial.println("new config saved");
}

void ReadConfig(){
  File ConfigFile = SD.open("config.txt");
  if (ConfigFile){
    Serial.println("Config existent. loading parameters...");
    while(ConfigFile.available()){
      inputChar = ConfigFile.read(); // gets one byte after another
      if (inputChar != ';'){ // ; as breaking char
        inputString[stringIndex] = inputChar;
        stringIndex++;
      } else {
        Serial.println("Parameters found");
        Serial.println(inputString);
        InputString = inputString; // changing char to String

        
        // Loading Configdate
        i = String_ConfigDate.length(); //length of ConfigDate String to know where to start reading
        Sindex = InputString.indexOf(String_ConfigDate);
        if (Sindex != -1){
          x = inputString[i + Sindex];
          x = x += inputString[i + Sindex + 1];
          x = x += inputString[i + Sindex + 2];
          x = x += inputString[i + Sindex + 3];
          x = x += inputString[i + Sindex + 4];
          x = x += inputString[i + Sindex + 5];
          x = x += inputString[i + Sindex + 6];
          x = x += inputString[i + Sindex + 7];
          x = x += inputString[i + Sindex + 8];
          x = x += inputString[i + Sindex + 9];
          x = x.trim();
          S_Configdate = x;
          Serial.print("Config from: ");
          Serial.println(S_Configdate);
        } else { Serial.println("no config date");}

        
        // loading Acc setting
        i = Acc_configStr.length();
        Sindex = InputString.indexOf(Acc_configStr);
        if(Sindex != -1){
          x = inputString[i+Sindex];
          x = x += inputString[i+Sindex+1];
          x = x.trim();
          AccCurrent = x.toInt();
          Serial.print("Accelerometer Sensitivity is set to: ");
          Serial.print(AccCurrent);
          Serial.println(" G");
        } else { Serial.println("No Accelerometer Settings were found");}

        // loading Gyroscope setting
        i = Gyr_configStr.length();
        Sindex = InputString.indexOf(Gyr_configStr);
        if(Sindex != -1){
          x = inputString[i+Sindex];
          x = x += inputString[i+Sindex+1];
          x = x += inputString[i+Sindex+2];
          x = x += inputString[i+Sindex+3];
          x = x.trim();
          GyrCurrent = x.toInt();
          Serial.print("Gyroscope Sensitivity is set to: ");
          Serial.print(GyrCurrent);
          Serial.println(" deg/s");
        } else { Serial.println("No Gyroscope Settings were found");}

         // loading DLPF setting
        i = IMU_DLPF_configStr.length();
        Sindex = InputString.indexOf(IMU_DLPF_configStr);
        if(Sindex != -1){
          x = inputString[i+Sindex];
          x = x += inputString[i+Sindex+1];
          x = x += inputString[i+Sindex+2];
          x = x.trim();
          DLPFCurrent = x.toInt();
          Serial.print("Digital Low Pass Filter Bandwidth is set to: ");
          Serial.print(DLPFCurrent);
          Serial.println(" Hz");
        } else { Serial.println("No DLPF Settings were found");}

        // loading SRD setting
        i = IMU_SRD_configStr.length();
        Sindex = InputString.indexOf(IMU_SRD_configStr);
        if(Sindex != -1){
          x = inputString[i+Sindex];
          x = x += inputString[i+Sindex+1];
          x = x.trim();
          SRDCurrent = x.toInt();
          Serial.print("Sample Rate Divider is set to: ");
          Serial.println(SRDCurrent);
          
        } else { Serial.println("No SRD Settings were found");}

        // loading Fluid Density
        i = FluidDensity_configStr.length();
        Sindex = InputString.indexOf(FluidDensity_configStr);
        if(Sindex != -1){
          x = inputString[i+Sindex];
          x = x += inputString[i+Sindex+1];
          x = x += inputString[i+Sindex+2];
          x = x += inputString[i+Sindex+3];
          x = x.trim();
          FluidDensity = x.toInt();
          Serial.print("Fluid Density is set to: ");
          Serial.print(FluidDensity);
          Serial.println(" [kg/m^3]");
          
        } else { Serial.println("No Fluid Density was found");}
  
        i = GPSOutrun_Str.length();
        Sindex = InputString.indexOf(GPSOutrun_Str);
        if(Sindex != -1){
          x = inputString[i+Sindex];
          x = x += inputString[i+Sindex+1];
          x = x += inputString[i+Sindex+2];
          x = x.trim();
          gpsOutrun = x.toInt();
          Serial.print("GPS Outrun Time is set to: ");
          Serial.print(gpsOutrun);
          Serial.println(" [s]");
          
        } else { Serial.println("No GPS Outrun Time was found");}
      }
    }
    Serial.println("Config File loaded");
  } else {
    Serial.println("Config not found. check SD!");
  }
}

void setDLPF(){
  if (DLPFCurrent == 5){
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
      } else if (DLPFCurrent == 10){
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
      } else if (DLPFCurrent == 20){
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
      } else if (DLPFCurrent == 41){
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
      } else if (DLPFCurrent == 92){
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);
      } else if (DLPFCurrent == 184){
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
      }
}

void setGyr(){
  if (GyrCurrent == 250){
        IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
      } else if (GyrCurrent == 500){
        IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
      } else if (GyrCurrent == 1000){
        IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
      } else if (GyrCurrent == 2000){
        IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
      }
}
void setAcc(){
  if (AccCurrent == 2){
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G); //
      } else if (AccCurrent == 4){
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G); //
      }else if (AccCurrent == 8){
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G); //
      }else if (AccCurrent == 16){
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G); //
      }
}
