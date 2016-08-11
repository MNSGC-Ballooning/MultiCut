//  Multi-Cut Box current implementation as of 6/6/16 *Updated 8/11/16*
//  By Noah Biniek (binie005@umn.edu)
#include <AltSoftSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>

//////////////////// Autonomous Behavior Variables ////////////////////

// At a certain altitude, burn specific payloads
long burnAAlt = 40000;         // Burn A at this altitude (in meters, 0 for never)
long burnBAlt = 55000;         // Burn B at this altitude (in meters, 0 for never)
long burnCAlt = 80000;         // Burn C at this altitude (in meters, 0 for never)
long burnDAlt = 0;             // Burn D at this altitude (in meters, 0 for never)

// After a certain amount of time (from start of flight), burn specific payloads
long burnATime = 3000;        // Burn A after this much time has elapsed (in seconds, 0 for never)
long burnBTime = 3900;         // Burn B after this much time has elapsed (in seconds, 0 for never)
long burnCTime = 5400;        // Burn C after this much time has elapsed (in seconds, 0 for never)
long burnDTime = 7200;        // Burn D after this much time has elapsed (in seconds, 0 for never)


// Class to support blinking LEDs that don't interrupt main loop
class IndicatorLED{
    private:
          uint8_t _ledPin;
          boolean _ledOn;
          boolean _burned;
          boolean _pulled;

    public:
          IndicatorLED(uint8_t pin);
          int getPin();
          void setOn();
          void setOff();
          // Turn the led on for a given amount of time (relies
          // on a call to check() in the main loop()).
          void burned();
          void pulled();
          void burnCheck();
          void pullCheck();
          boolean isOn();
};

// Constructor for flashing LED class
IndicatorLED::IndicatorLED(uint8_t ledPin) : _ledPin(ledPin){
    pinMode(_ledPin, OUTPUT);
    _ledOn = false;
}

int IndicatorLED::getPin(){
  return _ledPin;
}

boolean IndicatorLED::isOn(){
  return _ledOn;
}

// Enables the LED
void IndicatorLED::pulled(){
  _pulled = true;
}

// Disables the LED
void IndicatorLED::burned(){
  _burned = true;
}

// Turns the LED on
void IndicatorLED::setOn(){
  if(not _ledOn){
    digitalWrite(_ledPin, HIGH);
    _ledOn = true;
  }
}

// Turns the LED off
void IndicatorLED::setOff(){
  if(_ledOn){
    digitalWrite(_ledPin, LOW);
    _ledOn = false;
  }
}


// Checks and toggles the LED state when needed
void IndicatorLED::burnCheck(){
  if(_burned){
    setOn();
  }
}
void IndicatorLED::pullCheck(){
  if(_pulled){
    setOn();
  }
  if(not _pulled){
    setOff();
  }
}


const uint8_t led = 13;
const uint8_t burnerA = 9, burnerB = 5, burnerC = 7, burnerD = 11;
const uint8_t pullA = 8, pullB = 4, pullC = 6, pullD = 10;
const int chipSelect = 53;

const long xbeeBaud = 9600;
const long gpsBaud = 9600;

AltSoftSerial xBee; // TX:46, RX:48, PWM Unusuable:44,45 **These pins cannot be changed - this is built into the AltSoftSerial Library**
                    // If using Arduino Mega with SparkFun XBee shield, make sure switch is set to DLINE (not UART).
                    // This tells the XBee to communicate with pins 2 and 3 which must be jumped to pins 48 and 46 respectively.
                    // Make sure pins 2 and 3 on the shield are not inserted into the Arduino
#define s Serial1   // TX: 18, RX:19 (Connect GPS RX to 18, GPS TX to 19, VIN to 5V, GND to GND)
                    // Using Ultimate GPS Breakout v3
                        
//  SD Breakout Pinouts (with Arduino Mega):
//    DI - Pin 51
//    DO - Pin 50
//    CLK - Pin 52
//    CS - Pin 53

//  GPS Declaration  //
Adafruit_GPS GPS(&s);
String  gpsData;
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);

///////////////////////////////////// GLOBAL VARIABLES ///////////////////////////////////////////////
//  regular updates/box behavior  //
boolean gpsUpdates = false;
boolean statusUpdates = false;
boolean autoA = false, autoB = false, autoC = false, autoD = false;
long flightStart= 0;
boolean activatedA = false;
boolean activatedB = false;
boolean activatedC = false;
boolean activatedD = false;
long timeCheck = 0;
long interval = 500;
boolean flash1 = true;
boolean burn = false;
boolean flash2 = true;
boolean pull = false;
boolean resetTimeCheck = false;
boolean on = true;

  // Initialize LEDs    //
IndicatorLED burnerALED(33);
IndicatorLED burnerBLED(35);
IndicatorLED burnerCLED(37);
IndicatorLED burnerDLED(39);

// Logs data to the logfile, along with a timestamp
void logData(String text) {
  File logFile = SD.open("MC-LOG.txt", FILE_WRITE);
  if (logFile) {
    logFile.println(String(millis()/1000) + ": " + text);
    logFile.close();
  }
}

// Fires the burner specified in the argument
void fireBurner(int burner) {
  logData("Firing Burner " + (String)burner);
  sync_LEDs();
  delay(30000);                //  recharge capacitors for thirty seconds
  digitalWrite(burner, HIGH);  //  fire burner
  delay(3000);                 //  wait for three seconds
  digitalWrite(burner, LOW);   //  stop burner
}

void sync_LEDs(){
  burnerALED.setOff();
  burnerBLED.setOff();
  burnerCLED.setOff();
  burnerDLED.setOff();
}


// Fires the burner specificed in the first argument one time and then up to two more times if pull pin is still present
void fireAutonomousBurner(int burner, int pull) {
  fireBurner(burner);
  if(digitalRead(pull) == HIGH) {
    delay(30000);
    fireBurner(burner);
  } 
  if(digitalRead(pull) == HIGH) {
    delay(30000);
    fireBurner(burner);
  }
}

// Flashes the arduino's built in LED for 1.5 seconds
void testLED() {
  digitalWrite(led, HIGH);
  delay(1500);
  digitalWrite(led, LOW);
}

//  Return status string: "minutes:seconds A?B?C?D?" where ? may be O if pull is present or X if not present
String getPullStatus() {
  String status;
  int minutes = ((millis()/1000) - flightStart)/60;
  int seconds = ((millis()/1000) - flightStart)%60;
  status += String(minutes) + ":" + String(seconds) + " ";
  if(digitalRead(pullA) == LOW){
    activatedA = false;
    status += "AX";
  }
  else{
    if(not activatedA){
      activateIndicatorLED("A","P");
    }
    activatedA = true;
    status += "AO";
  }
  if(digitalRead(pullB) == LOW){
    activatedB = false;
    status += "BX";
  }
  else{
    if(not activatedB){
      activateIndicatorLED("B","P");
    }
    activatedB = true;
    status += "BO";
  }
  if(digitalRead(pullC) == LOW){
    activatedC = false;
    status += "CX";
  }
  else{
    if(not activatedC){
      activateIndicatorLED("C","P");
    }
    activatedC = true;
    status += "CO";
  }
  if(digitalRead(pullD) == LOW){
    activatedD = false;
    status += "DX";
  }
  else{
    if(not activatedD){
      activateIndicatorLED("D","P");
    }
    activatedD = true;
    status += "DO";
  }
  return status;
}

//  Interrupt is called once a millisecond, looks for any new GPS data, and stores it  //
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDRO      //Change made by Ben G 8/11/16 fixing interupt problem
  if(GPSECHO)
    if(c) UDRO = c;
#endif
}

//  GPS interrupt function  //
void useInterrupt(boolean v)
{
  if (v)
  {
    //  Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the "Compare A" function above  //
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else
  {
    //  Do not call the interrupt function COMPA anymore  //
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//  Return GPS string: "hours,minutes,seconds,lat,long,alt,satellites"
String getGPS() {
  String gpsData = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "," +
            String(GPS.latitudeDegrees, 6) + "," + String(GPS.longitudeDegrees, 6) + "," + String(GPS.altitude, 1) + "," + String(GPS.satellites);
  return gpsData;
}

// Checks for and responds to any commands received via XBee //
void xBeeCommand() {
  if (xBee.available()) {

    //  Grab transmission and parse command  //
    int len, num;
    String transmission, id, command;
    transmission = xBee.readStringUntil('!');
    len = transmission.length();
    num = transmission.indexOf('?');
    id = transmission.substring(0, num);
    command = transmission.substring(num + 1, len);
    logData("XBee Transmission received: " + transmission);
    Serial.println(transmission);

    //  Verify id and perform valid command //
    if (id == "MC") {
      Serial.println("Command acknowledged");
      sendToXbee("MC\n");                                  //  Command acknowledged
      logData("Command: " + command + " acknowledged");
      if (command == "BA") {                                //  "BA": Burn A
        logData("Commanded to Burn A");
        Serial.println("Burning A in thirty seconds");
        testLED();
        fireBurner(burnerA);
        activateIndicatorLED("A","B");   // Turn on the indicator light
        Serial.println("Burned A");
        sendToXbee("BA OK");
      }
      else if (command == "BB") {                         //  "BB": Burn B
        logData("Commanded to Burn B");
        Serial.println("Burning B in thirty seconds");
        testLED();
        fireBurner(burnerB);
        activateIndicatorLED("B","B");   // Turn on the indicator light
        Serial.println("Burned B");
        sendToXbee("BB OK");
      } 
      else if (command == "BC") {                         //  "BC": Burn C
        logData("Commanded to Burn C");
        Serial.println("Burning C in thirty seconds");
        testLED();
        fireBurner(burnerC);
        activateIndicatorLED("C","B");   // Turn on the indicator light
        Serial.println("Burned C");
        sendToXbee("BC OK");
      } 
      else if (command == "BD") {                         //  "BD": Burn D
        logData("Commanded to Burn D");
        Serial.println("Burning D in thirty seconds");
        testLED();
        fireBurner(burnerD);
        activateIndicatorLED("D","B");   // Turn on the indicator light
        Serial.println("Burned D");
        sendToXbee("BD OK");
      } 
      else if (command == "TG") {                         //  "TG": Transmit GPS
        String gpsString = getGPS();
        logData("GPS Data Requested: " + gpsString);
        sendToXbee(gpsString);
        Serial.println(gpsString);
      } 
      else if (command == "AG1") {                         //  "AG1": Turn on automatic GPS transmission
        gpsUpdates = true;
        logData("Automatic GPS updates turned on");
        Serial.println("GPS updates ON");
        sendToXbee("GOn");
      }
      else if (command == "AG0"){                           // "AG0": Turn off automatic GPS transmission
        gpsUpdates = false;
        logData("Automatic GPS updates turned off");
        Serial.println("GPS udpates OFF");
        sendToXbee("GOff");
      } 
      else if (command == "TP") {                         //  "TP": Transmit Pull Status
        String pullStatus = getPullStatus();
        logData("Pull Pin Status Requested: " + pullStatus);
        sendToXbee(pullStatus);
        Serial.println(pullStatus);
      }
      else if (command == "AS1") {                         //  "AS1": Turn on automatic status transmission
        statusUpdates = true;
        logData("Automatic status updates turned on");
        Serial.println("Status updates ON");
        sendToXbee("SOn");
      }
      else if(command == "AS0"){                          //  "AS0": Turn off automatic status transmission
        statusUpdates = false;
        logData("Automatic status updates turned off");
        sendToXbee("SOff");
      }
      else if (command == "AA1") {                         //  "AA1": Turn on autonomous A behavior
        autoA = true;
        logData("Autonomous A behavior turned on");
        Serial.println("Autonomous A behavior ON");
        sendToXbee("AAOn");
      }
      else if(command == "AA0"){                          //  "AA0": Turn off autonomous A behavior
        autoA = false;
        logData("Autonomous A behavior turned off");
        Serial.println("Autonomous A behavior OFF");
        sendToXbee("AAOff");
      }
      else if (command == "AB1") {                         //  "AB1": Turn on/off autonomous B behavior
        autoB = true;
        logData("Autonomous B behavior turned on");
        Serial.println("Autonomous B behavior ON");
        sendToXbee("ABOn");
      }
      else if(command == "AB0") {                         //  "AB0": Turn off autonomous B behavior
        autoB = false;
        logData("Autonomous B behavior turned off");
        Serial.println("Autonomous B behavior OFF");
        sendToXbee("ABOff");
      }
      else if (command == "AC1") {                         //  "AC1": Turn on autonomous C behavior
        autoC = true;
        logData("Autonomous C behavior turned on");
        Serial.println("Autonomous C behavior ON");
        sendToXbee("ACOn");
      }
      else if(command == "AC0"){                         //  "AC0": Turn off autonomous C behavior
        autoC = false;
        logData("Autonomous C behavior turned off");
        Serial.println("Autonomous C behavior OFF");
        sendToXbee("ACOff");
      }
      else if (command == "AD1") {                         //  "AD1": Turn on autonomous D behavior
        autoD = true;
        logData("Autonomous D behavior turned on");
        Serial.println("Autonomous D behavior ON");
        sendToXbee("ADOn");
      }
      else if(command == "AD0"){                         //  "AD0": Turn off autonomous D behavior
        autoD = false;
        logData("Autonomous D behavior turned off");
        Serial.println("Autonomous D behavior OFF");
        sendToXbee("ADOff");
      }
      else if (command == "TL") {                         //  "TL": Test LED
        logData("Commanded to Run LED Test");
        Serial.println("LED test");
        sendToXbee("LED");
        testLED();
      }
      else if (command == "XX") {                         //  "XX": Burn all modules
        logData("Commanded to Burn All Modules");
        Serial.println("Burning all modules");
        fireBurner(burnerA);
        Serial.println("Burned A");
        fireBurner(burnerB);
        Serial.println("Burned B");
        fireBurner(burnerC);
        Serial.println("Burned C");
        fireBurner(burnerD);
        Serial.println("Burned D");
        Serial.println("Burned all modules");
        sendToXbee("BAll OK");
      }
      else if (command == "RT"){                        // "RT": Reset flight timer
        logData("Commanded to Reset Flight Timer");
        resetFlightTimer();
        sendToXbee("Flight Timer Reset");
      }
      else {
        logData("Command not recognized");
        Serial.println("Command not recognized");
        sendToXbee("Command not Recognized");
      }
    }
  }
}

// Sends the argument string to the xbee with an "!" and a new line character appended to the end
void sendToXbee(String msg){
  if(msg != "MC\n"){
    xBee.print("MC;"+msg + "!" + "\n");
  }
  else{
    xBee.print(msg);
  }
  Serial.println("Sent " + msg + " to the xBee");
  logData("Sent " + msg + " to the xBee");
}

// Resets the time of the start of the flight to the current time
void resetFlightTimer(){
  Serial.println("Resetting Flight Timer");
  flightStart = millis()/1000;
  logData("Flight Timer Reset");
}

// Takes a string as an argument, and uses it to adjust the correct indicator LED
void activateIndicatorLED(String arg1,String arg2){
  if(arg1 == "A"){
    if(arg2 == "B"){
      logData("Turning on Burner A burn indicator light");
      Serial.println("Turning on Burner A burn indicator light");
      burnerALED.burned();
    }
    else if(arg2 == "P"){
      logData("Turning on Burner A pull indicator light");
      Serial.println("Turning on Burner A pull indicator light");
      burnerALED.pulled();
    }
  }
  else if(arg1 == "B"){
    if(arg2 == "B"){
      logData("Turning on Burner B burn indicator light");
      Serial.println("Turning on Burner B burn indicator light");
      burnerBLED.burned();      
    }
    else if(arg2 == "P"){
      logData("Turning on Burner B pull indicator light");
      Serial.println("Turning on Burner B pull indicator light");
      burnerBLED.pulled();
    }
  }
  else if(arg1 == "C"){
    if(arg2 == "B"){
      logData("Turning on Burner C burn indicator light");
      Serial.println("Turning on Burner C burn indicator light");
      burnerCLED.burned();      
    }
    else if(arg2 == "P"){
      logData("Turning on Burner C pull indicator light");
      Serial.println("Turning on Burner C pull indicator light");
      burnerCLED.pulled();
    }
  }
  else if(arg1 == "D"){
    if(arg2 == "B"){
      logData("Turning on Burner D burn indicator light");
      Serial.println("Turning on Burner D burn indicator light");
      burnerDLED.burned();      
    }
    else if(arg2 == "P"){
      logData("Turning on Burner D pull indicator light");
      Serial.println("Turning on Burner D pull indicator light");
      burnerDLED.pulled();
    }
  }
}

void indicateWithLEDs(){
  if(flash1){
    if(on){
      burnerALED.setOn();
      burnerBLED.setOn();
      burnerCLED.setOn();
      burnerDLED.setOn();
    }
    if(millis() - timeCheck > interval){
      if(resetTimeCheck){
        timeCheck = millis();
        resetTimeCheck = false;
      }
      Serial.println("Flash 1 Off");
      burnerALED.setOff();
      burnerBLED.setOff();
      burnerCLED.setOff();
      burnerDLED.setOff();
      on = false;
    }
    if(millis() - timeCheck > interval){
      timeCheck = millis();
      flash1 = false;
      burn = true;
      resetTimeCheck = true;
      on = true;
    }
  }
  if(burn){
    if(on){
      burnerALED.burnCheck();
      burnerBLED.burnCheck();
      burnerCLED.burnCheck();
      burnerDLED.burnCheck();
    }
    if(millis() - timeCheck > interval){
      if(resetTimeCheck){
        timeCheck = millis();
        resetTimeCheck = false;
      }
      burnerALED.setOff();
      burnerBLED.setOff();
      burnerCLED.setOff();
      burnerDLED.setOff();
      on = false;
    }
    if(millis() - timeCheck > interval){
      timeCheck = millis();
      burn = false;
      flash2 = true;
      resetTimeCheck = true;
      on = true;
    }
  }
  if(flash2){
    if(on){
      burnerALED.setOn();
      delay(100);
      burnerBLED.setOn();
      delay(100);
      burnerCLED.setOn();
      delay(100);
      burnerDLED.setOn();
      delay(100);
    }
    if(millis() - timeCheck > interval){
      if(resetTimeCheck){
        timeCheck = millis();
        resetTimeCheck = false;
      }
      burnerALED.setOff();
      burnerBLED.setOff();
      burnerCLED.setOff();
      burnerDLED.setOff();
      on = false;
    }
    if(millis() - timeCheck > interval){
      timeCheck = millis();
      flash2 = false;
      pull = true;
      resetTimeCheck = true;
      on = true;
    }
  }
  if(pull){
    if(on){
      burnerALED.pullCheck();
      burnerBLED.pullCheck();
      burnerCLED.pullCheck();
      burnerDLED.pullCheck();
    }
    if(millis() - timeCheck > interval){
      if(resetTimeCheck){
        timeCheck = millis();
        resetTimeCheck = false;
      }
      burnerALED.setOff();
      burnerBLED.setOff();
      burnerCLED.setOff();
      burnerDLED.setOff();
      on = false;
    }
    if(millis() - timeCheck > interval){
      timeCheck = millis();
      pull = false;
      flash1 = true;
      resetTimeCheck = true;
      on = true;
    }
  }
}

void setup () {
  //  Initialize Burners  //
    pinMode(burnerA, OUTPUT);
    pinMode(burnerB, OUTPUT);
    pinMode(burnerC, OUTPUT);
    pinMode(burnerD, OUTPUT);

    pinMode(led, OUTPUT); // initialize LED for testing

  //  Initialize Pulls  //
    pinMode(pullA, INPUT);
    pinMode(pullB, INPUT);
    pinMode(pullC, INPUT);
    pinMode(pullD, INPUT);

  //  Initialize Serials  //
  Serial.begin(9600);
  while (!Serial){;}      //  wait for serial port to connect
  xBee.begin(xbeeBaud);
  GPS.begin(gpsBaud);

  //  SD Data Log Setup //
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {    //  Check if SD card is present
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized");
  File logFile = SD.open("MC-LOG.txt", FILE_WRITE);
  if (logFile) {
    logFile.println("Multi-cut Box Data Log:\n");
    logFile.close();
  }
  else {
    Serial.println("Error opening MC-LOG.txt");
  }
  
  //  GPS Setup //
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //  selects data type to recieve
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //  sets update rate
  GPS.sendCommand(PGCMD_ANTENNA);               //  updates antenna status
  useInterrupt(true);
  delay(1000);
  s.println(PMTK_Q_RELEASE);

  resetFlightTimer();                            // Get the flightTimer reset before starting the loop to account for how long the setup takes
}

void loop (){
  indicateWithLEDs();                            // Blink the status of the LEDs
  logData(getGPS());                             // Log the GPS data
  logData(getPullStatus());                      // Log pull status
  
  xBeeCommand();                                 //  respond to XBee commands if any


  //  Update GPS  //
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA())){           
      return;
    }
  }

  if (gpsUpdates && ((millis()/1000)-flightStart)%30 == 0) {                    //  transmit GPS every 30 seconds if turned on
    sendToXbee(getGPS());
  }

  if(statusUpdates && ((millis()/1000)-flightStart)%60==0) {                    //  transmit pull pin status every minute if turned on
    sendToXbee(getPullStatus());
  }


  //  Autonomous Behavior //
  if(autoA) {                                          //  Module A
    if(burnAAlt != 0){
      if(GPS.altitude >= burnAAlt) {
        fireAutonomousBurner(burnerA,pullA);
        autoA = false;
      }
    }
    if(burnATime != 0){
      if(((millis()/1000)-flightStart) >= burnATime) {
        fireAutonomousBurner(burnerA,pullA);
        autoA = false;
      }
    }
  }
  if(autoB) {                                           // Module B
    if(burnBAlt != 0){
      if(GPS.altitude >= burnBAlt) {
        fireAutonomousBurner(burnerB,pullB);
        autoB = false;
      }
    }
    if(burnBTime != 0){
      if(((millis()/1000)-flightStart) >= burnBTime){
        fireAutonomousBurner(burnerB,pullB);
        autoB = false;
      }
    }
  }
  if(autoC) {                                          //  Module C
    if(burnCAlt != 0){
      if(GPS.altitude >= burnCAlt) {
        fireAutonomousBurner(burnerC,pullC);
        autoC = false;
      }
    }
    if(burnCTime != 0){
      if(((millis()/1000)-flightStart) >= burnCTime){
        fireAutonomousBurner(burnerC,pullC);
        autoC = false;
      }
    }
  }
  if(autoD) {                                          //  Module D
    if(burnDAlt  != 0){
      if((GPS.altitude >= burnDAlt)) {
        fireAutonomousBurner(burnerD,pullD);
        autoD = false;
      }
    }
    if(burnDTime != 0){
      if(((millis()/1000)-flightStart) >= burnDTime) {
        fireAutonomousBurner(burnerD,pullD);
        autoD = false;
      }
    }
  }
}

