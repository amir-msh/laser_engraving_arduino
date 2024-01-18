#include <SoftwareSerial.h>
#include "limited_stepper.h"
#include "pitches.h"
#include "sound_player.h"
#include "pins.h"

#define servoStepsPerRevolution 2038*8

// btMessage format = "#command\r\n"
#define btMessageStartChar '#'
#define btMessageEndChar '\r'
#define btMessageTerminatorChar '\n'

LimitedStepper xStepper(servoStepsPerRevolution, xStepperStepPin, xStepperDirPin, xStepperSwitchPin);
LimitedStepper yStepper(servoStepsPerRevolution, yStepperStepPin, yStepperDirPin, yStepperSwitchPin);

SoftwareSerial btSerial(btRxPin, btTxPin);
SoundPlayer soundPlayer(speakerPin);

#define btNormalTimeout 750
#define btFindingTimeout 100
#define btDataTimeout 5000
#define steppersRPM 5
#define isDebugMode true

enum ReportType {
  general,
  info,
  event,
  error,
};
const byte reportTypesLength = 4; 

#if isDebugMode
const byte showReports[] = {
  general,
  info,
  event,
  error,
};
#else
const byte showReports[] = { };
#endif

const byte showReportsLength = sizeof(showReports) / sizeof(showReports[0]); 

void report(String text, ReportType reportType = general, bool btReport = false) {
  if(showReportsLength==0) return;
  byte i;
  for (i = 0; i < showReportsLength; i++) {
    if (showReports[i] == (byte)reportType) break;
    else if (i == (showReportsLength - 1)) return;
  }

  String preText = "";
  preText = "(" + String(millis()) + " ms) ";
  switch (reportType) {
    case general:
      preText += "[General] : ";
      break;
    case info:
      preText += "[Info] : ";
      break;
    case event:
      preText += "[event] : ";
      break;
    case error:
      preText += "[error] : ";
      break;
  }
  text = preText + text;
  
  Serial.println(text);
  if(isBtConnected() && btReport){
    btSerial.println(text);
  }
}

void sendBtConfirmation(){
  report("Sending done message ...");
  delay(100);
  if(isBtConnected())
  {
    static String doneMessage = String(btMessageStartChar) + "done" + String(btMessageEndChar) + String(btMessageTerminatorChar);
    btSerial.print(doneMessage);
  }
}

void initLaser() {
  pinMode(laserRelayPin, OUTPUT);
  digitalWrite(laserRelayPin, 1);
}

void setLaser(bool isOn){
  digitalWrite(laserRelayPin, !isOn);
}

void initSteppers() {
  xStepper.setMaxDegree(70);
  yStepper.setMaxDegree(70);
  xStepper.setSpeedRPM(steppersRPM);
  yStepper.setSpeedRPM(steppersRPM);
  xStepper.calibrate();
  yStepper.calibrate();
  
  xStepper.moveToDeg(23);
  yStepper.moveToDeg(15);
  
  xStepper.setAsOrigin();
  yStepper.setAsOrigin();
}

void initShockSensor() {
  pinMode(shockSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(shockSensorPin), onShockDetected, FALLING);
}

void initBluetooth() {
  pinMode(btRxPin, INPUT);
  pinMode(btTxPin, OUTPUT);
  pinMode(btStatePin, INPUT);
  btSerial.setTimeout(btNormalTimeout);
  btSerial.begin(38400);
  delay(30);
  while(btSerial.available()) btSerial.read();
  delay(30);
}

bool isBtConnected(){
  return digitalRead(btStatePin);
}

void onShockDetected() {
  report("onShockDetected()", event);
  // TODO: implement
}

// OK
/// Reads a command from the bluetooth serial buffer (command format = "#{command}\r\n" )
String readBtCommand() {
  String cmd;
  cmd = btSerial.readStringUntil(btMessageTerminatorChar);
  int start = cmd.indexOf(btMessageStartChar);
  int end = cmd.indexOf(btMessageEndChar, 1);
  if(start == -1 || end == -1){
    return "";
  }
  clearBtBufferExcess();
  return cmd.substring(start+1,end);
}

/// Clears buffer excess (control charaters and spacing characters)
void clearBtBufferExcess() {
  char c;
  c = ' ';
  while(btSerial.available()) {
    c = btSerial.peek();
    if(c!='#' && (isControl(c) || isSpace(c) || !isAlphaNumeric(c)))
      btSerial.read();
    else break;
  }
}

/// Clears anything left inside of the bluetooth serial buffer
void clearBtBuffer(){
  while(btSerial.available()) btSerial.read();
}

// mode:RB({width},{buffer_size})
void selectEngravingMode(String cmd){
  int openPrIndex = cmd.indexOf('(');
  int modeNameStart = cmd.indexOf(':') + 1;

  if(openPrIndex<2 || modeNameStart<2) return;

  String mode = cmd.substring(
    modeNameStart,
    openPrIndex
  );

  if(mode.equals("rb")){
    int arg1End = cmd.indexOf(',', openPrIndex + 1);
    if(arg1End <= openPrIndex + 1) return;
    int arg2End = cmd.indexOf(')', arg1End + 1);
    if(arg2End < arg1End + 1) return;

    onRasterBlackWhiteModeSelected(
      cmd.substring(openPrIndex + 1, arg1End).toInt(),
      cmd.substring(arg1End + 1, arg2End).toInt()
    );
  }
  else if(mode.equals("live")){
    onLiveModeSelected();
  }
  else{
    report("Mode is not valid", error);
  }
}

void onLiveModeSelected(){
  sendBtConfirmation();
  report("onLiveModeSelected()", event);
  String moveTo = "";
  String xMove = "";
  String yMove = "";
  int xMoveDeg = 0;
  int yMoveDeg = 0;
  while(isBtConnected()){
    delay(25);
    if(btSerial.available()) {
      btSerial.setTimeout(btFindingTimeout);
      if(btSerial.find(btMessageStartChar)){
        String mode = btSerial.readStringUntil(':');
        if(mode.equals("stepper"))
        {
          btSerial.setTimeout(btNormalTimeout);
          moveTo = btSerial.readStringUntil(btMessageEndChar);
          clearBtBufferExcess();

          int cIndex = moveTo.indexOf(',');
          xMove = moveTo.substring(0, cIndex);
          yMove = moveTo.substring(cIndex+1, moveTo.indexOf(btMessageEndChar));
          xMove.replace(" ","");
          yMove.replace(" ","");

          // report(xMove + "," + yMove);

          if(xMove.startsWith("+")){
            xMove.remove(0, 1);
            xStepper.moveDeg(xMove.toDouble());
          }
          else if(xMove.startsWith("-")){
            xStepper.moveDeg(xMove.toDouble());
          }
          else {
            xStepper.moveToDeg(xMove.toDouble());;
          }

          if(yMove.startsWith("+")){
            yMove.remove(0, 1);
            yStepper.moveDeg(yMove.toDouble());
          }
          else if(yMove.startsWith("-")){
            yStepper.moveDeg(yMove.toDouble());
          }
          else {
            yStepper.moveToDeg(yMove.toDouble());;
          }
          delay(10);

          report("xServo Degree : " + String(xStepper.getAngle()), info);
          report("yServo Degree : " + String(yStepper.getAngle()) + "\n____________________________", info);
        }
        else if(mode.equals("laser")){
          btSerial.setTimeout(btNormalTimeout);
          String laserState = btSerial.readStringUntil(btMessageTerminatorChar);
          laserState = laserState.substring(0, laserState.indexOf(btMessageEndChar));
          laserState.replace(":","");
          setLaser(laserState.toInt());
          delay(10);
        }
      }
      else continue;
    }
  }
  btSerial.setTimeout(btNormalTimeout);
}

bool readData(uint8_t* rasterBuffer, const unsigned long bufferSize){
  int lastTimeOut = btSerial.getTimeout();
  btSerial.setTimeout(btFindingTimeout);
  if(btSerial.find((char*)"#data:")) {
    btSerial.setTimeout(btDataTimeout);
    btSerial.readBytes(rasterBuffer, bufferSize);
    btSerial.readStringUntil('\r');
    clearBtBufferExcess();
    btSerial.setTimeout(lastTimeOut);
    report("readData() => " + (String)(char*)rasterBuffer);
    return true;
  }
  else{
    btSerial.setTimeout(lastTimeOut);
    return false;
  }
}

void onIdleModeSelected(){
  report("onIdleModeSelected()", event);
}

bool engraveRasterBlackWhiteBuffer(uint8_t* buffer, unsigned long bufferSize, unsigned long imgWidth, unsigned long initialPixelIndex = 0) {
  unsigned long bufIndex = 0;
  unsigned long nextOffset = 0;
  unsigned long x=0, y=0, pixelIndex=initialPixelIndex;
  double dx=0, dy=0;
  double xDeg=0, yDeg=0;
  
  report("engraveRasterBlackWhiteBuffer() => bufferSize="+String(bufferSize));
  xStepper.setSpeedRPH(23);
  yStepper.setSpeedRPH(23);
  
  const double distanceMultiplier = 1.9;
  const double xDistanceFromTarget = 6 * distanceMultiplier;
  const double yDistanceFromTarget = 6 * distanceMultiplier + 6;
  const double xLength = 3.;
  const double yLength = 3.;

  double xTranslate = 1;
  double yTranslate = 1;

  unsigned long lastY = 0;
  
  for (unsigned long offset = 0; bufIndex < bufferSize; offset++, bufIndex = offset/8)
  {
    if(!isBtConnected()) return false;

    pixelIndex = offset + initialPixelIndex;
    x = pixelIndex % imgWidth;
    y = pixelIndex / imgWidth;
    if(y>=imgWidth){
      report("y-1>=imgWidth == true");
      break;
    }
    if(y!=lastY){
      report("y!=lastY == true");
      setLaser(0);
    }
    lastY = y;
    bool bit = bitRead(buffer[bufIndex], 7-(offset%8));
    
    dx = (((double)x / ((double)imgWidth-1.0)) * xLength) + xTranslate;
    dy = (((double)y / ((double)imgWidth-1.0)) * yLength) + yTranslate;

    xDeg = degrees(atan2(dx, xDistanceFromTarget));
    yDeg = degrees(atan2(dy, yDistanceFromTarget));

    xStepper.moveToDeg(xDeg);
    yStepper.moveToDeg(yDeg);

    setLaser(bit);

    report("* x=" + String(x) + ", y=" + String(y), info);
    report("* xDeg=" + String(xDeg) + ", yDeg=" + String(yDeg) + "\n------------------------------", info);
  }
  setLaser(0);
  report("engraveRasterBlackWhiteBuffer() => end", event);
  return true;
}

void onRasterBlackWhiteModeSelected(unsigned long imageWidth, unsigned long bufferSize){
  report("onRasterBlackWhiteModeSelected(" + String(imageWidth) + ", " + String(bufferSize) + ")", event);
  sendBtConfirmation();
  unsigned long packetNumber = 0;
  uint8_t rasterBuffer[bufferSize];
  while(isBtConnected()){
    if(btSerial.available()){
      if(readData(rasterBuffer, bufferSize)){
        if(engraveRasterBlackWhiteBuffer(
          rasterBuffer,
          bufferSize,
          imageWidth,
          (packetNumber++ * bufferSize) * 8
        )) {
          sendBtConfirmation();
        }
      }
      else continue;
    }
    delay(10);
  }
}

void onBtConnected(){
  report("onBtConnected()", event);
  soundPlayer.bluetoothConnectSound();
  delay(300);
  String cmd = "";
  while(isBtConnected()){
    if(btSerial.available()) {
      cmd = readBtCommand();
      if(cmd != ""){
        selectEngravingMode(cmd);
        cmd = "";
      }
    }
    delay(30);
  }
}

void onBtDisconnected(){
  report("onBtDisconnected()", event);
  soundPlayer.bluetoothDisconnectSound();
  delay(250);
}

void setup() {
  Serial.begin(38400);
  initBluetooth();
  report("setup()", event);
  soundPlayer.deviceOnSound();
  initLaser();
  initShockSensor();
  initSteppers();
  soundPlayer.deviceStartSound();
  delay(750);
}

// TODO: semantic versioning & compatibility matrix & tagging & change log file
void loop() {
  if (isBtConnected()){
    onBtConnected();
    // Wait till bluetooth gets disconnected
    while(isBtConnected()) delay(100);
    onBtDisconnected();
  }
  delay(66);
}

