#include <SoftwareSerial.h>
#include "limited_stepper.h"
#include "pitches.h"
#include "musics.h"
#include "pins.h"

#define servoStepsPerRevolution 2038*8

// btMessage format = "#command\r\n"
#define btMessageStartChar '#'
#define btMessageEndChar '\r'
#define btMessageTerminatorChar '\n'

LimitedStepper xStepper(servoStepsPerRevolution, xStepperStepPin, xStepperDirPin, xStepperSwitchPin);
LimitedStepper yStepper(servoStepsPerRevolution, yStepperStepPin, yStepperDirPin, yStepperSwitchPin);

SoftwareSerial btSerial(btRxPin, btTxPin);

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

void report(String text, ReportType reportType = general, bool btReport = true) {
  if(showReportsLength==0) return;
  static byte i;
  for (i = 0; i < showReportsLength; i++) {
    if (showReports[i] == (byte)reportType) break;
    else if (i == (showReportsLength - 1)) return;
  }

  static String preText = "";
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
  
  // verical :
  xStepper.moveToDeg(28);
  yStepper.moveToDeg(15);
  // xStepper.moveToDeg(55.0);
  // yStepper.moveToDeg(21.5);
  
  // center :
  // xStepper.moveToDeg(76.53);
  // yStepper.moveToDeg(25.83);
  
  xStepper.setAsOrigin();
  yStepper.setAsOrigin();

  setLaser(1);
  delay(1000);
  setLaser(0);
}

void initShockSensor() {
  pinMode(shockSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(shockSensorPin), onShockDetected, FALLING);
}

void initBluetooth(){
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
  static String cmd;
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
  static char c;
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
  else if(mode.equals("rg")){
    int arg1End = cmd.indexOf(',', openPrIndex + 1);
    if(arg1End <= openPrIndex + 1) return;
    int arg2End = cmd.indexOf(')', arg1End + 1);
    if(arg2End < arg1End + 1) return;

    onRasterGrayscaleModeSelected(
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
  // currentMode = rasterBlackWhite;
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
  // currentMode = idle;
  report("onIdleModeSelected()", event);
}

// void engravePoint(unsigned long x, unsigned long y, unsigned long imgWidth, byte power = 255, bool autoTurnOff = true) {
//   double xDegree = fractionalOffsetToDegree(((double)x)/(double)imgWidth);
//   double yDegree = fractionalOffsetToDegree(((double)y)/(double)imgWidth);
  
//   // if(xDegree>180) xDegree = 360 - xDegree;
//   // if(yDegree>180) yDegree = 360 - yDegree;

//   report("engravePoint() : x=" + String(x) + ", y=" + String(y) + ", xDegree=" + String(xDegree) + ", yDegree=" + String(yDegree) + "\n");

//   // moveServosSync(xDegree, yDegree);
  
//   setLaser(1);
//   delay(map(power,0,255,250,1000));
//   if(autoTurnOff) setLaser(0);
// }

bool engraveRasterBlackWhiteBuffer(uint8_t* buffer, unsigned long bufferSize, unsigned long imgWidth, unsigned long initialPixelIndex = 0) {
  unsigned long bufIndex = 0;
  unsigned long nextOffset = 0;
  unsigned long x=0, y=0, pixelIndex=initialPixelIndex;
  double dx=0, dy=0;
  double xDeg=0, yDeg=0;
  
  report("engraveRasterBlackWhiteBuffer() => bufferSize="+String(bufferSize));
  xStepper.setSpeedRPH(28);
  yStepper.setSpeedRPH(28);
  
  const double distanceMultiplier = 1.9;
  const double xDistanceFromTarget = 6 * distanceMultiplier;
  const double yDistanceFromTarget = 6 * distanceMultiplier + 3; // pre : 2
  // const double yDistanceFromTarget = 8 * distanceMultiplier;
  const double xLength = 3.;
  const double yLength = 3.;

  double xTranslate = 1;
  double yTranslate = 1;

  unsigned long lastY = 0;

  // <test>
  // for (unsigned int y = 0; y < imgWidth; y++)
  // {
  //   for (unsigned int x = 0; x < imgWidth; x++)
  //   {
  //     dx = (((double)x / ((double)imgWidth-1.0)) * xLength) + xTranslate;
  //     dy = (((double)y / ((double)imgWidth-1.0)) * yLength) + yTranslate;

  //     xDeg = degrees(atan2(dx, xDistanceFromTarget));
  //     yDeg = degrees(atan2(dy, yDistanceFromTarget));
      
  //     if(y!=lastY){
  //       report("(y!=lastY) == true");
  //       setLaser(0);
  //     }
  //     lastY = y;

  //     xStepper.moveToDeg(xDeg);
  //     yStepper.moveToDeg(yDeg);
  //     setLaser(1);

  //     report("* x=" + String(x) + ", y=" + String(y));
  //     report("* xDeg=" + String(xDeg) + ", yDeg=" + String(yDeg) + "\n------------------------------");

  //     delay(1);
  //   }    
  // }
  // setLaser(0);
  // xStepper.stepToDegree(0);
  // yStepper.stepToDegree(0);
  // return;
  // </test>
  
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

    // if((bool)bit) {
    //   delay(20);
    // }
    // else{
    //   delay(1);
    // }
    //
    
    // before :
    // nextOffset = offset+1;
    // bool nextBit = false;
    // if((nextOffset/8)+1 < bufferSize){
    //   nextBit = bitRead(buffer[(nextOffset/8)+1], 7-(nextOffset%8));
    // }

    // report("\nbufIndex=" + String(bufIndex) + "\nbuffer[bufIndex]=" + String(buffer[bufIndex],BIN) + ", bit=" + String(bit));
    // if(bit){
    //   engravePoint(x, y, imgWidth, 255, !nextBit);
    // }
  }
  setLaser(0);
  report("engraveRasterBlackWhiteBuffer() => end", event);
  return true;
}

void onRasterBlackWhiteModeSelected(unsigned long imageWidth, unsigned long bufferSize){
  // currentMode = rasterBlackWhite;
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
        // packetNumber++;
      }
      else continue;
    }
    delay(10);
  }
}

/// offset : offset from left-top corner (0.0<=offset<=1.0)
// double fractionalOffsetToDegree(double offset, double height) {
//   return round(atan2(offset, height) * 180 / PI);
//   // return round(atan2(1.0, (offset - 0.5)*2) * 180 / PI);
// }

void onRasterGrayscaleModeSelected(int imageWidth, int bufferSize) {
  sendBtConfirmation();
  // currentMode = rasterGrayscale;
  report("onRasterGrayscaleModeSelected(" + String(imageWidth) + ", " + String(bufferSize) + ")", event);
}

void onBtConnected(){
  report("onBtConnected()", event);
  pairMusic();
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
  unpairMusic();
  delay(250);
}

void setup() {
  Serial.begin(38400);
  initBluetooth();
  report("setup()", event);
  initSpeaker(speakerPin);
  onMusic();
  pinMode(speakerPin, OUTPUT);
  initLaser();
  initShockSensor();
  initSteppers();
  startMusic();
  delay(750);
}

// OK
void loop() {
  if (isBtConnected()){
    onBtConnected();
    /// Wait till bluetooth gets disconnected
    while(isBtConnected()) delay(100);
    onBtDisconnected();
  }
  delay(66);
}
