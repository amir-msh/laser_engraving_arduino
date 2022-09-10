#include <SoftwareSerial.h>
#include "limited_stepper.h"
#include "pitches.h"
#include "musics.h"
#include "pins.h"

#define servoStepsPerRevolution 2038*8

LimitedStepper xStepper(servoStepsPerRevolution, xStepperStepPin, xStepperDirPin, xStepperSwitchPin);
LimitedStepper yStepper(servoStepsPerRevolution, yStepperStepPin, yStepperDirPin, yStepperSwitchPin);

SoftwareSerial btSerial(btRxPin, btTxPin);

#define servo180DegRotateDelay 750
#define btNormalTimeout 750
#define btFindingTimeout 100
#define btDataTimeout 5000
#define steppersRPM 3

#define reportTypesLength 4

enum EngravingMode {
  rasterBlackWhite,
  rasterGrayscale,
  liveControl,
  idle,
};

EngravingMode currentMode = idle;

enum ReportType {
  general,
  info,
  event,
  error
};

int showReports[reportTypesLength] = {
  0, 1, 2, 3
};

void report(String text, ReportType reportType = general, bool btReport = true) {
  for (int i = 0; i < reportTypesLength; i++) {
    if (showReports[i] == reportType) break;
    else if (i == (reportTypesLength - 1)) return;
  }
  String preText = "(" + String(millis()) + " ms) ";
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

String engravingModeToCode(EngravingMode mode){
  switch (mode)
  {
    case rasterBlackWhite: return "rb";
    case rasterGrayscale: return "rg";
    case liveControl: return "live";
    case idle: return "idle";
    default: return "";
  }
}

void initLaser() {
  pinMode(laserRelayPin, OUTPUT);
  digitalWrite(laserRelayPin, 1);
}

void setLaser(bool isOn){
  // #laserm
  // return;
  digitalWrite(laserRelayPin, !isOn);
  //  digitalWrite(relay2Pin,!isOn);
}

void initSteppers() {
  xStepper.setMaxDegree(70);
  yStepper.setMaxDegree(70);
  xStepper.setSpeedRPM(steppersRPM);
  yStepper.setSpeedRPM(steppersRPM);
  xStepper.calibrate();
  yStepper.calibrate();
  
  // start :
  // xStepper.moveToDeg(62.89);

  // xStepper.moveToDeg(62.890);
  // yStepper.moveToDeg(35.000);
  
  xStepper.moveToDeg(76.53);
  yStepper.moveToDeg(25.83);
  xStepper.setAsOrigin();
  yStepper.setAsOrigin();

  // while(true){
  //   setLaser(1);
  //   delay(60);
  //   yStepper.moveToDeg(20);
  //   delay(60);
  //   yStepper.moveToDeg(37);
  //   setLaser(0);
  //   delay(5000);
  // }
}

void initShockSensor() {
  pinMode(shockSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(shockSensorPin), onShockDetected, FALLING);
//  report();
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
}

// OK
/// Reads a command from the bluetooth serial buffer (command format = "#{command}?\r" )
String readBtCommand() {
  String cmd = btSerial.readStringUntil('\r');
  int start = cmd.indexOf('#');
  int end = cmd.indexOf('?', 1);
  if(start == -1 || end == -1){
    return "";
  }
  clearBtBufferExcess();
  return cmd.substring(start+1,end);
}

/// Clears buffer excess (control charaters and spacing characters)
void clearBtBufferExcess() {
  char c = ' ';
  while(btSerial.available()) {
    c = btSerial.peek();
    if(isControl(c) || isSpace(c) || !isAlphaNumeric(c))
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
  else if(mode.equals("idle")){
    onIdleModeSelected();
    // report("mode is not valid", event);
  }
}

void onLiveModeSelected(){
  currentMode = rasterBlackWhite;
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
      if(btSerial.find('#')){
        String mode = btSerial.readStringUntil(':');
        if(mode.equals("stepper"))
        {
          btSerial.setTimeout(btNormalTimeout);
          moveTo = btSerial.readStringUntil('\r');
          clearBtBufferExcess();

          int cIndex = moveTo.indexOf(',');
          xMove = moveTo.substring(0, cIndex);
          yMove = moveTo.substring(cIndex+1, moveTo.indexOf('?'));
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

          report("xServo Degree : " + String(xStepper.getAngle()));
          report("yServo Degree : " + String(yStepper.getAngle()));
          report("____________________________");
        }
        else if(mode.equals("laser")){
          btSerial.setTimeout(btNormalTimeout);
          String laserState = btSerial.readStringUntil('\r');
          laserState = laserState.substring(0, laserState.indexOf('?'));
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

// void answerCommands(){
//   int lastTimeOut = btSerial.getTimeout();
//   btSerial.setTimeout(btFindingTimeout);
//   if(btSerial.find("$")){
//     String cmd = btSerial.readStringUntil('?');
//     clearBtBufferExcess();
//     btSerial.setTimeout(lastTimeOut);
//     if(cmd.startsWith("mode:")){
//       selectEngravingMode();
//     }
//     else if(cmd.equals("mode")){
//       btSerial.println("#mode:?");
      
//     }
//     else if(cmd.equals("")){
      
//     }
//   }
// }

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
  currentMode = idle;
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

void engraveRasterBlackWhiteBuffer(uint8_t* buffer, unsigned long bufferSize, unsigned long imgWidth, unsigned long initialPixelIndex = 0) {
  unsigned long bufIndex = 0;
  unsigned long nextOffset = 0;
  unsigned long x=0, y=0, pixelIndex=initialPixelIndex;
  
  report("engraveRasterBlackWhiteBuffer() => bufferSize="+String(bufferSize));
  
  for (unsigned long offset = 0; bufIndex < bufferSize; offset++, bufIndex = offset/8)
  {
    pixelIndex = offset + initialPixelIndex;
    x = pixelIndex % imgWidth;
    y = pixelIndex / imgWidth;

    bool bit = bitRead(buffer[bufIndex], 7-(offset%8));
    // setLaser(bit);
    report("* loop: x=" + String(x) + ", y=" + String(y) + "________________");

    
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
}

void onRasterBlackWhiteModeSelected(unsigned long imageWidth, unsigned long bufferSize){
  currentMode = rasterBlackWhite;
  report("onRasterBlackWhiteModeSelected(" + String(imageWidth) + ", " + String(bufferSize) + ")", event);
  unsigned long packetNumber = 0;
  uint8_t rasterBuffer[bufferSize];
  while(isBtConnected()){
    if(btSerial.available()){
      if(readData(rasterBuffer, bufferSize)){
        engraveRasterBlackWhiteBuffer(
          rasterBuffer,
          bufferSize,
          imageWidth,
          (packetNumber++ * bufferSize) * 8
        );
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
  currentMode = rasterGrayscale;
  report("onRasterGrayscaleModeSelected(" + String(imageWidth) + ", " + String(bufferSize) + ")", event);
}

void onBtConnected(){
  report("onBtConnected()", event);
  pairMusic();
  delay(500);
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
  Serial.begin(9600);
  delay(50);
  initBluetooth();
  delay(50);
  initSpeaker(speakerPin);
  onMusic();
  pinMode(speakerPin, OUTPUT);
  initLaser();
  initShockSensor();
  initSteppers();
  startMusic();
  delay(500);
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
