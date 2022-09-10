int sPin;

void initSpeaker(int pin){
  sPin = pin;
  pinMode(pin, OUTPUT);
  delay(50);
}

// delayed tone
void dt(int frequency, int noteDelay, int silence=0){
  tone(sPin, frequency, noteDelay);
  delay(noteDelay + silence + 5);
}

// asynchronous tone
void at(int frequency, int noteDuration){
  tone(sPin, frequency, noteDuration);
}

void startMusic(){
  noTone(sPin);
  dt(NOTE_C4, 100);
  dt(NOTE_E4, 100);
  dt(NOTE_G4, 100);
  at(NOTE_C5, 250);
}

void successMusic(){
  noTone(sPin);
  dt(NOTE_C4, 100, 100);
  dt(NOTE_C4, 100, 0);
  dt(NOTE_D4, 100, 200);
  dt(NOTE_C4, 100, 200);
  dt(NOTE_E4, 100, 200);
  at(NOTE_F4, 300);
}

void doneMusic(){
  noTone(sPin);
  dt(NOTE_C4, 50, 10);
  dt(NOTE_E4, 50, 10);

  dt(NOTE_D4, 50, 10);
  dt(NOTE_F4, 50, 10);
  
  dt(NOTE_E4, 50, 10);
  dt(NOTE_G4, 50, 10);
}

void onMusic(){
  noTone(sPin);
  dt(NOTE_C4, 50, 10);
  dt(NOTE_E4, 50, 10);

  dt(NOTE_D4, 50, 10);
  dt(NOTE_F4, 50, 10);
  
  dt(NOTE_E4, 50, 10);
  dt(NOTE_G4, 50, 10);
}

void pairMusic(){
  noTone(sPin);
  dt(NOTE_C4, 60, 10);
  dt(NOTE_E4, 60, 10);
  at(NOTE_G4, 60);
}

void unpairMusic(){
  noTone(sPin);
  dt(NOTE_G4, 70, 20);
  dt(NOTE_E4, 70, 20);
  at(NOTE_C4, 70);
}
