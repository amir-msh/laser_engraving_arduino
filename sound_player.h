#include "pitches.h"

class SoundPlayer {

  public:
    SoundPlayer(
      int speakerPin1
    ){
      this->speakerPin = speakerPin1;
      pinMode(speakerPin, OUTPUT);
      noTone(speakerPin);
    }

    void deviceStartSound(){
      noTone(speakerPin);
      dt(NOTE_C4, 100);
      dt(NOTE_E4, 100);
      dt(NOTE_G4, 100);
      at(NOTE_C5, 250);
    }

    void successSound(){
      noTone(speakerPin);
      dt(NOTE_C4, 100, 100);
      dt(NOTE_C4, 100, 0);
      dt(NOTE_D4, 100, 200);
      dt(NOTE_C4, 100, 200);
      dt(NOTE_E4, 100, 200);
      at(NOTE_F4, 300);
    }

    void doneSound(){
      noTone(speakerPin);
      dt(NOTE_C4, 50, 10);
      dt(NOTE_E4, 50, 10);

      dt(NOTE_D4, 50, 10);
      dt(NOTE_F4, 50, 10);
      
      dt(NOTE_E4, 50, 10);
      dt(NOTE_G4, 50, 10);
    }

    void deviceOnSound(){
      noTone(speakerPin);
      dt(NOTE_C4, 50, 10);
      dt(NOTE_E4, 50, 10);

      dt(NOTE_D4, 50, 10);
      dt(NOTE_F4, 50, 10);
      
      dt(NOTE_E4, 50, 10);
      dt(NOTE_G4, 50, 10);
    }

    void bluetoothConnectSound(){
      noTone(speakerPin);
      dt(NOTE_C4, 60, 10);
      dt(NOTE_E4, 60, 10);
      at(NOTE_G4, 60);
    }

    void bluetoothDisconnectSound(){
      noTone(speakerPin);
      dt(NOTE_G4, 70, 20);
      dt(NOTE_E4, 70, 20);
      at(NOTE_C4, 70);
    }
    
    // delayed tone
    void dt(int frequency, int noteDelay, int silence=0){
      tone(speakerPin, frequency, noteDelay);
      delay(noteDelay + silence + 5);
    }

    // asynchronous tone
    void at(int frequency, int noteDuration){
      tone(speakerPin, frequency, noteDuration);
    }

    int getSpeakerPin() {
      return speakerPin;
    }

  private:
    int speakerPin;
};
