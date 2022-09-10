#define maxDegreeLimit 360.1

enum RotationDirection {
    CW = 0, // clockwise
    CCW = 1, // counter clockwise
};

class LimitedStepper {
    public:
        LimitedStepper(
            uint32_t stepsPerRevolution,
            uint8_t stepperStepPin,
            uint8_t stepperDirPin,
            uint8_t switchPin,
            RotationDirection switchDirection = CCW,
            uint16_t maxStep = INT16_MAX
        ) {
            this->stepsPerRevolution = stepsPerRevolution;
            this->stepperStepPin = stepperStepPin;
            this->stepperDirPin = stepperDirPin;
            this->switchPin = switchPin;
            this->switchDirection = switchDirection;
            setMaxStep(maxStep);
            setSpeedRPM(1);

            pinMode(switchPin, INPUT_PULLUP);
            pinMode(stepperDirPin, OUTPUT);
            pinMode(stepperStepPin, OUTPUT);
            setRotationDirection(CW);
        }
        
        // RotationDirection getRotationDirection(){
        //     return rotationDirection;
        // }

        double getAngle(){
            return stepToDegree(offset);
        }

        void setMaxStep(uint16_t maxStep){
            maxStep = min(maxStep, degreeToStep(maxDegreeLimit));
        }

        void setMaxDegree(double maxDegree){
            maxStep = min(degreeToStep(maxDegreeLimit), degreeToStep(maxDegree));
        }

        void setSpeedRPM(uint32_t rpm)
        {
            rpm = max(rpm, 1UL);
            this->stepDelay = (60UL * 1000UL * 1000UL / (uint32_t) this->stepsPerRevolution) / (uint32_t) rpm;
        }

        void setSpeedRPH(uint32_t rph)
        {
            rph = max(rph, 1UL);
            this->stepDelay = (60UL * 60UL * 1000UL * 1000UL / (uint32_t) this->stepsPerRevolution) / (uint32_t) rph;
        }
        
        uint32_t getOffset(){
            return offset;
        }

        void setAsOrigin(){
            offset = 0;
        }

        void calibrate(){
            delay(50);
            uint32_t currentStepDelay = stepDelay;
            setSpeedRPM(1);
            setRotationDirection(switchDirection);
            
            while(!isAtBeginning())
            {
                step();
            }
            offset = 0;
            stepDelay = currentStepDelay;
            delay(50);
        }

        void step(){
            digitalWrite(stepperStepPin, LOW);
            delayMicroseconds(10);
            digitalWrite(stepperStepPin, HIGH);
            delayMicroseconds(stepDelay-10);
        }

        bool isAtBeginning() {
            return !digitalRead(switchPin);
        }

        bool hasReachedMaxStep(){
            return offset >= maxStep;
        }

        void stepMotor(int stepsToMove)
        {
            if(stepsToMove == 0) return;

            bool isDirectionCW = stepsToMove<0;
            setRotationDirection(static_cast<RotationDirection>(isDirectionCW));
            
            Serial.println("stepMotor() => offset : " + String(offset));
            stepsToMove = abs(stepsToMove);
            for (int i = 0; i < stepsToMove; i++)
            {
                if(isAtBeginning()){
                    // offset = 0;
                    if(switchDirection == rotationDirection){
                        offset = 0;
                        break;
                    }
                }
                // if (hasReachedMaxStep()){
                //     Serial.println("hasReachedMaxStep()");
                //     break;
                // }
                step();
                offset += isDirectionCW? -1: 1;
                // if(lastPercentage!=(int)((100L*(long)i)/(long)stepsToMove)) {
                //     lastPercentage=(uint16_t)((100L*(long)i)/(long)stepsToMove);
                //     if(lastPercentage%20==0){
                //         Serial.println(String(lastPercentage) + "% - step num = " + String(i+1));
                //     }
                // }
                // else if( i == stepsToMove-1){
                //     Serial.println("100% - step num = " + String(i+1));
                // }
                // if(isAtBeginning()){
                //     offset = 0;
                // }
            }
            if(isAtBeginning()) offset = 0;
            Serial.println("stepMotor() (finish) => offset : " + String(offset));
        }

        double stepToDegree(int step){
            return ((double) step * 360.0) / (double) stepsPerRevolution;
        }

        int degreeToStep(double degree){
            return round((degree * (double) stepsPerRevolution) / 360);
        }

        void moveDeg(double deg){
            // Serial.println(String(deg) + "deg => " + String(degreeToStep(deg)) + " step");
            stepMotor(degreeToStep(deg));
            // Serial.println("Stepping finished for " + String(deg) + "\n______________________________");
        }
        
        void moveToDeg(double targetDeg){
            int stepsToTarget = (int)((long)degreeToStep(targetDeg) - (long)offset);
            stepMotor(stepsToTarget);
        }

    private:
        uint32_t stepsPerRevolution;
        uint8_t stepperStepPin;
        uint8_t stepperDirPin;
        uint8_t switchPin;
        RotationDirection switchDirection = CCW;
        RotationDirection rotationDirection = CW;
        uint32_t stepDelay;
        uint16_t offset = 0;
        uint16_t maxStep = INT16_MAX;

        void setRotationDirection(RotationDirection dir){
            rotationDirection = dir;
            digitalWrite(stepperDirPin, (bool)dir);
        }
        void toggleRotationDirection(RotationDirection dir){
            rotationDirection = dir==CW? CCW : CW;
            digitalWrite(stepperDirPin, rotationDirection);
        }
};