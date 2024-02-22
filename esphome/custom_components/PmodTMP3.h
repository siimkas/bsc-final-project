#include "esphome.h"
#include <Wire.h> 
#define adress 0x48

class PmodTMP3 : public PollingComponent {
    private:

    
        float TMP3_getTemp_C(void) {
            // Launch of the measurement
            Wire.beginTransmission(adress);
            Wire.endTransmission();
            delay(10);

            //ask for measured data
            Wire.requestFrom(adress, 2);
            int tmp = (Wire.read() << 8) | Wire.read(); //retrieve data
            tmp >>= 4;  //use only 12-bits
            return (tmp / 16.00);
        }

            //get temperature in farenheit
        float TMP3_getTemp_F(void) {
            return TMP3_getTemp_C() * 9 / 5 + 32.2;
        }

            // Initialization of Pmod TMP3 module
        void TMP3_init(void) {
            //set 12-bit resolution
            Wire.beginTransmission(adress);
            Wire.write(0x01);
            Wire.write(0x60);
            Wire.endTransmission();

            //set address pointer
            Wire.beginTransmission(adress);
            Wire.write(0x00);
            Wire.endTransmission();
        }

    public:

        PmodTMP3() : PollingComponent(1000) { }

        Sensor *tempC = new Sensor();

        void setup() override{ 
            TMP3_init(); // initialization of Pmod TMP3
        }

        void update() override {
            tempC->publish_state(TMP3_getTemp_C());
        }

};