#include "esphome.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

static const char *TAG_ = "ADXL345";
bool r_interrupt;
bool r_single_tap;
bool r_double_tap;
bool r_activity;
bool r_inactivity;
bool r_free_fall;

class ADXL345Sensor : public PollingComponent {
  private:
    static void IRAM_ATTR interrupted()
    {

      r_interrupt = true;
      Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

      uint8_t interrupts = accel.readRegister(ADXL345_REG_INT_SOURCE);
      
      if (interrupts & (1 << 6)) {
        r_single_tap = true;
      }
      if (interrupts & (1 << 5)) {
        r_double_tap = true;
      }
      if (interrupts & (1 << 4)) {
        r_activity = true;
      }
      if (interrupts & (1 << 3)) {
        r_inactivity = true;
      }
      if (interrupts & (1 << 2)) {
        r_free_fall = true;
      }
    }

    void displaySensorDetails(void)
    {
      sensor_t sensor;
      accel.getSensor(&sensor);
      ESP_LOGI(TAG_,"------------------------------------");
      ESP_LOGI(TAG_,"Sensor:       %s",sensor.name);
      ESP_LOGI(TAG_,"Driver Ver:   %u",sensor.version);
      ESP_LOGI(TAG_,"Unique ID:    %u",sensor.sensor_id);
      ESP_LOGI(TAG_,"Max Value:    %f m/s^2",sensor.max_value);
      ESP_LOGI(TAG_,"Min Value:    %f m/s^2",sensor.min_value);
      ESP_LOGI(TAG_,"Resolution:   %f m/s^2",sensor.resolution);  
      ESP_LOGI(TAG_,"------------------------------------");
      delay(500);
    }

  public:
    Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
    // Sensor *accel_x_sensor = new Sensor();
    // Sensor *accel_y_sensor = new Sensor();
    // Sensor *accel_z_sensor = new Sensor();

    ADXL345Sensor() : PollingComponent(200) { }

    BinarySensor *activity = new BinarySensor();
    BinarySensor *single_tap = new BinarySensor();
    BinarySensor *double_tap = new BinarySensor();
    BinarySensor *free_fall = new BinarySensor();

    void setup() override {
    accel.begin();
    uint8_t pctrl = accel.readRegister(ADXL345_REG_POWER_CTL);

    if (!(pctrl & (1 << 3)) || true) {
      displaySensorDetails();

      ESP_LOGI(TAG_, "configuring ADXL");
      // offsets
      accel.writeRegister(ADXL345_REG_OFSX, 0);
      accel.writeRegister(ADXL345_REG_OFSY, 0);
      accel.writeRegister(ADXL345_REG_OFSZ, 0);
      
      accel.setDataRate(ADXL345_DATARATE_12_5_HZ);

      // Enable Low-Power mode for additional powersavings when using slower data-rates. 
      // gotta work around the library abit
      // accel.writeRegister(ADXL345_REG_BW_RATE, 0b10000 + ADXL345_DATARATE_100_HZ);

      // Datasheet Table 8 Typical Current Consumption vs. Data Rate, Low Power Mode 
      // for when using 400, 200, 100, 50, 25, 12.5 datarates
      // Downside are worse noise figures

      /* Set the range to whatever is appropriate for your project */
      accel.setRange(ADXL345_RANGE_2_G);

      // 10 * 62.5mg
      accel.writeRegister(ADXL345_REG_THRESH_ACT, 10);      
      // 2 * 62.5mg
      accel.writeRegister(ADXL345_REG_THRESH_INACT, 2); 
      // time inactive 5s
      accel.writeRegister(ADXL345_REG_TIME_INACT, 15);
      // ac-coupled, all axis enabled
      accel.writeRegister(ADXL345_REG_ACT_INACT_CTL, 255);
      // Free-fall
      accel.writeRegister(ADXL345_REG_THRESH_FF, 7); // (5 - 9) recommended - 62.5mg per increment
      accel.writeRegister(ADXL345_REG_TIME_FF, 30); // (20 - 70) recommended - 5ms per increment
      // tap detection all axis
      accel.writeRegister(0x2a,0b111);

      // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
        // adxl.setTapThreshold(50);           // 62.5 mg per increment
      accel.writeRegister(0x1d,50);
        // adxl.setTapDuration(15);            // 625 µs per increment
      accel.writeRegister(0x21,15);
        // adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
      accel.writeRegister(0x22,80);
        // adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
      accel.writeRegister(0x23,200);

      //pinMode(14, INPUT_PULLUP);
      //attachInterrupt(0, interrupted, LOW);

      // map all interrupts to int1 (binary flag, 1 is int2, 0 is int1)
      accel.writeRegister(ADXL345_REG_INT_MAP, 0);

      enable_activity_interrupt(true);
      enable_inactivity_interrupt(true);
      enable_double_interrupt(true);
      enable_single_tap_interrupt(true);

      // TODO powersaving mode
      // TODO can we only publish x,y,z on webgui (or choose what options we need)
      // TODO more configurable settings...
      // TODO can we avoid interrupt 2
      // TODO can ESPhome even do interrupts? (only on esp32 hardware wise anyway...)
      // TODO can we do more than 1 sensor?
      // TODO set default motion state

      accel.writeRegister(ADXL345_REG_FIFO_CTL, 0);
    }
  }

  void enable_activity_interrupt(bool enabled)
  {
    ESP_LOGI(TAG_, "activity interrupt: %u", enabled);
    uint8_t int_enable = accel.readRegister(ADXL345_REG_INT_ENABLE);
    if (enabled) {
      int_enable = int_enable | (1 << 4);
    } else {
      int_enable = int_enable & ~(1 << 4);
    }
    accel.writeRegister(ADXL345_REG_INT_ENABLE, int_enable);
  }

  void enable_inactivity_interrupt(bool enabled)
  {
    ESP_LOGI(TAG_, "inactivity interrupt: %u", enabled);
    uint8_t int_enable = accel.readRegister(ADXL345_REG_INT_ENABLE);
    if (enabled) {
      int_enable = int_enable | (1 << 3);
    } else {
      int_enable = int_enable & ~(1 << 3);
    }
    accel.writeRegister(ADXL345_REG_INT_ENABLE, int_enable);
  }

  void enable_single_tap_interrupt(bool enabled)
  {
    ESP_LOGI(TAG_, "single_tap interrupt: %u", enabled);
    uint8_t int_enable = accel.readRegister(ADXL345_REG_INT_ENABLE);
    if (enabled) {
      int_enable = int_enable | (1 << 6);
    } else {
      int_enable = int_enable & ~(1 << 6);
    }
    accel.writeRegister(ADXL345_REG_INT_ENABLE, int_enable);
  }

  void enable_double_interrupt(bool enabled)
  {
    ESP_LOGI(TAG_, "double_tap interrupt: %u", enabled);
    uint8_t int_enable = accel.readRegister(ADXL345_REG_INT_ENABLE);
    if (enabled) {
      int_enable = int_enable | (1 << 5);
    } else {
      int_enable = int_enable & ~(1 << 5);
    }
    accel.writeRegister(ADXL345_REG_INT_ENABLE, int_enable);
  }

  void update() override {
    if(r_single_tap && !r_double_tap){
      single_tap->publish_state(false);
      r_single_tap = false;
    }
    if(r_double_tap){
      double_tap->publish_state(false);
      r_double_tap = false;
    }
    if(r_activity){
      activity->publish_state(false);
      r_activity = false;
    }
    if(r_free_fall){
      free_fall->publish_state(false);
      r_free_fall = false;
    }

    sensors_event_t event;
    accel.getEvent(&event);
    // NOTE I don't need these but maybe you do?
    // accel_x_sensor->publish_state(event.acceleration.x);
    // accel_y_sensor->publish_state(event.acceleration.y);
    // accel_z_sensor->publish_state(event.acceleration.z);
    
    uint8_t interrupts = accel.readRegister(ADXL345_REG_INT_SOURCE);
    //ESP_LOGI(TAG_, "%u",interrupts % 128);

    if (interrupts & (1 << 6)) {
      r_single_tap = true;
      r_interrupt = true;
    }
    if (interrupts & (1 << 5)) {
      r_double_tap = true;
      r_interrupt = true;
    }
    if (interrupts & (1 << 4)) {
      r_activity = true;
      r_interrupt = true;
    }
    if (interrupts & (1 << 3)) {
      r_inactivity = true;
      r_interrupt = true;
    }
    if (interrupts & (1 << 2)) {
      r_free_fall = true;
      r_interrupt = true;
    }


    if (r_interrupt){
      r_interrupt = false;

      if(r_single_tap && !r_double_tap){
        single_tap->publish_state(true);
        //r_single_tap = false;
      }
      if(r_double_tap){
        double_tap->publish_state(true);
        //r_double_tap = false;
      }
      if(r_activity){
        activity->publish_state(true);
        //r_activity = false;
      }
      if(r_inactivity){
        activity->publish_state(false);
        //r_inactivity = false;
      }
      if(r_free_fall){
        free_fall->publish_state(true);
        //r_free_fall = false;
      }
    }

  }

};