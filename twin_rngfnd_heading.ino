#include <Wire.h>
#include <VL53L1X.h>
#include "common/mavlink.h"

//#define RIGHT_WALL
#define WAIT_FC_PWM

#define UWB_TAG_FRAME_NOT_FOUND 0
#define UWB_TAG_FRAME_HEADER 1
#define UWB_TAG_FRAME_FOUND 2
#define UWB_TAG_FRAME_OK 3
#define UWB_TAG_FRAME_BAD 4

#define DIST_TWIN_RNGFND_MM 300.0

#define SHUT_GPIO_RF 19
#define SHUT_GPIO_RR 18
#define SHUT_GPIO_LF 20
#define SHUT_GPIO_LR 21

VL53L1X sensor_rr;
VL53L1X sensor_rf;
VL53L1X sensor_lr;
VL53L1X sensor_lf;
uint16_t rr = 0;
uint16_t rf = 0;
byte state = UWB_TAG_FRAME_NOT_FOUND;
byte offset = 0;
byte chk_sum = 0;
uint32_t dist0 = 0;
float dist_wall_m = NAN;
float dist_uwb_m = 0;
float yaw = 0;  
unsigned long prv_ts = 0;

void setup()
{  
  //randomSeed(analogRead(3));
#ifdef WAIT_FC_PWM  
  pinMode(10, INPUT);
#endif  

  pinMode(SHUT_GPIO_RF, OUTPUT);
  pinMode(SHUT_GPIO_RR, OUTPUT);
  pinMode(SHUT_GPIO_LF, OUTPUT);
  pinMode(SHUT_GPIO_LR, OUTPUT);
  digitalWrite(SHUT_GPIO_RF, LOW);
  digitalWrite(SHUT_GPIO_RR, LOW);
  digitalWrite(SHUT_GPIO_LF, LOW);
  digitalWrite(SHUT_GPIO_LR, LOW);

  Serial1.begin(115200);
  Serial1.setTimeout(10);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C  

  Serial.println("init sensor_rf...");
  digitalWrite(SHUT_GPIO_RF, HIGH);
  delay(10);
  sensor_rf.setTimeout(500);  
  while (!sensor_rf.init())
  {
    Serial.println("Failed to detect and initialize sensor_rf!");
    while (1);
  }
  delay(100);
  sensor_rf.setAddress(0x37);
  Serial.println("sensor_rf ok");
  
  Serial.println("init sensor_rr...");
  digitalWrite(SHUT_GPIO_RR, HIGH);
  delay(10);
  sensor_rr.setTimeout(500);
  while (!sensor_rr.init())
  {
    Serial.println("Failed to detect and initialize sensor_rr!");
    while (1);
  }
  delay(100);
  sensor_rr.setAddress(0x35);
  Serial.println("sensor_rr ok");

  Serial.println("init sensor_lf...");
  digitalWrite(SHUT_GPIO_LF, HIGH);
  delay(10);
  sensor_lf.setTimeout(500);  
  while (!sensor_lf.init())
  {
    Serial.println("Failed to detect and initialize sensor_lf!");
    while (1);
  }
  delay(100);
  sensor_lf.setAddress(0x33);
  Serial.println("sensor_lf ok");

  Serial.println("init sensor_lr...");
  digitalWrite(SHUT_GPIO_LR, HIGH);
  delay(10);
  sensor_lr.setTimeout(500);
  while (!sensor_lr.init())
  {
    Serial.println("Failed to detect and initialize sensor_lr!");
    while (1);
  }
  Serial.println("sensor_lr ok");

#ifdef RIGHT_WALL      
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor_rf.setDistanceMode(VL53L1X::Long);
  sensor_rr.setMeasurementTimingBudget(80000);
  
  sensor_rr.setDistanceMode(VL53L1X::Long);
  sensor_rr.setMeasurementTimingBudget(80000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor_rf.startContinuous(90);
  sensor_rr.startContinuous(90);  
#else  
  sensor_lf.setDistanceMode(VL53L1X::Long);
  sensor_lr.setMeasurementTimingBudget(80000);
  
  sensor_lr.setDistanceMode(VL53L1X::Long);
  sensor_lr.setMeasurementTimingBudget(80000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor_lf.startContinuous(90);
  sensor_lr.startContinuous(90);  
#endif  
  
#ifdef WAIT_FC_PWM  
  int pwm_value = 1000;  
  while (pwm_value < 1500) {
    delay(10);
    pwm_value = pulseIn(10, HIGH);    
  }
  Serial.println("setup done");
#endif  
}

void loop()
{  
  byte buf[128];
  size_t count = Serial1.readBytes(buf, 128);
  for (int i = 0; i < count; i++) {
    byte c = buf[i];
    if (state == UWB_TAG_FRAME_NOT_FOUND) {
      if (c == 0x55) {
        state = UWB_TAG_FRAME_HEADER;
        chk_sum = c;
      }
    } else if (state == UWB_TAG_FRAME_HEADER) {
      if (c == 0x1) {
          state = UWB_TAG_FRAME_FOUND;
          offset = 1;
          chk_sum = chk_sum + c;
      } else {
          state = UWB_TAG_FRAME_NOT_FOUND;
      }
    } else if (state == UWB_TAG_FRAME_FOUND) {
        offset = offset + 1;
        if (offset == 22)
            dist0 = c;
        else if (offset == 23)
            dist0 = dist0 | (c << 8);
        else if (offset == 24)
            dist0 = dist0 | (c << 16);
        else if (offset == 127) {
            chk_sum = chk_sum & 0xff;
            if (c == chk_sum) {
                state = UWB_TAG_FRAME_NOT_FOUND;
                dist_uwb_m = dist0 * 0.001;
#if 0
                String msg = "dist0:";
                Serial.println(msg + dist0);
#endif
            } else {
                state = UWB_TAG_FRAME_NOT_FOUND;
                //print 'bad', c, chk_sum
            }
        }
        chk_sum = chk_sum + c;
    }
  }
#ifdef RIGHT_WALL    
  if (sensor_rf.dataReady()) rf = sensor_rf.read();
  if (sensor_rr.dataReady()) rr = sensor_rr.read();
#else
  if (sensor_lf.dataReady()) rf = sensor_lf.read();
  if (sensor_lr.dataReady()) rr = sensor_lr.read();
#endif  
  if (rr > 0 && rf > 0) {    
    double theta = 0;
#ifdef RIGHT_WALL    
    if (rf < rr) {
      theta = atan2(rr - rf, DIST_TWIN_RNGFND_MM);
      yaw = theta;
      dist_wall_m = (rf + rr) * 0.5 * cos(theta) * -0.001;
    } else {      
      theta = atan2(rf - rr, DIST_TWIN_RNGFND_MM);
      yaw = 2 * PI - theta;
      dist_wall_m = (rf + rr) * 0.5 * cos(theta) * -0.001;
    }
#else
    if (rf < rr) {
      theta = atan2(rr - rf, DIST_TWIN_RNGFND_MM);
      yaw = 2 * PI - theta;
      dist_wall_m = (rf + rr) * 0.5 * cos(theta) * 0.001;
    } else {      
      theta = atan2(rf - rr, DIST_TWIN_RNGFND_MM);
      yaw = theta;
      dist_wall_m = (rf + rr) * 0.5 * cos(theta) * 0.001;
    }
#endif    
#if 0
    String msg = "rng:";
    msg = msg + rf + ":" + rr + ":" + theta + ":" + dist_wall_m;
    Serial.println(msg);
#endif
    rr = 0;
    rf = 0;
  }
  if (!isnan(dist_wall_m) && dist_uwb_m > 0 && dist_uwb_m < 1000) {
#if 0
    String dbg = "send ";
    dbg = dbg + dist_wall_m + "," + dist_uwb_m + "," + yaw;
    Serial.println(dbg);
#endif    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_vision_position_estimate_pack(0, 0, &msg, micros(), dist_uwb_m, dist_wall_m, 0, 0, 0, yaw);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
    dist_wall_m = NAN;
    dist_uwb_m = 0;    
  }
}
