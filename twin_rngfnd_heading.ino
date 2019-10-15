#include <Wire.h>
#include <VL53L1X.h>
#include "common/mavlink.h"

//#define RIGHT_WALL

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

VL53L1X sensor_r;
VL53L1X sensor_f;
uint16_t rr = 0;
uint16_t rf = 0;
byte state = UWB_TAG_FRAME_NOT_FOUND;
byte offset = 0;
byte chk_sum = 0;
uint32_t dist0 = 0;
float dist_wall_m = 0;
float dist_uwb_m = 0;
float yaw = 0;  
unsigned long prv_ts = 0;

void setup()
{  
  //randomSeed(analogRead(3));

  pinMode(SHUT_GPIO_RF, OUTPUT);
  pinMode(SHUT_GPIO_RR, OUTPUT);
  pinMode(SHUT_GPIO_LF, OUTPUT);
  pinMode(SHUT_GPIO_LR, OUTPUT);
  digitalWrite(SHUT_GPIO_RF, LOW);
  digitalWrite(SHUT_GPIO_RR, LOW);
  digitalWrite(SHUT_GPIO_LF, LOW);
  digitalWrite(SHUT_GPIO_LR, LOW);
  delay(10);     

  Serial1.begin(115200);
  Serial1.setTimeout(10);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C  

  Serial.println("init sensor_f...");
#ifdef RIGHT_WALL
  digitalWrite(SHUT_GPIO_RF, HIGH);
#else
  digitalWrite(SHUT_GPIO_LF, HIGH);
#endif
  delay(10);
  sensor_f.setTimeout(500);  
  while (!sensor_f.init())
  {
    Serial.println("Failed to detect and initialize sensor_f!");
#ifdef RIGHT_WALL    
    digitalWrite(SHUT_GPIO_RF, LOW);
    delay(10);
    digitalWrite(SHUT_GPIO_RF, HIGH);
    delay(10);
#else
    digitalWrite(SHUT_GPIO_LF, LOW);
    delay(10);
    digitalWrite(SHUT_GPIO_LF, HIGH);
    delay(10);
#endif    
  }
  sensor_f.setAddress(0x2b);
  Serial.println("sensor_f ok");
  
  Serial.println("init sensor_r...");
#ifdef RIGHT_WALL
  digitalWrite(SHUT_GPIO_RR, HIGH);
#else
  digitalWrite(SHUT_GPIO_LR, HIGH);  
#endif  
  delay(10);
  sensor_r.setTimeout(500);
  while (!sensor_r.init())
  {
    Serial.println("Failed to detect and initialize sensor_r!");
#ifdef RIGHT_WALL
    digitalWrite(SHUT_GPIO_RR, LOW);
    delay(10);
    digitalWrite(SHUT_GPIO_RR, HIGH);
    delay(10);
#else
    digitalWrite(SHUT_GPIO_LR, LOW);
    delay(10);
    digitalWrite(SHUT_GPIO_LR, HIGH);
    delay(10);
#endif    
  }
  Serial.println("sensor_r ok");
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor_r.setDistanceMode(VL53L1X::Long);
  sensor_r.setMeasurementTimingBudget(80000);

  sensor_f.setDistanceMode(VL53L1X::Long);
  sensor_f.setMeasurementTimingBudget(80000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor_r.startContinuous(90);
  sensor_f.startContinuous(90);
  
#ifdef RIGHT_WALL
  Serial.println("right wall");
#else
  Serial.println("left wall");
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
  //if (good_dist >0) {
  //  String msg = "dist:";
  //  Serial.println(msg + good_dist);
  //}  
  if (sensor_r.dataReady()) rr = sensor_r.read();
  if (sensor_f.dataReady()) rf = sensor_f.read();
#if 0
  if (rr > 0 || rf > 0) {
    String msg = "rng:";
    msg = msg + rr + ":" + rf;
    Serial.println(msg);    
  }  
#endif  
  if (rr > 0 && rf > 0) {    
    double theta = 0;
    if (rf < rr) {
      theta = atan2(rr - rf, DIST_TWIN_RNGFND_MM);
      yaw = PI * 0.5 + theta;
      dist_wall_m = (rf + rr) * 0.5 * cos(theta) * 0.001;
    } else {      
      theta = atan2(rf - rr, DIST_TWIN_RNGFND_MM);
      yaw = PI * 0.5 - theta;
      dist_wall_m = (rf + rr) * 0.5 * cos(theta) * 0.001;
    }
#ifndef RIGHT_WALL
      dist_wall_m = -dist_wall_m;    
#endif
#if 1
    String msg = "rng:";
    msg = msg + rr + ":" + rf + ":" + theta + ":" + dist_wall_m;
    Serial.println(msg);
#endif
    rr = 0;
    rf = 0;
  }
  if (dist_wall_m > 0 && dist_uwb_m > 0 && dist_uwb_m < 1000) {
#if 0
    String dbg = "send ";
    dbg = dbg + dist_wall_m + "," + dist_uwb_m + "," + yaw;
    Serial.println(dbg);
#endif    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_vision_position_estimate_pack(0, 0, &msg, micros(), dist_wall_m, dist_uwb_m, 0, 0, 0, yaw);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
    dist_wall_m = 0;
    dist_uwb_m = 0;    
  }
}
