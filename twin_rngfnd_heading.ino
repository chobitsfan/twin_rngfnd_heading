#include <Wire.h>
#include <VL53L1X.h>
#include "common/mavlink.h"

#define UWB_TAG_FRAME_NOT_FOUND 0
#define UWB_TAG_FRAME_HEADER 1
#define UWB_TAG_FRAME_FOUND 2
#define UWB_TAG_FRAME_OK 3
#define UWB_TAG_FRAME_BAD 4

#define TUNNEL_WIDTH_MM 1000.0f
#define DIST_TWIN_RNGFND_MM 10.0f

//#define MY_DEBUG

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
  randomSeed(analogRead(0));
  
  delay(3000);
  
  Serial.begin(9600);
  Serial1.setTimeout(10);
  Serial1.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW);  
  digitalWrite(8, LOW);
  delay(500);

  digitalWrite(8, HIGH);
  delay(500);
  sensor_f.setTimeout(500);
  if (!sensor_f.init())
  {
    Serial.println("Failed to detect and initialize sensor_f!");
    while (1);
  }
  sensor_f.setAddress(0x2b);

  digitalWrite(9, HIGH);
  delay(500);
  if (!sensor_r.init())
  {
    Serial.println("Failed to detect and initialize sensor_r!");
    while (1);
  }
  
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
  sensor_r.startContinuous(100);
  
  sensor_f.startContinuous(100);
}

void loop()
{  
#if 0
  unsigned long now_ts = millis();
  if (now_ts - prv_ts > 100) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_vision_position_estimate_pack(0, 0, &msg, micros(), 1.0f, 1.0f + random(10)*0.1, 0, 0, 0, 1.0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
    prv_ts = now_ts;
  }
#else
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
                //String msg = "dist0:";
                //Serial.println(msg + dist0);
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
  if (rr > 0 && rf > 0) {    
    float angle_a = 0.0f;
    if (rf < rr) {
      angle_a = atan2f(rr, rf);
      yaw = PI * 0.5 - PI * 0.25 + angle_a;
      dist_wall_m = rf * sinf(angle_a) * 0.001;
    } else {      
      angle_a = atan2f(rf, rr);
      yaw = PI * 0.5 + PI * 0.25 - angle_a; 
      dist_wall_m = rr * sinf(angle_a) * 0.001;
    }        
    //String msg = "rng:";
    //msg = msg + yaw + ":" + dist_wall_m;
    //Serial.println(msg);
    rr = 0;
    rf = 0;
  }
  if (dist_wall_m > 0 && dist_uwb_m > 0) {
#ifdef MY_DEBUG    
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
#endif
}
