#include <Wire.h>
#include <VL53L1X.h>
#include "common/mavlink.h"

#define UWB_TAG_FRAME_NOT_FOUND 0
#define UWB_TAG_FRAME_HEADER 1
#define UWB_TAG_FRAME_FOUND 2
#define UWB_TAG_FRAME_OK 3
#define UWB_TAG_FRAME_BAD 4

VL53L1X sensor;
VL53L1X sensor2;
uint16_t r1 = 0;
uint16_t r2 = 0;
byte state = UWB_TAG_FRAME_NOT_FOUND;
byte offset = 0;
byte chk_sum = 0;
uint32_t dist0 = 0;

void setup()
{
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
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setAddress(0x2b);

  digitalWrite(9, HIGH);
  delay(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(80000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(100);

  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(80000);
  sensor2.startContinuous(100);
}

void loop()
{
  uint32_t good_dist = 0;
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
                good_dist = dist0;
                //print 'ok', dist0
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
  //String msg = "range:";
  if (sensor.dataReady()) r1 = sensor.read();
  if (sensor2.dataReady()) r2 = sensor2.read();
  if (r1 > 0 && r2 > 0) {
    //msg = msg + r1 + ":" + r2;    
    //Serial.println(msg);
    r1 = r2 = 0;
  }
  //if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
}
