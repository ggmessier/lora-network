#include "mbed.h"
/**
 ******************************************************************************
 * @file    main.cpp
 * @author  NW
 * @version V1.0.0
 * @date    07-May-2019
 * @brief   Modified Example application for using the X_NUCLEO_IKS01A2 
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board
 *          Using a ticker timer and event queuing.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/ 

/* Includes */
#include "mbed.h"
#include "XNucleoIKS01A2.h"

/* Configures PC serial port */
Serial pc(USBTX, USBRX);
DigitalOut led1(LED1);

uint8_t id;
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue printfQueue;
EventQueue eventQueue;

/* Defines the timer */
Timer t;
time_t whattime;

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if (fractPart >= i) {
      break;
    }
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}

/* Reads the sensor board sensors */
/* Reads the current board time */
/* Compares the current time to the last time it was measured */
void Read_Sensors() {
  // this runs in the normal priority thread
  led1 = !led1;
  hum_temp->get_temperature(&temp1);
  hum_temp->get_humidity(&humid1);
  press_temp->get_temperature(&temp2);
  press_temp->get_pressure(&humid2);
  magnetometer->get_m_axes(axes1);
  accelerometer->get_x_axes(axes2);
  acc_gyro->get_x_axes(axes3);
  acc_gyro->get_g_axes(axes4);
  usTime2 = usTime1;
  usTime1 = t.read_high_resolution_us();
  usDeltaTime = usTime1 - usTime2;
  whattime = time(NULL);
}

/* Prints to the serial console */
void Print_Sensors() {
  // this runs in the lower priority thread
  printf("----------------------\r\n");
  printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", print_double(buffer1, temp1), print_double(buffer2, humid1));
  printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", print_double(buffer3, temp2), print_double(buffer4, humid2));
  printf("LSM303AGR [mag/mgauss]:  %6ld, %6ld, %6ld\r\n", axes1[0], axes1[1], axes1[2]);
  printf("LSM303AGR [acc/mg]:  %6ld, %6ld, %6ld\r\n", axes2[0], axes2[1], axes2[2]);
  printf("LSM6DSL [acc/mg]:      %6ld, %6ld, %6ld\r\n", axes3[0], axes3[1], axes3[2]);
  printf("LSM6DSL [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes4[0], axes4[1], axes4[2]);
  printf("----------------------\r\n");
  printf("Current us Time: %lld us\n\r", usTime1);
  printf("Delta us Timer: %lld us\n\r", usDeltaTime);
  printf("Current Epoch Time: %u\n\r", (unsigned int)whattime);
}

/* Converts standard time into Epoch time */
time_t asUnixTime(int year, int mon, int mday, int hour, int min, int sec) {
    struct tm   t;
    t.tm_year = year - 1900;
    t.tm_mon =  mon - 1;        // convert to 0 based month
    t.tm_mday = mday;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;            // Is Daylight saving time on? 1 = yes, 0 = no, -1 = unknown
 
    return mktime(&t);          // returns seconds elapsed since January 1, 1970 (begin of the Epoch)
}


/* Simple main function */
int main() {
  pc.baud(115200);
  /* Sets an arbitrary starting date */
  /* TODO: read in from serial console to start */
  set_time(asUnixTime(2019,03,24,16,10,30));

  /* resets and starts the timer */
  t.reset();
  t.start();
  usTime1 = t.read_high_resolution_us();
  
  /* Enable all sensors */
  hum_temp->enable();
  press_temp->enable();
  magnetometer->enable();
  accelerometer->enable();
  acc_gyro->enable_x();
  acc_gyro->enable_g();
  wait(1.5);
  printf("\r\n--- Starting new run ---\r\n");

  hum_temp->read_id(&id);
  printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
  press_temp->read_id(&id);
  printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);
  magnetometer->read_id(&id);
  printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
  accelerometer->read_id(&id);
  printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
  acc_gyro->read_id(&id);
  printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
  printf("---\r\n");
    
    // normal priority thread for other events
  Thread eventThread(osPriorityNormal);
  eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
  
  // low priority thread for calling printf()
  Thread printfThread(osPriorityLow);
  printfThread.start(callback(&printfQueue, &EventQueue::dispatch_forever));
  
  // call read_sensors 1 every second, automatically defering to the eventThread
  Ticker ReadTicker;
  Ticker PrintTicker;
  ReadTicker.attach(eventQueue.event(&Read_Sensors), 1.0f);
  PrintTicker.attach(printfQueue.event(&Print_Sensors), 1.0f);
 
  wait(osWaitForever);
}