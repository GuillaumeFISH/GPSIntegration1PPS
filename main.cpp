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
#include "mbed-os/mbed.h"
#include "X_NUCLEO_IKS01A2/XNucleoIKS01A2.h"
#include "MBed_Adafruit-GPS-Library/MBed_Adafruit_GPS.h"
#include "mbed-os/platform/mbed_error.h"

#define MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED true

//Configures pins and serial port
Serial pc(USBTX, USBRX);
InterruptIn PPS(A1,PullDown);
DigitalOut led(LED1);

uint8_t id;
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;
int int_time=0;
bool set_rtc = true;
//Payload
uint8_t Buffer[32];

//Configure GPS
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here


/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue printfQueue;
EventQueue eventQueue;
EventQueue GPSQueue;

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
  //led = !led;
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
    pc.printf("%u ", (unsigned int)whattime);
    pc.printf("%d ", int_time);
    pc.printf("%lld ", usTime1);
    pc.printf("%7s %s ", print_double(buffer1, temp1), print_double(buffer2, humid1));
    pc.printf("%7s %s ", print_double(buffer3, temp2), print_double(buffer4, humid2));
    pc.printf("%6ld %6ld %6ld ", axes1[0], axes1[1], axes1[2]);
    pc.printf("%6ld %6ld %6ld", axes2[0], axes2[1], axes2[2]);
    pc.printf("%6ld %6ld %6ld", axes3[0], axes3[1], axes3[2]);
    pc.printf("%6ld %6ld %6ld\r\n", axes4[0], axes4[1], axes4[2]);
    /*
    //Building the payload
    //Microcontroller time
    Buffer[0] = (int)((whattime >> 24) & 0xFF) ;
    Buffer[1] = (int)((whattime >> 16) & 0xFF) ;
    Buffer[2] = (int)((whattime >> 8) & 0XFF);
    Buffer[3] = (int)((whattime & 0XFF));
    //
    */
}

/* Converts standard time into Epoch time. Could delete this if no longer needed.*/
time_t asUnixTime(int year, int mon, int mday, int hour, int min, int sec, int msec) {
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

void Detect1PPS() {
    led = !led;
}

//Function that attempts to fetch gps information and parse it
//Will be triggered by pps eventually or by a high priority queue
void GPS_data() {
    //pc.printf("Inside GPS Data Function");
    int received = 0;
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                continue;
            }
            else
                received++;
                //pc.printf("Received...%d", received);  
        }
    } while(received != 2);
    int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);    
    Read_Sensors();
}

/* Simple main function */
int main() {
    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                    //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
    
    myGPS.sendCommand(PMTK_AWAKE);
    pc.printf("Wake Up GPS...\r\n");
    //PPS.fall(&Detect1PPS);

    wait(2);
    //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    
    pc.printf("Connection established at 115200 baud...\r\n");
    
    /* Prints the date in standard format, used to test the gps data fetched
    pc.printf("year: %d", myGPS.year);
    pc.printf("month: %d", myGPS.month);
    pc.printf("day: %d", myGPS.day);
    pc.printf("hour: %d", myGPS.hour);
    pc.printf("minute: %d", myGPS.minute);
    pc.printf("seconds: %d", myGPS.seconds);
    pc.printf("\r\n%d", int_time);
    */
  
    /* resets and starts the timer */
    t.reset();
    t.start();
    usTime1 = t.read_high_resolution_us();

    pc.printf("Timer Reset and Started...\r\n");
  
    /* Enable all sensors */
    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();
    wait(1.5);

    pc.printf("Sensors Enabled...\r\n");

    
    do{
        GPS_data();   //queries the GPS
        if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        pc.printf("Waiting for GPS Fix...%d\r\n",myGPS.fix);
        pc.printf("Fix Quality...%d\r\n",myGPS.fixquality);
        wait(1);

    }while(myGPS.fix == false);
    //set rtc when called for the first time (sync to gps on startup)
    int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
    set_time(int_time);
    
    //Prints headers for each measurement. Unsure if the acc, mag, and gyro
    //directions are accurate. (Don't know if accx actually measures in x direction)
    pc.printf("\r\nEPOC GPST RUNT TEP1 HUM TEP2 PRES MAGX MAGY MAGZ AC1X AC1Y AC1Z AC2X AC2Y AC2Z GYRX GYRY GYRZ\r\n");
    
    // Set Nucleos time according to gps data most recently fetched, 
    // could maybe get rid of. Depends on how we pps will be used.
    //Currently replaced by the last two lines in GPS_data
    //GPS_data();
    //int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);
    //set_time(int_time);

    pc.printf("Current GPS Epoch Time...%d\r\n",int_time);
    
    //High priority thread for gps
    Thread gpsThread(osPriorityNormal);
    gpsThread.start(callback(&GPSQueue, &EventQueue::dispatch_forever));
    
    // normal priority thread for other events
    Thread eventThread(osPriorityNormal);
    eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
  
    // low priority thread for calling printf()
    Thread printfThread(osPriorityLow);
    printfThread.start(callback(&printfQueue, &EventQueue::dispatch_forever));
    
    // call read_sensors 1 every second, automatically defering to the eventThread
    //Ticker GPSTicker;
    //Ticker ReadTicker;
    //Ticker PrintTicker;
    //GPSTicker.attach(GPSQueue.event(&GPS_data), 1.0f);
    //ReadTicker.attach(eventQueue.event(&Read_Sensors), 1.0f);
    //PrintTicker.attach(printfQueue.event(&Print_Sensors), 1.0f);

    PPS.rise(GPSQueue.event(&GPS_data));
    //PPS.rise(eventQueue.event(&Read_Sensors));
    PPS.fall(printfQueue.event(&Print_Sensors));


    //PPS.fall(GPSQueue.event(&Detect1PPS));
    
    wait(osWaitForever);
}