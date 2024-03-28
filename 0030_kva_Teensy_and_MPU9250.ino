/* MPU9250_MS5637_AHRS_t3
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the MPU9250+MS5637 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MPU9250+MS5637 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#define VERSION_NUMBER 1.0

/*  COMPILE DIRECTIVES */
//#define COMPILE_MS5637 // compile for pressure sensor or not


/* definition of debug levels
    mainly replacing Serial.print and Serial.println by nothing when not needed
    that is:
     DEBUG_LEVEL_0 used to debug HMI
     DEBUG_LEVEL_1 used to debug handle_emergency
*/ 
#define DEBUG_LEVEL_0 0 // 1 means debug level 0 is active 0 means it's not
#if DEBUG_LEVEL_0 == 1
  #define debug(x) Serial.print(x)
  #define debug_2arg(x, y) Serial.print(x, y)
  #define debugln(x) Serial.println(x)
  #define debugln_2arg(x, y) Serial.println(x, y)
#else
  #define debug(x)
  #define debug_2arg(x, y)
  #define debugln(x)
  #define debugln_2arg(x, y)
#endif

#include "Wire.h"   
// this to request an end of transmission with restart condition
#define I2C_NOSTOP false

//#include <i2c_t3.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "mpu9250_definitions.h"
#ifdef COMPILE_MS5637
  #include "MS5637_code.h"
#endif
#include "i2c_general_functions.h"
#include "mpu9250_code.h"

bool volatile display_paused{false};     // display is paused
void isr_fonction() {
  display_paused = false;
}

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno and Teensy 3.2 as well
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno and Teensy 3.2 as well
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 4, 3);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!

/*******************************************************************
 SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP
*/

#define CLIC_PIN 2 // pin for clics, interrupt switch
void setup()
{
  Wire.begin();
  Wire.setClock(400000);


//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
//  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(100);
  #if DEBUG_LEVEL_0 == 1
  Serial.begin(38400);
  #endif
  
  // Set up the interrupt pins, its set as active high, push-pull
  pinMode(intPin, INPUT);
  pinMode(CLIC_PIN, INPUT);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  attachInterrupt(CLIC_PIN, isr_fonction, RISING);
  //attachInterrupt(digitalPinToInterrupt(CLIC_PIN), isr_fonction, RISING);
  
  display.begin(); // Initialize the display
  display.setContrast(25); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("MPU9250");

  //display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("9-DOF 16-bit");

  display.setCursor(0, 20);
  display.print("motion sensor");

  display.setCursor(0,30);
  display.print("60 ug LSB");

  display.setCursor(0,40);
  display.print("paused");

  display.display();
  display_paused = true;
  while(display_paused);
  //delay(8000);

/* Display tests */
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("max 14 caracte");
  display.setCursor(0,8);
  display.print("ligne 2");

  display.setCursor(0,16);
  display.print("ligne 3");
  display.print("<--");

  display.setCursor(0,24);
  display.print("ligne 4");

  display.setCursor(0,32);
  display.print("ligne 5");

  display.setCursor(0,40);
  display.print("ligne 6 paused");

  display.display();

  display_paused = true;
  while(display_paused);

  display.display();

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  I2Cscan();// look for I2C devices on the bus
    
  // Read the WHO_AM_I register, this is a good test of communication
  debugln("MPU9250 9-axis motion sensor...");
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  debug("MPU9250 ");
  debug("I AM ");
  debug_2arg(c, HEX);
  debug(" I should be ");
  debugln_2arg(0x71, HEX);

  display.setCursor(20,0);
  display.print("MPU9250");

  display.setCursor(0,10);
  display.print("Unit ID:");

  //display.setCursor(0,20);
  display.setCursor(50,10);
  display.print(c, HEX);

  display.setCursor(0,30);
  //display.print("I Should Be");
  display.print("paused");
  //display.setCursor(0,40);
  //display.print(0x71, HEX); 
  display.display();
  display_paused = true;
  while(display_paused);
  //delay(1000); 

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    debugln("MPU9250 is online...");
    
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    debug("x-axis self test: acceleration trim within : ");
    debug_2arg(SelfTest[0],1);
    debugln("% of factory value");
    debug("y-axis self test: acceleration trim within : ");
    debug_2arg(SelfTest[1],1);
    debugln("% of factory value");
    debug("z-axis self test: acceleration trim within : ");
    debug_2arg(SelfTest[2],1);
    debugln("% of factory value");
    debug("x-axis self test: gyration trim within : ");
    debug_2arg(SelfTest[3],1);
    debugln("% of factory value");
    debug("y-axis self test: gyration trim within : ");

    debug_2arg(SelfTest[4],1);
    debugln("% of factory value");
    debug("z-axis self test: gyration trim within : ");
    debug_2arg(SelfTest[5],1);
    debugln("% of factory value");
    delay(1000);
    
   // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   getMres();
    
   debugln(" Calibrate gyro and accel");
   accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
   debugln("accel biases (mg)");
   debugln(1000. * accelBias[0]);
   debugln(1000. * accelBias[1]);
   debugln(1000. * accelBias[2]);
   debugln("gyro biases (dps)");
   debugln(gyroBias[0]);
   debugln(gyroBias[1]);
   debugln(gyroBias[2]);

  display.clearDisplay();
     
  display.setCursor(0, 0); display.print("MPU9250 bias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(66, 24); display.print("o/s");   
 
  display.display();
  delay(10000);  
   
  initMPU9250(); 
  debugln("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
  debug("AK8963 ");
  debug("I AM ");
  debug_2arg(d, HEX);
  debug(" I should be ");
  debugln_2arg(0x48, HEX);
  display.clearDisplay();
  display.setCursor(20,0);
  display.print("AK8963");
  display.setCursor(0,10);
  display.print("I AM");
  display.setCursor(0,20);
  display.print(d, HEX);  
  display.setCursor(0,30);
  display.print("I Should Be");
  display.setCursor(0,40);
  display.print(0x48, HEX);  
  display.display();
  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  initAK8963(magCalibration);
  debugln("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  
  magcalMPU9250(magBias, magScale);
  debugln("AK8963 mag biases (mG)");
  debugln(magBias[0]);
  debugln(magBias[1]);
  debugln(magBias[2]); 
  debugln("AK8963 mag scale (mG)");
  debugln(magScale[0]);
  debugln(magScale[1]);
  debugln(magScale[2]); 
  delay(2000); // add delay to see results before serial spew of data
   
//  debugln("Calibration values: ");
  debug("X-Axis sensitivity adjustment value ");
  debugln_2arg(magCalibration[0], 2);
  debug("Y-Axis sensitivity adjustment value ");
  debugln_2arg(magCalibration[1], 2);
  debug("Z-Axis sensitivity adjustment value ");
  debugln_2arg(magCalibration[2], 2);
  
  display.clearDisplay();
  display.setCursor(20,0);
  display.print("AK8963");
  display.setCursor(0,10);
  display.print("ASAX ");
  display.setCursor(50,10);
  display.print(magCalibration[0], 2);
  display.setCursor(0,20);
  display.print("ASAY ");
  display.setCursor(50,20);
  display.print(magCalibration[1], 2);
  display.setCursor(0,30);
  display.print("ASAZ ");
  display.setCursor(50,30);
  display.print(magCalibration[2], 2);
  display.display();
  delay(1000);  

#ifdef COMPILE_MS5637
  // Reset the MS5637 pressure sensor
  MS5637Reset();
  delay(100);
  debugln("MS5637 pressure sensor reset...");
  
  // Read PROM data from MS5637 pressure sensor
  MS5637PromRead(Pcal);
  debugln("PROM dta read:");
  debug("C0 = ");
  debugln(Pcal[0]);
  unsigned char refCRC = Pcal[0] >> 12;
  debug("C1 = ");
  debugln(Pcal[1]);
  debug("C2 = ");
  debugln(Pcal[2]);
  debug("C3 = ");
  debugln(Pcal[3]);
  debug("C4 = ");
  debugln(Pcal[4]);
  debug("C5 = ");
  debugln(Pcal[5]);
  debug("C6 = ");
  debugln(Pcal[6]);
  
  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  debug("Checksum = ");
  debug(nCRC);
  debug(" , should be ");
  debugln(refCRC);  
  
  display.clearDisplay();
  display.setCursor(20,0);
  display.print("MS5637");
  display.setCursor(0,10);
  display.print("CRC is ");
  display.setCursor(50,10);
  display.print(nCRC);
  display.setCursor(0,20);
  display.print("Should be ");
  display.setCursor(50,30);
  display.print(refCRC);
  display.display();
  delay(1000);  
#endif
  attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250
  }
  else
  {
    debug("Could not connect to MPU9250: 0x");
    debugln_2arg(c, HEX);
    while(1); // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // If intPin goes high, all data registers have new data
   if(newData == true) {  // On interrupt, read data
     newData = false;  // reset newData flag
     readMPU9250Data(MPU9250Data); // INT cleared on any read
 //   readAccelData(accelCount);  // Read the x/y/z adc values
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)MPU9250Data[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)MPU9250Data[1] * aRes - accelBias[1];   
    az = (float)MPU9250Data[2] * aRes - accelBias[2];  
   
 //   readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)MPU9250Data[4] * gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)MPU9250Data[5] * gRes;  
    gz = (float)MPU9250Data[6] * gRes;   
  
    readMagData(magCount);  // Read the x/y/z adc values
   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    if(newMagData == true) {
      newMagData = false; // reset newMagData flag
      mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
      my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];  
      mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];  
      mx *= magScale[0];
      my *= magScale[1];
      mz *= magScale[2]; 
    } 
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
  // we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
  // positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
  // function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(-ax, ay, az, gx * PI/180.0f, -gy * PI/180.0f, -gz * PI/180.0f,  my,  -mx, mz);
//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
    debug("ax = ");
    debug((int)1000 * ax);  
    debug(" ay = ");
    debug((int)1000 * ay); 
    debug(" az = ");
    debug((int)1000 * az);
    debugln(" mg");
    debug("gx = ");
    debug_2arg( gx, 2); 
    debug(" gy = ");
    debug_2arg( gy, 2); 
    debug(" gz = ");
    debug_2arg( gz, 2);
    debugln(" deg/s");
    debug("mx = ");
    debug( (int)mx ); 
    debug(" my = ");
    debug( (int)my ); 
    debug(" mz = ");
    debug( (int)mz );
    debugln(" mG");
    
    debug("q0 = ");
    debug(q[0]);
    debug(" qx = ");
    debug(q[1]); 
    debug(" qy = ");
    debug(q[2]); 
    debug(" qz = ");
    debugln(q[3]); 

    tempCount = readTempData();  // Read the gyro adc values
    temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    debug("Gyro temperature is ");
    debug_2arg(temperature, 1);
    debugln(" degrees C"); // Print T values to tenths of s degree C

 #ifdef COMPILE_MS5637

    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
  //
  // Second order corrections
    if(Temperature > 20) 
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
 // End of second order corrections
 //
     Temperature = Temperature - T2/100;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa
  
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = Pressure; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;

    float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));
   
    debug("Digital temperature value = ");
    debug( (float)Temperature, 2);
    debugln(" C"); // temperature in degrees Celsius
    /*
    debug("Digital temperature value = ");
    debug(9.*(float) Temperature/5. + 32., 2);
    debugln(" F"); // temperature in degrees Fahrenheit
    */
    debug("Digital pressure value = ");
    debug((float) Pressure, 2);
    debugln(" mbar");// pressure in millibar
    debug("Altitude = ");
    debug(altitude, 2); debugln(" feet");
 #endif // #ifdef COMPILE_MS5637


   // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:
 //   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
 //   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
 //   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
 //   pitch *= 180.0f / PI;
 //   yaw   *= 180.0f / PI; 
 //   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
 //   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
 //   roll  *= 180.0f / PI;
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;

    debug("Yaw, Pitch, Roll: ");
    debug_2arg(yaw, 2);
    debug(", ");
    debug_2arg(pitch, 2);
    debug(", ");
    debugln_2arg(roll, 2);

    debug("Grav_x, Grav_y, Grav_z: ");
    debug_2arg(-a31 * 1000, 2);
    debug(", ");
    debug_2arg(-a32 * 1000, 2);
    debug(", ");
    debug_2arg(a33 * 1000, 2);
    debugln(" mg");
    debug("Lin_ax, Lin_ay, Lin_az: ");
    debug_2arg(lin_ax * 1000, 2);
    debug(", ");
    debug_2arg(lin_ay * 1000, 2);
    debug(", ");
    debug_2arg(lin_az * 1000, 2);
    debugln(" mg");
    
    debug("rate = ");
    debug_2arg((float)sumCount / sum, 2);
    debugln(" Hz");
   
    display.clearDisplay();    
 
    display.setCursor(0, 0);
    display.print(" x   y   z ");

    display.setCursor(0,  8);
    display.print((int)(1000 * ax)); 
    display.setCursor(24, 8);
    display.print((int)(1000 * ay)); 
    display.setCursor(48, 8);
    display.print((int)(1000 * az)); 
    display.setCursor(72, 8);
    display.print("mg");
    
    display.setCursor(0,  16);
    display.print((int)(gx)); 
    display.setCursor(24, 16);
    display.print((int)(gy)); 
    display.setCursor(48, 16);
    display.print((int)(gz)); 
    display.setCursor(66, 16);
    display.print("o/s");    

    display.setCursor(0,  24);
    display.print((int)(mx)); 
    display.setCursor(24, 24);
    display.print((int)(my)); 
    display.setCursor(48, 24);
    display.print((int)(mz)); 
    display.setCursor(72, 24);
    display.print("mG");    
 
    display.setCursor(0,  32);
    display.print((int)(yaw)); 
    display.setCursor(24, 32);
    display.print((int)(pitch)); 
    display.setCursor(48, 32);
    display.print((int)(roll)); 
    display.setCursor(66, 32);
    display.print("ypr");  
  
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
#ifdef COMPILE_MS5637
    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz"); 
#endif //#ifdef COMPILE_MS5637

    display.display();
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    } // if delt_t > 500
} // end loop
