#include "mbed.h"
#include "MPU9250.h"
 
char buffer[14];
 
MPU9250 mpu9250;

Timer t;
        
int main() {
  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C    
  
  t.start();        
    
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x71\n\r");
  
  if (whoami == 0x71) { // WHO_AM_I should always be 0x68
    printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
    printf("MPU9250 is online...\n\r");
    sprintf(buffer, "WHO_AM_I 0x%x", whoami);
    wait_us(1000);
    
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    printf("x-axis self test: acceleration trim within : %f p of factory value\n\r", SelfTest[0]);  
    printf("y-axis self test: acceleration trim within : %f p of factory value\n\r", SelfTest[1]);  
    printf("z-axis self test: acceleration trim within : %f p of factory value\n\r", SelfTest[2]);  
    printf("x-axis self test: gyration trim within : %f p of factory value\n\r", SelfTest[3]);  
    printf("y-axis self test: gyration trim within : %f p of factory value\n\r", SelfTest[4]);  
    printf("z-axis self test: gyration trim within : %f p of factory value\n\r", SelfTest[5]);  
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    printf("x gyro bias = %f\n\r", gyroBias[0]);
    printf("y gyro bias = %f\n\r", gyroBias[1]);
    printf("z gyro bias = %f\n\r", gyroBias[2]);
    printf("x accel bias = %f\n\r", accelBias[0]);
    printf("y accel bias = %f\n\r", accelBias[1]);
    printf("z accel bias = %f\n\r", accelBias[2]);
    wait_us(2000);
    mpu9250.initMPU9250(); 
    printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of accelerometer, gyroscope, and temperature
    printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
    printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
    wait_us(1000);
  } else {
    printf("Could not connect to MPU9250: \n\r");
    printf("%#x \n",  whoami);
 
    sprintf(buffer, "WHO_AM_I 0x%x", whoami);
 
    while(1); // Loop forever if communication doesn't happen
  }
 
  mpu9250.getAres(); // Get accelerometer sensitivity
  mpu9250.getGres(); // Get gyro sensitivity
  printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
  printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
 
  while(1) {
    // If intPin goes high, all data registers have new data
    if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    
        mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes - accelBias[1];   
        az = (float)accelCount[2]*aRes - accelBias[2];  
    
        mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes - gyroBias[1];  
        gz = (float)gyroCount[2]*gRes - gyroBias[2];   
    }
    mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);

    printf("ax = %f", ax); 
    printf(" ay = %f", ay); 
    printf(" az = %f  mg\n\r", az); 

    printf("gx = %f", gx); 
    printf(" gy = %f", gy); 
    printf(" gz = %f  rad/s\n\r", gz); 

    tempCount = mpu9250.readTempData();  // Read the adc values
    temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
    printf(" temperature = %f  C\n\r", temperature);    
    
    ThisThread::sleep_for(1s);
  }
}