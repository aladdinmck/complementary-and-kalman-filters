#include <Wire.h>
#include <math.h>
#include <MLX90393.h> 
#include <SparkFunLSM6DSO.h>



MLX90393 MLX;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)
LSM6DSO LSM;

#define ACCELEROMETER_SENSITIVITY 32543.83
#define GYROSCOPE_SENSITIVITY 65.536
//#define M_PI 3.14159265359
#define dt 0.01

float magX, magY, magZ, magTemp, accelX, accelY, accelZ, gyrX, gyrY, gyrZ, IMUTemp;
uint16_t accelXRaw, accelYRaw, accelZRaw, gyrXRaw, gyrYRaw, gyrZRaw, accelRange, gyroRange;
float gyroDataRange, accelDataRange;
float aPitch;
float aRoll;

float pitch;
float roll;

float angleX = 0;
float angleY = 0;
float angleZ = 0;

float gRoll = 0;
float gPitch = 90;
float gYaw = 0;

float mYaw = 0;


//float dt = millis();

//float pitchAcc;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  //failsafe error checking to prevent irri\tation with no output. basically checks to make sure both sensors are connected and running
  if (!LSM.begin()) {
    //Serial.println("IMU not found. Check wiring!");
    delay(500);
  }
  if (!MLX.begin()) {
    //Serial.println("Magnetometer not found. Check wiring!");
    delay(500);
  }
  
  //gain is set as 1 for this example, basically meaning no gain is applied to signal.
  MLX.setGainSel(1);

  //resolution is set to 0 for each axis, representing the finest resolution the sensor is capable of.
  MLX.setResolution(0, 0, 0); //x, y, z

  //oversampling and digital filtering are untouched
  MLX.setOverSampling(0);
  MLX.setDigitalFiltering(0);
  
  if (LSM.initialize(BASIC_SETTINGS)) {
    //Serial.println("Loaded Basic Settings");
  }
  delay(500);
}//end of setup




//Below are functions for pulling data from sensors. Each 3-axis "piece" is passed into individual functions to make it easier to manipulate. 
//Simply call the function in the loop and then decide which data you want to print/use

//function to read magnetometer data
void read_Mag_Data() {
  MLX.readData(data);
  magX = data.x;
  magY = data.y;
  magZ = data.z;
}

//function to read magnetometer temp data
void read_Mag_Temp() {
  MLX.readData(data);
  magTemp = data.t;
}

//function to read IMU accelerometer data
void read_LSM_Accel_Data() {
  accelX = LSM.readFloatAccelX();
  accelY = LSM.readFloatAccelY();
  accelZ = LSM.readFloatAccelZ();
  accelXRaw = LSM.readRawAccelX();
  accelYRaw = LSM.readRawAccelY();
  accelZRaw = LSM.readRawAccelZ();
  accelRange = LSM.getAccelRange();
}

//function to read IMU gyro data
void read_LSM_Gyro_Data() {
  gyrX = LSM.readFloatGyroX();
  gyrY = LSM.readFloatGyroY();
  gyrZ = LSM.readFloatGyroZ();
  gyrXRaw = LSM.readRawGyroX();
  gyrYRaw = LSM.readRawGyroY();
  gyrZRaw = LSM.readRawGyroZ();
  gyroRange = LSM.getGyroRange();
}

//function to read IMU temp data
void read_LSM_Temp() {
  IMUTemp = LSM.readTempF();
}

void ComplementaryFilter(float accX, float accY, float accZ, float gX, float gY, float gZ, float *pitch, float *roll) {
    float pitchAcc, rollAcc;

    *pitch += ((float)gX / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll += ((float)gY / GYROSCOPE_SENSITIVITY) * dt;  // Angle around the Y-axis

    int forceMagnitudeApprox = abs(accX) + abs(accY) + abs(accZ);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accY, (float)accZ) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accX, (float)accZ) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }

    //printf("&pitch   = %p\n", (void *) &pitch);
    //printf("&roll    = %p\n", (void *) &roll);

    //Serial.println((void *) &pitch);
    //Serial.println((void *) &roll);
    
    //Serial.println(String(**pitch));
    //Serial.println(String(**roll));
}

//loop below polls functions above to gather and then print data to serial monitor
void loop()
{
  //function calling section. Will "pull" all data (accelX, gyrX etc.) from functions above to be called individually later in the loop
  //ultimately gives the user easy power over what data is used vs what isn't
  read_LSM_Accel_Data();
  read_LSM_Gyro_Data();
  read_LSM_Temp();
  read_Mag_Data();
  read_Mag_Temp();

  // Pieter Jan code
  
//ComplementaryFilter(accelX, accelX, accelX, gyrX, gyrX, gyrX, pitch, roll);

  //this same structure applies to all functions. Just change acceleration and the value name to your respective function. 
  //Serial.println("Acceleration Values:");
  /*
  Serial.print("Acceleration Range: ");
  Serial.println(accelRange);
  Serial.print("Acceleration X: ");
  Serial.println(accelX);
  Serial.print("Acceleration Y: ");
  Serial.println(accelY);
  Serial.print("Acceleration Z: ");
  Serial.println(accelZ);

  Serial.print("Gyrometer Range: ");
  Serial.println(gyroRange);
  Serial.println("Gyrometer Values:");
  Serial.print("Gyrometer X: ");
  Serial.println(gyrX);
  Serial.print("Gyrometer Y: ");
  Serial.println(gyrY);
  Serial.print("Gyrometer Z: ");
  Serial.println(gyrZ);
  
  Serial.println("Magnetometer Values:");
  Serial.print("Magnetometer X: ");
  Serial.println(magX);
  Serial.print("Magnetometer Y: ");
  Serial.println(magY);
  Serial.print("Magnetometer Z: ");
  Serial.println(magZ);
  */



/*
  accelXRaw = LSM.readRawAccelX();
  accelYRaw = LSM.readRawAccelY();
  accelZRaw = LSM.readRawAccelZ();
*/
  
  
/*
  angleX = 0.98 * ((pitch + gyrXRaw) * 0.01) + 0.02 * (accelX);
  
  angleY = 0.98 * ((pitch + gyrYRaw) * 0.01) + 0.02 * (accelY);
  
  //angleZ = 0.98 * (pitch + gyrZRaw * 0.01) + 0.02 * (accelZ);

  Serial.println(String("     AngleX: ") + angleX + String("     AngleY: ") + angleY + String("     AngleZ: ") + angleZ);
  
  if (angleX > 200) {
    angleX = angleX / 10; 
  }
  if (angleY > 200) {
    angleY = angleY / 10; 
  }
  if (angleZ > 200) {
    angleZ = angleX / 10; 
  }
  */

  /*
  float mag_norm = sqrt((magX * magX) + (magY * magY) + (magZ * magZ));
  magX = magX / mag_norm;
  magY = magY / mag_norm;
  magZ = magZ / mag_norm;
  */
  
  // magnetometer yaw
  //mYaw = 10 * atan2((-magY * cos(roll) + magZ * sin(roll)), (magX * cos(pitch) + magY * sin(pitch) * sin(roll) + magZ * sin(pitch) * cos(roll)));


  
  //Serial.println(String("Roll: ") + roll + String("    Pitch: ") + pitch);
  //Serial.println(String("accelXRaw: ") + gyrX + String("        accelYRaw: ") + gyrY + String("        accelZRaw: ") + gyrZ);

  // accelerometer roll and pitch 
  aRoll = atan2(accelYRaw, accelZRaw) * 180 / M_PI;
  aPitch = acos(accelX) * 180 / M_PI;

  roll = aRoll;
  pitch = aPitch;

  // gyrometer roll, pitch and yaw
  gRoll += ((gyrX + gyrY * sin(aRoll) * tan(aPitch) + gyrZ * cos(aRoll) * tan(aPitch)) * 0.04);
  gPitch += ((gyrY * cos(aRoll) - gyrZ * sin(aPitch)) * 0.04); // use 0.04 if dt isnt enough
  gYaw += (gyrX + gyrY * sin(aRoll) * (1 / cos(aPitch)) + gyrZ * cos(aRoll) * (1 / cos(aPitch)) * dt);

  // Complementary Filter Variable 
  float rollCF = 0.90 * (roll + gyrX * dt) + 0.1 * (accelX);
  float pitchCF = 0.90 * (pitch + gyrY * dt) + 0.1 * (accelY);
  

  Serial.println(String("        pitchCF: ") + abs(pitchCF) + String("        pitchAcc: ") + abs(aPitch) + String("        gPitch: ") + abs(gPitch));  
  // String(" tch: ") + abs(aPitch) + 
  
  if (gRoll > 180) {
    gRoll = 0;
  }
  if (gPitch > 180) {
    gPitch = 90;  
  }
  if (gYaw > 180) {
    gYaw = 0;
  }
}
