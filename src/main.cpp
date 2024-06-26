#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ClosedCube_OPT3001.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

ClosedCube_OPT3001 sensor_A;
ClosedCube_OPT3001 sensor_B;
ClosedCube_OPT3001 sensor_C;   // NAFFED!!! //
LSM9DS1 imu;

#define SENSOR_ADDRESS_A 0x47  // ADDR - SCL 0x47(71)
#define SENSOR_ADDRESS_B 0x46  // ADDR - SDA 0x46(70)
#define SENSOR_ADDRESS_C 0x44  // ADDR - GND 0x44(68)
#define SENSOR_MAG_ADRESS 0x1E // 0x1C if SDO_M is low
#define SENSOR_ACCEL_GYRO_ADRESS 0x6B // 0x6A if SDO_AG is low
//#define PRINT_RAW // parameter in void print---()
#define PRINT_CALCULATED // parameter in void print---()
#define DECLINATION 0.016 // in Newcastle upon time 2024


/* Sun Sensor */
void OPT3001_Setup();
void configureSensor(ClosedCube_OPT3001 sensor);
void printResult(String text, OPT3001 result);
void printError(String text, OPT3001_ErrorCode error);
void printLux(float valA, float valB, float valC);
void pError(String text, OPT3001 result);
float getValue(OPT3001 result);
float angle_BA(float senA, float senB);
float angle_CA(float senA, float senC);

/* ISU */
void printGyro();
void printMag();
void printAccel();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

unsigned long starttime;  //timer

// ==============================================================================
void setup() 
{
    Wire.begin();
    Serial.begin(9600);

    sensor_A.begin(SENSOR_ADDRESS_A);
    sensor_B.begin(SENSOR_ADDRESS_B);
    sensor_C.begin(SENSOR_ADDRESS_C);

    OPT3001_Setup();    

    if (imu.begin() == false)
    {
        Serial.println("Failed to communicate with IMU");
        while (1);
    }
    starttime = millis(); // timer
}

// ==============================================================================
void loop()
{
    if (millis() - starttime >= 60000) // milliseconds  
    {
        while(true);
    } //timer

    /*sun sensor*/
	OPT3001 result_A = sensor_A.readResult();
    delay(10);
    OPT3001 result_B = sensor_B.readResult();
    delay(10);
    OPT3001 result_C = sensor_C.readResult();

    float valA = getValue(result_A);
    float valB = getValue(result_B);
    float valC = getValue(result_C);

    printLux(valA, valB, valC);

    if (valA != -1)
    {
        if (valB != -1){
            float theta = angle_BA(valA, valB);
            Serial.print("Angle AB: ");
            Serial.print(theta);
            Serial.println(" Degrees.");
        }
        if (valC != -1){
            float psi = angle_CA(valA, valC);
            Serial.print("Angle AC: ");
            Serial.print(psi);
            Serial.println(" Degrees."); 
        }
    }
    else
    {
        pError("Sensor A", result_A);
        Serial.println("System not valid, Sensor A privatol");
    }

    /*IMU*/
    if ( imu.magAvailable())
    {
        imu.readMag();
    }

    printMag();
    printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.my, imu.mz);
    Serial.println();

    delay(500);
}

// ==============================================================================
/* Sun Sensor */
void OPT3001_Setup()
{
    Serial.print("OPT3001 Manufacturer ID: ");
    Serial.println(sensor_A.readManufacturerID());
    Serial.print("OPT3001 Device ID");
    Serial.println(sensor_A.readDeviceID());

    configureSensor(sensor_A); 
    Serial.println("Sensor A");
    printResult("High-Limit", sensor_A.readHighLimit());
    printResult("Low-Limit", sensor_A.readLowLimit());
    Serial.println("----"); 

    configureSensor(sensor_B);
    Serial.println("Sensor B");
    printResult("High-Limit", sensor_B.readHighLimit());
    printResult("Low-Limit", sensor_B.readLowLimit());
    Serial.println("----");

    configureSensor(sensor_C);
    Serial.println("Sensor C");
    printResult("High-Limit", sensor_C.readHighLimit());
    printResult("Low-Limit", sensor_C.readLowLimit());
    Serial.println("----");
}
void configureSensor(ClosedCube_OPT3001 sensor) 
{
    OPT3001_Config newConfig;
    
    newConfig.RangeNumber = B1100;	
    newConfig.ConvertionTime = B0;
    newConfig.Latch = B1;
    newConfig.ModeOfConversionOperation = B11;

    OPT3001_ErrorCode errorConfig = sensor.writeConfig(newConfig);
   
    if (errorConfig != NO_ERROR)
    {
        printError("OPT3001 configuration", errorConfig);
    }
    else 
    {
        OPT3001_Config sensorConfig = sensor.readConfig();
        Serial.println("OPT3001 Current Config:");
        Serial.println("------------------------------");
        
        Serial.print("Conversion ready (R):");
        Serial.println(sensorConfig.ConversionReady,HEX);

        Serial.print("Conversion time (R/W):");
        Serial.println(sensorConfig.ConvertionTime, HEX);

        Serial.print("Fault count field (R/W):");
        Serial.println(sensorConfig.FaultCount, HEX);

        Serial.print("Flag high field (R-only):");
        Serial.println(sensorConfig.FlagHigh, HEX);

        Serial.print("Flag low field (R-only):");
        Serial.println(sensorConfig.FlagLow, HEX);

        Serial.print("Latch field (R/W):");
        Serial.println(sensorConfig.Latch, HEX);

        Serial.print("Mask exponent field (R/W):");
        Serial.println(sensorConfig.MaskExponent, HEX);

        Serial.print("Mode of conversion operation (R/W):");
        Serial.println(sensorConfig.ModeOfConversionOperation, HEX);

        Serial.print("Polarity field (R/W):");
        Serial.println(sensorConfig.Polarity, HEX);

        Serial.print("Overflow flag (R-only):");
        Serial.println(sensorConfig.OverflowFlag, HEX);

        Serial.print("Range number (R/W):");
        Serial.println(sensorConfig.RangeNumber, HEX);

        Serial.println("------------------------------");
    }    
}
void printResult(String text, OPT3001 result) 
{
	if (result.error == NO_ERROR) 
  {
      Serial.print(text);
      Serial.print(": ");
      Serial.print(result.lux);
      Serial.println(" lux");
	}
	else 
  {
	  	printError(text,result.error);
	}
}
void printLux(float valA, float valB, float valC)
{
    //Serial.println("Sensor A, Sensor B, Sensor C ");
    if (valA != -1){
        Serial.print(valA);
    }else{Serial.print("Error");}
    Serial.print(", ");
    if (valB != -1){
       Serial.print(valB);
    }else{Serial.print("Error");}
    Serial.print(", ");
    if (valC != -1){
       Serial.print(valC);
    }//else{Serial.print("Error");}
    Serial.println(" Lux");
}
void printError(String text, OPT3001_ErrorCode error) 
{
    Serial.print(text);
    Serial.print(": [ERROR] Code #");
    Serial.println(error);
}
float getValue(OPT3001 result)
{
    if (result.error == NO_ERROR) {
        return result.lux;
    } else {
        return -1;
    }  
}
void pError(String text, OPT3001 result) 
{
    if (result.error != NO_ERROR)
    {
        Serial.print(text);
        Serial.print(": [ERROR] Code #");
        Serial.println(result.error);
    }
}
float angle_BA(float senA, float senB)
{
    float theta = PI / 4 - atan(senB / senA);
    theta *= 180 / PI;
    return theta;
}
float angle_CA(float senA, float senC)
{
    float psi = PI / 4 - atan(senC / senA);
    psi *= 180 / PI;
    return psi;
}
/* IMU */
void printGyro()
{
    // Now we can use the gx, gy, and gz variables as we please.
    // Either print them as raw ADC values, or calculated in DPS.
    Serial.print("G: ");

    #ifdef PRINT_CALCULATED
        // If you want to print calculated values, you can use the
        // calcGyro helper function to convert a raw ADC value to
        // DPS. Give the function the value that you want to convert.
        Serial.print(imu.calcGyro(imu.gx), 2);
        Serial.print(", ");
        Serial.print(imu.calcGyro(imu.gy), 2);
        Serial.print(", ");
        Serial.print(imu.calcGyro(imu.gz), 2);
        Serial.println(" deg/s");
    #elif defined PRINT_RAW
        Serial.print(imu.gx);
        Serial.print(", ");
        Serial.print(imu.gy);
        Serial.print(", ");
        Serial.println(imu.gz);
    #endif
}
void printAccel()
{
    // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
    Serial.print("A: ");

    #ifdef PRINT_CALCULATED
        // If you want to print calculated values, you can use the
        // calcAccel helper function to convert a raw ADC value to
        // g's. Give the function the value that you want to convert.
        Serial.print(imu.calcAccel(imu.ax), 2);
        Serial.print(", ");
        Serial.print(imu.calcAccel(imu.ay), 2);
        Serial.print(", ");
        Serial.print(imu.calcAccel(imu.az), 2);
        Serial.println(" g");
    #elif defined PRINT_RAW
        Serial.print(imu.ax);
        Serial.print(", ");
        Serial.print(imu.ay);
        Serial.print(", ");
        Serial.println(imu.az);
    #endif
}
float AbsoluteMag(float x, float y, float z)
{
    float AbsMag = pow((imu.calcMag(x)*100)*(imu.calcMag(x)*100)+ 
                       (imu.calcMag(y)*100)*(imu.calcMag(y)*100)+
                       (imu.calcMag(z)*100)*(imu.calcMag(z)*100), 0.5);
    return AbsMag;
}
void printMag()
{
    // Now we can use the mx, my, and mz variables as we please.
    // Either print them as raw ADC values, or calculated in Gauss.
    Serial.print("M: ");

    #ifdef PRINT_CALCULATED
        // If you want to print calculated values, you can use the
        // calcMag helper function to convert a raw ADC value to
        // Gauss. Give the function the value that you want to convert.
        Serial.print(imu.calcMag(imu.mx)*100, 2);
        Serial.print(", ");
        Serial.print(imu.calcMag(imu.my)*100, 2);
        Serial.print(", ");
        Serial.print(imu.calcMag(imu.mz)*100, 2);
        Serial.print(" Abs: ");
        Serial.print(AbsoluteMag(imu.mx,imu.my,imu.my), 2);
        Serial.println(" microTesla");
    #elif defined PRINT_RAW
        Serial.print(imu.mx);
        Serial.print(", ");
        Serial.print(imu.my);
        Serial.print(", ");
        Serial.println(imu.mz);
    #endif
}
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    /* Calculate pitch, roll, and heading.
    Pitch/roll calculations taken from this app note:
    http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
    Heading calculations taken from this app note:
    http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf */

    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float heading;

    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    Serial.print("Pitch, Roll: ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    Serial.print("Heading: "); Serial.println(heading, 2);
}
