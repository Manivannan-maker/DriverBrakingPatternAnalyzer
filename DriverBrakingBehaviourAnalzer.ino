

#include <Arduino_LSM9DS1.h>
#include <String.h>
#include <Wire.h>


#define CONVERT_G_TO_MS2    9.80665f
//Constants:
const int flexPin = A1; //pin A0 to read analog input
const int SpeedPin=A2; // speed analog

uint16_t FullbrakeUpperthreshold=0;
uint16_t FullbrakeLowerthreshold=0;
uint8_t brakepedalpressedVal=0;

float time_ms=0;

//Variables:
int FlexSensorvalue; //save analog value
float avgFlexValue=0;

boolean CalibrateFlexBrakepedalvalue=0 ;
boolean CalibrateSimulateSpeedSensorvalue=0;
boolean internalBrakecaln=0;
boolean I2cCommunicationEna=1;
float DefaultvehicleSpeed=25; // 25kmph

//Speed variables
float VehicleSpeed=DefaultvehicleSpeed;

float MinPotentiometerVal=0;

float avgSpeedValue=0;

float SpeedSensorvalue=0;

float  MaxPotentiometerVal=0;
float ReadSpeedSensor=0;

// Speed simulation

boolean SpeedSimulationEnable =1;

const int vehicleSpeedSimulatorPin = 2;     
const int vehicleSpeedResetPin=3; 

int vehicleSpeedSimulatorState=0;
int vehicleSpeedResetState=0;

float speedreducingfactor=0.108;





int y=0;

//acceleration
  float x_axis, y_axis, z_axis;

int Linearinterpolation( float x , float x0, float x1, float y0, float y1);


void setup() {
  Serial.begin(115200);
    Wire.begin();                              //Begins I2C communication at pin (A4,A5)
     // initialize the pushbutton pin as an input:
  pinMode(vehicleSpeedSimulatorPin, INPUT_PULLUP);
  pinMode(vehicleSpeedResetPin, INPUT_PULLUP);

 
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

 
}

void loop() {


  //Calibrating Brake pedal value
  if(CalibrateFlexBrakepedalvalue ==1 )
    {
         delay(5000);

       Serial.println("Press Full Brake pedal for 7 seconds");               //Print 
    delay(2000);

    
    
    for(int i =0 ; i<50;i++)
    {
       FlexSensorvalue = analogRead(flexPin);         //Read and save analog value from potentiometer
     
       avgFlexValue=FlexSensorvalue+avgFlexValue;
       Serial.println(FlexSensorvalue);               //Print value
       delay(100);      
    }

    FullbrakeUpperthreshold=avgFlexValue/50;
    avgFlexValue=0;
    Serial.print("FullbrakeUpperthreshold  :  ");
    Serial.print("\t");
    Serial.println(FullbrakeUpperthreshold);
    

     Serial.println("Release Brake pedal for 7 seconds");               //Print 
    delay(5000);

      for(int i =0 ; i<50;i++)
    {
       FlexSensorvalue = analogRead(flexPin);         //Read and save analog value from potentiometer
     
       avgFlexValue=FlexSensorvalue+avgFlexValue;
       Serial.println(FlexSensorvalue);               //Print value
       delay(100);      
    }
    FullbrakeLowerthreshold=avgFlexValue/50;
    avgFlexValue=0;
    Serial.print("FullbrakeLowerthreshold  :  ");
    Serial.print("\t");
    Serial.println(FullbrakeLowerthreshold);
 delay(3000);
    CalibrateFlexBrakepedalvalue=0;
    
    }


    if(CalibrateSimulateSpeedSensorvalue==1)
    {
  delay(2000);

       Serial.println("Tune Max Value");               //Print 
    delay(2000);
 for(int i =0 ; i<50;i++)
    {
       SpeedSensorvalue = analogRead(SpeedPin);         //Read and save analog value from potentiometer
     
       avgSpeedValue=SpeedSensorvalue+avgSpeedValue;
       Serial.println(SpeedSensorvalue);               //Print value
       delay(100);      
    }

    MaxPotentiometerVal=avgSpeedValue/50;
    avgSpeedValue=0;
    Serial.print("MaxPotentiometerVal  :  ");
    Serial.print("\t");
    Serial.println(MaxPotentiometerVal);
    

     Serial.println("Tune Min value");               //Print 
    delay(5000);

      for(int i =0 ; i<50;i++)
    {
       SpeedSensorvalue = analogRead(SpeedPin);         //Read and save analog value from potentiometer
     
       avgSpeedValue=SpeedSensorvalue+avgSpeedValue;
       Serial.println(SpeedSensorvalue);               //Print value
       delay(100);      
    }
    MinPotentiometerVal=avgSpeedValue/50;
    avgSpeedValue=0;
    Serial.print("MinPotentiometerVal  :  ");
    Serial.print("\t");
    Serial.println(MinPotentiometerVal);
 delay(5000);
    

      CalibrateSimulateSpeedSensorvalue=0;
    }

  

   

// Brake caln

     if(internalBrakecaln==1)
     {

           FlexSensorvalue = analogRead(flexPin);         //Read and save analog value 

   
  // Bringing to threshold value
  if(FlexSensorvalue<FullbrakeUpperthreshold)
  {
    FlexSensorvalue=FullbrakeUpperthreshold;
  }
 
  if(FlexSensorvalue>FullbrakeLowerthreshold)
  {
    FlexSensorvalue=FullbrakeLowerthreshold;
  }
  
  brakepedalpressedVal=Linearinterpolation(FlexSensorvalue,FullbrakeUpperthreshold,FullbrakeLowerthreshold,100,0);
     }


//Speed simulation enable
     if(SpeedSimulationEnable==1)
     {
vehicleSpeedSimulatorState = digitalRead(vehicleSpeedSimulatorPin);

vehicleSpeedResetState = digitalRead(vehicleSpeedResetPin);

if(vehicleSpeedSimulatorState==LOW && VehicleSpeed >0)
{
VehicleSpeed=VehicleSpeed-speedreducingfactor;
if(VehicleSpeed<=speedreducingfactor){
  VehicleSpeed=0;
}
  
}

if(vehicleSpeedResetState==LOW)
{
VehicleSpeed=DefaultvehicleSpeed;
  
}

      
     }

//I2C 
     if(I2cCommunicationEna==1)
     {
     Wire.requestFrom(1,1);                           // request 1 byte from slave arduino (8)
     brakepedalpressedVal = Wire.read();                // receive a byte from the slave arduino and store in MasterReceive

      
     }

    



     if(CalibrateSimulateSpeedSensorvalue==1)
     {
     ReadSpeedSensor=analogRead(SpeedPin);         //Read and save analog value 

  VehicleSpeed=Linearinterpolation(ReadSpeedSensor,MaxPotentiometerVal,MinPotentiometerVal,25,5);
     }
  
   IMU.readAcceleration(x_axis, y_axis, z_axis);

   z_axis *= CONVERT_G_TO_MS2;
   
    Serial.print(z_axis);
  Serial.print('\t');
  Serial.print(brakepedalpressedVal);
   Serial.print('\t');
   Serial.println(VehicleSpeed);

    delay(10);
  
}


// Linear interpolation formula to calculate brake pedal value
int Linearinterpolation( float x , float x0, float x1, float y0, float y1)
{



y= y0+(((x-x0)*(y1-y0))/(x1-x0));
return y;
  
}
