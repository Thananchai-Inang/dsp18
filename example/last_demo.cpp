#include <Arduino.h>
#include <SPI.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

//----------------------------Gyroscope parameters----------------------------
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int valx , valy , valz;
char rd;
int prevVal;
int pin11 = 5 , pin10 = 4 ;
int val1 , val2 ;
int valgy1 = 0 , valgy2 = 0;

//----------------------------Depth Sensor parameters----------------------------
int pwmChannel = 0;
int frequence = 32768;
int resolution = 8;
int pwmPin = 5;
int pH2O = 1;
int pHG = 13.56;
bool init_PAir = false;
float PAir = 0.0;
//----------------------------Depth sensor function----------------------------
void resetsensor() {
    SPI.setDataMode(SPI_MODE0); 
    SPI.transfer(0x15);
    SPI.transfer(0x55);
    SPI.transfer(0x40);
}
//----------------------------Control parameters----------------------------
float kp = 6; //kp for depth
float kd = 1; //kd for depth
float kp_gyro1 = 6, kp_gyro2 = 6, kp_gyro3 = 6, kp_gyro4 = 6; //kp for gyro 1,2,3,4
float kd_gyro1 = 1, kd_gyro2 = 1, kd_gyro3 = 1, kd_gyro4 = 1; //kd for gyro 1,2,3,4
float err;
float Lasterr = 0;
float der;
float Depth;
//gyro control parameters
float err1, err2, err3, err4; //Propotional term
float der1, der2, der3, der4; //Derivatives term
float Lasterr1 = 0, Lasterr2 = 0, Lasterr3 = 0, Lasterr4 = 0; //for PD control

float DepthTarget = 20;////////define target///////////
float LastDepth = 0;
unsigned long previousTime = 0;

int gyro_condition = 0; //-> after 3-axis condition satisfied this equals 1

//plotter
int low = 0;
int high = 40;

//----------------------------define motor drive pins----------------------------
//-----------------------------pump 1 pins------------------------------------
const int pump1Pin1 = 25;  // IN1 of motor drive L
const int pump1Pin2 = 26;  // IN2 of motor drive L
const int pump1PinPWM = 27;  
const int pwmChannelPump1 = 1; //define pump 1 as channel 1

//-----------------------------pump 2 pins------------------------------------
const int pump2Pin1 = 4;  // IN1 of motor drive R
const int pump2Pin2 = 2;  // IN2 of motor drive R
const int pump2PinPWM = 15;  
const int pwmChannelPump2 = 2; //define pump 2 as channel 2

//-----------------------------pump 3 pins------------------------------------
const int pump3Pin1 = 13;  // IN3 of motor drive L
const int pump3Pin2 = 12;  // IN4 of motor drive L
const int pump3PinPWM = 14;  
const int pwmChannelPump3 = 3; //define pump 3 as channel 3

//-----------------------------pump 4 pins------------------------------------
const int pump4Pin1 = 16;  // IN3 of motor drive R
const int pump4Pin2 = 33;  // IN4 of motor drive R
const int pump4PinPWM = 32;  
const int pwmChannelPump4 = 4; //define pump 4 as channel 4

//other PWM param
const int freq = 30000;
const int resolutionPump = 8;
int dutyCycle = 120;
float controlP1; 
float controlP2;
float controlP3;
float controlP4;
float controlPall;

void setup() {
    Serial.begin(115200);

    //-----------------------------pump 1 setup------------------------------------
    pinMode(pump1Pin1, OUTPUT); //motor 1
    pinMode(pump1Pin2, OUTPUT);
    pinMode(pump1PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump1, freq, resolutionPump);
    ledcAttachPin(pump1PinPWM, pwmChannelPump1); //map pwm channel 1 to motor1PinPWM

    //-----------------------------pump 2 setup------------------------------------
    pinMode(pump2Pin1, OUTPUT); //motor 2
    pinMode(pump2Pin2, OUTPUT);
    pinMode(pump2PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump2, freq, resolutionPump);
    ledcAttachPin(pump2PinPWM, pwmChannelPump2); //map pwm channel 2 to motor2PinPWM
    //-----------------------------pump 3 setup------------------------------------
    pinMode(pump3Pin1, OUTPUT); //motor 3
    pinMode(pump3Pin2, OUTPUT);
    pinMode(pump3PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump3, freq, resolutionPump);
    ledcAttachPin(pump3PinPWM, pwmChannelPump3); //map pwm channel 3 to motor3PinPWM

    //-----------------------------pump 4 setup------------------------------------
    pinMode(pump4Pin1, OUTPUT); //motor 4
    pinMode(pump4Pin2, OUTPUT);
    pinMode(pump4PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump4, freq, resolutionPump);
    ledcAttachPin(pump4PinPWM, pwmChannelPump4); //map pwm channel 4 to motor4PinPWM

    //----------------------------Depth Sensor setup----------------------------
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    ledcSetup(pwmChannel, frequence, resolution);
    ledcAttachPin(pwmPin, pwmChannel);

    //----------------------------Gyroscope setup----------------------------
    Wire.begin();
    Serial.begin(115200);
    Serial.print("Initialize_MPU:");
    mpu.initialize();
    Serial.print(mpu.testConnection() ? "Connected" : "Connection_failed");
  }

void loop() {
    //----------------------------main Depth Sensor----------------------------
    //normal sensor function call
    ledcWrite(pwmChannel, 127);
    resetsensor();
    unsigned int result1 = 0;
    unsigned int inbyte1 = 0;
    SPI.transfer(0x1D); 
    SPI.transfer(0x50);
    SPI.setDataMode(SPI_MODE1);
    result1 = SPI.transfer(0x00);
    result1 = result1 << 8;
    inbyte1 = SPI.transfer(0x00);
    result1 = result1 | inbyte1;
    resetsensor(); 
    unsigned int result2 = 0;
    byte inbyte2 = 0; 
    SPI.transfer(0x1D);
    SPI.transfer(0x60);
    SPI.setDataMode(SPI_MODE1); 
    result2 = SPI.transfer(0x00);
    result2 = result2 <<8;
    inbyte2 = SPI.transfer(0x00);
    result2 = result2 | inbyte2;
    resetsensor();
    unsigned int result3 = 0;
    byte inbyte3 = 0;
    SPI.transfer(0x1D);
    SPI.transfer(0x90); 
    SPI.setDataMode(SPI_MODE1); 
    result3 = SPI.transfer(0x00);
    result3 = result3 <<8;
    inbyte3 = SPI.transfer(0x00);
    result3 = result3 | inbyte3;
    resetsensor();
    unsigned int result4 = 0;
    byte inbyte4 = 0;
    SPI.transfer(0x1D);
    SPI.transfer(0xA0);
    SPI.setDataMode(SPI_MODE1); 
    result4 = SPI.transfer(0x00);
    result4 = result4 <<8;
    inbyte4 = SPI.transfer(0x00);
    result4 = result4 | inbyte4;
    long c1 = (result1 >> 1) & 0x7FFF;
    long c2 = ((result3 & 0x003F) << 6) | (result4 & 0x003F);
    long c3 = (result4 >> 6) & 0x03FF;
    long c4 = (result3 >> 6) & 0x03FF;
    long c5 = ((result1 & 0x0001) << 10) | ((result2 >> 6) & 0x03FF);
    long c6 = result2 & 0x003F;
    resetsensor();

    //----------------------------main Pressure----------------------------
    //normal sensor function call
    unsigned int presMSB = 0; 
    unsigned int presLSB = 0; 
    unsigned int D1 = 0;
    SPI.transfer(0x0F);
    SPI.transfer(0x40);
    delay(35);
    SPI.setDataMode(SPI_MODE1);
    presMSB = SPI.transfer(0x00);
    presMSB = presMSB << 8;
    presLSB = SPI.transfer(0x00);
    D1 = presMSB | presLSB;
    resetsensor();

    //----------------------------main Temperature----------------------------
    //normal sensor function call
    unsigned int tempMSB = 0;
    unsigned int tempLSB = 0;
    unsigned int D2 = 0;
    SPI.transfer(0x0F);
    SPI.transfer(0x20);
    delay(35);
    SPI.setDataMode(SPI_MODE1);
    tempMSB = SPI.transfer(0x00);
    tempMSB = tempMSB << 8;
    tempLSB = SPI.transfer(0x00);
    D2 = tempMSB | tempLSB;
    const long UT1 = (c5 << 3) + 20224;
    const long dT = D2 - UT1;
    const long TEMP = 200 + ((dT * (c6 + 50)) >> 10);
    const long OFF  = (c2 * 4) + (((c4 - 512) * dT) >> 12);
    const long SENS = c1 + ((c3 * dT) >> 10) + 24576;
    const long X = (SENS * (D1 - 7168) >> 14) - OFF;
    long PCOMP = ((X * 10) >> 5) + 2500;
    float TEMPREAL = TEMP/10;
    //----------------------------Initialize the pressure sensor----------------------------
    //normal sensor initialization
    if (!init_PAir) {
        PAir = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
        init_PAir = true;
        // Serial.print("Initialize PAir variable");
    }
    //----------------------------Calculate Depth----------------------------
    //normal sensor calculation
    float PCOMPHG = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06  
    Depth = (PCOMPHG-PAir)*(pHG/pH2O)/10;
        
    //----------------------------print Depth----------------------------
    // Serial.print(" ,Compensated_pressure_in_mmHg= ");
    // Serial.print(PCOMPHG);
    // Serial.print("Depth_in_cm = ,");
    // Serial.print(Depth);

    //----------------------------get Gyroscope Value----------------------------
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    valx = map(ax, -17000, 17000, 0, 179);
    valy = map(ay, -17000, 17000, 0, 179);
    valz = map(az, -17000, 17000, 0, 179);
    //print gyro value
    Serial.print("axis_x = ,") ;
    Serial.print(valx) ;
    Serial.print(", axis_y = ,") ;
    Serial.print(valy) ;
    Serial.print(", axis_z = ,") ;
    Serial.print(valz) ;

    //---------------------------- main Control----------------------------
    //cal time diff for differential calculation
    unsigned long currentTime = millis();
    unsigned long timeDifference = currentTime - previousTime; 

    //------------gyro error------------
    //--gyro error pump 1--
    err1 = valx - valy;
    der1 = (err1 - Lasterr1)/(timeDifference);
    previousTime = currentTime;
    Lasterr1 = err1;
    //--gyro error pump 2--
    err2 = 180 - valx - valy;
    der2 = (err2 - Lasterr2)/(timeDifference);
    previousTime = currentTime;
    Lasterr2 = err2;
    //--gyro error pump 3--
    err3 = valx + valy - 180;
    der3 = (err3 - Lasterr3)/(timeDifference);
    previousTime = currentTime;
    Lasterr3 = err3;
    //--gyro error pump 4--
    err4 = valy - valx;
    der4 = (err4 - Lasterr4)/(timeDifference);
    previousTime = currentTime;
    Lasterr4 = err4;

    //------------depth error------------
    err = DepthTarget - Depth;
    // derivative = (err - lasterr)/(time-lasttime);
    der = (err - Lasterr)/(timeDifference);
    previousTime = currentTime;
    Lasterr = err;

    //---------------------------- Control Gyro ----------------------------
    if (gyro_condition < 30){
        //-----------------------------pump 1 control gyro------------------------------------
        controlP1 = float((kp_gyro1 * err1)+(kd_gyro1 * der1));
        int PWM1 = map(abs(controlP1),0,190,150,255); //change pump1 PWM value 
        if (controlP1 >= 0) {
            digitalWrite(pump1Pin1, HIGH); //water in
            digitalWrite(pump1Pin2, LOW);
            ledcWrite(pwmChannelPump1, PWM1);
            Serial.print(", P1_IN_PWM: ,");
            Serial.print(PWM1);
        } 
        else {
            digitalWrite(pump1Pin1, LOW); //water out
            digitalWrite(pump1Pin2, HIGH);
            ledcWrite(pwmChannelPump1, PWM1);
            Serial.print(", P1_OUT_PWM: ,");
            Serial.print(PWM1);
        }

        //-----------------------------pump 2 control gyro------------------------------------
        controlP2 = float((kp_gyro2 * err2)+(kd_gyro2 * der2));
        int PWM2 = map(abs(controlP2),0,190,150,255); //change pump2 PWM value 
        if (controlP2 >= 0) {
            digitalWrite(pump2Pin1, HIGH); //water in
            digitalWrite(pump2Pin2, LOW);
            ledcWrite(pwmChannelPump2, PWM2);
            Serial.print(", P2_IN_PWM: ,");
            Serial.print(PWM2);
        } 
        else {
            digitalWrite(pump2Pin1, LOW); //water out
            digitalWrite(pump2Pin2, HIGH);
            ledcWrite(pwmChannelPump2, PWM2);
            Serial.print(", P2_OUT_PWM: ,");
            Serial.print(PWM2);
        }

        //-----------------------------pump 3 control gyro------------------------------------
        controlP3 = float((kp_gyro3 * err3)+(kd_gyro3 * der3));
        int PWM3 = map(abs(controlP3),0,190,150,255); //change pump3 PWM value 
        if (controlP3 >= 0) {
            digitalWrite(pump3Pin1, HIGH); //water in
            digitalWrite(pump3Pin2, LOW);
            ledcWrite(pwmChannelPump3, PWM3);
            Serial.print(", P3_IN_PWM: ,");
            Serial.print(PWM3);
        } 
        else {
            digitalWrite(pump3Pin1, LOW); //water out
            digitalWrite(pump3Pin2, HIGH);
            ledcWrite(pwmChannelPump3, PWM1);
            Serial.print(", P3_OUT_PWM: ,");
            Serial.print(PWM3);
        }

        //-----------------------------pump 4 control gyro------------------------------------
        controlP4 = float((kp_gyro4 * err4)+(kd_gyro4 * der4));
        int PWM4 = map(abs(controlP4),0,190,150,255); //change pump4 PWM value 
        if (controlP4 >= 0) {
            digitalWrite(pump4Pin1, HIGH); //water in
            digitalWrite(pump4Pin2, LOW);
            ledcWrite(pwmChannelPump4, PWM4);
            Serial.print(", P4_IN_PWM: ,");
            Serial.print(PWM4);
        } 
        else {
            digitalWrite(pump4Pin1, LOW); //water out
            digitalWrite(pump4Pin2, HIGH);
            ledcWrite(pwmChannelPump4, PWM4);
            Serial.print(", P4_OUT_PWM: ,");
            Serial.print(PWM4);
        }

    }
    else{
        //reset every balance control
            controlP1 = 0;
            controlP2 = 0;
            controlP3 = 0;
            controlP4 = 0;
        //---------------------------- Control depth----------------------------
        controlPall = float((kp * err)+(kd * der));
        //-----------------------------pump 1,2,3,4 control depth------------------------------------
        int PWM_all = map(abs(controlPall),0,190,145,255); //change all pump PWM value 
        if (controlPall >= 0) {
            //water in all pump
            //pump 1 water_in enable
            digitalWrite(pump1Pin1, HIGH); //water in
            digitalWrite(pump1Pin2, LOW);
            //pump 2 water_in enable
            digitalWrite(pump2Pin1, HIGH); //water in
            digitalWrite(pump2Pin2, LOW);
            //pump 3 water_in enable
            digitalWrite(pump3Pin1, HIGH); //water in
            digitalWrite(pump3Pin2, LOW);
            //pump 4 water_in enable
            digitalWrite(pump4Pin1, HIGH); //water in
            digitalWrite(pump4Pin2, LOW);
            //set all pwmChannelPump 
            ledcWrite(pwmChannelPump1, PWM_all);
            ledcWrite(pwmChannelPump2, PWM_all);
            ledcWrite(pwmChannelPump3, PWM_all);
            ledcWrite(pwmChannelPump4, PWM_all);

            Serial.print(", All_IN_PWM: ,");
            Serial.print(PWM_all);
        } 
        else {
            //pump 1 water_out enable
            digitalWrite(pump1Pin1, LOW); //water out
            digitalWrite(pump1Pin2, HIGH);
            //pump 2 water_out enable
            digitalWrite(pump2Pin1, LOW); //water in
            digitalWrite(pump2Pin2, HIGH);
            //pump 3 water_out enable
            digitalWrite(pump3Pin1, LOW); //water in
            digitalWrite(pump3Pin2, HIGH);
            //pump 4 water_out enable
            digitalWrite(pump4Pin1, LOW); //water in
            digitalWrite(pump4Pin2, HIGH);
            //set all pwmChannelPump 
            ledcWrite(pwmChannelPump1, PWM_all);
            ledcWrite(pwmChannelPump2, PWM_all);
            ledcWrite(pwmChannelPump3, PWM_all);
            ledcWrite(pwmChannelPump4, PWM_all);

            Serial.print(", All_OUT_PWM: ,");
            Serial.print(PWM_all);
        }
        
    }    
    
    //change gyro_condition if it is satisfied
    if (abs(controlP1) < 30 && abs(controlP2) < 30 && abs(controlP3) < 30 && abs(controlP4) < 30){
        gyro_condition += 1;
    }
    else {
        gyro_condition == 0;
    }

    // Serial.print(", low_lim: ,");
    // Serial.print(low); // To freeze the lower limit
    // Serial.print(" ,");
    // Serial.print("high_lim: ");
    // Serial.print(high); // To freeze the upper limit
    // Serial.print(" ,");
    Serial.print(", depth_err: ,");
    Serial.print(err); //depth error

    Serial.print(", Depth: ,");
    Serial.print(Depth); //depth now

    Serial.print(", Depth_target: ,");
    Serial.print(DepthTarget); //depth target
    
    Serial.print(", CPall: ,");
    Serial.print(controlPall); //controlPall
    Serial.print(", CP1: ,");
    Serial.print(controlP1); //controlP1
    Serial.print(", CP2: ,");
    Serial.print(controlP2); //controlP2
    Serial.print(", CP3: ,");
    Serial.print(controlP3); //controlP3
    Serial.print(", CP4: ,");
    Serial.print(controlP4); //controlP4

    Serial.print(", gyro_cond: ,"); //gyro condition
    Serial.println(gyro_condition);


    delay(200);
}

