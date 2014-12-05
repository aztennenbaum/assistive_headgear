    // MPU-6050 Short Example Sketch
    // By Arduino User JohnChi
    // August 17, 2014
    // Public Domain
    #include<Wire.h>
    #define M_OUTPUT(x) (int)(3.3*255.0*sqrt(x)/5.0)
    #define M_SIDE(x) sqrt(abs(x)/32768.0)
    #define M_CENTER 6
    #define M_LEFT 10
    #define M_RIGHT 3
    #define M_FRONT 5    
    #define M_BACK 11
    #define M_DELAY 5
    const int MPU=0x68;  // I2C address of the MPU-6050
    int16_t temp;
    float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    float init_AcX,init_AcY,init_AcZ,init_Tmp,init_GyX,init_GyY,init_GyZ;
    float i,j,k,g,g2,t;
    int m[12] = {0};
    
    void setup(){
      
      
      pinMode(M_CENTER, OUTPUT);
      pinMode(M_LEFT, OUTPUT);
      pinMode(M_RIGHT, OUTPUT);
      pinMode(M_FRONT, OUTPUT);
      pinMode(M_BACK, OUTPUT);
      Wire.begin();
      Wire.beginTransmission(MPU);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);

      delay(50);      

      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
      temp=Wire.read()<<8|Wire.read();init_AcX=(float)temp;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      temp=Wire.read()<<8|Wire.read();init_AcY=(float)temp;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      temp=Wire.read()<<8|Wire.read();init_AcZ=(float)temp;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      temp=Wire.read()<<8|Wire.read();init_Tmp=(float)temp;  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      temp=Wire.read()<<8|Wire.read();init_GyX=(float)temp;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      temp=Wire.read()<<8|Wire.read();init_GyY=(float)temp;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      temp=Wire.read()<<8|Wire.read();init_GyZ=(float)temp;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      g=init_AcX*init_AcX+init_AcY*init_AcY+init_AcZ*init_AcZ;
      
//motor test
      analogWrite(M_LEFT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_LEFT, 0);
      delay(200);

      
      analogWrite(M_RIGHT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_RIGHT, 0);
      delay(200);

      analogWrite(M_FRONT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_FRONT, 0);
      delay(200);


      analogWrite(M_BACK, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_BACK, 0);
      delay(200);

      analogWrite(M_CENTER, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_CENTER, 0);
      delay(200);

      /*Serial.begin(9600);*/
    }
    void loop(){
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
      temp=Wire.read()<<8|Wire.read();AcX=(float)temp;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      temp=Wire.read()<<8|Wire.read();AcY=(float)temp;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      temp=Wire.read()<<8|Wire.read();AcZ=(float)temp;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      temp=Wire.read()<<8|Wire.read();Tmp=(float)temp;  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      temp=Wire.read()<<8|Wire.read();GyX=(float)temp;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      temp=Wire.read()<<8|Wire.read();GyY=(float)temp;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      temp=Wire.read()<<8|Wire.read();GyZ=(float)temp;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

//cross product of initial acceleration vector with final
      i=(init_AcY*AcZ - init_AcZ*AcY);
      j=(init_AcZ*AcX - init_AcX*AcZ);
      k=(init_AcX*AcY - init_AcY*AcX);
      g2=AcX*AcX+AcY*AcY+AcZ*AcZ;      
      t=(i*i+j*j+k*k)/(g*g2);
      
    analogWrite(M_CENTER, M_OUTPUT(t));
    if(GyY<0.0) {
//run left motor for up to 500 ms based on speed
       if (m[M_LEFT]>M_DELAY) {
         if(m[M_LEFT]<M_DELAY+(int)(M_SIDE(GyY)*10)) analogWrite(M_LEFT, 255);
         else m[M_LEFT]=0;
       } else {
         analogWrite(M_LEFT, 0);
       }
       analogWrite(M_RIGHT, 0);
       
       m[M_LEFT]++;
       m[M_RIGHT]=0;
     
     } else {
//run right motor for up to 500 ms based on speed
       if (m[M_RIGHT]>M_DELAY) {
         if(m[M_RIGHT]<M_DELAY+(int)(M_SIDE(GyY)*10)) analogWrite(M_RIGHT, 255);
         else m[M_RIGHT]=0;
       } else {
         analogWrite(M_RIGHT, 0);
       }
       analogWrite(M_LEFT, 0);
       
       m[M_RIGHT]++;
       m[M_LEFT]=0;
     
     }
      
     if(GyX<0.0) {
//run front motor for up to 500 ms based on speed
       if (m[M_FRONT]>M_DELAY) {
         if(m[M_FRONT]<M_DELAY+(int)(M_SIDE(GyX)*10)) analogWrite(M_FRONT, 255);
         else m[M_FRONT]=0; 
       } else {
         analogWrite(M_FRONT, 0);
       }
       analogWrite(M_BACK, 0);
       
       m[M_FRONT]++;
       m[M_BACK]=0;
     
     } else {
//run back motor for up to 500 ms based on speed
       if (m[M_BACK]>M_DELAY) {
         if(m[M_BACK]<M_DELAY+(int)(M_SIDE(GyX)*10)) analogWrite(M_BACK, 255);
         else m[M_BACK]=0; 
       } else {
         analogWrite(M_BACK, 0);
       }
       analogWrite(M_FRONT, 0);
       
       m[M_BACK]++;
       m[M_FRONT]=0;
      
     }
/*      analogWrite(M_RIGHT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_RIGHT, 0);
      delay(200);
      analogWrite(M_LEFT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_LEFT, 0);
      delay(200);
      
      
      analogWrite(M_RIGHT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_RIGHT, 0);
      delay(200);


      analogWrite(M_FRONT, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_FRONT, 0);
      delay(200);


      analogWrite(M_BACK, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_BACK, 0);
      delay(200);

      analogWrite(M_CENTER, M_OUTPUT(1.0));
      delay(200);
      analogWrite(M_CENTER, 0);
      delay(200);*/

  /*

      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" | AcY = "); Serial.print(AcY);
      Serial.print(" | AcZ = "); Serial.print(AcZ);
      Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
      Serial.print(" | GyX = "); Serial.print(GyX);
      Serial.print(" | GyY = "); Serial.print(GyY);
      Serial.print(" | GyZ = "); Serial.println(GyZ);
*/
      delay(50);
    }
