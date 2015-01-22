
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//This code is provided under the GNU General Public Licence
//12/19/2012
#include "openIMUL.h"
//#include <Streaming.h>



openIMU::openIMU(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, 
float *accZ, float *scAccX, float *scAccY, float *scAccZ, float *magX, float *magY, float *magZ, float *xGPS , float *yGPS, float *zBaro,float *xGPSVel ,float *yGPSVel,float *zBaroVel,float *G_Dt, float *H_Dt){

  //constructor for the 10DOF system
  gx = gyroX;
  gy = gyroY;
  gz = gyroZ;

  ax = accX;
  ay = accY;
  az = accZ;

  mx = magX;
  my = magY;
  mz = magZ;

  sax = scAccX;
  say = scAccY;
  saz = scAccZ;

  XPosGPS = xGPS;
  YPosGPS = yGPS;
  ZPosBaro = zBaro;
  XVelGPS = xGPSVel;
  YVelGPS = yGPSVel;
  ZVelBaro = zBaroVel;
  dt = G_Dt;
  dt2 = H_Dt;
  q0.val = 1;
  q1.val = 0;
  q2.val = 0;
  q3.val = 0;

  XEst.val = 0;
  velX.val = 0;

  YEst.val = 0;
  velY.val = 0;

  velZ.val = 0;
  ZEst.val = 0;


  kPosGPS = 0.20;
  kVelGPS = 0.36;
  kAccGPS = 0.00003;

  kPosBaro = 0.09;
  kVelBaro = 0.09;
  kAccBaro = 1e-6;

  for (int i = 0; i < LAG_SIZE; i++){
    XEstHist[i] = 0;
    YEstHist[i] = 0;
    XVelHist[i] = 0;
    YVelHist[i] = 0;
  }
  for (int i = 0; i < LAG_SIZE_BARO; i++){
    ZEstHist[i] = 0;
    ZVelHist[i] = 0;
  }


}

void openIMU::GetGravOffset(void){
  initialAccMagnitude.val = -1.0 * sqrt(*sax * *sax + *say * *say + *saz * *saz) ;

}
void openIMU::GetInertial(void){

  inertialX.val = ((R11 * (*sax)) + (R21 * (*say))) + (R31 * (*saz));// - inertialXOffSet.val;
  inertialY.val = R12 * *sax + R22 * *say + R32 * *saz;// - inertialYOffSet.val;
  inertialZGrav.val = R13 * *sax + R23 * *say + R33 * *saz;
  //inertialZ.val = R13 * *sax + R23 * *say + R33 * *saz - initialAccMagnitude.val;// - inertialZOffSet.val;
  inertialZ.val = inertialZGrav.val - initialAccMagnitude.val;// - inertialZOffSet.val;
}


void openIMU::UpdateLagIndex(void){
  /*
  currentEstIndex++;
   
   lagIndex++;
   if (currentEstIndex == LAG_SIZE){
   currentEstIndex = 0;
   }
   if (lagIndex == LAG_SIZE){
   lagIndex = 0;
   
   }*/


  currentEstIndex++;
  //0.7 sec lag
  if (currentEstIndex == (LAG_SIZE)){
    currentEstIndex = 0;
  }
  //50Hz main loop * 0.7 sec = 35
  lagIndex = currentEstIndex - 53;
  /*
  if (lagAmt > 0){
   lagIndex = currentEstIndex - 53;
   }
   else{
   lagIndex = currentEstIndex;
   }*/


  if (lagIndex < 0){
    lagIndex = LAG_SIZE + lagIndex;
  }


  //0.3sec lag
  currentEstIndex_z++;
  if (currentEstIndex_z == (LAG_SIZE_BARO)){
    currentEstIndex_z = 0;
  }
  //50Hz main loop 8 0.3 sec = 15
  lagIndex_z = currentEstIndex_z - 23;

  if (lagIndex_z < 0){
    lagIndex_z = LAG_SIZE_BARO + lagIndex_z;
  }
}



void openIMU::InitialQuat(){
  //calculate the ypr from sensors convert to quaternion and rotation matrix
  radPitch.val = atan2(*sax,sqrt(*say * *say + *saz * *saz));
  radRoll.val = atan2(-1.0 * *say, -1.0 * *saz);
  //Serial.println(ToDeg(radPitch.val));
  //Serial.println(ToDeg(radRoll.val));

  R11 = cos(radPitch.val);
  R13 = -1.0 * sin(radPitch.val);
  R21 = sin(radRoll.val)*sin(radPitch.val);
  R22 = cos(radRoll.val);
  R23 = cos(radPitch.val)*sin(radRoll.val);

  bx = *mx * R11 + *mz * R13;
  by = *mx * R21 + *my * R22 + *mz * R23;
  radYaw.val = atan2(-1.0 * by, bx) - DECLINATION;

  q0.val = cos(radYaw.val/2.0)*cos(radPitch.val/2.0)*cos(radRoll.val/2.0) + sin(radYaw.val/2.0)*sin(radPitch.val/2.0)*sin(radRoll.val/2.0); 
  q1.val = cos(radYaw.val/2.0)*cos(radPitch.val/2.0)*sin(radRoll.val/2.0) - sin(radYaw.val/2.0)*sin(radPitch.val/2.0)*cos(radRoll.val/2.0); 
  q2.val = cos(radYaw.val/2.0)*sin(radPitch.val/2.0)*cos(radRoll.val/2.0) + sin(radYaw.val/2.0)*cos(radPitch.val/2.0)*sin(radRoll.val/2.0); 
  q3.val = sin(radYaw.val/2.0)*cos(radPitch.val/2.0)*cos(radRoll.val/2.0) - cos(radYaw.val/2.0)*sin(radPitch.val/2.0)*sin(radRoll.val/2.0);
  magnitude.val = sqrt(q0.val *  q0.val + q1.val *  q1.val + q2.val *  q2.val + q3.val *  q3.val); 
  q0.val = q0.val / magnitude.val;
  q1.val = q1.val / magnitude.val;
  q2.val = q2.val / magnitude.val;
  q3.val = q3.val / magnitude.val;


  q0q0 = q0.val*q0.val;
  q1q1 = q1.val*q1.val;
  q2q2 = q2.val*q2.val;
  q3q3 = q3.val*q3.val;

  q0q1 = q0.val*q1.val;
  q0q2 = q0.val*q2.val;
  q0q3 = q0.val*q3.val;

  q1q2 = q1.val*q2.val;
  q1q3 = q1.val*q3.val;

  q2q3 = q2.val*q3.val;
  //generate rotation matrix
  R11 = 2*(q0q0-0.5+q1q1);
  R12 = 2*(q1q2+q0q3);
  R13 = 2*(q1q3-q0q2);
  R21 = 2*(q1q2-q0q3);
  R22 = 2*(q0q0-0.5+q2q2);
  R23 = 2*(q2q3+q0q1);
  R31 = 2*(q1q3+q0q2);
  R32 = 2*(q2q3-q0q1);
  R33 = 2*(q0q0-0.5+q3q3);  

  //rotate by declination 
  COS_DEC = cos(DECLINATION);
  SIN_DEC = sin(DECLINATION);
  R11 = R11*COS_DEC - R12*SIN_DEC;
  R12 = R12*COS_DEC + R11*SIN_DEC;

  R21 = R21*COS_DEC - R22*SIN_DEC;
  R22 = R22*COS_DEC + R21*SIN_DEC;

  R31 = R31*COS_DEC - R32*SIN_DEC;
  R32 = R32*COS_DEC + R31*SIN_DEC;





}

void openIMU::Predict(void){



  biasedX = (*sax - accelBiasX.val);
  biasedY = (*say - accelBiasY.val);
  biasedZ = (*saz - accelBiasZ.val);
  inertialXBiased.val = R11 * biasedX + R21 * biasedY + R31 * biasedZ;//  - inertialXOffSet.val;
  inertialYBiased.val = R12 * biasedX + R22 * biasedY + R32 * biasedZ;// - inertialYOffSet.val;
  inertialZBiased.val = R13 * biasedX + R23 * biasedY + R33 * biasedZ - initialAccMagnitude.val;// - inertialZOffSet.val; 



  velX.val = velX.val + inertialXBiased.val * *dt;
  velY.val = velY.val + inertialYBiased.val * *dt;
  velZ.val = velZ.val + inertialZBiased.val * *dt;


  XEst.val = XEst.val + velX.val * *dt;
  YEst.val = YEst.val + velY.val * *dt;
  ZEst.val = ZEst.val + velZ.val * *dt;



  XEstHist[currentEstIndex] = XEst.val;
  YEstHist[currentEstIndex] = YEst.val;


  XVelHist[currentEstIndex] = velX.val;
  YVelHist[currentEstIndex] = velY.val;


  ZEstHist[currentEstIndex_z] = ZEst.val;
  ZVelHist[currentEstIndex_z] = velZ.val;
  lagZVel.val = -1.0 * ZVelHist[lagIndex_z];


  ZEstUp.val = -1.0 * ZEst.val;
  velZUp.val = -1.0 * velZ.val;


}
void openIMU::CorrectGPS(void){
  xPosError.val = XEstHist[lagIndex] - *XPosGPS;
  yPosError.val = YEstHist[lagIndex] - *YPosGPS;

  xVelError.val = XVelHist[lagIndex] - *XVelGPS;
  yVelError.val = YVelHist[lagIndex] - *YVelGPS;

  XEst.val = XEst.val - kPosGPS * xPosError.val;
  YEst.val = YEst.val - kPosGPS * yPosError.val;

  velX.val = velX.val - kVelGPS * xVelError.val;
  velY.val = velY.val - kVelGPS * yVelError.val;




  accelBiasXEF = R11 * accelBiasX.val + R21 * accelBiasY.val + R31 * accelBiasZ.val;
  accelBiasYEF = R12 * accelBiasX.val + R22 * accelBiasY.val + R32 * accelBiasZ.val;
  accelBiasZEF = R13 * accelBiasX.val + R23 * accelBiasY.val + R33 * accelBiasZ.val;


  accelBiasXEF = accelBiasXEF + kAccGPS * xVelError.val;
  accelBiasYEF = accelBiasYEF + kAccGPS * yVelError.val;

  accelBiasX.val = R11*accelBiasXEF + R12*accelBiasYEF + R13*accelBiasZEF;
  accelBiasY.val = R21*accelBiasXEF + R22*accelBiasYEF + R23*accelBiasZEF;
  accelBiasZ.val = R31*accelBiasXEF + R32*accelBiasYEF + R33*accelBiasZEF;


}
/*void openIMU::SetBias(void){
 accelBiasX.val = R11*inertialAvgX + R12*inertialAvgY + R13*inertialAvgZ;
 accelBiasY.val = R21*inertialAvgX + R22*inertialAvgY + R23*inertialAvgZ;
 accelBiasZ.val = R31*inertialAvgX + R32*inertialAvgY + R33*inertialAvgZ;
 }*/
void openIMU::CorrectAlt(void){
  zPosError.val = ZEstHist[lagIndex_z] + *ZPosBaro;
  zVelError.val = ZVelHist[lagIndex_z] + *ZVelBaro;

  ZEst.val = ZEst.val - kPosBaro * zPosError.val;
  velZ.val = velZ.val - kVelBaro * zVelError.val;

  accelBiasXEF = R11*accelBiasX.val + R21*accelBiasY.val + R31*accelBiasZ.val;
  accelBiasYEF = R12*accelBiasX.val + R22*accelBiasY.val + R32*accelBiasZ.val;
  accelBiasZEF = R13*accelBiasX.val + R23*accelBiasY.val + R33*accelBiasZ.val;


  accelBiasZEF = accelBiasZEF + kAccBaro * zVelError.val;

  accelBiasX.val = R11*accelBiasXEF + R12*accelBiasYEF + R13*accelBiasZEF;
  accelBiasY.val = R21*accelBiasXEF + R22*accelBiasYEF + R23*accelBiasZEF;
  accelBiasZ.val = R31*accelBiasXEF + R32*accelBiasYEF + R33*accelBiasZEF;

  ZEstUp.val = -1.0 * ZEst.val;
  velZUp.val = -1.0 * velZ.val;
}

void openIMU::IntegrateGyro(){
  /*  magnitude.val =  sqrt(*ax * *ax + *ay * *ay + *az * *az);
   magnitudeDifference.val = fabs(initialAccMagnitude.val +  magnitude.val);
   if (magnitudeDifference.val > FEEDBACK_LIMIT){
   skipFeedBack = true;
   }*/
  if (kiAcc > 0){

    *gx = *gx + integralFBX;
    *gy = *gy + integralFBY;
    *gz = *gz + integralFBZ;  
  }
  dtby2 = *dt2 * 0.5;
  q0.val += -1 * dtby2*(*gx * q1.val + *gy * q2.val + *gz * q3.val);
  q1.val +=      dtby2*(*gx * q0.val - *gy * q3.val + *gz * q2.val);
  q2.val +=      dtby2*(*gx * q3.val + *gy * q0.val - *gz * q1.val);
  q3.val +=      dtby2*(*gy * q1.val - *gx * q2.val + *gz * q0.val);


  //normalize the quaternion
  recipNorm = InvSqrt(q0.val * q0.val + q1.val * q1.val + q2.val * q2.val + q3.val * q3.val);
  q0.val *= recipNorm;
  q1.val *= recipNorm;
  q2.val *= recipNorm;
  q3.val *= recipNorm;



}


void openIMU::IMUupdate() {
  //normalize the sensor readings
  magnitude.val =  sqrt(*ax * *ax + *ay * *ay + *az * *az);
  magnitudeDifference.val = fabs(initialAccMagnitude.val +  magnitude.val);

  if (magnitudeDifference.val < FEEDBACK_LIMIT ){


    feedBack = 2;
    recipNorm = 1/magnitude.val;
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;



    vx = R13;
    vy = R23;
    vz = R33;


    exa = (*ay * vz - *az * vy);
    eya = (*az * vx - *ax * vz);
    eza = (*ax * vy - *ay * vx);

    kiDTAcc = kiAcc * *dt2;
    if (kiAcc > 0){
      integralFBX += exa * kiDTAcc;
      integralFBY += eya * kiDTAcc;
      integralFBZ += eza * kiDTAcc;
      *gx = *gx + integralFBX;
      *gy = *gy + integralFBY;
      *gz = *gz + integralFBZ;  
    }
    else{
      integralFBX = 0;
      integralFBY = 0;
      integralFBZ = 0;  
    }
    *gx += exa * kpAcc;
    *gy += eya * kpAcc;
    *gz += eza * kpAcc;
  }
  else{
    feedBack = 0;
  }

  dtby2 = *dt2 * 0.5;
  q0.val += -1.0 * dtby2*(*gx * q1.val + *gy * q2.val + *gz * q3.val);
  q1.val +=      dtby2*(*gx * q0.val - *gy * q3.val + *gz * q2.val);
  q2.val +=      dtby2*(*gx * q3.val + *gy * q0.val - *gz * q1.val);
  q3.val +=      dtby2*(*gy * q1.val - *gx * q2.val + *gz * q0.val);


  //normalize the quaternion
  recipNorm = 1/sqrt(q0.val * q0.val + q1.val * q1.val + q2.val * q2.val + q3.val * q3.val);
  q0.val *= recipNorm;
  q1.val *= recipNorm;
  q2.val *= recipNorm;
  q3.val *= recipNorm;

}

void openIMU::AHRSupdate() {
  //normalize the sensor readings
  magnitude.val =  sqrt(*ax * *ax + *ay * *ay + *az * *az);
  magnitudeDifference.val = fabs(initialAccMagnitude.val +  magnitude.val);

  if (magnitudeDifference.val < FEEDBACK_LIMIT ){

    recipNorm = 1/magnitude.val;
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;
    if (magFlag == 1){
      recipNorm = InvSqrt(*mx * *mx + *my * *my + *mz * *mz);
      *mx *= recipNorm;
      *my *= recipNorm;
      *mz *= recipNorm;

      hx = R11 * *mx + R21 * *my + R31 * *mz;
      hy = R12 * *mx + R22 * *my + R32 * *mz;
      hz = R13 * *mx + R23 * *my + R33 * *mz;


      bx = sqrt(hx * hx + hy * hy);
      bz = hz;


      wx = R11*bx + R13*bz;
      wy = R21*bx + R23*bz;
      wz = R31*bx + R33*bz;


      exm = (*my * wz - *mz * wy);
      eym = (*mz * wx - *mx * wz);
      ezm = (*mx * wy - *my * wx);
    }
    else{
      exm = 0;
      eym = 0;
      ezm = 0;
    }


    vx = R13;
    vy = R23;
    vz = R33;


    exa = (*ay * vz - *az * vy);
    eya = (*az * vx - *ax * vz);
    eza = (*ax * vy - *ay * vx);

    kiDTAcc = kiAcc * *dt;
    kiDTMag = kiMag * *dt;
    if (kiAcc > 0){
      integralFBX += exa * kiDTAcc+ exm * kiDTMag;
      integralFBY += eya * kiDTAcc+ eym * kiDTMag;
      integralFBZ += eza * kiDTAcc+ ezm * kiDTMag;
      *gx = *gx + integralFBX;
      *gy = *gy + integralFBY;
      *gz = *gz + integralFBZ;  
    }
    else{
      integralFBX = 0;
      integralFBY = 0;
      integralFBZ = 0;  
    }
    *gx += exa * kpAcc + exm * kpMag;
    *gy += eya * kpAcc + eym * kpMag;
    *gz += eza * kpAcc + ezm * kpMag;
  }


  dtby2 = *dt * 0.5;
  q0.val += -1.0 * dtby2*(*gx * q1.val + *gy * q2.val + *gz * q3.val);
  q1.val +=      dtby2*(*gx * q0.val - *gy * q3.val + *gz * q2.val);
  q2.val +=      dtby2*(*gx * q3.val + *gy * q0.val - *gz * q1.val);
  q3.val +=      dtby2*(*gy * q1.val - *gx * q2.val + *gz * q0.val);


  //normalize the quaternion
  recipNorm = 1/sqrt(q0.val * q0.val + q1.val * q1.val + q2.val * q2.val + q3.val * q3.val);
  q0.val *= recipNorm;
  q1.val *= recipNorm;
  q2.val *= recipNorm;
  q3.val *= recipNorm;
  /*
  q0q0 = q0.val*q0.val;
   q1q1 = q1.val*q1.val;
   q2q2 = q2.val*q2.val;
   q3q3 = q3.val*q3.val;
   
   q0q1 = q0.val*q1.val;
   q0q2 = q0.val*q2.val;
   q0q3 = q0.val*q3.val;
   
   q1q2 = q1.val*q2.val;
   q1q3 = q1.val*q3.val;
   
   q2q3 = q2.val*q3.val;
   //generate rotation matrix
   R11 = 2*(q0q0-0.5+q1q1);
   R12 = 2.0*(q1q2+q0q3);
   R13 = 2.0*(q1q3-q0q2);
   R21 = 2.0*(q1q2-q0q3);
   R22 = 2.0*(q0q0-0.5+q2q2);
   R23 = 2.0*(q2q3+q0q1);
   R31 = 2.0*(q1q3+q0q2);
   R32 = 2.0*(q2q3-q0q1);
   R33 = 2.0*(q0q0-0.5+q3q3);  
   
   //rotate by declination 
   R11 = R11*COS_DEC - R12*SIN_DEC;
   R12 = R12*COS_DEC + R11*SIN_DEC;
   
   R21 = R21*COS_DEC - R22*SIN_DEC;
   R22 = R22*COS_DEC + R21*SIN_DEC;
   
   R31 = R31*COS_DEC - R32*SIN_DEC;
   R32 = R32*COS_DEC + R31*SIN_DEC;
   */
}

void openIMU::AHRSupdate2() {
  //normalize the sensor readings
  magnitude.val =  sqrt(*ax * *ax + *ay * *ay + *az * *az);
  magnitudeDifference.val = fabs(initialAccMagnitude.val +  magnitude.val);

  if (magnitudeDifference.val < FEEDBACK_LIMIT ){


    feedBack = 2;
    recipNorm = 1/magnitude.val;
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;
    if (magFlag == 1){
      recipNorm = InvSqrt(*mx * *mx + *my * *my + *mz * *mz);
      *mx *= recipNorm;
      *my *= recipNorm;
      *mz *= recipNorm;

      hx = R11 * *mx + R21 * *my + R31 * *mz;
      hy = R12 * *mx + R22 * *my + R32 * *mz;
      hz = R13 * *mx + R23 * *my + R33 * *mz;


      bx = sqrt(hx * hx + hy * hy);
      bz = hz;


      wx = R11*bx + R13*bz;
      wy = R21*bx + R23*bz;
      wz = R31*bx + R33*bz;


      exm = (*my * wz - *mz * wy);
      eym = (*mz * wx - *mx * wz);
      ezm = (*mx * wy - *my * wx);
    }
    else{
      exm = 0;
      eym = 0;
      ezm = 0;
    }


    vx = R13;
    vy = R23;
    vz = R33;


    exa = (*ay * vz - *az * vy);
    eya = (*az * vx - *ax * vz);
    eza = (*ax * vy - *ay * vx);

    kiDTAcc = kiAcc * *dt2;
    kiDTMag = kiMag * *dt2;
    if (kiAcc > 0){
      integralFBX += exa * kiDTAcc+ exm * kiDTMag;
      integralFBY += eya * kiDTAcc+ eym * kiDTMag;
      integralFBZ += eza * kiDTAcc+ ezm * kiDTMag;
      *gx = *gx + integralFBX;
      *gy = *gy + integralFBY;
      *gz = *gz + integralFBZ;  
    }
    else{
      integralFBX = 0;
      integralFBY = 0;
      integralFBZ = 0;  
    }
    *gx += exa * kpAcc + exm * kpMag;
    *gy += eya * kpAcc + eym * kpMag;
    *gz += eza * kpAcc + ezm * kpMag;
  }
  else{
    feedBack = 0;
  }

  dtby2 = *dt2 * 0.5;
  q0.val += -1.0 * dtby2*(*gx * q1.val + *gy * q2.val + *gz * q3.val);
  q1.val +=      dtby2*(*gx * q0.val - *gy * q3.val + *gz * q2.val);
  q2.val +=      dtby2*(*gx * q3.val + *gy * q0.val - *gz * q1.val);
  q3.val +=      dtby2*(*gy * q1.val - *gx * q2.val + *gz * q0.val);


  //normalize the quaternion
  recipNorm = 1/sqrt(q0.val * q0.val + q1.val * q1.val + q2.val * q2.val + q3.val * q3.val);
  q0.val *= recipNorm;
  q1.val *= recipNorm;
  q2.val *= recipNorm;
  q3.val *= recipNorm;

}
void openIMU::GenerateRotationMatrix(void){
  q0q0 = q0.val*q0.val;
  q1q1 = q1.val*q1.val;
  q2q2 = q2.val*q2.val;
  q3q3 = q3.val*q3.val;

  q0q1 = q0.val*q1.val;
  q0q2 = q0.val*q2.val;
  q0q3 = q0.val*q3.val;

  q1q2 = q1.val*q2.val;
  q1q3 = q1.val*q3.val;

  q2q3 = q2.val*q3.val;
  //generate rotation matrix
  R11 = 2*(q0q0-0.5+q1q1);
  R12 = 2.0*(q1q2+q0q3);
  R13 = 2.0*(q1q3-q0q2);
  R21 = 2.0*(q1q2-q0q3);
  R22 = 2.0*(q0q0-0.5+q2q2);
  R23 = 2.0*(q2q3+q0q1);
  R31 = 2.0*(q1q3+q0q2);
  R32 = 2.0*(q2q3-q0q1);
  R33 = 2.0*(q0q0-0.5+q3q3);  
  COS_DEC = cos(DECLINATION);
  SIN_DEC = sin(DECLINATION);
  //rotate by declination 
  R11 = R11*COS_DEC - R12*SIN_DEC;
  R12 = R12*COS_DEC + R11*SIN_DEC;

  R21 = R21*COS_DEC - R22*SIN_DEC;
  R22 = R22*COS_DEC + R21*SIN_DEC;

  R31 = R31*COS_DEC - R32*SIN_DEC;
  R32 = R32*COS_DEC + R31*SIN_DEC;


}

void openIMU::GetEuler(void){
  rawRoll.val = ToDeg(FastAtan2(2 * (q0.val * q1.val + q2.val * q3.val),1 - 2.0 * (q1.val * q1.val + q2.val * q2.val)));
  roll.val=  rawRoll.val - rollOffset.val;

  rawPitch.val = ToDeg(asin(2.0 * (q0.val * q2.val - q3.val * q1.val)));
  pitch.val =  rawPitch.val - pitchOffset.val;

  yaw.val = ToDeg(FastAtan2(2.0 * (q0.val * q3.val + q1.val * q2.val) , 1 - 2.0* (q2.val * q2.val + q3.val * q3.val)));

  if (yaw.val < 0){
    yaw.val +=360;
  }

}
void openIMU::GetPitch(void){
  rawPitch.val = ToDeg(asin(2.0 * (q0.val * q2.val - q3.val * q1.val)));
  pitch.val =  rawPitch.val - pitchOffset.val;
}

void openIMU::GetRoll(void){
  rawRoll.val = ToDeg(FastAtan2(2.0 * (q0.val * q1.val + q2.val * q3.val),1 - 2.0 * (q1.val * q1.val + q2.val * q2.val)));
  roll.val=  rawRoll.val - rollOffset.val;
}

void openIMU::GetYaw(void){
  yaw.val = ToDeg(FastAtan2(2.0 * (q0.val * q3.val + q1.val * q2.val) , 1 - 2.0* (q2.val * q2.val + q3.val * q3.val))) ;
  if (yaw.val < 0){
    yaw.val +=360;
  }
}
















































