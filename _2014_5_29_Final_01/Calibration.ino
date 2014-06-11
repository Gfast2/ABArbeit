
int breakFlag = false; //when endstop hitted, turn to true till canncel this move totally then reset to false.

//------------------------------------------------------------------------------
// x,y,z are the targeted coordinate.
// Sure, this function is using Bresenham algorithmus
// Decide to divide caliLine() in smaller pecise, in order to get the coordinate when endstop hitted.
void caliLine(float x,float y,float z) {
  Serial.println(F("Jump in caliLine() function"));
  long l1,l2,l3;
  IK(x,y,z,l1,l2,l3);
  a[0].delta = l1 - laststep1;
  a[1].delta = l2 - laststep2;
  a[2].delta = l3 - laststep3;
  long maxsteps = 0; 
  for(int i=0; i<3; ++i){
    a[i].absdelta = abs(a[i].delta);
    a[i].dir = a[i].delta > 0 ? 0 : 1; 
    a[i].over=0;
    if( maxsteps < a[i].absdelta) maxsteps = a[i].absdelta;
  }  
  for( long i=0; i<maxsteps; ++i){
    
    Serial.print(F("I'm making "));
     Serial.print(i);
     Serial.println(F("th steps."));
     
    if(digitalRead(ORIG_X_MIN_PIN) == 0 || digitalRead(ORIG_Y_MIN_PIN) == 0 || digitalRead(ORIG_Z_MIN_PIN) == 0) { 
      posx=x; //roughly transmitt the position info, in oder to let it run next position in right way.
      posy=y;
      posz=z;
      breakFlag = true;
      Serial.println("EStop get pushed.");
      break;
    }    
    for(int j=0; j<3; ++j){
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps){
        a[j].over -= maxsteps;
        Monestep(j,a[j].dir);
        switch(j){         //add or minus the one step from "laststep"
          case 0: laststep1 = a[j].dir > 0 ? --laststep1 : ++laststep1; break; //figure out the motor direction and laststep relatiion.
          case 1: laststep2 = a[j].dir > 0 ? --laststep2 : ++laststep2; break;
          case 2: laststep3 = a[j].dir > 0 ? --laststep3 : ++laststep3; break;
        }
      }
    }
    delay(1); delayMicroseconds(200);
  }
  /* //this step will be done when motor move each step.
   laststep1=l1;
   laststep2=l2;
   laststep3=l3;
   */
  posx=x;
  posy=y;
  posz=z;
  //Serial.println(F("Finished one caliLine()"));
}






// dived lines into smaller pieces. In order get a more accurate position.
void caliLine_small(float x, float y, float z){
  float dx = x - posx;
  float dy = y - posy;
  float dz = z - posz;
  float len = sqrt(dx*dx + dy*dy + dz*dz);
  //Serial.println("len:" + String((long)len));
  
  if(len <= MM_PER_SEGMENT){
    caliLine(x,y,z);
    return;
  }
  // too long
  long pieces = floor(len / MM_PER_SEGMENT);
  //Serial.println("pieces:" + String((long)pieces));
  float x0 = posx;
  float y0 = posy;
  float z0 = posz;
  float a;
  for(long j=0; j<=pieces; ++j){
    Serial.print(F("now I'm in line pieces: "));
    Serial.println(j);
    if(breakFlag == false){
      a = (float)j / (float)pieces;
      /*
      Serial.print("a: "); 
      Serial.println(a);
      Serial.print("target coord: "); 
      Serial.print((x-x0)*a+x0); 
      Serial.print(",");
      Serial.print( (y-y0)*a+y0);
      Serial.print(",");
      Serial.println((z-z0)*a+z0);
      */
      caliLine( (x-x0)*a+x0, (y-y0)*a+y0, (z-z0)*a+z0);
    } 
    else {
      breakFlag = false;
      Serial.println(F("Cancel the X axis movements and step breakFlag to false."));
      break;
    }
  }
  //line(x,y,z); if only used in calibration phase, it will never reach that far.
}

//float back = 5; //go back value when hit Endstop







void calibration(){
  Serial.println(F("Enter calibration."));
  // 1. go to find calibration point near mount point A
  /*
  PRINTLN(COORDMOTORA[X]);
   PRINTLN(COORDMOTORA[Y]);
   PRINTLN(COORDMOTORA[Z]);
   */
  float COORDMOTORA_mm[3]; //mount point A coordination
  COORDMOTORA_mm[X] = cos(DELTA_ALPHA_A * M_PI / 180) * DELTA_RADIUS;
  COORDMOTORA_mm[Y] = sin(DELTA_ALPHA_A * M_PI / 180) * DELTA_RADIUS;
  COORDMOTORA_mm[Z] = DELTA_Z;
  //caliLine(COORDMOTORA_mm[X],COORDMOTORA_mm[Y],COORDMOTORA_mm[Z]); //on theory, if go to mount A, for sure the Endstop A will be pushed on the road. !!!function line()'s arguments is based unit: mm
  caliLine_small(COORDMOTORA_mm[X],COORDMOTORA_mm[Y],COORDMOTORA_mm[Z]);

  // reset line A long
  Serial.println("lastStep1 before recalculate:" + String(laststep1));
  laststep1 = mmToStep(2200.0); //calib line long on mount A. Seile Endstop pad is mount on the point distance to mount point A 2.2 meter far
  //should go back a little bit to release the endstop. (using line() but not caliLine())
  //Serial.println("now go on to go next line().");
  Serial.println(F("step up laststep1 finished"));
  Serial.println("lastStep1 now: " + String(laststep1));
  /* can't make GUANG relese Estop by this way. suspect the laststep2 and laststep3 didn't updated correctely so it moves not correct.    
  Serial.println(F("posx,posy,posz: "));
  Serial.println(posx);
  Serial.println(posy);
  Serial.println(posz);
  Serial.println(F("(posx-back),(posy-back),(posz-back): "));
  Serial.println(posx-back);
  Serial.println(posy-back);
  Serial.println(posz-back);
  line((posx-back),(posy-back),(posz-back));
  */  
  for(int k=0; k<(50*25); ++k){ //hard coded release solution.
    //Serial.println("check move!");
    Monestep(X,0); //a[0].dir==1 => laststep1 get shorter, ==0, laststep1 get longer
    ++laststep1; 
    delay(1); delayMicroseconds(500);
  }  
  //Serial.println("laststep1 now: " + String(laststep1));
  //Serial.println("first Estop check finished.");
  delay(1000);
    
  // 2. go to find calibration point near mount point B
  float COORDMOTORB_mm[3]; //mount point A coordination
  COORDMOTORB_mm[X] = cos(DELTA_ALPHA_B * M_PI / 180) * DELTA_RADIUS;
  COORDMOTORB_mm[Y] = sin(DELTA_ALPHA_B * M_PI / 180) * DELTA_RADIUS;
  COORDMOTORB_mm[Z] = DELTA_Z;
  Serial.println("start to run next caliLine_small().");
  caliLine_small(COORDMOTORB_mm[X],COORDMOTORB_mm[Y],COORDMOTORB_mm[Z]);
  Serial.println("finish run next caliLine_small().");
  // reset line B long
  laststep2 = mmToStep(2200.0); //calib value
  // release Y Endstop
  for(int k=0; k<(50*25); ++k){ //hard coded release solution.
    //Serial.println("check move!");
    Monestep(Y,0); //a[0].dir==1 => laststep1 get shorter, ==0, laststep1 get longer
    ++laststep2; 
    delay(1); delayMicroseconds(500);
  }
   delay(1000);
  
  
  
  
    /*
   // 3. go to find calibration point near mount point C
   caliLine(COORDMOTORC[X],COORDMOTORC[Y],COORDMOTORC[Z]);  
   // reset line C long
   laststep3 = mmToStep(2200.0); //calib value
   delay(1000);
   
   caliLine(0,0,1500); //Here use line(x,,y,z); will cause parser don't move coordinate when its minus value.
   */
  Serial.println("Calibration finisched.");
}


