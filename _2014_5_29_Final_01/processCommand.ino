


//------------------------------------------------------------------------------
void releaseMotor(int num){
  switch(num){
    case 0: digitalWrite(ORIG_X_ENABLE_PIN,HIGH); break;//TODO: try out if this pin get low will disable the motors
    case 1: digitalWrite(ORIG_Y_ENABLE_PIN,HIGH); break;
    case 2: digitalWrite(ORIG_Z_ENABLE_PIN,HIGH); break;
  }
}

//------------------------------------------------------------------------------
void activeMotor(int num){
  switch(num){
    case 0: digitalWrite(ORIG_X_ENABLE_PIN,LOW); break;//TODO: try out if this pin get low will disable the motors
    case 1: digitalWrite(ORIG_Y_ENABLE_PIN,LOW); break;
    case 2: digitalWrite(ORIG_Z_ENABLE_PIN,LOW); break;
  }
}
//------------------------------------------------------------------------------
static void where() {
  Serial.print("X:");    Serial.print(posx);
  Serial.print("mm Y:"); Serial.print(posy);
  Serial.print("mm Z:"); Serial.print(posz);
  Serial.print("mm F");    Serial.print(123);//printFeedRate();
  Serial.print("\n");
}

void fanOn(){
  digitalWrite(HEATER_0_PIN,HIGH);
  Serial.println(F("Mainboard Fan On"));  
}

void fanOff(){
  digitalWrite(HEATER_0_PIN,LOW);
  Serial.println(F("Mainboard Fan Off"));
}

void motorDisable(){
  digitalWrite(ORIG_X_ENABLE_PIN, HIGH); //disable all motor.
  digitalWrite(ORIG_Y_ENABLE_PIN, HIGH);
  digitalWrite(ORIG_Z_ENABLE_PIN, HIGH);
  Serial.println(F("motors disabled."));
}

void motorEnable(){
  digitalWrite(ORIG_X_ENABLE_PIN, LOW); //disable all motor.
  digitalWrite(ORIG_Y_ENABLE_PIN, LOW);
  digitalWrite(ORIG_Z_ENABLE_PIN, LOW);
  Serial.println(F("motors enabled."));
}

void ledCOn (){ //cold led control

}

void ledCOff (){

}

void ledWOn (){

}

void ledWOff (){

}

//Input coordinate y, get x on this line
float ac(float y){
  float x;
  //y=k·x+(COORDMOTORC[Y])
  //k = (Ya-Yc) / (Xa-Xc)
  float k = ((float)COORDMOTORA[Y]-(float)COORDMOTORC[Y]) / ((float)COORDMOTORA[X] - (float)COORDMOTORC[X]);// huge fund! if wanna get float value from integer value based 
  /*
  Serial.print("COORDMOTORA[Y]");
  Serial.println(COORDMOTORA[Y]);
  Serial.print("COORDMOTORC[Y]");
  Serial.println(COORDMOTORC[Y]);  
  Serial.print("COORDMOTORA[X]");
  Serial.println(COORDMOTORA[X]);  
  Serial.print("COORDMOTORC[X]");
  Serial.println(COORDMOTORC[X]);  
  Serial.print("k: ");
  Serial.println(k);
  */
  x = (DELTA_RADIUS + y) / k;
  return x;
}

//this return the value x is minus the same value from ac()
float bc(){
  float x;    
  return x;
}

// Check if possible get target coordinate.
int coordcheck(float x, float y, float z){
  // check y value legel.
  /*
  Serial.println(F("X,Y,Z in coordcheck function: "));
  Serial.println(x);  Serial.println(y);  Serial.println(z);  
  */
  if(y <= COORDMOTORC[Y] || y >= COORDMOTORA[Y]){
    Serial.println(F("y coord inpossible."));
    return 0;
  }
  /* //Deactive this z check for original Test.
  // check z value legel
  if(z < 0 || z > 2000){
    Serial.println(F("z coord inpossible."));
    return 0;
  }
  */
  //Through y get x value on both line BC and AC.  
  float tempX = ac(y); //tempX should always positive. and -tempX should always on line BC
  Serial.println(tempX);
  if(x >= tempX || x <= -tempX){
    Serial.println(F("line coord inpossible."));
    return 0;
  }
  return 1; //legal
}

/*
//------------------------------------------------------------------------------
static void printConfig() {
  Serial.print(m1d);        Serial.print("=");  Serial.print(limit_top);  Serial.print(",");
  Serial.print(limit_left); Serial.print("\n");
  Serial.print(m2d);        Serial.print("=");  Serial.print(limit_top);  Serial.print(",");
  Serial.print(limit_right);Serial.print("\n");
  Serial.print("Bottom=");  Serial.println(limit_bottom);
  Serial.print("Feed rate=");  printFeedRate();
}
*/

//------------------------------------------------------------------------------
static void help() {
  Serial.println();
  Serial.println(F("== DRAWBOT - http://github.com/i-make-robots/Drawbot/ =="));
  Serial.println(F("All commands end with a semi-colon."));
  Serial.println(F("Input M100 to show up this help."));
  Serial.println(F("CONFIG [Tx.xx] [Bx.xx] [Rx.xx] [Lx.xx];"));
  Serial.println(F("       - display/update this robot's configuration."));
  Serial.println(F("TELEPORT [Xx.xx] [Yx.xx]; - move the virtual plotter."));
  Serial.println(F("support the following G-codes (http://en.wikipedia.org/wiki/G-code):"));
  Serial.println(F("G00,G01,G02,G03,G04,G20,G21,G28,G90,G91,M18,M114"));
}

/*
//------------------------------------------------------------------------------
static void printFeedRate() {
  //Serial.print("f1= ");
  Serial.print(feed_rate * 60.0 / mode_scale);
  Serial.print(mode_name);
  Serial.println("/min");
}
*/

//------------------------------------------------------------------------------
// feed rate is given in units/min and converted to cm/s ************************
// the units/min, units can be in "mm" or in "in"
static void setFeedRate(float v) {
  /*
  float v1 = v * mode_scale / 60.0;
  if( feed_rate != v1 ) {
    feed_rate = v1; //feed rate is now in unit: cm/s
    if(feed_rate > MAX_VEL) feed_rate=MAX_VEL;
    if(feed_rate < MIN_VEL) feed_rate=MIN_VEL;
  }
  
  long step_delay1 = 1000000.0 / (feed_rate/THREADPERSTEP1); //where comes the 1000000
  long step_delay2 = 1000000.0 / (feed_rate/THREADPERSTEP2); //in () unit: steps/s
  step_delay = step_delay1 > step_delay2 ? step_delay1 : step_delay2;
  
  Serial.print("step_delay=");
  Serial.println(step_delay);
  //printFeedRate();
  */
}

/*
//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
static void teleport(float x,float y) {
  posx=x;
  posy=y;

  // @TODO: posz?
  long L1,L2; //teleport the new set point
  IK(posx,posy,L1,L2);
  laststep1=L1;
  laststep2=L2;
}

//------------------------------------------------------------------------------
static int processSubcommand() {
  int found=0;
  char *ptr=buffer;
  
  while(ptr && ptr<buffer+sofar && strlen(ptr)) {
    if(!strncmp(ptr,"G20",3)) {
      Serial.println("input Unite: inches");
      mode_scale=25.4f;  // inches -> mm, 1 inche == 25.4 mm
      strcpy(mode_name,"in");
      //printFeedRate();
      found=1;
    } 
    else if(!strncmp(ptr,"G21",3)) { //say the imput parameter is always set to "mm" and be here convertered.
      Serial.println("input Unite: mm");
      mode_scale=1;  // mm -> mm, 1 mm = 1 mm
      strcpy(mode_name,"mm");
      //printFeedRate();
      found=1;
    }
    else if(!strncmp(ptr,"G90",3)) {
      // absolute mode
      absolute_mode=1;
      found=1;
      Serial.println("in absolute mode");
    } 
    else if(!strncmp(ptr,"G91",3)) {
      // relative mode
      absolute_mode=0;
      found=1;
      Serial.println("in relative mode");
    }
    //ptr=strchr(ptr,' ')+1; //here is the deal!!
    //when strchr() return NULL, this ChipKIT IDE can not handle
    ptr=strchr(ptr,' '); 
    if(ptr != NULL){
      ptr += 1;
    }
    
  }
  return found;
}



*/


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy,float npz) {
  // here is a good place to add sanity tests
  posx=npx;
  posy=npy;
  posz=npz;  
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code, float val){
  char *ptr=buffer;
  while(ptr && *ptr && ptr<buffer+sofar){
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr=strchr(ptr,' ') + 1;
  }
  return val;
}

/**
 * The Serial command parser
 */
void processCommand(){
  int cmd = parsenumber('G', -1);
  switch(cmd){
    case  0:
    case  1: //move as line
      //motorEnable();
      /* //original code:
      line( parsenumber('X',(mode_abs?posx:0)) + (mode_abs?0:posx),
            parsenumber('Y',(mode_abs?posy:0)) + (mode_abs?0:posy),
            parsenumber('Z',(mode_abs?posz:0)) + (mode_abs?0:posz)  );
      */    
      /*
      float X = parsenumber('X',(mode_abs?posx:0));
      float Y = parsenumber('Y',(mode_abs?posy:0));
      float Z = parsenumber('Z',(mode_abs?posz:0));
      */
      float X,Y,Z;
      X = parsenumber('X',(mode_abs?posx:0));
      Y = parsenumber('Y',(mode_abs?posy:0));
      Z = parsenumber('Z',(mode_abs?posz:0));      
      if(coordcheck(X,Y,Z)){
        line(X,Y,Z);
      } else {
        Serial.println(F("coordinate beyond the possible area"));
      }      

      /*
      line( parsenumber('X',(mode_abs?posx:0)) ,
            parsenumber('Y',(mode_abs?posy:0)) ,
            parsenumber('Z',(mode_abs?posz:0)) );
      */
      //motorDisable();
      break;
    case  90:  mode_abs=1;  break;  //absolute mode
    case  91:  mode_abs=0;  break;  //relative mode
    case  92:  //set logical position (Teleport)
      position( parsenumber('X',0),
                parsenumber('Y',0),
                parsenumber('Z',0) );
      break;
  }
  cmd = parsenumber('M', -1);
  switch(cmd){
    case  84:   motorDisable(); break;
    case  85:   motorEnable();  break;
    case  100:  help();  break;
    case  114:  where(); break;
    case  106:  fanOn(); break;
    case  107:  fanOff();break;
    case  200:  ledCOn();break;
    case  201:  ledCOff();  break;
    case  202:  ledWOn();   break;
    case  203:  ledWOff();  break;
    case  204:  parsenumber('c',-1);
    case  205:  parsenumber('w',-1);
    case  251:  calibration();
  }
}

