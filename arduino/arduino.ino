#include <NewPing.h>

//for the distance sensor----
const int trigPin = 12;
const int echoPin = 9;
//for the DC-motor-----------
const int transistorPin = 6;
//---------------------------

int error_val;
int error_prev_val;
int cherror_val;

// GLOBAL VARIABLES
double max_h = 1.0;

char junk = ' ';
bool haveHeight = false;
bool serialBegun = false;

double getSmallBaseSize(int a, int b, double h) {
        return (b-a)/2.0 - (((b-(b-a)/2.0)-a)*h*1.0);
}

int f_nl = 0;
int f_ns = 1;
int f_z = 2;
int f_ps = 3;
int f_pl = 4;

int force [5][5] = {
		      {f_pl, f_pl, f_pl, f_ps, f_z},
         	      {f_pl, f_pl, f_ps, f_z, f_ns},
		      {f_pl, f_ps, f_z, f_ns, f_nl},
		      {f_ps, f_z, f_ns, f_nl, f_nl},
		      {f_z, f_ns, f_nl, f_nl, f_nl}};

struct Members {
  //distance measured in microseconds
  int pl1;
  int pl2;
  int ph1;
  int ph2;
};

struct Members e_mem[5];
struct Members ce_mem[5];
struct Members f_mem[5];

int desiredHeight = 0;
int heightDifference = 500;

int error_nl;
int error_ns;
int error_z;
int error_ps;
int error_pl;

int cherror_nl = -300;
int cherror_ns = -150;
int cherror_z = 0;
int cherror_ps = 150;
int cherror_pl = 300;

int force_nl_1 = 205; //possible [0, 255]
int force_nl = 215;
int force_ns = 225;
int force_z = 235;
int force_ps = 245;
int force_pl = 255;
int force_pl_1 = 265;

void initEmem() {//OK
  //NL
  e_mem[0].pl1 = -10000; //a very small number
  e_mem[0].ph1 = -5000; // a very small number
  e_mem[0].ph2 = error_nl;
  e_mem[0].pl2 = error_ns;

  //NS
  e_mem[1].pl1 = error_nl;
  e_mem[1].ph1 = error_ns;
  e_mem[1].ph2 = error_ns;
  e_mem[1].pl2 = error_z;

  //Z
  e_mem[2].pl1 = error_ns;
  e_mem[2].ph1 = error_z;
  e_mem[2].ph2 = error_z;
  e_mem[2].pl2 = error_ps;

  //PS
  e_mem[3].pl1 = error_z;
  e_mem[3].ph1 = error_ps;
  e_mem[3].ph2 = error_ps;
  e_mem[3].pl2 = error_pl;

  //PL
  e_mem[4].pl1 = error_ps;
  e_mem[4].ph1 = error_pl;
  e_mem[4].ph2 = 50000; //a very big number
  e_mem[4].pl2 = 100000; //a very big number
}

void initCemem() {//OK
  ce_mem[0].pl1 = -10000;
  ce_mem[0].ph1 = -5000;
  ce_mem[0].ph2 = cherror_nl;
  ce_mem[0].pl2 = cherror_ns;

  ce_mem[1].pl1 = cherror_nl;
  ce_mem[1].ph1 = cherror_ns;
  ce_mem[1].ph2 = cherror_ns;
  ce_mem[1].pl2 = cherror_z;

  ce_mem[2].pl1 = cherror_ns;
  ce_mem[2].ph1 = cherror_z;
  ce_mem[2].ph2 = cherror_z;
  ce_mem[2].pl2 = cherror_ps;

  ce_mem[3].pl1 = cherror_z;
  ce_mem[3].ph1 = cherror_ps;
  ce_mem[3].ph2 = cherror_ps;
  ce_mem[3].pl2 = cherror_pl;

  ce_mem[4].pl1 = cherror_ps;
  ce_mem[4].ph1 = cherror_pl;
  ce_mem[4].ph2 = 5000;
  ce_mem[4].pl2 = 10000;
}

void initFmem() {//OK
  //NL
  f_mem[0].pl1 = force_nl_1;
  f_mem[0].ph1 = force_nl;
  f_mem[0].ph2 = force_nl;
  f_mem[0].pl2 = force_ns;
  
  //NS
  f_mem[1].pl1 = force_nl;
  f_mem[1].ph1 = force_ns;
  f_mem[1].ph2 = force_ns;
  f_mem[1].pl2 = force_z;

  //Z
  f_mem[2].pl1 = force_ns;
  f_mem[2].ph1 = force_z;
  f_mem[2].ph2 = force_z;
  f_mem[2].pl2 = force_ps;

  //PS
  f_mem[3].pl1 = force_z;
  f_mem[3].ph1 = force_ps;
  f_mem[3].ph2 = force_ps;
  f_mem[3].pl2 = force_pl;

  //PL
  f_mem[4].pl1 = force_ps;
  f_mem[4].ph1 = force_pl;
  f_mem[4].ph2 = force_pl;
  f_mem[4].pl2 = force_pl_1;
}

double degreeOfMembership(struct Members member, int distance) {//OK
  if (distance <= member.pl1 || member.pl2 <= distance) {
    //if distance outside of the function
    return 0;
  } else if (member.ph1 <= distance && distance <= member.ph2) {
    //if between the two upmost points
    return max_h;
  } else if (distance <= member.ph1) {
    //if on left slope
    return max_h * (distance - member.pl1) / (member.ph1 - member.pl1);
  } else if (distance >= member.ph2) {
    return max_h * (member.pl2 - distance) / (member.pl2 - member.ph2);
  }
}

int getError() {
  ///////////////////////////////////////////////////////////////
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  double duration, dist;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  // convert the time into a distance
  dist = microsecondsToCentimeters(duration);

  //return dist * 100 - 40 * 100;
  return 43 * 100 - dist * 100;

  /////////////////////////////////////////////////////////////////
  
}
void printSomething(double something){
//  return;
  Serial.print(something);
  Serial.print("\t");
}
void printSomething(int something){
//  return;
  Serial.print(something);
  Serial.print("\t");
}
void printSomething(char* something){
//  return;
  Serial.print(something);
  Serial.print("\t");
}

void getDesiredHeight(bool changing){
  if (changing){
    desiredHeight = Serial.parseInt();
    Serial.println();
    Serial.print("Dave, the desired height value has changed to ");
    Serial.println(desiredHeight);
    Serial.println();
    while (Serial.available() > 0){
      junk = Serial.read(); 
    }
  } else {
    Serial.println("Hello, Dave. Please enter the desired height.");
  }
  
  while (Serial.available() == 0 && !haveHeight){   // Wait here until input buffer has a character
    //get Height from user input
    desiredHeight = Serial.parseInt();
    //clear keyboard buffer if there is  >0 characters
    while (Serial.available() > 0){
      junk = Serial.read(); 
    }      
    //if user gets a Height other than 0, continue
    if (desiredHeight != 0){
      break;
    }
  }
  
  error_nl = desiredHeight - heightDifference * 2;
  error_ns = desiredHeight - heightDifference; //= > /10 cm
  error_z = desiredHeight;
  error_ps = desiredHeight + heightDifference;
  error_pl = desiredHeight + heightDifference * 2;
  initEmem();

  Serial.print("Desired Height, Dave: ");
  Serial.println(desiredHeight);
}

void setup() {
  Serial.begin(9600);
  pinMode(transistorPin, OUTPUT); //Sets the pin as output

  initCemem();
  initEmem();
  initFmem();

  error_val = getError(); //Read sensor value to error_val

}

void loop() {
  
  // ********* READING INPUT *********
  if (!haveHeight){
    getDesiredHeight(false);   //not changing => because we are getting value for the first time
  }
  haveHeight = true;
  
  if (Serial.available() > 0){
    getDesiredHeight(true);    //changing => because we previously had a value
  }
  // ********* END OF READING INPUT *********
        
  error_prev_val = error_val;
  error_val = getError();

  cherror_val = error_val - error_prev_val;

  //e[i], ... are the indexes for the functions, for e and e´
  //so we have 0 => NL, 1 => NS, ....
  int e[2] = {-1, -1};
  int ce[2] = {-1, -1};
  double degreeOfForce[5] = {0}; //degreeOfForce

  for(int i = 0; i < 5; i++) {
    double temp_degree;
    
    temp_degree = degreeOfMembership(e_mem[i], error_val);
    printSomething(temp_degree);
    if(temp_degree != 0 ) {
      if (e[0] == -1) {
        e[0] = i;
      } else {
        e[1] = i;
      }
    }

    temp_degree = degreeOfMembership(ce_mem[i], cherror_val);
    printSomething(temp_degree);
    if(temp_degree != 0 ) {
      if (ce[0] == -1) {
        ce[0] = i;
      } else {
        ce[1] = i;
      }
    }
  }

  //this is the strength of the rule e# and ce#
  double strengthOfRule[4];

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      int loopI = i*2 + j;
      
      if (e[i] != -1 && ce[j] != -1) {
        strengthOfRule[loopI] = min(degreeOfMembership(e_mem[e[i]], error_val), degreeOfMembership(ce_mem[ce[j]], cherror_val));
    
        //Serial.print("Strength of ");
        //Serial.print(strengthOfRule[loopI]);
    
        int f_index = force[e[i]][ce[j]];
        degreeOfForce[f_index] = max(degreeOfForce[f_index], strengthOfRule[loopI]);
       /* Serial.print(" Degree of force: index = ");
        Serial.print(f_index);
        Serial.print(", degree = " );
        Serial.println(degreeOfForce[f_index]);//*/
      }
    }
  }
  /********* COG *********/
  
  //this is the force functions that the rule outputs
  //so we have the function and the strength to compute the COG

  double sumAreasAndMiddle = 0, sumAreas = 0;
  for (int i = 0; i < 5; i++) {
    printSomething(degreeOfForce[i]);
    if (degreeOfForce[i] != 0) {
      //big base + small base * (height / 2)
      double area = ((f_mem[i].pl2-f_mem[i].pl1) + getSmallBaseSize(f_mem[i].pl1, f_mem[i].pl2, degreeOfForce[i])) * degreeOfForce[i]/2.0;
      double middle = f_mem[i].pl2-(f_mem[i].pl2-f_mem[i].pl1)/2.0;
      /*
      Serial.println("fmem1, fmem2, small base, degree of force");
      Serial.println(f_mem[i].pl1);
      Serial.println(f_mem[i].pl2);
      Serial.println(getSmallBaseSize(f_mem[i].pl1, f_mem[i].pl2, degreeOfForce[i]));
      Serial.println(degreeOfForce[i]);
      */
      sumAreasAndMiddle = sumAreasAndMiddle + (area * middle);
      sumAreas = sumAreas + area;
      /*
      Serial.println(sumAreasAndMiddle);
      Serial.println(sumAreas);//*/
    } else { 
      continue;
    }
  }
  //Final Output!
  double output = sumAreasAndMiddle / sumAreas;
  
  /********* END OF COG *********/
  
  if (output > 255.0) {
	output = 255.0;
  } else if (output < 50.0) {
	output = 170.0;
  }

  analogWrite(transistorPin, output);
  //*
  printSomething(error_val - desiredHeight);
  printSomething(cherror_val);
  printSomething(output);
  //*/
  printSomething(desiredHeight);
  Serial.println();
  /*
  Serial.print(error_val / 100.0);
  Serial.print(" cm,");
  Serial.print("speed (ce): ");
  Serial.print(cherror_val);
  Serial.print(", output = ");
  Serial.println(output);
  */
  delay(1);
}

double microsecondsToCentimeters(double microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}



