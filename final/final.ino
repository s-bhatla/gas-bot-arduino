#include <AFMotor.h>
#include <NewPing.h> //need to install a lib for this
#include <Servo.h> 
#include <deque.h>

AF_DCMotor motor1(1, MOTOR12_8KHZ); 
AF_DCMotor motor2(2, MOTOR12_8KHZ);
AF_DCMotor motor3(3, MOTOR12_8KHZ); 
AF_DCMotor motor4(4, MOTOR12_8KHZ);
int val;
int actions[4];
Servo myservo;	// create servo object to control a servo
int pos = 0;	// variable to store the servo position

long duration=0; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
deque<int> movlist(4);

#define TRIG_PIN A0 
#define ECHO_PIN A1 
#define MAX_DISTANCE 200 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
int distance = 100;

float pinSensor = A5;
float gasValue;
float prevgasValue = 0;

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

int checkleft(){
  for(pos = 0; pos <= 90; pos += 1) 
	{
		myservo.write(pos);
		delay(15);
	}
  int dist = readPing();
  delay(2000);
	// sweeps from 90 degrees to 0 degrees
	for(pos = 90; pos>=0; pos-=1)
	{
		myservo.write(pos);
		delay(15);
	}
  return dist;
}

int checkright(){
  for(pos = 0; pos >= -90; pos -= 1) 
	{
		myservo.write(pos);
		delay(15);
	}
	// sweeps from -90 degrees to 0 degrees
	for(pos = -90; pos<=0; pos+=1)
	{
		myservo.write(pos);
		delay(15);
	}
  int dist = readPing();
  delay(2000);
}


void forward(){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void left(){
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);//needs calibration
}

void right(){
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);//needs calibration
}

void backward(){
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);//change
}

void stop(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void detect(){
  int dis = readping();
  if(dis <= 15)
  {
    stop();
  }
}

void setup() {
  // put your setup code here, to run once:
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  myservo.attach(10);   
  pinMode(pinSensor, INPUT);
  Serial.begin(9600);
}

int state=1;
movelist.enqueue(1);
void loop() {
  // put your main code here, to run repeatedly:
  gasValue = analogRead(pinSensor);
  int last = movelist.pop_back();
  int seclast = movelist.pop_back();
  movelist.push_back(seclast);
  movelist.push_back(last);

  if (gasValue > prevgasValue){
      if(last==2)
      {
        //oneeighty()
        forward();
        while(duration<4000){
          detect();
          wait(400);
          duration=duration+400;
        }
        movlist.push_back(2);
      }
      else{
        forward();
        while(duration<4000){
          detect();
          wait(400);
          duration=duration+400;
        }
        stop();
        movlist.push_back(1);
      }
  }
  if(gasValue < prevgasValue){
    if(state==1)
    {
      backward();
      while(duration<4000){
        detect();
        wait(400);
        duration=duration+400;
      }
      stop();
      movlist.push_back(2);
    }
    if(state==2)
    {
      
    }
  }
  prevgasValue = gasValue;

}
