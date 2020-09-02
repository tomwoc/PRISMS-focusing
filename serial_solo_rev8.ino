#include "AccelStepper.h" //library used to control motor. See https://www.airspayce.com/mikem/arduino/AccelStepper/

const byte buffer_size = 64; //buffter for serial input data
char received_chars[buffer_size];
char temp_chars[buffer_size];
char message_raw[buffer_size] = {0};
long integer_raw = 0; //integer part of input
boolean new_data = false; //boolean if new data is detected in buffer
bool enable = false;
long stp_rev = 800; //steps per revolution 
long end_limit = stp_rev*46; //end limit in steps
float max_speed = stp_rev*6; //maximum motor speed
float acceleration = stp_rev*3; 
bool stepper_active = false; //true if routine in progress
bool switch_activated = false; 

int PUL = 5; //define Pulse pin
int DIR = 4; //define Direction pin
int ENA = 3; //define Enable pin
int ALM = 2; //define Alarm detect pin
int SWI = 12; //define Switch detect pin
int PWR = 11; //define Power detect pin

AccelStepper stepper(1, PUL, DIR);

void setup() {
  pinMode (PUL, OUTPUT); //Set pin states
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ALM, INPUT);
  pinMode (SWI, INPUT);
  pinMode (PWR, INPUT);

  stepper.setPinsInverted(true, false, false); //Invert direction. Ensure that going from step 0 to a positive step number results in the focusing shaft rotating clockwise (screwing in to body)
  stepper.setEnablePin(ENA);
  stepper.disableOutputs();
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(acceleration);
  stepper.setMinPulseWidth(2.5); //Min pulse width of driver board


  Serial.begin(9600);
  Serial.println("Ardu_Stepper");

  while (digitalRead(PWR) == LOW) {
    Serial.println("Stepper drive not detected"); 
  } //Begin loop when power detected
  Serial.println("Stepper drive detected"); //This message in the serial buffer means the driver is detected and motor is primed
}

void loop() {
  recieve_input(); //Reads input information sandwhiched between @ and #
  if (new_data == true) {
    strcpy(temp_chars, received_chars); //Strips raw input data to integer and characters
    parse_data(); //Function to split the received inputs up to a delimiter
    process_data(); //Turns the integer and character part in to motor command
  }
  single_step(); //Iterates a single step
  alm_detect(); //Looks for alarm pin on driver board
  swi_detect(); //Switch pin detect
}

void recieve_input() {
  static byte index = 0;
  static boolean in_progress = false;
  char start_char = '@'; //Start character
  char end_char = '#'; //End character
  char raw;

  while (Serial.available() > 0 && new_data == false) { //Serial.avaliable is 0 when the buffer is empty
    raw = Serial.read();
    if (in_progress == true) { 
      if (raw != end_char) { //Iterates buffer, adding characters until the # is detected
        received_chars[index] = raw; 
        index++;
        if (index >= buffer_size) {
          index = buffer_size - 1;
        }
      }
      else {
        received_chars[index] = '\0'; //True if the end character is #
        in_progress = false; //End of input read
        index = 0;
        new_data = true;
      }
    }
    else if (raw == start_char) { //Searches for first character in buffer
      in_progress = true;
    }
  }
}

void parse_data() {
  char * strtok_index;
  strtok_index = strtok(temp_chars, ":"); //Splits input at the : character
  strcpy(message_raw, strtok_index); //Copies string
  strtok_index = strtok(NULL, ":"); 
  integer_raw = atol(strtok_index); //Converts string to a long integer
}

void process_data() {
  if (message_raw[0] == 'H') { //Home command, moves to position 0
    if (enable ==  true) {
      stepper.moveTo(0);
      stepper_active = true;
    }
  }
  else if (message_raw[0] == 'F') { //Move to end command (kept during testing but otherwise not used)
    if (enable ==  true) {
      stepper.moveTo(end_limit);
      stepper_active = true;
    }
  }
  
  else if (message_raw[0] == 'S') { //Soft stop command. Position information retained 
    bool stopping = true;
    stepper.stop();
  }
  else if (message_raw[0] == 'D') { //Disable command. Position retained, however mechanism can move as motor is powered off
    enable = false;
    stepper.disableOutputs();
    delayMicroseconds(5);
    print_input();
  }
  else if (message_raw[0] == 'E') { //Enable command, locks motor in place
    enable = true;
    stepper.enableOutputs();
    delayMicroseconds(5);
    print_input();
  }
  else if (message_raw[0] == 'P') { //Returns current position 
    Serial.print("@");
    Serial.print(stepper.currentPosition());
    Serial.print("#");
  }
  else if (message_raw[0] == 'M') { //Move to command
    if (enable ==  true) {
      if (stepper.currentPosition()+integer_raw <= end_limit && stepper.currentPosition() + integer_raw >= 0) { //Ensures command falls within limits
        stepper.move(integer_raw);
        stepper_active = true;
      }
    }
  }
  else if (message_raw[0] == 'V') { //Set velocity
    max_speed = integer_raw;
    stepper.setMaxSpeed(max_speed);
    print_input();
  }
  else if (message_raw[0] == 'Z') { //Starts the zeroing routine
    zero_routine();
  }
  else if (message_raw[0] == 'T') { //Starts the microswitch test procedure, note the code is blocking here so cannot escape once started
    zero_test();
  }
  new_data = false;
}

void single_step() { //Iterates a single step if required
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  else if (stepper_active == true) {
    print_input();
    stepper_active = false;
  }
}

void zero_routine() { //Zeroing routine. Position 0 defined as 200 steps forward from when microswitch activated
  if (enable == true) {
    while (digitalRead(SWI) == LOW) {
      stepper.move(-1);
      stepper.run();
      alm_detect();
    }
    stepper.setCurrentPosition(0);
    stepper.runToNewPosition(200);
    stepper.setCurrentPosition(0);
    print_input();
  }
}

void zero_test() { 
  if (enable == true) {
    while (digitalRead(SWI) == LOW) {
      stepper.move(-1);
      stepper.run();
      alm_detect();
    }
    //Serial.print("@");
    Serial.print(stepper.currentPosition());
    Serial.print(',');
    
    while (digitalRead(SWI) == HIGH) {
      stepper.move(1);
      stepper.run();
      alm_detect();
    }
    //Serial.print("@");
    Serial.println(stepper.currentPosition());
    //Serial.println('#');
    stepper.runToNewPosition(0);
  }
}

void alm_detect() { //Reads the alarm pins and disables output if alarm detected. Likely due to an acceleration parameter too high or something stopping the gear train from moving
  if (digitalRead(ALM) == HIGH) {
    enable = false;
    stepper.disableOutputs();
    Serial.println("Alarm");
  }
}

void swi_detect() { //Reads switch state and stops output if the microswitch is detected outside of the zeroing routine
  if (digitalRead(SWI) == HIGH) {
    if (enable == true) {
      stepper.disableOutputs();
      enable = false;
      Serial.println("Zero switch triggered");
    }
  }
}

void print_input() { //Prints input commands inbetween the start and end characters
  Serial.print("@");
  Serial.print(received_chars);
  Serial.print('#');
}
