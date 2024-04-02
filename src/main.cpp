#include <EnableInterrupt.h>
#include <math.h> // For tan function

#define NUM_CHANNELS 2 // Number of channels you want to support
const int INPUT_CHANNEL_PINS[] = {A0, A1}; // Corrected pin assignments

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// InputChannel structure
struct InputChannel {
  int pin;
  volatile uint32_t startTime;
  volatile uint16_t pulseDuration;
  int value;
};

// Array of input channels
InputChannel inputChannels[NUM_CHANNELS];

// Function prototype for handleInterrupt
void handleInterrupt(int channelIndex);

// Interrupt handler functions for each channel
void handleInterruptChannel0() {
  handleInterrupt(0);
}

void handleInterruptChannel1() {
  handleInterrupt(1);
}

// Interrupt handler function
void handleInterrupt(int channelIndex) {
  InputChannel& channel = inputChannels[channelIndex];
  if (digitalRead(channel.pin) == HIGH) {
    channel.startTime = micros();
  } 
  else {
    channel.pulseDuration = micros() - channel.startTime;
    // Map the pulse duration to a value (e.g., 1000-2000us to 0-100%)
    channel.value = map(channel.pulseDuration, 1000, 2000, -100, 100);
    // Ensure the value stays within bounds
    channel.value = constrain(channel.value, -100, 100);
        
    // Now you can use channel.value to control your device
    // For example, write it to a motor controller or log it to Serial
    }
}

// Function to initialize input channels
void setupInputChannels() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    inputChannels[i].pin = INPUT_CHANNEL_PINS[i];
    pinMode(inputChannels[i].pin, INPUT);
    // Use a different ISR for each channel
    if (i == 0) {
        enableInterrupt(inputChannels[i].pin, handleInterruptChannel0, CHANGE);
    } 
    else if (i == 1) {
        enableInterrupt(inputChannels[i].pin, handleInterruptChannel1, CHANGE);
    }
 }
}

// Placeholder function to set the speed of an engine
void setEngineSpeed(float powerA, float powerB) {

  int powerA_int = static_cast<int>(map(powerA, -100, 100, -255, 255));
  int powerB_int = static_cast<int>(map(powerB, -100, 100, -255, 255));
  
  // Motor A
  if (powerA_int >= 0){ // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (powerA_int < 0){ // Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  Serial.println(powerA_int);
  analogWrite(enA, abs(powerA_int)); // Power

 // Motor B
  if (powerB_int >= 0){ // Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (powerB_int < 0){ // Backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  Serial.println(powerB_int);
  analogWrite(enB, abs(powerB_int)); // Power

}

void setup() {  
  Serial.begin(9600);
  setupInputChannels();
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

float engine1Power = 0;
float engine2Power = 0;

float deadzone(float value, float threshold) {
  if (abs(value) <= threshold) {
    return 0.0f;
  }
  return value;
}

const float ALPHA = 0.3f;
float smoothen(float new_value, float previous_value) {
  return ALPHA * new_value + (1 - ALPHA) * previous_value;
}

float last_steering = 0;
float last_power = 0;

void loop() {
  // Calculate the steering angle and power based on the input values
  float steeringAngle = deadzone(inputChannels[0].value, 12); // Assuming the first channel controls steering
  float power = deadzone((inputChannels[1].value) * -1, 1.); // Assuming the second channel controls power
  
  last_steering = smoothen(steeringAngle, last_steering);
  last_power = smoothen(power, last_power);

  // Calculate the power distribution between the engines
  engine1Power = (last_power + last_steering) / 2;
  engine2Power = (last_power - last_steering) / 2;

  // Ensure the power values are within the expected range
  engine1Power = constrain(engine1Power, -100, 100);
  engine2Power = constrain(engine2Power, -100, 100);

  // Set the speed of the engines
  setEngineSpeed(engine1Power, engine2Power);

  // For debugging purposes, print the values to the serial monitor
  // Serial.print("Steering Angle: ");
  // Serial.print(last_steering);
  // Serial.print("\tPower: ");
  // Serial.print(last_power);
  // Serial.print("\tEngine 1 Power: ");
  // Serial.print(engine1Power);
  // Serial.print("\tEngine 2 Power: ");
  // Serial.println(engine2Power);

  delay(500);
}
