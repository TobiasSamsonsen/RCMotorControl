#include <EnableInterrupt.h>
#include <StandardCplusplus.h>
#include <math.h>

#define NUM_CHANNELS 2 // Number of channels you want to support
const int INPUT_CHANNEL_PINS[] = {A0, A1}; // Corrected pin assignments

// Motor A connections
#define enA 9
#define in1 8
#define in2 7
// Motor B connections
#define enB 3
#define in3 5
#define in4 4

// generic helper function for mapping one range into another
template <typename T, typename U, typename V>
V mapRange(T value, T in_min, T in_max, U out_min, U out_max) {
    U out_value;
    if (value >= in_min) {
        out_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    } else {
        out_value = out_min;
    }
    out_value = constrain(out_value, out_min, out_max);
    return out_value;
}

class InputChannel {
public:
  int pin;
  volatile uint32_t startTime;
  volatile uint16_t pulseDuration;
  float value;
  uint16_t rangeMin;
  uint16_t rangeMax;

  InputChannel(int pin, uint16_t rangeMin, uint16_t rangeMax) : pin(pin), rangeMin(rangeMin), rangeMax(rangeMax) {
    startTime = 0;
    pulseDuration = 0;
    value = 0.0f;
  }

  void handleInterrupt() {
    if (digitalRead(pin) == HIGH) {
      startTime = micros();
    } else {
      pulseDuration = micros() - startTime;
      if (pulseDuration > 3000) {
        pulseDuration = (this->rangeMax + this->rangeMin) / 2;
      }
      double low=-1.0, high=1.0;
      // Map the pulse duration to a value (e.g., 1000-2000us to -1.0 to 1.0)
      value = mapRange<uint16_t, double, float>(static_cast<uint16_t>(pulseDuration), this->rangeMin, this->rangeMax, low, high);
      // sensetivity function      
      value = sin((PI/2) * value);
    }
  }

  void debug() {
    if (pin == A1) {
      Serial.print("\rPulse Duration: " + String(pulseDuration) + " \t\t\t");
      Serial.print("\r\t\t\tChannel value: " + String(value) + "\t\t\t");
    }
  }
};

// Array of input channels
InputChannel inputChannels[NUM_CHANNELS] = {
    InputChannel(A0, 920, 1792),
    InputChannel(A1, 888, 1972)
};

// Interrupt handler functions for each channel
void handleInterruptChannel0() {
  inputChannels[0].handleInterrupt();
}

void handleInterruptChannel1() {
  inputChannels[1].handleInterrupt();
}

// Function to initialize input channels
void setupInputChannels() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
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

  int powerA_int = mapRange<float, int, int>(powerA, -1.0, 1.0, -255, 255);
  int powerB_int = mapRange<float, int, int>(powerB, -1.0, 1.0, -255, 255);
  
  Serial.print("\rLeft: " + String(powerA_int) + "\tRight: " + String(powerB_int));

  // Motor A
  if (powerA_int >= 0){ // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (powerA_int < 0){ // Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, abs(powerA_int * 0.4)); // Power

 // Motor B
  if (powerB_int >= 0){ // Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (powerB_int < 0){ // Backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  // Serial.println("Left: " + String(powerA_int) + "\tRight: " + String(powerB_int));
  analogWrite(enB, abs(powerB_int * 0.4)); // Power
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

// Function to calculate the absolute value of a float number
#define absf(x) ((x)>0?(x):-(x))

float deadzone(float value, float threshold) {
  if (absf(value) <= threshold) {
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
  float steeringAngle = deadzone(inputChannels[0].value*-1, 0.1) * 0.5; // Assuming the first channel controls steering
  float power = deadzone(inputChannels[1].value*-1, 0.25); // Assuming the second channel controls power
  
  last_steering = smoothen(steeringAngle, last_steering);
  last_power = smoothen(power, last_power);

  // Calculate the power distribution between the engines
  engine1Power = ((last_power + last_steering) / 2);
  engine2Power = ((last_power - last_steering) / 2);
  // Serial.println("Left: " + String(engine1Power) + "\tRight: " + String(engine2Power));

  // Ensure the power values are within the expected range
  engine1Power = constrain(engine1Power, -1.0, 1.0);
  engine2Power = constrain(engine2Power, -1.0, 1.0);
  // Set the speed of the engines
  // Serial.print("\rLeft: " + String(engine1Power) + "\tRight: " + String(engine2Power));
  setEngineSpeed(engine1Power, engine2Power);

}
