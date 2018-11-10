boolean senseFlag;
static const int NUM_STEPS = 5; // Number of values to store in rolling average filter. Probably too small
unsigned long timesteps[NUM_STEPS];
static const int SPOKES = 5; // Number of times per revolution the sensor will be tripped
unsigned int DEBOUNCETHRESHHOLD = 50; // For real testing, this should be 0.
int counter;


void setup() {
  pinMode(2, INPUT); 
  attachInterrupt(digitalPinToInterrupt(2), recordData, FALLING); //Catches tripped sensor
  Serial.begin(9600);
  counter = 0; // Index for rolling average array


  // Initialize array with zeros
  for (int i = 0; i < NUM_STEPS; i++) {
    timesteps[i] = 0;
  }
}

void loop() {
  boolean fullTimeFlag = true; // Initialize flagcheck
  // Reset counter to 0 after 5 iterations
  
  
  if (senseFlag) {
    static unsigned long lastPass = 0; // Initializes lastPass as 0 the first time
    unsigned long currentPass = millis(); // Grabs current time

    unsigned long timestep = currentPass - lastPass; // Finds the elapsed time since the last time the sensor was tripped
    timesteps[counter] = timestep; // Stores that value in the rolling average filter
    Serial.print("hello");  
    // Checks to see if average filter is full
    for (int i = 0; i < NUM_STEPS; i++) {
      if (timesteps[i] == 0) {
        fullTimeFlag = false;
        break;
      }
    }
    
    // Calculate revolutions per second of wheel
    if (fullTimeFlag) {
      double averageTimeStep = average(timesteps); // Gets average from array
      float rps = 1/(averageTimeStep * SPOKES * 0.001); // Calculates revolutions per second
      //message = (uint8_t) rps
      
      unsigned char message[] = {0, 0, 0, 0, 0, 0, 0, 0};
      String bin_rps = String(rps, BIN);
      bin_rps.toCharArray(message, 8);
      // Can stuff(message);
      Serial.print("Revolutions per second: ");
      Serial.println(rps);
//    Serial.println(bin_rps);
      CAN.sendMsgBuf(0x18, 0, 8, messaage);
      delay(100); 
    }

    // Reset flags and counter
    lastPass = currentPass;
    senseFlag = false;
    counter++;
    if (counter == NUM_STEPS){
      counter = 0;
    }
  }
  
}

void recordData() {
  // When metal passes in front of the hall effect sensor, set flag
  //We did debouncing here but at fast speeds we should be careful because at high speeds debouncing is bad
  //However, this may be a limitation of the sensor, as it seems to "bounce" approximately 40 milliseconds apart, which isn't actually that fast
  
 
  static unsigned long lastTrigger = 0;
  unsigned long currentTrigger = millis();
  if(currentTrigger-lastTrigger > DEBOUNCETHRESHHOLD)
    senseFlag = true;
  lastTrigger = currentTrigger;
}

double average(unsigned long timesteps[]) {
  double average = 0;
  
  for (int i = 0; i < NUM_STEPS; i++) {
    average += timesteps[i];
  }

  average /= NUM_STEPS;

  return average;
}
