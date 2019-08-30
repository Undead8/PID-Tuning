#include <Wire.h>
#include <DFRobot_SHT20.h> // https://github.com/DFRobot/DFRobot_SHT20
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <pidautotuner.h> // https://github.com/jackw01/arduino-pid-autotuner

// Tune using this output target and a specific PID loop interval
const float TARGET = 20.0;
const unsigned long LOOP_INTERVAL = 300000;

// Pin constants
const int OUTPUT_PIN = 3;

// Other variables
float temperature;
float humidity;

SerLCD lcd; // Default I2C address is 0x72
DFRobot_SHT20 sht20; //  Default I2C address is 0x40
PIDAutotuner tuner = PIDAutotuner();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // LCD - Init
  lcd.begin(Wire); //Set up the LCD for I2C communication
  lcd.setBacklight(0, 0, 0); //Set backlight to black
  lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  
  // SHT20 - Init 
  sht20.initSHT20();

  // Set pin modes
  pinMode(OUTPUT_PIN, OUTPUT);
  
  // Delay to make sure everything is init.
  delay(100);

  tuner.setTargetInputValue(TARGET);
  tuner.setLoopInterval(LOOP_INTERVAL);
  tuner.setOutputRange(0, 255);
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
  tuner.startTuningLoop();

  long microseconds;
  while (!tuner.isFinished()) {

    long prevMicroseconds = microseconds;
    microseconds = micros();

    // Get input value here (temperature, encoder position, velocity, etc)
    checkTempHum();

    double output = tuner.tunePID(temperature);
    analogWrite(OUTPUT_PIN, output);

    while (micros() - microseconds < LOOP_INTERVAL) delayMicroseconds(1);
   }
  
  // Turn the output off here.
  digitalWrite(OUTPUT_PIN, LOW);

  // Get PID gains - set your PID controller's gains to these
  double kp = tuner.getKp();
  double ki = tuner.getKi();
  double kd = tuner.getKd();

  // Display on LCD
  lcd.clear();
  lcd.print("kp: ");
  lcd.print(kp);
  lcd.setCursor(9, 0);
  lcd.print("ki: ");
  lcd.print(ki);
  lcd.setCursor(0, 1);
  lcd.print("kd: ");
  lcd.print(kd);  
}

void loop() {

}


// Check temperature and humidity, then display on screen
void checkTempHum() {
  temperature = sht20.readTemperature();
  humidity = sht20.readHumidity();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(static_cast<int>(temperature));
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(static_cast<int>(humidity));
  lcd.print("%");
}
