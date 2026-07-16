/*

  Wiring:
    PWMA -> D3   AIN1 -> D5   AIN2 -> D4
    PWMB -> D6   BIN1 -> D8   BIN2 -> D7
    STBY -> D9
    IMU SDA -> A4   SCL -> A5

  Serial commands (115200 baud):
    kp 18.5      set proportional gain
    ki 0.75      set integral gain
    kd 1.4       set derivative gain
    set -3.6     set balance setpoint (degrees)
  -----------------------------------------------------------------------
*/

#include <Wire.h>

// ---- IMU ----
const uint8_t MPU_ADDR = 0x68;
float gyroBiasX = 0;
float accelAngleOffset = 0; // set automatically during calibration 
float angle = 0;
unsigned long lastTime = 0;
bool debug = true;
String inputBuffer = "";

// How often the PID/control step runs. Lower number = faster loop,
// higher number = slower. 10000 = 100 Hz, 20000 = 50 Hz, etc.

const unsigned long LOOP_PERIOD_US = 5000UL; // 200 Hz -- was running near max speed before

// ---- Motor pins (TB6612FNG) ----
const uint8_t PWMA = 3;
const uint8_t AIN1 = 5;
const uint8_t AIN2 = 4;
const uint8_t PWMB = 6;
const uint8_t BIN1 = 8;
const uint8_t BIN2 = 7;
const uint8_t STBY = 9;


 uint8_t DEADZONE = 10; // MIN MOTOR SPEED



const float OUTPUT_THRESHOLD = 0;


float MAX_OUTPUT = 250; //MAX MOTOR SPEED CLAMPED

// ---- PID tuning----
float Kp = 31.0;
float Ki = 1.0;
float Kd = 0.22;
float setpoint = 0.0; // OFFSET IS ALREADY CALCULATED DURING CALLIBRATION

float integral = 0;
float lastError = 0;

// ---- Serial command buffer ----
String serialBuffer;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRaw(int16_t &ay, int16_t &az, int16_t &gx) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperature, discard
  gx = (Wire.read() << 8) | Wire.read();
}

void driveMotors(float output) {
  output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);
  
  // Stops the motors completely when the tilt angle exceeds 45 degree so that the motor stops when the robot falls
  if (fabs(output) < OUTPUT_THRESHOLD) {
 
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    return;
  }

  bool forward = output >= 0;
  uint8_t pwm = (uint8_t)fabs(output);

  if (pwm < DEADZONE) {
    pwm = DEADZONE; // remap so a real (non-negligible) command actually moves the wheels
  }

  digitalWrite(AIN2, forward ? HIGH : LOW);
  digitalWrite(AIN1, forward ? LOW : HIGH);
  analogWrite(PWMA, pwm);

  digitalWrite(BIN2, forward ? LOW : HIGH);
  digitalWrite(BIN1, forward ? HIGH : LOW);
  analogWrite(PWMB, pwm);
}

// ---- Serial command handling ----
void processCommand(String cmd)
{
    cmd.trim();

    cmd.toLowerCase();

    if(cmd.startsWith("kp "))
    {
        Kp = cmd.substring(3).toFloat();
    }

    else if(cmd.startsWith("ki "))
    {
        Ki = cmd.substring(3).toFloat();
        integral = 0;
    }

    else if(cmd.startsWith("kd "))
    {
        Kd = cmd.substring(3).toFloat();
    }

    else if(cmd.startsWith("set "))
    {
        setpoint = cmd.substring(4).toFloat();
    }

    else if(cmd.startsWith("max "))
    {
        MAX_OUTPUT = cmd.substring(4).toFloat();
    }

    else if(cmd.startsWith("dead "))
    {
        DEADZONE = cmd.substring(5).toInt();
    }

    else if(cmd=="debug on")
    {
        debug = true;
    }

    else if(cmd=="debug off")
    {
        debug = false;
    }

    else if(cmd=="status")
    {
        Serial.println();
        Serial.println("-----STATUS-----");

        Serial.print("Kp: "); Serial.println(Kp);
        Serial.print("Ki: "); Serial.println(Ki);
        Serial.print("Kd: "); Serial.println(Kd);
        Serial.print("Setpoint: "); Serial.println(setpoint);
        Serial.print("Angle: "); Serial.println(angle);

        Serial.println();
        Serial.println("-----STATUS-----");

        Serial.print("Kp: "); Serial.println(Kp);
        Serial.print("Ki: "); Serial.println(Ki);
        Serial.print("Kd: "); Serial.println(Kd);
        Serial.print("Setpoint: "); Serial.println(setpoint);
        Serial.print("Angle: "); Serial.println(angle);

        return;
    }

    Serial.print("Kp=");
    Serial.print(Kp);

    Serial.print(" Ki=");
    Serial.print(Ki);

    Serial.print(" Kd=");
    Serial.print(Kd);

    Serial.print(" Set=");
    Serial.println(setpoint);

    Serial.print("Kp=");
    Serial.print(Kp);

    Serial.print(" Ki=");
    Serial.print(Ki);

    Serial.print(" Kd=");
    Serial.print(Kd);

    Serial.print(" Set=");
    Serial.println(setpoint);
}

void handleInput(Stream &port)
{
    while (port.available())
    {
        char c = port.read();

        if (c == '\r' || c == '\n')
        {
            if (inputBuffer.length())
            {
                processCommand(inputBuffer);
                inputBuffer = "";
            }
        }
        else
        {
            inputBuffer += c;
        }
    }
}

void setup() {
    
 Serial.begin(9600);

  Serial.println("Balance Robot Ready!");
  Wire.begin();

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  writeRegister(0x6B, 0x01); // wake up, clock source = gyro PLL
  delay(100);
  writeRegister(0x1A, 0x04); // DLPF ~21 Hz
  writeRegister(0x1B, 0x08); // gyro +-500 deg/s -> 65.5 LSB per deg/s
  writeRegister(0x1C, 0x08); // accel +-4g

  Serial.println(F("Calibrating -- hold the robot still, at the exact angle"));
  Serial.println(F("it should balance at. That pose becomes the new zero."));
  long sumGyro = 0;
  double sumAccelAngle = 0;
  int16_t ay, az, gx;
  for (int i = 0; i < 1000; i++) {
    readRaw(ay, az, gx);
    sumGyro += gx;
    sumAccelAngle += atan2((float)ay, (float)az) * 57.2958;
    delay(3);
  }
  gyroBiasX = sumGyro / 1000.0;
  accelAngleOffset = sumAccelAngle / 1000.0;
  Serial.println(F("Calibration done."));
  Serial.println(F("Commands: kp <val> | ki <val> | kd <val> | set <val>"));

  lastTime = micros();
}

void runControlStep(float dt) {
  int16_t ay, az, gx;
  readRaw(ay, az, gx);

  float gyroRate = (gx - gyroBiasX) / 65.5;              // deg/s
  float accelAngle = atan2((float)ay, (float)az) * 57.2958 - accelAngleOffset; // degrees, zero-corrected

  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accelAngle;

  // ---- Simple PID ----
  float error = setpoint - angle;
  integral += error * dt;


  float derivative = -gyroRate;
  lastError = error; // kept for reference/debugging, no longer used by D term

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Minimal safety net: cut power if it's fallen over, and stop the
  // integral from winding up while it's down.
  if (abs(angle) > 45.0) {
    output = 0;
    integral = 0;
  }

  driveMotors(output);

if(debug)
{
    Serial.print(angle);
    Serial.print(',');
    Serial.println(output);

    Serial.print(angle);
    Serial.print(',');
    Serial.println(output);
}
}

void loop() {
  unsigned long now = micros();
  if (now - lastTime >= LOOP_PERIOD_US) {
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    runControlStep(dt);
  }

  handleInput(Serial);
  // checked every pass, independent of the control rate above
}