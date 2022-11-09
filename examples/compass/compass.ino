// QMC 5883L
#include<Wire.h>
#include<math.h>

#define MAG_ADDR    0x0D
#define MAG_OUT_X_L 0x00
#define MODE_REG    0x09

uint8_t mag_xm, mag_xl, mag_ym, mag_yl, mag_zm, mag_zl;

float mag_x, mag_y, mag_z;
int azimuth;

typedef struct Vector
{
  float x, y, z;
} Vector;

void setupMAG()
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MODE_REG);
  //delay(100);
  Wire.write(0b00001101); // Continuous mode with +/-2 Gauss
  Wire.endTransmission();
}

Vector getMAG()
{
  Vector MAG;
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_OUT_X_L);
  Wire.endTransmission();

  Wire.requestFrom(MAG_ADDR, 6);
  if(Wire.available() >= 6)
  {
    mag_xl = Wire.read();
    mag_xm = Wire.read();
    mag_yl = Wire.read();
    mag_ym = Wire.read();
    mag_zl = Wire.read();
    mag_zm = Wire.read();

    mag_x = ((mag_xm << 8)|mag_xl)*0.0000833333;
    mag_y = ((mag_ym << 8)|mag_yl)*0.0000833333;
    mag_z = ((mag_zm << 8)|mag_zl)*0.0000833333;

    MAG.x = mag_x;
    MAG.y = mag_y;
    MAG.z = mag_z;
    
    return MAG;
  }
}

Vector calibrateCompass()
{
  Vector rawMag, trueMag;
  rawMag = getMAG();

  float calib[3];
  float raw[3] = {rawMag.x, rawMag.y, rawMag.z};
  // float bias[3] = {0.368478,0.132804,-0.217310};
  // float scale[3][3] = {{0.971691, -0.037843, -0.027270},
  //                      {-0.037843, 0.899053, -0.000747},
  //                      {-0.027270, -0.000747, 0.899432}};

  float bias[3] = {0.376457,0.119874,-0.165460};
  float scale[3][3] = {{0.880161, -0.029439, -0.022056},
                       {-0.029439, 0.969466, 0.014079},
                       {-0.022056, 0.014079, 0.949999}};

  // float bias[3] = {0.367769, 0.128360, -0.157271};
  // float scale[3][3] = {{0.930362, -0.011549, 0.004401},
  //                      {-0.011549, 0.896397, 0.019743},
  //                      {0.004401, 0.019743, 0.890681}};

  for (int i = 0; i < 3; i++)
  {
    calib[i] = raw[i] - bias[i];
  }

  for (int i = 0; i < 3; i++)
  {
    calib[i] = scale[i][0]*calib[0] + scale[i][1]*calib[1] + scale[i][2]*calib[2];
  }

  trueMag.x = calib[0];
  trueMag.y = calib[1];
  trueMag.z = calib[2];
  return trueMag;
}

float getAzimuth()
{
  Vector Mag_read;
  Mag_read = calibrateCompass();
  azimuth = atan2(Mag_read.y,Mag_read.x)*180/M_PI;
  return azimuth < 0 ? 360 + azimuth:azimuth;
}

void transmitData(Vector mag)
{
  float x = mag.x;
  float y = mag.y;
  float z = mag.z;

  uint8_t bufferData[13];
  bufferData[0] = 0xff; // Identify hardware
  for (int i = 0; i<sizeof(x); i++)
  {
    bufferData[i+1] = ((uint8_t*)&x)[i];
    bufferData[i+5] = ((uint8_t*)&y)[i];
    bufferData[i+9] = ((uint8_t*)&z)[i];
  }
  Serial.write(bufferData, 13);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  setupMAG();
}

void loop() {
  
  float head;

  // byte error;
  // Wire.beginTransmission(MAG_ADDR);
  // error = Wire.endTransmission();
  // Serial.println(error);

  Vector Mag;
  //Mag = calibrateCompass();
  
  // Serial.print("X : ");
  // Serial.print(Mag.x);
  // Serial.print("  Y : ");
  // Serial.print(Mag.y);
  // Serial.print("  Z : ");
  // Serial.print(Mag.z);
  // head = getAzimuth();
  // Serial.print("  YAW : ");
  // Serial.println(head);
  Mag = getMAG(); // raw data
  transmitData(Mag);
  delay(50);

}
