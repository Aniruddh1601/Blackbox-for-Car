#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>
#include <stdlib.h>
#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
int t;
char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i)
{ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

int L=600;                        //Use 600 data point to pull data into matrix 
boolean data[1000];               //Need to be more then above L
boolean result[100];              //Need to be more then 40 
int resultcount=0;                //End result shoudl be 39.
int data_pin=24;                   //Set the data pin location
int extract_data_threadhold = 8;  //This threadhold between 0 or 1. It is the counts of high status between low voltage.
int DHT_type=22;                  //11 means DHT11, 22 means DHT22.  It will only be used at the end of data caculation
double HR=0, TM=0;  

const int AOUTpin=A0;         //the AOUT pin of the Data_file sensor goes into analog pin A0 of the arduino
int i,alco_high,alco_low,Data_file,data1[200];

int CO=A1;
int airqua;
int apin=A2;
int LD1=A3;
int LD2=A4;
int LD3=A5;                           //LDR and ultrasonic
int trig=A6;
int echo=A7;
float dist,dur;
String string_;
String airq,di,ld,co,al,moi,a;

TinyGPS gps;
SoftwareSerial ss(0,1);
SoftwareSerial BTSerial(4,5);
static char dtostrfbuffer[20];
int CS = 53;
int LED = 13;

//Define String
String SD_date_time = "invalid";
String SD_lat = "invalid";
String SD_lon = "invalid";
String dataString ="";

static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec, int SD_val);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

void setup()
{
  pinMode(CS, OUTPUT);  //Chip Select Pin for the SD Card
  pinMode(LED, OUTPUT);  //LED Indicator
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(LD1,INPUT);
  pinMode(LD2,INPUT);
  pinMode(LD3,INPUT);
  //Serial interfaces
  Serial.begin(115200);
  ss.begin(4800);
  
  //Connect to the SD Card
  if(!SD.begin(CS))
  {
    Serial.println("Card Failure");
    return;
  }
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.println(TinyGPS::library_version());
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();
  Serial.println("Sats HDOP Latitude Longitude Fix  Date       Time       Date Alt     Course Speed Card  Data_file Course Card  Chars Sentences Checksum");
  Serial.println("          (deg)    (deg)     Age                        Age  (m)     --- from GPS ----  ---- to London  ----  RX    RX        Fail");
  Serial.println("--------------------------------------------------------------------------------------------------------------------------------------");

  pinMode(AOUTpin, INPUT);//sets the pin as an input to the arduino
for(i=2;i<=5;i++)
{
  pinMode(i,OUTPUT);//sets the pin as an output of the arduino
}
 File Data_file=SD.open("mpu6050_output.csv", FILE_WRITE);
 String Data_file_header="Accleration x:, Acceleration y:, Accleration z:, temperature:, gyro x:, gyro y:, gyro z:";
 Data_file.print(Data_file_header);
 Data_file.close();

 File moist= SD.open("MOIST.csv",FILE_WRITE);
 String mois_header="moisture:, Temperautre:";
 moist.print(mois_header);
 moist.close();

 File alcohol_=SD.open("alcohol_level.csv",FILE_WRITE);
 String alcohol_header="alcohol level,";
 alcohol_.print(alcohol_header);
 alcohol_.close();
 
 File co_= SD.open("carbon_monoxide.csv", FILE_WRITE);
 String co_header="CO level:,";
 co_.print(co_header);
 co_.close();
 
 File air_= SD.open("Air_Quality.csv", FILE_WRITE);
 String air_header="Air Quality: ,";
 air_.print(air_header);
 air_.close();
 
 File dis_=SD.open("distance_.csv", FILE_WRITE);
 String dist_header="distance_file: ,";
 dis_.print(dist_header);
 dis_.close();

 File ldr_=SD.open("visibility.csv", FILE_WRITE); 
 String visi_header="LDR1: ,LDR2: ,LDR3: ";
 ldr_.print(visi_header);
 ldr_.close();

 SoftwareSerial BTSerial(4,5); //tx,rx
 BTSerial.begin(38400);
  
}

void mpu()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  t=temperature/340.00+36.53;
  String ax=String(convert_int16_to_str(accelerometer_x));
  String ay=String(convert_int16_to_str(accelerometer_y));
  String az=String(convert_int16_to_str(accelerometer_z));
  String tmp=String(t);
  String gx=String(convert_int16_to_str(gyro_x));
  String gy=String(convert_int16_to_str(gyro_y));
  String gz=String(convert_int16_to_str(gyro_z));
  a="Accleration x: "+ax+", Acceleration y: "+ay+", Accleration z: "+az+", temperature: "+tmp+", gyro x:"+gx+", gyro y: "+gy+", gyro z: "+gz;
  File Data_file=SD.open("mpu6050_output.csv", FILE_WRITE);
  if(Data_file)
  {
    Data_file.println(a);
    Data_file.close();
    // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(t);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  }
  else
  {
    Serial.println("couldn't open mpu6050 file");
  }
}

void Get_Data()
{
  for (int i = 0; i <= L; i++)
  {
    data[i]=digitalRead(data_pin);
 
   }
}
void dht_ani()
{
  readDHT();
    String hr=String(HR);
    String tm=String(tm);
  moi="moisture: "+hr+", Temperautre: "+tm; 
  File moist= SD.open("MOIST.csv",FILE_WRITE);
  if(moist)
  {
    moist.println(moi);
    moist.close();
    Serial.print("DHT22, HR = ");Serial.print(HR,1);Serial.print("%, Temperature = ");Serial.print(TM,1);
  }
}
   
void mq3()
{
   alco_high=0; // initially Data_file level is zero
 alco_low = 1023; // low threshold is 1023

 for(i=200; i>0; i--)
 {
      data1[i] = data1[i-1]; // decrement of data  
      if(data1[i]>alco_high)
      alco_high=data1[i];
      if(data1[i]<alco_low)
      alco_low=data1[i];
 }        
      
data1[0]= analogRead(AOUTpin);//reads the analaog value from the Data_file sensor's AOUT pin
Data_file=map(data1[0],0,1023,0,600);
 al=",";
al+=String(Data_file);

 File alcohol_=SD.open("alcohol_level.csv",FILE_WRITE);
if(alcohol_)
{
  alcohol_.println(al);
  alcohol_.close();
  Serial.print("Data_file value: ");
  Serial.println(Data_file);
}
else
{
  Serial.println("error opening sd card Data_file level file");
}

}

void mq7()
{
  // read the input on analog pin 0:
  int CO_Value = analogRead(CO);
   co=",";
  co+= String(CO_Value);
  File co_= SD.open("carbon_monoxide.csv", FILE_WRITE);
  if(co_)
  {
    co_.println(co);
    co_.close();
    Serial.println("CO level:");
    Serial.println(CO_Value);
  }
}

void mq135()
{
  airq=",";
airqua = analogRead(apin);              // read analog input pin 0
airq+=String(airqua);
File air_= SD.open("Air_Quality.csv", FILE_WRITE);
if(air_)
{
  air_.println(airq);
  air_.close();
  Serial.print("AirQua=");
  Serial.print(airqua, DEC);               // prints the value read
  Serial.println(" PPM");
                        
}
else
{
  Serial.println("error opening sd card air quality file");
}

}

void ultra()
{
  digitalWrite(trig,LOW);
  delay(2);
  digitalWrite(trig,HIGH);
  delay(10);
  digitalWrite(trig,LOW);
  dur=pulseIn(echo,HIGH);
  dist=(dur*0.034)/2;    
   di=",";
  di+=String(dist);
   File dis_=SD.open("distance_.csv", FILE_WRITE);
  if(dis_)
  {
    dis_.println(di);
    dis_.close();
    Serial.println("Data_file: "+di);
  }
  else
  {
    Serial.println("error opening Data_file file");
  }
}

void LDR()
{
  String ld="";
  ld=String(analogRead(LD1))+","+String(analogRead(LD2))+","+String(analogRead(LD3));
  File ldr_=SD.open("visibility.csv", FILE_WRITE);
  if(ldr_)
  {
    ldr_.println(ld);
    ldr_.close();
    Serial.println(ld);
   }
   else
   {
    Serial.println("error opening visibility file");
   }
}

void loop()
{
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    if (feedgps())
      newdata = true;
  }
  
  gpsdump(gps);
  
  //Write the newest information to the SD Card
  dataString = SD_date_time + "," + SD_lat + "," + SD_lon;
  if(SD_date_time != "invalid")
    digitalWrite(LED, HIGH);
  else
    digitalWrite(LED, LOW);
    
  //Open the Data CSV File
  File gps_file = SD.open("GPS.csv", FILE_WRITE);
  if (gps_file)
  {
    gps_file.println(dataString);
    Serial.println(dataString);
    gps_file.close();
  }
  else
  {
    Serial.println("\nCouldn't open the gps file!");
  }

  mpu();
  delay(50);
  dht_ani();
  delay(50);
  mq3();
  delay(50);
  mq7();
  delay(50);
  mq135();
  delay(50);
  ultra();
  delay(50);                                                                                                           
}

static void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const float LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age); 
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5, 1); //LATITUDE
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5, 2); //LONGITUDE
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);

  print_date(gps); //DATE AND TIME

  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 8, 2, 0);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2, 0);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2, 0);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0UL : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : TinyGPS::course_to(flat, flon, 51.508131, -0.128002), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2, 0);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  feedgps();
}

static void print_float(float val, float invalid, int len, int prec, int SD_val)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    Serial.print(sz);
    if(SD_val == 1) SD_lat = sz;
    else if(SD_val == 2) SD_lon = sz;
  }
  else
  {
    Serial.print(val, prec);
    if (SD_val == 1) SD_lat = dtostrf(val,10,5,dtostrfbuffer);
    else if (SD_val == 2) SD_lon = dtostrf(val,10,5,dtostrfbuffer);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(" ");
  }
  feedgps();
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
  {
    Serial.print("*******    *******    ");
    SD_date_time = "invalid";
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
    SD_date_time = sz;
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  feedgps();
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}

static bool feedgps()
{
  while (ss.available())
  {
    if (gps.encode(ss.read()))
      return true;
  }
  return false;
}

void readDHT() 
{
  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.  Pull from HIGH to LOW to tell DHT11 prepare sending dada out
  pinMode(data_pin, OUTPUT);
  //Pull up the data pin prepare for the next step
  digitalWrite(data_pin, HIGH);delay(100);  
  // First set data line low for 20 milliseconds to trigger DHT11 to start generate result.
  // Page 5 in datasheet, it needs >18ms.  
  digitalWrite(data_pin, LOW);delay(20); 

  // End the start signal by setting data line high for 40 microseconds.
  digitalWrite(data_pin, HIGH);
  delayMicroseconds(40);

  // Now start reading the data line to get the value from the DHT sensor.
  pinMode(data_pin, INPUT_PULLUP);  
  delayMicroseconds(40);  // Delay a bit to let sensor pull data line low, and wait till low before get data.   
  Get_Data();
  //Display_Result();  //You can use Serial Plot to see the data like oscolscope
  extract_data();  //Extract data and show end reult.
}


void extract_data()
{
  int high=0;
  resultcount=0;
  
  for (int i = 0; i< L-1;i++){
    if ((data[i]==true)){high++;}
    if ((data[i]==false)){
      if (high>0){if(high > extract_data_threadhold){result[resultcount]=true;high=0;resultcount++;}}
      if (high>0){if(high < extract_data_threadhold){result[resultcount]=false;high=0;resultcount++;}}
      }
    }
  
  //for (int j = 0; j<= resultcount;j++){
  for (int j = 0; j< 40;j++){
    result[j]=result[j+1];
    Serial.print(result[j]);
    if (((j+1)/8.0)==int((j+1)/8.0)){Serial.print(" ");}//Add a space every 8 bits
    }  
  Serial.println();   
  //Prepare 2^i.  Arduino IDE do not understand ^ operant 
  double powerof2[16];
  powerof2[0]=1;
  for (int i=1;i<16;i++){
    powerof2[i]=1;
    for (int j=0;j<i;j++){
      powerof2[i]=powerof2[i]*2;  
    }
  }
   //Caculate for DHT22
  if (DHT_type==22){ 
    int startpoint=0;
    //Extract HR, temperature, and verify code
    for (int i=0;i<16;i++){
      HR=HR+powerof2[15-i]*(result[i]*1.0);
      TM=TM+powerof2[15-i]*(result[16+i]*1.0);
      //Serial.print("Debug:");Serial.print(" i=");Serial.print(i);Serial.print(" powerof2[15-i]=");Serial.print(powerof2[15-i]);Serial.print(", result[i]=");Serial.print(result[i]);Serial.print(" HR=");Serial.print(HR);Serial.print(" TM=");Serial.print(TM);Serial.println();
      //delay(500);
    }
    //Check SUM with 5th byte, ignore overflow
    int VR1=0,VR2=0,VR3=0,VR4=0,VRR=0,VR=0;
    for (int i=0;i<8;i++){
      VR1=VR1+powerof2[7-i]*(result[i]*1.0);    //1st byte
      VR2=VR2+powerof2[7-i]*(result[i+8]*1.0);  //2nd byte
      VR3=VR3+powerof2[7-i]*(result[i+16]*1.0); //3rd byte
      VR4=VR4+powerof2[7-i]*(result[i+24]*1.0); //4th byte
      VR=VR+powerof2[7-i]*(result[i+32]*1.0);   //VR byte      
    }
    VRR=VR1+VR2+VR3+VR4;
    if (VRR>=256)VRR=VRR-256;  //Remove overflow 
    HR=HR/10.0;
    TM=TM/10.0;
    
  }
}
