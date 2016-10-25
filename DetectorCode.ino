#include <dht.h>
#include <SoftwareSerial.h>
#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"
#include <math.h>

#define         MQ135_SENSOR                 (2)
#define         MQ_DEFAULTPPM 399
#define         MQ_DEFAULTRO 68550 
#define         RL_VALUE135                     (22000) 
#define         CALIBRATION_SAMPLE_TIMES     (50)    
#define         CALIBRATION_SAMPLE_INTERVAL  (500)  
#define         READ_SAMPLE_INTERVAL         (50)   
#define         READ_SAMPLE_TIMES            (5)     
#define         GAS_CO2                      (2)
#define         GAS_NO2                      (4)


#define dht_dpin 7 

#define         MQ_PIN                       (0)    
#define         RL_VALUE                     (5)    
#define         RO_CLEAN_AIR_FACTOR          (9.83) 
                                                    
 
#define         CALIBARAION_SAMPLE_TIMES     (50)   
#define         CALIBRATION_SAMPLE_INTERVAL  (500)  
                                                     
#define         READ_SAMPLE_INTERVAL         (50)    
#define         READ_SAMPLE_TIMES            (5)      
                                                     
 
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
float           CO2Curve[2]     =  {113.7105289, -3.019713765}; //MQ135
float           NO2Curve[2]     =  {84.07117895, -4.41107687}; 
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   
                                                   
                                                    
float           COCurve[3]  =  {2.3,0.72,-0.34};  
                                                   
float           SmokeCurve[3] ={2.3,0.53,-0.44};                                                       
float           Ro = 10;               
float mq_ro = 10000.0;  
int val = 0;               
int valr = 0;               
float valAIQ =0.0;
float valAIQ2 =0.0;
float valAIQro =0.0;
float lastAIQ =0.0;
int  value =0;


const int RX_PIN = 2;
const int TX_PIN = 3;
SoftwareSerial Bserial(RX_PIN, TX_PIN);
dht DHT;

int temperature;
int humidity;
float lpg,co,smoke;
String ts,hs,ls,cs,ss,c1s,c2s,co1s,co2s,c22s,ns;
char t[10],h[10],l[10],c[10],s[10],ct1[10],ct2[10],co1[10],co2[10],c2[10],n[10];
char tp;

int pin = 8;
int pin2 = 6;

unsigned long duration;
unsigned long duration2;

unsigned long starttime;
unsigned long sampletime_ms = 1000;//sampe 1s ;

unsigned long lowpulseoccupancy = 0;
unsigned long lowpulseoccupancy2 = 0;

float ratio = 0;
float ratio2 = 0;

float concentration = 0;
float concentration2 = 0;

float con;
float con2;

void setup() {
  Serial.begin(9600);
  Bserial.begin(9600);
  Bserial.println("\nCalibrating Sensor..!!");
  Ro = MQ2Calibration(MQ_PIN); 
  mq_ro=MQ135Calibration(MQ135_SENSOR,MQ_DEFAULTPPM,CO2Curve);
  pinMode(dht_dpin,INPUT);
  pinMode(8,INPUT);

  starttime = millis();//get the current time;
  Bserial.println("\nCalibrating Done...!!");
  Bserial.write("\nClick Refresh to get Reading.....\n");
}

void loop() {
  readTemperatureAndHumidity();
  readMQ2();
  readMQ135();
  readPPD42NS();
  convert();
  if(Bserial.available()){

      tp = (char)Bserial.read();
      if(tp == 'R'){
        Serial.print(tp);
        Serial.write("\nTemperature = ");
        Serial.write(t);
        Serial.println();
        Serial.write("\nHumidity =  ");
        Serial.write(h);
        Serial.println();
        Serial.print("\nLPG = "); 
        Serial.print(l);
        Serial.print( "ppm" );
        Serial.println();
        Serial.print("\nCO = "); 
        Serial.print(c);
        Serial.print( "ppm" );
        Serial.println();
        Serial.print("\nNO2 = "); 
        Serial.print(c2);
        Serial.print( "ppm" );
        Serial.println();
        Serial.print("\nCO2 = "); 
        Serial.print(n);
        Serial.print( "ppm" );
        Serial.println();
        Serial.print("\nSMOKE = "); 
        Serial.print(s);
        Serial.print( "ppm" );
        Serial.println();
        Serial.print("\nPM2.5 count = ");
        Serial.print(concentration);
        Serial.print(" pcs/0.01cf");
        Serial.println();
        Serial.print("\nPM2.5 Concentration = ");
        Serial.print(con);
        Serial.print("ug\\m3");
        Serial.println();
        Serial.print("\nPM10 count = ");
        Serial.print(concentration2);
        Serial.print(" pcs/0.01cf");
        Serial.println();
        Serial.print("\nPM10 Concentration = ");
        Serial.print(con2);
        Serial.print("ug\\m3");
        Serial.println();
        
        Bserial.write("\nTemperature = ");
        Bserial.write(t);
        Bserial.println();
        Bserial.write("\nHumidity =  ");
        Bserial.write(h);
        Bserial.println();
        Bserial.print("\nLPG = "); 
        Bserial.print(l);
        Bserial.print( "ppm" );
        Bserial.println();
        Bserial.print("\nCO = "); 
        Bserial.print(c);
        Bserial.print( "ppm" );
        Bserial.println();
        Bserial.print("\nNO2 = "); 
        Bserial.print(c2);
        Bserial.print( "ppm" );
        Bserial.println();
        Bserial.print("\nCO2 = "); 
        Bserial.print(n);
        Bserial.print( "ppm" );
        Bserial.println();
        Bserial.print("\nSMOKE = "); 
        Bserial.print(s);
        Bserial.print( "ppm" );
        Bserial.println();
        Bserial.print("\nPM2.5 count = ");
        Bserial.print(ct1);
        Bserial.print(" pcs/0.01cf");
        Bserial.println();
        Bserial.print("\nPM2.5 Concentration = ");
        Bserial.print(co1);
        Bserial.print("ug\\m3");
        Bserial.println();
        Bserial.print("\nPM10 count = ");
        Bserial.print(ct2);
        Bserial.print(" pcs/0.01cf");
        Bserial.println();
        Bserial.print("\nPM10 Concentration = ");
        Bserial.print(co2);
        Bserial.print("ug\\m3");
        Bserial.println();
    }
  }
 }

void readMQ135(){
  valAIQ = MQ135GetGasPercentage(MQ135Read(MQ135_SENSOR),mq_ro,GAS_NO2,MQ135_SENSOR);
  valAIQ /= 10000;
  valAIQ2 = MQ135GetGasPercentage(MQ135Read(MQ135_SENSOR),mq_ro,GAS_CO2,MQ135_SENSOR);
}

float MQ135Calibration(int mq_pin, double ppm, float *pcurve )
{
  int i;
  float val=0;

  for (i=0;i<CALIBRATION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQ135ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBRATION_SAMPLE_TIMES;                   //calculate the average value
  //Ro = Rs * sqrt(a/ppm, b) = Rs * exp( ln(a/ppm) / b )

  return  (long)val*exp((log(pcurve[0]/ppm)/pcurve[1]));

}

float MQ135ResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE135*(4095-raw_adc)/raw_adc));
}


float MQ135Read(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQ135ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}

int  MQ135GetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio/ro), pcurve[1]));
}

int MQ135GetGasPercentage(float rs_ro_ratio, float ro, int gas_id, int sensor_id)
{
    if ( gas_id == GAS_CO2 ) {
     return MQ135GetPercentage(rs_ro_ratio,ro,CO2Curve);     //MQ135
    } else if ( gas_id == GAS_NO2 ) {
     return MQ135GetPercentage(rs_ro_ratio,ro,NO2Curve);     //MQ135
    } 
  return 0;
}

void readPPD42NS(){
   duration = pulseIn(pin, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;

  duration2 = pulseIn(pin2,LOW);
  lowpulseoccupancy2 = lowpulseoccupancy2+duration2;

  if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
    ratio2 = lowpulseoccupancy2/(sampletime_ms*10.0); 
    
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    concentration2 = 1.1*pow(ratio2,3)-3.8*pow(ratio2,2)+520*ratio2+0.62;
    //concentration -= concentration2;
    
    double pi = 3.14159;
    double density = 1.65*pow(10,12);
    double K = 3531.5;
    
    double r25 = 0.44*pow(10,-6);
    double vol25 = (4/3)*pi*pow(r25,3);
    double mass25 = density*vol25;
    con = (concentration)*K*mass25;

    
    double r10 = 0.44*pow(10,-6);
    double vol10 = (4/3)*pi*pow(r10,3);
    double mass10 = density*vol10;
    con2 = (concentration2)*K*mass10;
    
    lowpulseoccupancy = 0;
    lowpulseoccupancy2 = 0;
    
    starttime = millis();
  }
}
 
void readTemperatureAndHumidity() {
  DHT.read11(dht_dpin);
  humidity = (int) DHT.humidity;
  temperature = (int) DHT.temperature;
}
void readMQ2(){
  lpg = MQ2GetGasPercentage(MQ2Read(MQ_PIN)/Ro,GAS_LPG);
  co = MQ2GetGasPercentage(MQ2Read(MQ_PIN)/Ro,GAS_CO);
  smoke = MQ2GetGasPercentage(MQ2Read(MQ_PIN)/Ro,GAS_SMOKE);
}

void convert(){
  ts = (String)temperature;
  ts.toCharArray(t,10);
  
  hs = (String)humidity;
  hs.toCharArray(h,10);

  ls = (String)lpg;
  ls.toCharArray(l,10);

  cs = (String)co;
  cs.toCharArray(c,10);

  ss = (String)smoke;
  ss.toCharArray(s,10);

  c1s = (String)concentration;
  c1s.toCharArray(ct1,10);

  c2s = (String)concentration2;
  c2s.toCharArray(ct2,10);

  co1s = (String)con;
  co1s.toCharArray(co1,10);

  co2s = (String)con2;
  co2s.toCharArray(co2,10);

  c22s = (String)valAIQ;
  c22s.toCharArray(c2,10);

  ns = (String)valAIQ2;
  ns.toCharArray(n,10);


}


float MQ2ResistanceCalculation(int raw_adc){
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
float MQ2Calibration(int mq_pin){
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}

float MQ2Read(int mq_pin){
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

int MQ2GetGasPercentage(float rs_ro_ratio, int gas_id){
  if ( gas_id == GAS_LPG ) {
     return MQ2GetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQ2GetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQ2GetPercentage(rs_ro_ratio,SmokeCurve);
  }    
  return 0;
}

int  MQ2GetPercentage(float rs_ro_ratio, float *pcurve){
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
