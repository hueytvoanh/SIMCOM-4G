#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <String.h> 
#include <avr/wdt.h>

#define LED7_CMD_0           A2
#define LED7_CMD_1           A3
#define LED7_CMD_2           A4
#define LED7_CMD_3           A5
#define LED7_SCL             4
#define LED7_SDA             3

#define dataPin               4
#define clockPin              3

#define INPUT_MAIN_PULSE      2
#define INPUT_GEN             A7

#define INPUT_ACQ             A6                       //AIN0
#define INPUT_TMP             A7                       //AIN1
#define INPUTVOLTAGE          A0                       //AIN2
#define INPUT_CURRENT         A1                       //AIN3
#define INPUT_DOOR            5
#define INPUT_AC              2

#define OUTPUT_RELAY          9
#define OUTPUT_LOAD           8
#define OUTPUT_SIG            10
   
#define MENU_BUTTON           13 
#define INC_BUTTON            12
#define DEC_BUTTON            11

#define NO_SOURCE             0
#define MAIN_SOURCE           1
#define GEN_SOURCE            2
#define TEMP_SOURCE           3
#define NHOT_SOURCE           4
#define ACQ_SOURCE            5
#define CURRENT_SOURCE        6
#define MAIN_VOLTAGE          7
#define LED7_ACQ_H            8
#define LED7_ACQ_L            9
#define LED7_TEMP_H           10
#define LED7_TEMP_L           11
#define LED7_SOURCE_10        12
#define LED7_SOURCE_100       13
#define LED7_GSM_CODE_E1      14
#define LED7_GSM_CODE_E2      15
#define LED7_GSM_CODE_E3      16
#define LED7_GSM_CODE_E4      17
#define LED7_GSM_CODE_E5      18
#define LED7_GSM_CODE_E6      19
#define LED7_GSM_CODE_E7      20
#define LED7_GSM_CODE_E8      21
#define LED7_AC_OK            22
#define LED7_AC_NOK           23
#define LED7_GSM_SMS_ERROR    24       

#define AC_DISABLE            HIGH

#define LED7_END_SETUP        50
#define LED7_HZ               50
#define LED7_HZ_LONG          100
#define LED7_CONFIG_HL        50
#define LED7_CONFIG_BEGIN     70
#define LED7_CONFIG_HZ        15

#define RELAY_ACTIVE          HIGH
#define RELAY_DEACTIVE        LOW
#define LOAD_ACTIVE           HIGH
#define LOAD_DEACTIVE         LOW
#define SIG_ACTIVE            LOW
#define SIG_DEACTIVE          HIGH

#define SETUP_NONE            0
#define SETUP_ACQ_HIGH        1
#define SETUP_ACQ_LOW         2
#define SETUP_TEMP_HIGH       3
#define SETUP_TEMP_LOW        4

#define AC_NORMAL             0
#define AC_ON_RELAY           1
#define DOOR_OPEN             HIGH
#define DOOR_CLOSE            LOW

#define ACQ_ERRR_VALUE        1        
#define V_TH_G                120
#define V_TH_M                120
#define A_TH                  864
#define A_REF                 15.0

#define TEMP_HIGH_TH          45
#define TEMP_LOW_TH           40
#define VL_ACQ_TH             10
#define VH_ACQ_TH             14

#define SMSLENGTH             150
#define NAMEMAX               7
#define SMS_INTERVAL          30000

#define SCAN_NUMBER           5

#define SMS_NONE              0
#define SMS_AC_OUT            1
#define SMS_AC_IN             2
#define SMS_IN_NONE           0
#define SMS_IN_INFOR          1
#define FRE_COUNT             10

//#define DEBUG
#define DEBUG_GSM
#define GSM_SMS_ERROR
#define GSM_FUNCTION
#define SMS_WARNING
#define WARN_AC
#define WARN_ACQ
//#define WARN_DOOR
//#define WARN_TEMP
//#define MQTT_FUNCTION

//#define AUTHOR_TEST
//#define ACCOUNT_ADMIN
#define ACCOUNT_CM1
#define MQTT_STATE_ERROR


#ifdef ACCOUNT_ADMIN
const String myphone="0945818332";
#else
const String myphone="0919861955";
#endif

//String phoneSet ="";

char myphone2[11] = "";
char myphone3[11] = "";
char RxBuff[256]= "";
char atsName[10] = "ATS      ";
char msgChar[SMSLENGTH] = "";
unsigned long now, currentMillis, startCheckingTime;
float factorInM, factorInG, factorAcq;
boolean mainState, mainStateLast, acqState, acqStateLast, doorState, doorStateLast, tempState, tempStateLast, loadState;
boolean phone2Exist, phone3Exist;
char smsControl;
String atsTkStr = "";
boolean  checkingTk, inputTk;
float tempValue = 589.8;
float acqValue = 12.5;
float currentValue = 5.5;
char fMenu, fAuto, fTest, but3, scanMc;
float vThConfig;
float temp_L_Config, temp_H_Config, vAcq_L_Config, vAcq_H_Config;
boolean configVoltage;
char setUpState;

//////////////////////////////////////////////////////////////////////////////////////////////MQTT///////////////////////////////////////////////////////////////////////////////////////////////
#define TIME_UPLOAD_SECOND                  10
#define SYS_MQTT_CONNECT                    0
#define SYS_MQTT_CONNECT_AT                 1
#define SYS_MQTT_CONNECT_AT_CHECK           2
#define SYS_MQTT_CONNECT_CIPSHUT            3
#define SYS_MQTT_CONNECT_CIPSHUT_CHECK      4
#define SYS_MQTT_CONNECT_CIPMUX             5
#define SYS_MQTT_CONNECT_CIPMUX_CHECK       6
#define SYS_MQTT_CONNECT_CGATT              7
#define SYS_MQTT_CONNECT_CGATT_CHECK        8
#define SYS_MQTT_CONNECT_CSTT               9
#define SYS_MQTT_CONNECT_CSTT_CHECK         10
#define SYS_MQTT_CONNECT_CIICR              11
#define SYS_MQTT_CONNECT_CIICR_CHECK        12
#define SYS_MQTT_CONNECT_CIFSR              13
#define SYS_MQTT_CONNECT_CIFSR_CHECK        14

#define SYS_MQTT_UPLOAD                     20
#define SYS_MQTT_WAIT                       21
#define SYS_MQTT_FINISH                     22
#define SYS_MQTT_ERROR                      23
#define SYS_MQTT_HALT                       24
#define SYS_MQTT_CONNECT_CHECK              25
#define SYS_MQTT_UPLOAD_CHECK               26


#define SYS_SMS_INFOR_OUT                   34
#define SYS_SMS_INFOR_OUT_SENDING           35
#define SYS_SMS_PHONE2_ADDED                36
#define SYS_SMS_PHONE2_ADDED_SENDING        37
#define SYS_SMS_PHONE3_ADDED                38
#define SYS_SMS_PHONE3_ADDED_SENDING        39

#define SERIAL_CHECK_MS                     20000   // 20 ms
 

volatile int systemState;
int mqttUploadRetryNumber;
float i1, i2, i3, v1, v2, v3, p1, p2, p3;
volatile boolean mqttUploadStatus, mqttUploadChecking;

bool vinaNetwork, itelNetwork;

unsigned long currentTime, lastUploadTime, startMqttCheckingTime;

int setupCount;

static int networkError = 0;

byte connectPacket[ 19 ] =
{
  0x10, 0x11, 0x00, 0x04, 0x4D, 0x51, 0x54, 0x54, 0x04, 0x02, 0x00, 0x3C, 0x00, 0x05, 0x54, 0x32, 0x54, 0x49, 0x44
};
/*
byte connectPacketPass[ 28 ] =
{
  0x10, 0x1A, 0x00, 0x04, 0x4D, 0x51, 0x54, 0x54, 0x04, 0xC2, 0x00, 0x3C, 0x00, 0x08, 0x6E, 0x68, 0x61, 0x74, 0x6E, 0x68, 0x61, 0x74, 0x31, 0x35, 0x39, 0x31, 0x32, 0x33
};
*/
/*
  byte publishPacket[ 26 ] =
  {
  0x30, 0x18, 0x00, 0x0E, 0x54, 0x32, 0x54, 0x5F, 0x54, 0x6F, 0x70, 0x69, 0x63, 0x5F, 0x44, 0x65, 0x6D, 0x6F, 0x54, 0x65, 0x6D, 0x70, 0x3A, 0x20, 0x33, 0x30

  };
*/
/*
byte publishPacket[ 14 ] =
{
  0x30, 0x0C, 0x00, 0x07, 0x73, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x73, 0x4F, 0x46, 0x46

};

byte publishPacketSensor[ 15 ] =
{
  0x30, 0x0D, 0x00, 0x07, 0x73, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x73, 0x37, 0x38, 0x2E, 0x35

};
*/
/*
byte publishPacket_IU[ 46] =
{
0x30, 0x2C, 0x00, 0x07, 0x73, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x73, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x2C, 0x31, 0x31, 0x2E, 0x31, 0x31, 0x2C, 0x32, 0x32, 0x2E, 0x32, 0x32, 0x2C, 0x33, 0x33, 0x2E, 0x33, 0x33, 0x2C, 0x34, 0x34, 0x2E, 0x34, 0x34, 0x2C, 0x35, 0x35, 0x2E, 0x35, 0x35
};   ////// 0x27 + 2
*/
#ifdef ACCOUNT_ADMIN
byte publishPacket_IU[ 55] =
{
0x30, 0x35, 0x00, 0x07, 0x73, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x73, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x2C, 0x31, 0x31, 0x2E, 0x31, 0x31, 0x2C, 0x32, 0x32, 0x2E, 0x32, 0x32, 0x2C, 0x33, 0x33, 0x2E, 0x33, 0x33, 0x2C, 0x34, 0x34, 0x2E, 0x34, 0x34, 0x2C, 0x35, 0x35, 0x2E, 0x35, 0x35, 0x2C, 0x30, 0x31, 0x2C, 0x61, 0x64, 0x6D, 0x69, 0x6e
};
#endif

#ifdef ACCOUNT_CM1
byte publishPacket_IU[ 56] =
{
0x30, 0x36, 0x00, 0x07, 0x73, 0x65, 0x6E, 0x73, 0x6F, 0x72, 0x73, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x2C, 0x31, 0x31, 0x2E, 0x31, 0x31, 0x2C, 0x32, 0x32, 0x2E, 0x32, 0x32, 0x2C, 0x33, 0x33, 0x2E, 0x33, 0x33, 0x2C, 0x34, 0x34, 0x2E, 0x34, 0x34, 0x2C, 0x35, 0x35, 0x2E, 0x35, 0x35, 0x2C, 0x30, 0x32, 0x2C, 0x63, 0x6D, 0x31, 0x33, 0x5F, 0x61
};
#endif

///////////////////////////////////////////////////////////////////////////////////////////////MQTT///////////////////////////////////////////////////////////////////////////////////////////////

int pulsecount;
unsigned long nowAc, previousAc;

const char string_0[SMSLENGTH] PROGMEM = "START";
    const char string_1[SMSLENGTH] PROGMEM = "Canh bao. Mat dien luoi \n";
    const char string_2[SMSLENGTH] PROGMEM = "Vbat||Ddien||Tmp||AC \n" 
                                             "15.5||15.5||55.5||NOK||DOOR ----- \n";
    const char string_3[SMSLENGTH] PROGMEM = "ADDED PHONE2 xxxxxxxxxx \n";
    const char string_4[SMSLENGTH] PROGMEM = "ADDED PHONE3 xxxxxxxxxx \n"; 
    const char* const string_table[] PROGMEM = {string_0, string_1, string_2, string_3, string_4};

byte nump[] = {
 B11111100, // Zero
 B01100000, // One
 B11011010, // Two
 B11110010, // Three
 B01100110, // Four
 B10110110, // Five
 B10111110, // Six
 B11100000, // Seven
 B11111110, // Eight
 B11110110, // Nine
 B11111111, // None
 B00000010, // -
 B11000110, // TMP 12 
 B00011100, // L   
 B00111010, // o    14
 B10110111, // s
 B10110111, // s    16
 B01101111, // H    
 B00011101, // L    18      
 B11101110, // A   
 B01111100, // U    20   
 B10011111, // E    21    
 B10011100, // C    22                  
};

byte numD[] = {
 B11111101, // Zero
 B01100001, // One
 B11011011, // Two
 B11110011, // Three
 B01100111, // Four
 B10110111, // Five
 B10111111, // Six
 B11100001, // Seven
 B11111111, // Eight
 B11110111, // Nine
};


void ISR_HZ() {
    //digitalWrite(OUTPUT_RELAY, RELAY_ACTIVE);
    //digitalWrite(OUTPUT_LOAD, LOAD_ACTIVE);
    //digitalWrite(OUTPUT_SIG, SIG_ACTIVE);
    
    nowAc = millis();
    pulsecount++;            
    //detachInterrupt(digitalPinToInterrupt(INPUT_MAIN_PULSE)); 
    //digitalWrite(OUTPUT_RELAY, RELAY_DEACTIVE);
    //digitalWrite(OUTPUT_LOAD, LOAD_DEACTIVE);
    //digitalWrite(OUTPUT_SIG, SIG_DEACTIVE);
          
}


int calFre(){

    previousAc = millis();
    pulsecount = -1;
    int aa;
    aa = 0;
    attachInterrupt(digitalPinToInterrupt(INPUT_MAIN_PULSE), ISR_HZ, RISING);     
    while(1){
        //if (pulsecount < 20){ 
        if (pulsecount < FRE_COUNT){ 
            aa++;
            delay_ms(500);
        }            
        else{
            break;      
        }
         
        //if(aa == 20){
        if(aa == FRE_COUNT){
            break;
        }
    }     
    detachInterrupt(digitalPinToInterrupt(INPUT_MAIN_PULSE));   
    //if(pulsecount > 15){
    if(pulsecount > FRE_COUNT){
         mainState = true;
    }
    else{
         mainState = false;
    }

    //digitalWrite(OUTPUT_RELAY, RELAY_DEACTIVE);
    //digitalWrite(OUTPUT_LOAD, LOAD_DEACTIVE);
    //digitalWrite(OUTPUT_SIG, SIG_DEACTIVE);
               
    return 1;
}



void delay_ms(int x)   {
    for(int i=0; i<=x; i++){
        wdt_reset();
        delayMicroseconds(800);
    }
}
void delayMinute(int T){
    int p = 0;
    int longDelayInSeconds;
    longDelayInSeconds = T*60;
    while (p < longDelayInSeconds) {
        wdt_reset();
        delay_ms(1000);
        p++;  
    }
}                                     
// -------------------------------------------------------------------------------------
//Function: Gsm init and make sms
//--------------------------------------------------------------------------------------
void Gsm_Init()
{        
    startCheckingTime = 0;       
    Serial.begin(9600); 
    Serial.println(F("ATE0")); 
    delay_ms(500);
    Serial.println(F("AT+IPR=9600"));
    delay_ms(500);
    Serial.println(F("AT+CMGF=1"));
    delay_ms(500); 
    Serial.println(F("AT+CMGD=1,4"));
    delay_ms(500); 
    Serial.println(F("AT+CNMI=2,2"));
    delay_ms(500); 
    //Serial.println("AT+CBAND=\"ALL_BAND\"" );  
    Serial.println("AT+CBAND=\"EGSM_MODE\"" );  
    delay_ms(500);
    Serial.println("AT+CPMS=\"ME\"" );  
    delay_ms(500);
    Serial.println(F("AT&W"));  
    delay_ms(500);
    Serial.println("AT+CFUN=0" );  
    delay_ms(500);
    Serial.println("AT+CFUN=1" );
    delay_ms(500);
    Serial.setTimeout(50);
}

void makeSms(char* msgTk, String phoneNum){
    strcpy(RxBuff, ""); 
    Serial.println("AT+CMGS=\"" + phoneNum + "\"");    
    delay_ms(2000);
    Serial.flush();
    for(int i=0; i<SMSLENGTH; i++){
        Serial.print(msgTk[i]);
    }
    Serial.flush();
    strcpy(RxBuff, "");  
    Serial.print("\r\n");   
    Serial.println((char)26); 
    delay_ms(3500);
    //strcpy(RxBuff, "");  
}

void GsmMakeSmsChar(char* msgTk){
    makeSms(msgTk, myphone);
    if(phone2Exist == true) { 
      delay_ms(10000);   
      myphone2[10] = '\0';
      String phone2Send = "0123456789";
      phone2Send = String(myphone2);
      makeSms(msgTk, phone2Send);
    }
    if(phone3Exist == true) { 
      delay_ms(10000);   
      myphone3[10] = '\0';
      String phone3Send = "0123456789";
      phone3Send = String(myphone3);
      makeSms(msgTk, phone3Send);
    }
    startCheckingTime = millis();
}

//------------------------------------------------------------------------------------------------------------
// Function to read Serial Data
//------------------------------------------------------------------------------------------------------------
void serialEvent() {  
    static int pos = 0;
    while (Serial.available()!=0) {
        delay_ms(20);
        char inChar = (char)Serial.read(); 
        switch (inChar){
            case '\n': // Ignore new-lines
                break;
            case '\r': // Return on CR               
                pos = 0;  // Reset position index ready for next time                
                break;
            default:
               if (pos < 255) {
                   RxBuff[pos++] = inChar;
               }
               else{
                   pos = 0;
                   strcpy(RxBuff, "");
               }
               break;                           
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// Function to readBuff and check SMS
//------------------------------------------------------------------------------------------------------------
int checkBuff(){    
    char *outputInfor = NULL;
    outputInfor = strstr (RxBuff,"INFOR");
    if(outputInfor) {
        systemState = SYS_SMS_INFOR_OUT;
        strcpy(RxBuff, "");        
        startCheckingTime = 0;
        startMqttCheckingTime = 0;
        return 1;
    }

    outputInfor = strstr (RxBuff,"START");
    if(outputInfor) {
        smsControl = 2;
        strcpy(RxBuff, "");
        return 1;
    } 

    outputInfor = strstr (RxBuff,"OFF");
    if(outputInfor) {   
        smsControl = 3;
        strcpy(RxBuff, "");
        return 1;
    }   

    outputInfor = strstr (RxBuff,"RUN");
    if(outputInfor) {   
        smsControl = 4;
        strcpy(RxBuff, "");
        return 1;
    } 
    
    outputInfor = strstr (RxBuff,"MAIN_FORCE");
    if(outputInfor) {   
        smsControl = 6;
        strcpy(RxBuff, "");
        return 1;
    }   

    outputInfor = strstr (RxBuff,"CHECK_GEN");
    if(outputInfor) {   
        smsControl = 7;
        strcpy(RxBuff, "");
        return 1;
    }                 

    outputInfor = strstr (RxBuff,"CHECKTK");
    if(outputInfor) {   
        smsControl = 8;
        strcpy(RxBuff, "");
        return 1;
    }

    outputInfor = strstr (RxBuff,"NAP");
    if(outputInfor) {   
        String atsTk = String(outputInfor);
        atsTkStr = atsTk.substring(3, 17);        
        smsControl = 9;
        strcpy(RxBuff, "");       
        return 1;
    }    

    outputInfor = strstr (RxBuff,"TK");
    if(outputInfor){  
        strcpy(msgChar, outputInfor); 
        smsControl = 10;
        strcpy(RxBuff, "");
        return 1;
    }

    outputInfor = strstr (RxBuff,"NAME");
    if(outputInfor){  
        String atsSet = String(outputInfor);
        String atsNameStr = atsSet.substring(4, 4+NAMEMAX);
        for(int i = 0; i < NAMEMAX; i ++){
            atsName[i] = atsNameStr[i];
        }
        EEPROM.write(8, atsName[0]);
        EEPROM.write(10, atsName[1]);
        EEPROM.write(12, atsName[2]);
        EEPROM.write(14, atsName[3]);
        EEPROM.write(16, atsName[4]);
        EEPROM.write(18, atsName[5]);
        EEPROM.write(20, atsName[6]);
        smsControl = 11;
        strcpy(RxBuff, "");
        return 1;    
    }

    outputInfor = strstr (RxBuff,"PHONE22");
    if(outputInfor){   
            String phoneSet = String(outputInfor);    
            //phoneSet = String(outputInfor);    
            String myphone2Char = phoneSet.substring(7,17);
            myphone2Char.trim();             
            delay_ms(5000);
            for(int i = 0; i < 10; i ++){
                myphone2[i] = myphone2Char[i];
            }
            EEPROM.write(30, myphone2[0]);
            EEPROM.write(32, myphone2[1]);
            EEPROM.write(34, myphone2[2]);
            EEPROM.write(36, myphone2[3]);
            EEPROM.write(38, myphone2[4]);
            EEPROM.write(40, myphone2[5]);
            EEPROM.write(42, myphone2[6]);
            EEPROM.write(44, myphone2[7]);
            EEPROM.write(46, myphone2[8]);
            EEPROM.write(48, myphone2[9]);
            phone2Exist = true;
            systemState = SYS_SMS_PHONE2_ADDED;
            strcpy(RxBuff, "");
            return 1;    
    }  

    outputInfor = strstr (RxBuff,"PHONE33");
    if(outputInfor){   
            String phoneSet = String(outputInfor);    
            String myphone3Char = phoneSet.substring(7,17);
            myphone3Char.trim();
            for(int i = 0; i < 10; i ++){
                myphone3[i] = myphone3Char[i];
            }
            EEPROM.write(56, myphone3[0]);
            EEPROM.write(58, myphone3[1]);
            EEPROM.write(60, myphone3[2]);
            EEPROM.write(62, myphone3[3]);
            EEPROM.write(64, myphone3[4]);
            EEPROM.write(66, myphone3[5]);
            EEPROM.write(68, myphone3[6]);
            EEPROM.write(70, myphone3[7]);
            EEPROM.write(72, myphone3[8]);
            EEPROM.write(74, myphone3[9]);
            phone3Exist = true;
            systemState = SYS_SMS_PHONE3_ADDED;
            strcpy(RxBuff, "");
            return 1;    
    }

    outputInfor = strstr (RxBuff,"SETT1");
    if(outputInfor){  
        smsControl = 13;
        strcpy(RxBuff, "");
        return 1;   
    }

    outputInfor = strstr (RxBuff,"RING");
    if(outputInfor){  
        smsControl = 14;
        strcpy(RxBuff, "");
        return 1;    
    }



    currentMillis = millis(); 
                        
    if(startMqttCheckingTime != 0){  
        switch(systemState){
            case SYS_MQTT_CONNECT_AT_CHECK:                                                    //1111111111111111111111111111111111111111111111111111111111111111111111
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        systemState = SYS_MQTT_CONNECT_CIPSHUT; 
                        networkError = 0;  
                        startMqttCheckingTime = 0;
                    }
                    else{
                        return 1;      
                    }
                }
                else{                    
                    systemState = SYS_MQTT_CONNECT_AT;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E1);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    startMqttCheckingTime = 0;
                }
                break;

            case SYS_MQTT_CONNECT_CIPSHUT_CHECK:                                                //222222222222222222222222222222222222222222222222222222222222222222222
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        systemState = SYS_MQTT_CONNECT_CIPMUX; 
                        networkError = 0; 
                        startMqttCheckingTime = 0; 
                     }
                    else{
                        return 1;      
                    }
                }    
                else{
                    systemState = SYS_MQTT_CONNECT_CIPSHUT;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E2);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    startMqttCheckingTime = 0;
                }
                break;

           case SYS_MQTT_CONNECT_CIPMUX_CHECK:                                                        //3333333333333333333333333333333333333333333333333333333333
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        systemState = SYS_MQTT_CONNECT_CGATT; 
                        networkError = 0; 
                        startMqttCheckingTime = 0; 
                     }
                     else{
                         return 1; 
                     }
                } 
                else{
                    systemState = SYS_MQTT_CONNECT_CIPMUX;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E3);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    else{
                        return 1;      
                    }
                    startMqttCheckingTime = 0;
                }
                break;

            case SYS_MQTT_CONNECT_CGATT_CHECK:                                                         //44444444444444444444444444444444444444444444444444444
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        systemState = SYS_MQTT_CONNECT_CSTT; 
                        networkError = 0;  
                        startMqttCheckingTime = 0;
                     }
                    else{
                        return 1;      
                    }
                }    
                else{
                    systemState = SYS_MQTT_CONNECT_CGATT;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E4);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    startMqttCheckingTime = 0;
                }
                break;

            case SYS_MQTT_CONNECT_CSTT_CHECK:                                                           //5555555555555555555555555555555555555555555555555555555
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        systemState = SYS_MQTT_CONNECT_CIICR; 
                        networkError = 0;  
                        startMqttCheckingTime = 0;
                     }
                    else{
                        return 1;      
                    }
                }    
                else{
                    systemState = SYS_MQTT_CONNECT_CSTT;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E5);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    startMqttCheckingTime = 0;
                }
                break;


            case SYS_MQTT_CONNECT_CIICR_CHECK:                                                            //66666666666666666666666666666666666666666666666666666666666
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        systemState = SYS_MQTT_CONNECT_CIFSR; 
                        networkError = 0; 
                        startMqttCheckingTime = 0; 
                     }
                    else{
                        return 1;      
                    }
                }
                else{
                    systemState = SYS_MQTT_CONNECT_CIICR;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E6);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    startMqttCheckingTime = 0;
                }
                break;


            case SYS_MQTT_CONNECT_CIFSR_CHECK:                                                             //777777777777777777777777777777777777777777777777777777777777
                if ((currentMillis - startMqttCheckingTime) <= SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"10.");
                    if(outputInfor){   
                        systemState = SYS_MQTT_UPLOAD; 
                        networkError = 0; 
                        startMqttCheckingTime = 0; 
                     }
                     else{
                         return 1; 
                     }
                } 
                else{
                    systemState = SYS_MQTT_CONNECT_CIFSR;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E7);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    else{
                        return 1;      
                    }
                    startMqttCheckingTime = 0;
                }
                break;

             case SYS_MQTT_UPLOAD_CHECK:                                                             //888888888888888888888888888888888888888888888888888888888888888888
                if ((currentMillis - startMqttCheckingTime) < SERIAL_CHECK_MS){
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){   
                        lastUploadTime = millis();
                        systemState = SYS_MQTT_WAIT; 
                        networkError = 0;  
                        startMqttCheckingTime = 0;
                     }
                     else{
                         return 1; 
                     }
                }     
                else{
                    systemState = SYS_MQTT_CONNECT_AT;
                    networkError++;
                    if(networkError >= 3){
                        #ifdef MQTT_STATE_ERROR
                        for(int i = 0; i < 200; i++){ 
                           displayLed7(99.99, LED7_GSM_CODE_E8);
                        }
                        #endif
                        Gsm_Init();  
                        systemState = SYS_MQTT_CONNECT_AT;
                    }
                    else{
                        return 1;      
                    }
                    startMqttCheckingTime = 0;
                }
                break;
                
            default:
                break;       
        }
    }


    // CHECK SENDING SMS OK
    if(startCheckingTime != 0){
        switch(systemState){
            case SYS_SMS_INFOR_OUT_SENDING:                                                             //888888888888888888888888888888888888888888888888888888888888888888
                if ((currentMillis - startCheckingTime) < SMS_INTERVAL) {
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){    
                        systemState = SYS_MQTT_CONNECT_AT; 
                        networkError = 0; 
                        startCheckingTime = 0; 
                     }
                     else{
                        return 1; 
                     }
                }
                else{
                    systemState = SYS_SMS_INFOR_OUT;
                    networkError++;
                    if(networkError >= 2){
                        Gsm_Init();  
                        systemState = SYS_SMS_INFOR_OUT;
                    }
                    startCheckingTime = 0;
                }
                break;

            case SYS_SMS_PHONE2_ADDED_SENDING:                                                             //888888888888888888888888888888888888888888888888888888888888888888
                if ((currentMillis - startCheckingTime) < SMS_INTERVAL) {
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){    
                        systemState = SYS_MQTT_CONNECT_AT; 
                        networkError = 0; 
                        startCheckingTime = 0; 
                     }
                     else{
                        return 1; 
                     }
                }
                else{
                    //systemState = SYS_SMS_PHONE2_ADDED;
                    networkError++;
                    if(networkError >= 2){
                        Gsm_Init();  
                        //systemState = SYS_SMS_PHONE2_ADDED;
                    }
                    startCheckingTime = 0;
                }
                break;

            case SYS_SMS_PHONE3_ADDED_SENDING:                                                             //888888888888888888888888888888888888888888888888888888888888888888
                if ((currentMillis - startCheckingTime) < SMS_INTERVAL) {
                    outputInfor = strstr (RxBuff,"OK");
                    if(outputInfor){    
                        //systemState = SYS_MQTT_CONNECT_AT; 
                        networkError = 0; 
                        startCheckingTime = 0; 
                     }
                     else{
                        return 1; 
                     }
                }
                else{
                    //systemState = SYS_SMS_PHONE3_ADDED;
                    networkError++;
                    if(networkError >= 2){
                        Gsm_Init();  
                        //systemState = SYS_SMS_PHONE3_ADDED;
                    }
                    startCheckingTime = 0;
                }
                break;   

            default:
                break;
        }
    }
        
    return 1;
}

void checkSms(){     
    switch(smsControl){
        
        default:
            break;    
    } 
    smsControl = 0;          
}
void smsNapTk(){
    inputTk = true;
    String nap = "";
    nap = "AT+CUSD=1,\"*100*"+atsTkStr+"#\"";
    Serial.println(nap);
    Serial.flush();    
    strcpy(RxBuff, "");
}



void activeLed1(void){
  digitalWrite(LED7_CMD_0, HIGH);
  digitalWrite(LED7_CMD_1, HIGH);
  digitalWrite(LED7_CMD_2, LOW);
  digitalWrite(LED7_CMD_3, HIGH);
}

void activeLed2(void){
  digitalWrite(LED7_CMD_0, HIGH);
  digitalWrite(LED7_CMD_1, LOW);
  digitalWrite(LED7_CMD_2, HIGH);
  digitalWrite(LED7_CMD_3, HIGH);
}

void activeLed3(void){
  digitalWrite(LED7_CMD_0, LOW);
  digitalWrite(LED7_CMD_1, HIGH);
  digitalWrite(LED7_CMD_2, HIGH);
  digitalWrite(LED7_CMD_3, HIGH);
}

void activeLed4(void){
  digitalWrite(LED7_CMD_0, HIGH);
  digitalWrite(LED7_CMD_1, HIGH);
  digitalWrite(LED7_CMD_2, HIGH);
  digitalWrite(LED7_CMD_3, LOW);
}


void displayLed7(float dataIn, int type){
 int led_100 = 0;
 int led_10 = 0;
 int led_1 = 0;
 int led_dot = 0;

 //led_100 = dataIn/100;
 //led_10 = (dataIn - (led_100*100))/10;
 //led_1 = (dataIn - (led_100*100)) - (led_10*10);
 //led_dot = (int(dataIn *10.0))%10; 

 //dataIn = 14.5;

 
 switch(type){
  case ACQ_SOURCE:
       led_100 = dataIn/10;
       led_10 = (dataIn - (led_100*10));
       led_1 = (int(dataIn *10.0))%10;
       led_dot = 20;        
       break;
  case TEMP_SOURCE:
       led_100 = dataIn/10;
       led_10 = (dataIn - (led_100*10));
       led_1 = (int(dataIn *10.0))%10;
       led_dot = 12;        
       break;

  case CURRENT_SOURCE:
       if(currentValue >= 10.0){
           led_100 = dataIn/10;
           led_10 = (dataIn - (led_100*10));
           led_1 = (int(dataIn *10.0))%10;
           led_dot = 19;        
       }
       if(currentValue < 10.0){
           led_100 = 0;
           led_10 = (int(dataIn))%10;
           led_1 = (int(dataIn *10.0))%10;
           led_dot = 19;        
       }
       
       break;
  case MAIN_SOURCE:       
       if(mainState == false){
           led_100 = dataIn/10;
           led_10 = (dataIn - (led_100*10));
           led_1 = (int(dataIn *10.0))%10;
           led_dot = 12;        
       } 
       break; 
  case LED7_SOURCE_100:
       led_100 = dataIn/100;
       led_10 = (dataIn - (led_100*100))/10;
       led_1 = (dataIn - (led_100*100)) - (led_10*10);
       led_dot = (int(dataIn *10.0))%10; 
       break; 

  case LED7_ACQ_H:
       led_100 = 20;
       led_10 = 20;
       led_1 = 17;
       led_dot = 17; 
       break;

    case LED7_ACQ_L:
       led_100 = 20;
       led_10 = 20;
       led_1 = 18;
       led_dot = 18; 
       break;

    case LED7_TEMP_H:
       led_100 = 12;
       led_10 = 12;
       led_1 = 17;
       led_dot = 17; 
       break;

    case LED7_TEMP_L:
       led_100 = 12;
       led_10 = 12;
       led_1 = 18;
       led_dot = 18; 
       break;

    case LED7_END_SETUP:
       led_100 = 11;
       led_10 = 11;
       led_1 = 11;
       led_dot = 11; 
       break;
    case LED7_GSM_CODE_E1:
       led_100 = 21;
       led_10 = 1;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_GSM_CODE_E2:
       led_100 = 21;
       led_10 = 2;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_GSM_CODE_E3:
       led_100 = 21;
       led_10 = 3;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_GSM_CODE_E4:
       led_100 = 21;
       led_10 = 4;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_GSM_CODE_E5:
       led_100 = 21;
       led_10 = 5;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_GSM_CODE_E6:
       led_100 = 21;
       led_10 = 6;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_GSM_CODE_E7:
       led_100 = 21;
       led_10 = 7;
       led_1 = 10;
       led_dot = 10; 
       break;
    case LED7_AC_OK:
       led_100 = 19;
       led_10 = 22;
       led_1 = 19;
       led_dot = 22; 
       break;
    case LED7_AC_NOK:
       led_100 = 19;
       led_10 = 22;
       led_1 = 21;
       led_dot = 21; 
       break;
    case LED7_GSM_SMS_ERROR:
       led_100 = 15;
       led_10 = 15;
       led_1 = 15;
       led_dot = 15; 
       break;
       
  default: 
       break;
 }

 activeLed1();
 shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_100]);
 switch(type){
  case MAIN_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[13]);
       break;
  case LED7_SOURCE_100:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_100]);
       break;
  default:
      break;
 }
 
 delay(5);
 
 activeLed2();
 shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_10]);
 switch(type){
  case LED7_SOURCE_100:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_10]);
       break;
  case ACQ_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~numD[led_10]);
       break;
  case TEMP_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~numD[led_10]);
       break;
  case CURRENT_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~numD[led_10]);
       break;
  case MAIN_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[14]);
       break;
  default:
      break;
 }
 delay(5);
 
 activeLed3();
 shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_1]);
 switch(type){
  case LED7_SOURCE_100:
       shiftOut(dataPin, clockPin, LSBFIRST, ~numD[led_1]);
       break;
  case LED7_SOURCE_10:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_1]);
       break;
  case MAIN_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[15]);
       break;
  default:
      break;
 }
 delay(5);
 
 activeLed4();
 shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_dot]);
 switch(type){
  case LED7_SOURCE_100:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[led_dot]);
       break;
  case MAIN_SOURCE:
       shiftOut(dataPin, clockPin, LSBFIRST, ~nump[16]);
       break;
  default:
      break;
 }
 delay(5);
}


int readInput(char indexSource){
    int V_In;
    float volOut;

    switch(indexSource){
        case MAIN_SOURCE:
            
            if(digitalRead(INPUT_MAIN_PULSE)){
                mainState = true;          
                return 1;
            }
            else{     
                mainState = false;
                return 1;
            }
            break;
            /*
            V_In = analogRead(INPUT_MAIN_PULSE);
            Vout = V_In/1.0;
            
            if(volOut > 200){
                mainState = true;   
                return 1;
            }
            else{      
                mainState = false;   
                return 1;
            }
            break;
            */
        case GEN_SOURCE:
            V_In = analogRead(INPUT_GEN);
            volOut = V_In/factorInG;
            if(volOut > V_TH_G){       
                return V_In;
            }
            else{            
                return 0;
            }
            break;
        case TEMP_SOURCE:
            V_In = analogRead(INPUT_TMP);
            return V_In;
            break;
        case ACQ_SOURCE:
            V_In = analogRead(INPUT_ACQ);
            return V_In;
            break;
        case CURRENT_SOURCE:
            V_In = analogRead(INPUT_CURRENT);
            return V_In;
            break;
        default:
            break;        
    }      
}

void getSensorValue(void){  
  tempValue = readInput(TEMP_SOURCE)*(-0.078)+50.8;
  acqValue = (readInput(ACQ_SOURCE))*0.09 + 0.36;
  currentValue = (readInput(CURRENT_SOURCE))*0.1345 - 1.8;   
  calFre();

  if(digitalRead(INPUT_DOOR) == LOW){
      doorState = false;
      //for(int i = 0; i < 80; i++){
      //    displayLed7(11.11, ACQ_SOURCE);
      //}
  }
  else{
      doorState = true;
      //for(int i = 0; i < 80; i++){
      //    displayLed7(22.22, ACQ_SOURCE);
      //}
  }

  if(acqState == true){
      if((acqValue > vAcq_H_Config)||(acqValue < vAcq_L_Config)){
          acqState = false;  
      }
  }
  else{
      if((acqValue <= (vAcq_H_Config-ACQ_ERRR_VALUE))&&(acqValue >= (vAcq_L_Config+ACQ_ERRR_VALUE))){
          acqState = true;
      }
  }

  if(tempState == true){
      //if((tempValue > temp_H_Config)||(tempValue < temp_L_Config)){
      if(tempValue > TEMP_HIGH_TH){
          tempState = false;
      }
  }
  else{
      if(tempValue <= (TEMP_HIGH_TH - 2)){
          tempState = true;
      }
  }  

  #ifdef WARN_AC
  if((mainStateLast == true)&&(mainState == false)){
     mainStateLast = false;
     systemState = SYS_SMS_INFOR_OUT;      
  }
  if((mainStateLast == false)&&(mainState == true)){
     mainStateLast = true;
     systemState = SYS_SMS_INFOR_OUT;
  }
  #endif

  #ifdef WARN_DOOR
  if((doorStateLast == false)&&(doorState == true)){
     doorStateLast = true;
     systemState = SYS_SMS_INFOR_OUT;
  }
  if((doorStateLast == true)&&(doorState == false)){
     doorStateLast = false;
     systemState = SYS_SMS_INFOR_OUT;
  }
  #endif

  #ifdef WARN_ACQ
  if((acqStateLast == false)&&(acqState == true)){
     acqStateLast = true;
     systemState = SYS_SMS_INFOR_OUT;
  }
  if((acqStateLast == true)&&(acqState == false)){
     acqStateLast = false;
     systemState = SYS_SMS_INFOR_OUT;
  }
  #endif

  if(tempValue < 0){
      tempValue = 0;
  }
  
  #ifdef WARN_TEMP
  if((tempStateLast == false)&&(tempState == true)){
     tempStateLast = true;
     systemState = SYS_SMS_INFOR_OUT;
  }
  if((tempStateLast == true)&&(tempState == false)){
     tempStateLast = false;
     systemState = SYS_SMS_INFOR_OUT;
  }
  
  #endif
}

void fanControl(){

  if(tempState == true){
      digitalWrite(OUTPUT_LOAD, LOAD_ACTIVE); 
  }
  else{
      digitalWrite(OUTPUT_LOAD, LOAD_DEACTIVE);   
  }
}

void relayControl(){
  if(acqState == false){
      digitalWrite(OUTPUT_RELAY, RELAY_ACTIVE); 
  }
  else{
      digitalWrite(OUTPUT_RELAY, RELAY_DEACTIVE);       
  }
  
}

void outSigControl(){
  if(mainState == false){
    digitalWrite(OUTPUT_SIG, SIG_ACTIVE);
  }
  else{
    digitalWrite(OUTPUT_SIG, SIG_DEACTIVE);
  } 
   
}

int checkInputButtons(){   
    //char acq_H_value, acq_L_value, temp_H_value, temp_L_value;

    if(digitalRead(MENU_BUTTON)==LOW){
        for(int i = 0; i < LED7_CONFIG_BEGIN; i++){
             displayLed7(vAcq_L_Config, LED7_ACQ_L);  
        }
        setUpState = SETUP_ACQ_LOW;
        setupCount = 0;
    }

   while( setUpState != SETUP_NONE){
    
    switch(setUpState){
        case SETUP_NONE:
            if(digitalRead(MENU_BUTTON)==LOW){
                for(int i = 0; i < LED7_CONFIG_HL; i++){
                    displayLed7(vAcq_L_Config, LED7_ACQ_L);  
                }
                setUpState = SETUP_ACQ_LOW;
            }
            break;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
        case SETUP_ACQ_LOW:
            if(digitalRead(INC_BUTTON)==LOW){
                vAcq_L_Config++;   
            }
            if(digitalRead(DEC_BUTTON)==LOW){
                vAcq_L_Config--;   
            }
            if(digitalRead(MENU_BUTTON)==LOW){
                EEPROM.write(0, vAcq_L_Config);               
                setUpState = SETUP_ACQ_HIGH;
                for(int l = 0; l < LED7_CONFIG_HL; l++){
                    displayLed7(vAcq_L_Config, LED7_ACQ_H);
                }
            }
            
            for(int l = 0; l < LED7_CONFIG_HZ; l++){
                displayLed7(vAcq_L_Config, ACQ_SOURCE);
            }
            break;
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
        case SETUP_ACQ_HIGH:
            if(digitalRead(INC_BUTTON)==LOW){
                vAcq_H_Config++;   
            }
            if(digitalRead(DEC_BUTTON)==LOW){
                vAcq_H_Config--;   
            }
            if(digitalRead(MENU_BUTTON)==LOW){
                EEPROM.write(2, vAcq_H_Config);
                setUpState = SETUP_TEMP_LOW;                        
                for(int j = 0; j < LED7_CONFIG_HL; j++){
                    displayLed7(vAcq_H_Config, LED7_TEMP_L);
                }
            }

            for(int k = 0; k < LED7_CONFIG_HZ; k++){
                displayLed7(vAcq_H_Config, ACQ_SOURCE);
            }
            break;
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case SETUP_TEMP_LOW:
            if(digitalRead(INC_BUTTON)==LOW){
                temp_L_Config++;   
            }
            if(digitalRead(DEC_BUTTON)==LOW){
                temp_L_Config--;   
            }
            if(digitalRead(MENU_BUTTON)==LOW){
                EEPROM.write(4, temp_L_Config);                
                setUpState = SETUP_TEMP_HIGH;
                for(int l = 0; l < LED7_CONFIG_HL; l++){
                    displayLed7(temp_L_Config, LED7_TEMP_H);
                }
            }
            
            for(int l = 0; l < LED7_CONFIG_HZ; l++){
                displayLed7(temp_L_Config, TEMP_SOURCE);
            }
            break;
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        case SETUP_TEMP_HIGH:
            if(digitalRead(INC_BUTTON)==LOW){
                temp_H_Config++;   
            }
            if(digitalRead(DEC_BUTTON)==LOW){
                temp_H_Config--;   
            }
            if(digitalRead(MENU_BUTTON)==LOW){
                EEPROM.write(6, temp_H_Config);
                setUpState = SETUP_NONE;
                for(int l = 0; l < LED7_CONFIG_HL; l++){
                    displayLed7(temp_H_Config, LED7_END_SETUP);
                }
            }
            
            for(int l = 0; l < LED7_CONFIG_HZ; l++){
                displayLed7(temp_H_Config, TEMP_SOURCE);
            }
            break;
         
        default:
            break;
        
    }
    setupCount++;
    if(setupCount >= 400){
        setUpState = SETUP_NONE;
        break;  
    }
   }
}

int displayAcq(){
  //acqValue = 19.5;
  if(setUpState != SETUP_NONE){
    return 1; 
  }
  
  //acqValue = 29.8;
  for(int i = 0; i < LED7_HZ; i++){
  //for(int i = 0; i < 3000; i++){
     displayLed7(acqValue, ACQ_SOURCE);
  }
  
  return 1;
}

int displayTemp(){
  if(setUpState != SETUP_NONE){
    return 1; 
  }
  
  //tempValue = 27.8;
  //for(int i = 0; i < 3000; i++){
  for(int i = 0; i < LED7_HZ; i++){
     displayLed7(tempValue, TEMP_SOURCE);
  }

  return 1;
}

int displayCurent(){
  if(setUpState != SETUP_NONE){
    return 1; 
  }
  
  //currentValue = 13.4;
  for(int i = 0; i < LED7_HZ; i++){
  //for(int i = 0; i < 3000; i++){
     displayLed7(currentValue, CURRENT_SOURCE);
  }

  return 1;
}

int displayAc(){
  if(setUpState != SETUP_NONE){
    return 1; 
  }
  
  if(mainState == true ){
      for(int i = 0; i < LED7_HZ; i++){
          displayLed7(currentValue, LED7_AC_OK);
      }
  }
  else{
      for(int i = 0; i < LED7_HZ; i++){
          displayLed7(currentValue, LED7_AC_NOK);
      }
  }

  return 1;
}


void configIO(void){
  pinMode(LED7_CMD_0  , OUTPUT);
  pinMode(LED7_CMD_1  , OUTPUT);
  pinMode(LED7_CMD_2  , OUTPUT);
  pinMode(LED7_CMD_3  , OUTPUT);  
  pinMode(LED7_SCL  , OUTPUT);
  pinMode(LED7_SDA  , OUTPUT);

  pinMode(dataPin  , OUTPUT);
  pinMode(clockPin  , OUTPUT);

  pinMode(OUTPUT_LOAD  , OUTPUT);
  pinMode(OUTPUT_SIG  , OUTPUT);
  pinMode(OUTPUT_RELAY  , OUTPUT);

  pinMode(INPUT_ACQ  , INPUT);
  pinMode(INPUT_TMP  , INPUT);
  pinMode(INPUT_MAIN_PULSE  , INPUT);
  pinMode(INPUT_DOOR , INPUT_PULLUP);  

  pinMode(MENU_BUTTON , INPUT_PULLUP);
  pinMode(INC_BUTTON , INPUT_PULLUP);
  pinMode(DEC_BUTTON , INPUT_PULLUP);    
}

void makeCall(){  
  Serial.println("ATD" + myphone + ";");
  delay_ms(15000);
  Serial.println("ATH");
  Serial.println("ATD" + myphone + ";");
  delay_ms(15000);
  Serial.println("ATH");
}

void smsInfor(){        
    String AcquyStr = String(acqValue);
    String currentStr = String(currentValue);
    String tempStr = String(tempValue);
    
    strcpy_P(msgChar, (char*)pgm_read_word(&(string_table[2])));

    msgChar[22] = AcquyStr[0];
    msgChar[23] = AcquyStr[1];
    msgChar[24] = AcquyStr[2];
    msgChar[25] = AcquyStr[3];

    msgChar[28] = currentStr[0];
    msgChar[29] = currentStr[1];
    msgChar[30] = currentStr[2];
    msgChar[31] = currentStr[3];
    
    msgChar[34] = tempStr[0];
    msgChar[35] = tempStr[1];
    msgChar[36] = tempStr[2];
    msgChar[37] = tempStr[3];


    if(mainState == true){
        msgChar[40] = ' ';
        //msgChar[41] = 'O';
        //msgChar[42] = 'K';
    }
    else{
        msgChar[40] = 'N';
        //msgChar[41] = 'O';
        //msgChar[42] = 'K';
    }

    if(doorState == true){
        msgChar[50] = 'O';
        msgChar[51] = 'P';
        msgChar[52] = 'E';
        msgChar[53] = 'N';
    }
    else{
        msgChar[50] = 'C';
        msgChar[51] = 'L';
        msgChar[52] = 'O';
        msgChar[53] = 'S';
        msgChar[54] = 'E';
    }
    
}

void sendSmsTaskFunction(){
  switch(systemState){
     case SYS_SMS_INFOR_OUT:
         startCheckingTime = 0;
         startMqttCheckingTime = 0;
         strcpy_P(msgChar, (char*)pgm_read_word(&(string_table[2])));
         smsInfor();
         GsmMakeSmsChar(msgChar);
         systemState = SYS_SMS_INFOR_OUT_SENDING;
         break;
         
     case SYS_SMS_PHONE2_ADDED:
         startCheckingTime = 0;
         startMqttCheckingTime = 0;
         strcpy_P(msgChar, (char*)pgm_read_word(&(string_table[3]))); 
         
         msgChar[13] = myphone2[0]; 
         msgChar[14] = myphone2[1]; 
         msgChar[15] = myphone2[2]; 
         msgChar[16] = myphone2[3]; 
         msgChar[17] = myphone2[4]; 
         msgChar[18] = myphone2[5]; 
         msgChar[19] = myphone2[6]; 
         msgChar[20] = myphone2[7]; 
         msgChar[21] = myphone2[8]; 
         msgChar[22] = myphone2[9]; 
                
         GsmMakeSmsChar(msgChar);
         systemState = SYS_SMS_PHONE2_ADDED_SENDING;
         break;
     
     case SYS_SMS_PHONE3_ADDED:
         startCheckingTime = 0;
         startMqttCheckingTime = 0;
         strcpy_P(msgChar, (char*)pgm_read_word(&(string_table[4])));        

         msgChar[13] = myphone3[0]; 
         msgChar[14] = myphone3[1]; 
         msgChar[15] = myphone3[2]; 
         msgChar[16] = myphone3[3]; 
         msgChar[17] = myphone3[4]; 
         msgChar[18] = myphone3[5]; 
         msgChar[19] = myphone3[6]; 
         msgChar[20] = myphone3[7]; 
         msgChar[21] = myphone3[8]; 
         msgChar[22] = myphone3[9]; 
         
         
         GsmMakeSmsChar(msgChar);
         systemState = SYS_SMS_PHONE2_ADDED_SENDING;
         break;
    
    default:
        break;
  }
}

int readROMData(){  
    char vAcq_L_th, vAcq_H_th, tTemp_L_th, tTemp_H_th;
    /*
    char vAcq_L_th = EEPROM.read(0);
    char vAcq_H_th = EEPROM.read(2);  

    char tTemp_L_th = EEPROM.read(4);
    char tTemp_H_th = EEPROM.read(6);

    if((vAcq_L_th>100)||(vAcq_L_th<5)){
      vAcq_L_th = 40;
    }
    vAcq_L_Config = vAcq_L_th;
    //vAcq_L_Config = float(vAcq_L_th) * 1.0;  
      
    if((vAcq_H_th>100)||(vAcq_H_th<5)){
      vAcq_H_th = 45;
    }
    vAcq_H_Config = vAcq_H_th;
    //vAcq_H_Config = float(vAcq_H_th) * 1.0;
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if((tTemp_L_th>100)||(tTemp_L_th<5)){
      tTemp_L_th = 20;
    }
    temp_L_Config = float(tTemp_L_th) * 1.0;   
     
    if((tTemp_H_th>100)||(tTemp_H_th<5)){
      tTemp_H_th = 65;
    }
    temp_H_Config = float(tTemp_H_th) * 1.0;
    */
    vAcq_L_th = EEPROM.read(0);
    vAcq_H_th = EEPROM.read(2);  
    tTemp_L_th = EEPROM.read(4);
    tTemp_H_th = EEPROM.read(6);

    vAcq_L_Config = float(vAcq_L_th) * 1.0; 
    vAcq_H_Config = float(vAcq_H_th) * 1.0; 
    temp_L_Config = float(tTemp_L_th) * 1.0; 
    temp_H_Config = float(tTemp_H_th) * 1.0; 

    if(vAcq_L_Config < 5){
        vAcq_L_Config = 10;
    }
    
    if(vAcq_H_Config < 5){
        vAcq_H_Config = 15;
    }

    
    if(temp_L_Config < 5){
        temp_L_Config = 10;
    }
    
    if(temp_H_Config < 5){
        temp_H_Config = 15;
    }
    
    
    /*
    vAcq_L_Config = EEPROM.read(0);
    vAcq_H_Config = EEPROM.read(2);
    temp_L_Config = EEPROM.read(4);
    temp_H_Config = EEPROM.read(6);
    */

    myphone2[0] = char(EEPROM.read(30)) ;
    if(myphone2[0] == '0'){
        myphone2[1] = char(EEPROM.read(32)) ;
        myphone2[2] = char(EEPROM.read(34)) ;
        myphone2[3] = char(EEPROM.read(36)) ;
        myphone2[4] = char(EEPROM.read(38)) ;
        myphone2[5] = char(EEPROM.read(40)) ;
        myphone2[6] = char(EEPROM.read(42)) ;
        myphone2[7] = char(EEPROM.read(44)) ;
        myphone2[8] = char(EEPROM.read(46)) ;
        myphone2[9] = char(EEPROM.read(48)) ;
        phone2Exist = true;
    }
    else{
        strcpy(myphone2, "");  
        phone2Exist = false; 
    }

    myphone3[0] = char(EEPROM.read(56)) ;
    if(myphone3[0] == '0'){
        myphone3[1] = char(EEPROM.read(58)) ;
        myphone3[2] = char(EEPROM.read(60)) ;
        myphone3[3] = char(EEPROM.read(62)) ;
        myphone3[4] = char(EEPROM.read(64)) ;
        myphone3[5] = char(EEPROM.read(66)) ;
        myphone3[6] = char(EEPROM.read(68)) ;
        myphone3[7] = char(EEPROM.read(70)) ;
        myphone3[8] = char(EEPROM.read(72)) ;
        myphone3[9] = char(EEPROM.read(74)) ;
        phone3Exist = true;
    }
    else{
        strcpy(myphone3, "");  
        phone3Exist = false; 
    }

    
    return 1;
}

void updateSensorPacket(){
    /*
  i1 = 99.99;
  i2 = 88.88;  
  i3 = 77.77;

  v1 = 66.66;
  v2 = 12.34;  
  v3 = 56.78;
  */
  i1 = tempValue;
  v1 = acqValue;
  i2 = currentValue;
  if(mainState == true){
      v2 = 1.0;
  }
  else{
      v2 = 0; 
  }
  
  String i1_Str, i1_Str_1, i1_Str_2, i1_Str_3, i1_Str_4, i1_Str_5;
  String i2_Str, i2_Str_1, i2_Str_2, i2_Str_3, i2_Str_4, i2_Str_5;
  String i3_Str, i3_Str_1, i3_Str_2, i3_Str_3, i3_Str_4, i3_Str_5;

  String v1_Str, v1_Str_1, v1_Str_2, v1_Str_3, v1_Str_4, v1_Str_5;
  String v2_Str, v2_Str_1, v2_Str_2, v2_Str_3, v2_Str_4, v2_Str_5;
  String v3_Str, v3_Str_1, v3_Str_2, v3_Str_3, v3_Str_4, v3_Str_5;

 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  i1_Str = String(i1);
  if(i1 < 10.00){
      i1_Str_1 =  String(0);
      i1_Str_2 =  String(i1_Str[0]);
      i1_Str_3 =  String(i1_Str[1]);
      i1_Str_4 =  String(i1_Str[2]);
      i1_Str_5 =  String(i1_Str[3]);
  }
  if(i1 >= 10.00){
      i1_Str_1 =  String(i1_Str[0]);
      i1_Str_2 =  String(i1_Str[1]);
      i1_Str_3 =  String(i1_Str[2]);
      i1_Str_4 =  String(i1_Str[3]);
      i1_Str_5 =  String(i1_Str[4]);
  }
  byte buffer_i1_1[i1_Str_1.length() + 1];
  byte buffer_i1_2[i1_Str_2.length() + 1];
  byte buffer_i1_4[i1_Str_4.length() + 1];
  byte buffer_i1_5[i1_Str_5.length() + 1];

  i1_Str_1.getBytes(buffer_i1_1, i1_Str_1.length() + 1);
  i1_Str_2.getBytes(buffer_i1_2, i1_Str_2.length() + 1);
  i1_Str_4.getBytes(buffer_i1_4, i1_Str_4.length() + 1);
  i1_Str_5.getBytes(buffer_i1_5, i1_Str_5.length() + 1);

  publishPacket_IU[11] = buffer_i1_1[0];
  publishPacket_IU[12] = buffer_i1_2[0];
  publishPacket_IU[14] = buffer_i1_4[0];
  publishPacket_IU[15] = buffer_i1_5[0];
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  i2_Str = String(i2);
    if(i2 < 10.00){
      i2_Str_1 =  String(0);
      i2_Str_2 =  String(i2_Str[0]);
      i2_Str_3 =  String(i2_Str[1]);
      i2_Str_4 =  String(i2_Str[2]);
      i2_Str_5 =  String(i2_Str[3]);
  }
  if(i2 >= 10.00){
      i2_Str_1 =  String(i2_Str[0]);
      i2_Str_2 =  String(i2_Str[1]);
      i2_Str_3 =  String(i2_Str[2]);
      i2_Str_4 =  String(i2_Str[3]);
      i2_Str_5 =  String(i2_Str[4]);
  }
  byte buffer_i2_1[i2_Str_1.length() + 1];
  byte buffer_i2_2[i2_Str_2.length() + 1];
  byte buffer_i2_4[i2_Str_4.length() + 1];
  byte buffer_i2_5[i2_Str_5.length() + 1];

  i2_Str_1.getBytes(buffer_i2_1, i2_Str_1.length() + 1);
  i2_Str_2.getBytes(buffer_i2_2, i2_Str_2.length() + 1);
  i2_Str_4.getBytes(buffer_i2_4, i2_Str_4.length() + 1);
  i2_Str_5.getBytes(buffer_i2_5, i2_Str_5.length() + 1);

  publishPacket_IU[17] = buffer_i2_1[0];
  publishPacket_IU[18] = buffer_i2_2[0];
  publishPacket_IU[20] = buffer_i2_4[0];
  publishPacket_IU[21] = buffer_i2_5[0];
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    i3_Str = String(i3);
    if(i3 < 10.00){
      i3_Str_1 =  String(0);
      i3_Str_2 =  String(i3_Str[0]);
      i3_Str_3 =  String(i3_Str[1]);
      i3_Str_4 =  String(i3_Str[2]);
      i3_Str_5 =  String(i3_Str[3]);
  }
  if(i3 >= 10.00){
      i3_Str_1 =  String(i3_Str[0]);
      i3_Str_2 =  String(i3_Str[1]);
      i3_Str_3 =  String(i3_Str[2]);
      i3_Str_4 =  String(i3_Str[3]);
      i3_Str_5 =  String(i3_Str[4]);
  }
  byte buffer_i3_1[i3_Str_1.length() + 1];
  byte buffer_i3_2[i3_Str_2.length() + 1];
  byte buffer_i3_4[i3_Str_4.length() + 1];
  byte buffer_i3_5[i3_Str_5.length() + 1];

  i3_Str_1.getBytes(buffer_i3_1, i3_Str_1.length() + 1);
  i3_Str_2.getBytes(buffer_i3_2, i3_Str_2.length() + 1);
  i3_Str_4.getBytes(buffer_i3_4, i3_Str_4.length() + 1);
  i3_Str_5.getBytes(buffer_i3_5, i3_Str_5.length() + 1);

  publishPacket_IU[23] = buffer_i3_1[0];
  publishPacket_IU[24] = buffer_i3_2[0];
  publishPacket_IU[26] = buffer_i3_4[0];
  publishPacket_IU[27] = buffer_i3_5[0];
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    v1_Str = String(v1);
    if(v1 < 10.00){
      v1_Str_1 =  String(0);
      v1_Str_2 =  String(v1_Str[0]);
      v1_Str_3 =  String(v1_Str[1]);
      v1_Str_4 =  String(v1_Str[2]);
      v1_Str_5 =  String(v1_Str[3]);
  }
  if(v1 >= 10.00){
      v1_Str_1 =  String(v1_Str[0]);
      v1_Str_2 =  String(v1_Str[1]);
      v1_Str_3 =  String(v1_Str[2]);
      v1_Str_4 =  String(v1_Str[3]);
      v1_Str_5 =  String(v1_Str[4]);
  }
  byte buffer_v1_1[v1_Str_1.length() + 1];
  byte buffer_v1_2[v1_Str_2.length() + 1];
  byte buffer_v1_4[v1_Str_4.length() + 1];
  byte buffer_v1_5[v1_Str_5.length() + 1];

  v1_Str_1.getBytes(buffer_v1_1, v1_Str_1.length() + 1);
  v1_Str_2.getBytes(buffer_v1_2, v1_Str_2.length() + 1);
  v1_Str_4.getBytes(buffer_v1_4, v1_Str_4.length() + 1);
  v1_Str_5.getBytes(buffer_v1_5, v1_Str_5.length() + 1);

  publishPacket_IU[29] = buffer_v1_1[0];
  publishPacket_IU[30] = buffer_v1_2[0];
  publishPacket_IU[32] = buffer_v1_4[0];
  publishPacket_IU[33] = buffer_v1_5[0];

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   v2_Str = String(v2);
    if(v2 < 10.00){
      v2_Str_1 =  String(0);
      v2_Str_2 =  String(v2_Str[0]);
      v2_Str_3 =  String(v2_Str[1]);
      v2_Str_4 =  String(v2_Str[2]);
      v2_Str_5 =  String(v2_Str[3]);
  }
  if(v2 >= 10.00){
      v2_Str_1 =  String(v2_Str[0]);
      v2_Str_2 =  String(v2_Str[1]);
      v2_Str_4 =  String(v2_Str[3]);
      v2_Str_5 =  String(v2_Str[4]);
  }
  byte buffer_v2_1[v2_Str_1.length() + 1];
  byte buffer_v2_2[v2_Str_2.length() + 1];
  byte buffer_v2_4[v2_Str_4.length() + 1];
  byte buffer_v2_5[v2_Str_5.length() + 1];

  v2_Str_1.getBytes(buffer_v2_1, v2_Str_1.length() + 1);
  v2_Str_2.getBytes(buffer_v2_2, v2_Str_2.length() + 1);
  v2_Str_4.getBytes(buffer_v2_4, v2_Str_4.length() + 1);
  v2_Str_5.getBytes(buffer_v2_5, v2_Str_5.length() + 1);

  publishPacket_IU[35] = buffer_v2_1[0];
  publishPacket_IU[36] = buffer_v2_2[0];
  publishPacket_IU[38] = buffer_v2_4[0];
  publishPacket_IU[39] = buffer_v2_5[0];
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
    v3_Str = String(v3);
    if(v3 < 10.00){
      v3_Str_1 =  String(0);
      v3_Str_2 =  String(v3_Str[0]);
      v3_Str_3 =  String(v3_Str[1]);
      v3_Str_4 =  String(v3_Str[2]);
      v3_Str_5 =  String(v3_Str[3]);
  }
  if(v3 >= 10.00){
      v3_Str_1 =  String(v3_Str[0]);
      v3_Str_2 =  String(v3_Str[1]);
      v3_Str_3 =  String(v3_Str[2]);
      v3_Str_4 =  String(v3_Str[3]);
      v3_Str_5 =  String(v3_Str[4]);
  }
  byte buffer_v3_1[v3_Str_1.length() + 1];
  byte buffer_v3_2[v3_Str_2.length() + 1];
  byte buffer_v3_4[v3_Str_4.length() + 1];
  byte buffer_v3_5[v3_Str_5.length() + 1];

  v3_Str_1.getBytes(buffer_v3_1, v3_Str_1.length() + 1);
  v3_Str_2.getBytes(buffer_v3_2, v3_Str_2.length() + 1);
  v3_Str_4.getBytes(buffer_v3_4, v3_Str_4.length() + 1);
  v3_Str_5.getBytes(buffer_v3_5, v3_Str_5.length() + 1);

  publishPacket_IU[41] = buffer_v3_1[0];
  publishPacket_IU[42] = buffer_v3_2[0];
  publishPacket_IU[44] = buffer_v3_4[0];
  publishPacket_IU[45] = buffer_v3_5[0];
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
}


int mqttUploadTaskFunction( ) {
    delay(50);
    switch (systemState) {

        case SYS_MQTT_CONNECT_AT:
            #ifdef MQTT_STATE_CONNECT
            for(int i = 0; i < 200; i++){ 
                displayLed7(00.11, ACQ_SOURCE);
            }
            #endif
            
            systemState = SYS_MQTT_CONNECT_AT_CHECK;
            strcpy(RxBuff, "");
            Serial.println("AT" );
            delay(500);
            startMqttCheckingTime = millis(); 
            break;

        
        case SYS_MQTT_CONNECT_CIPSHUT:
             #ifdef MQTT_STATE_CONNECT
             for(int i = 0; i < 200; i++){ 
                displayLed7(11.1, ACQ_SOURCE);
             }
             #endif
             
             strcpy(RxBuff, "");
             Serial.println("AT+CIPSHUT" );
             delay(500);
             startMqttCheckingTime = millis();              
             systemState = SYS_MQTT_CONNECT_CIPSHUT_CHECK;
             break;

        case SYS_MQTT_CONNECT_CIPMUX:
             #ifdef MQTT_STATE_CONNECT
             for(int i = 0; i < 200; i++){ 
                displayLed7(22.2, ACQ_SOURCE);
             }
             #endif
             
             strcpy(RxBuff, "");
             Serial.println("AT+CIPMUX=0" );
             delay(500);
             startMqttCheckingTime = millis(); 
             systemState = SYS_MQTT_CONNECT_CIPMUX_CHECK;
             break;

        case SYS_MQTT_CONNECT_CGATT:                                                //LOOP LOOP
             #ifdef MQTT_STATE_CONNECT
             for(int i = 0; i < 200; i++){ 
                displayLed7(33.3, ACQ_SOURCE);
             }
             #endif
             
             Serial.println("AT" );
             delay(1000);
             strcpy(RxBuff, "");
             Serial.println("AT+CGATT=1" );
             delay(500);
             startMqttCheckingTime = millis(); 
             systemState = SYS_MQTT_CONNECT_CGATT_CHECK;
             break;
                  
        case SYS_MQTT_CONNECT_CSTT:
             #ifdef MQTT_STATE_CONNECT
             for(int i = 0; i < 200; i++){ 
                displayLed7(44.4, ACQ_SOURCE);
             }
             #endif
             
             strcpy(RxBuff, "");
             
             //Serial.println("AT+CSTT=\"e-internet\"");
             if(vinaNetwork == true){
                Serial.println("AT+CSTT=\"m3-world\",\"mms\",\"mms\"" );
             }
             else{
                Serial.println("AT+CSTT=\"e-internet\"" );
             }
             
             delay(500);
             startMqttCheckingTime = millis(); 
             systemState = SYS_MQTT_CONNECT_CSTT_CHECK;
             break;

        case SYS_MQTT_CONNECT_CIICR:
             #ifdef MQTT_STATE_CONNECT
             for(int i = 0; i < 200; i++){ 
                displayLed7(55.5, ACQ_SOURCE);
             }
             #endif
             
             strcpy(RxBuff, "");
             Serial.println("AT+CIICR" );
             delay(1000);
             startMqttCheckingTime = millis(); 
             systemState = SYS_MQTT_CONNECT_CIICR_CHECK;
             break;
             
        case SYS_MQTT_CONNECT_CIFSR:
             #ifdef MQTT_STATE_CONNECT
             for(int i = 0; i < 200; i++){ 
                displayLed7(66.66, ACQ_SOURCE);
             }
             #endif
             
             strcpy(RxBuff, "");
             Serial.println("AT+CIFSR");
             delay(500);
             startMqttCheckingTime = millis(); 
             systemState = SYS_MQTT_CONNECT_CIFSR_CHECK;
             break;
                  
        case SYS_MQTT_UPLOAD:
            #ifdef MQTT_STATE_CONNECT
            for(int i = 0; i < 200; i++){
                displayLed7(77.7, ACQ_SOURCE);
            }
            #endif
            
            updateSensorPacket();
            
            mqttUploadRetryNumber++;
            mqttUploadChecking = true;
            mqttUploadStatus = true;
        
            //Serial.println("GO TO MQTT UPLOADING");
            delay(2000);            
            //Serial.println("AT+CIPSTART=\"TCP\", \"sunnyiot.duckdns.org\", \"1883\"" );
            Serial.println("AT+CIPSTART=\"TCP\", \"sunnyiot.org\", \"1883\"" );
            delay(15000);    
            //Serial.println("BEGIN SENDING"); 
            Serial.println("AT+CIPSEND" );
            delay(3000);
            Serial.write( connectPacket, sizeof( connectPacket ) );
            //Serial.write( connectPacketPass, sizeof( connectPacketPass) );
            Serial.write(0x1A);
            delay(500);

            //strcpy(RxBuff, "");
            //PUBLISH TO MQTT
            Serial.println("AT+CIPSEND" );
            delay(3000);
            //sim800.write( publishPacketSensor, sizeof( publishPacketSensor ) );
            strcpy(RxBuff, "");
            Serial.write( publishPacket_IU, sizeof( publishPacket_IU ) );
            Serial.write(0x1A); 
            delay(500);

            systemState = SYS_MQTT_UPLOAD_CHECK;
            startMqttCheckingTime = millis();
            
            break;

      case SYS_MQTT_WAIT: 
          #ifdef DEBUG
          digitalWrite(OUTPUT_RELAY, RELAY_ACTIVE);
          digitalWrite(OUTPUT_LOAD, LOAD_ACTIVE);
          digitalWrite(OUTPUT_SIG, SIG_ACTIVE);
          #endif
          currentTime = millis(); 
          if((currentTime - lastUploadTime) >= 600000) {
              #ifdef DEBUG 
              digitalWrite(OUTPUT_RELAY, RELAY_DEACTIVE);
              digitalWrite(OUTPUT_LOAD, LOAD_DEACTIVE);
              digitalWrite(OUTPUT_SIG, SIG_DEACTIVE);
              #endif
              //systemState = MQTT_CONNECT_AT;
              systemState = SYS_MQTT_UPLOAD;
          }
          break;

      case SYS_MQTT_ERROR:
          systemState = SYS_MQTT_HALT;
          break;
          
      case SYS_MQTT_HALT:
          //Serial.println("MQTT HALTTTTTTTTTTTTTTTTTT. GO TO RESTART ESP");
          delay(20000);
          //ESP.restart();
          break;    

      default:
        break;

    }
    return 1;
 }

void setup() {
  factorAcq = 4.0;
  factorInM = 1.4;
  loadState = false;
  vThConfig = 220.0;
  configVoltage = false;
  tempState = true; 
  tempStateLast = true;
  acqState = true;
  acqStateLast = true; 
  doorState = true;
  doorStateLast = true;  
  mainState = true;
  mainStateLast = true;
  pulsecount = 0;
  setupCount = 0;

  #ifdef ACCOUNT_ADMIN
  vinaNetwork = false;
  #else
  vinaNetwork = true;
  #endif
  
  phone2Exist = false, phone3Exist = false;    
  setUpState = SETUP_NONE;
  systemState = SYS_MQTT_CONNECT_AT;
  
  configIO();
  readROMData();

  #ifdef GSM_FUNCTION
  delay_ms(20000);
  Gsm_Init();
  #endif
  
  #ifdef GSM_TEST
  makeCall();
  delay_ms(15000);
  strcpy_P(msgChar, (char*)pgm_read_word(&(string_table[0])));
  GsmMakeSmsChar(msgChar); 
  #endif
}

void loop() {
  checkInputButtons();
  getSensorValue();
  displayAc();
  displayCurent();
  relayControl();
  fanControl();
  displayAcq();
  outSigControl();
  displayTemp();
  #ifdef GSM_FUNCTION
  sendSmsTaskFunction();   
  #endif
  #ifdef MQTT_FUNCTION
  mqttUploadTaskFunction();
  #endif
  #ifdef GSM_FUNCTION
  serialEvent();
  checkBuff();
  #endif
}
