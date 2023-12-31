/*
|__________________________[Menu do programa]____________________________|
|       Nome      | Linha | tipo |              Propósito                |
|_________________|_______|______|_______________________________________|
|      wi-fi      |  41   |      | Declarar a rede wifi                  |
|    ThingSpeak   |  45   |      | Informações do canal/campos           |
|      MQ-135     |  61   |      | Declarar as definições do MQ-135      |
|       MPU       |  71   |      | Declarar as definições do MPU-6050    |
|      tempo      |  74   |      | Definir marcas temporais              |
|       DHT       |  78   |      | Definir pinos e tipo do DHT           | 
|       KY        |  83   |      | Definir pinos e tipo do KY-037        | 
|     LED-RGB     |  90   |      | Definir pinos do LED-RGB              |  
|     microSD     |  96   |      | Definir o pino CS                     |
|     mudarCor    |  105  | void | recebe as cores do LED-RGB            |
|       NTP       |  115  |      | Declarar as definições do NTP         |
|   conectarMQTT  |  120  | void | estabelecer conecção com broker       |
|     callback    |  142  | void | receber mensagens do broker           |
|  publishMessage |  155  | void | publicar mensagens em um canal        |
|     escrever    |  160  | bool | Anexar um valor no arquivo do cartão  |
|      setup      |  181  | void | inicializar diversos aspectos         |
|      loop       |  324  | void | rodar o programa continuamente        |
|________________________________________________________________________|

Boas Festas :D
*/

//Inclusões
#include "DHT.h"
#include <WiFi.h>
#include "ThingSpeak.h"
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <SD.h>
#include <SPI.h>
#include "time.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <MQUnifiedsensor.h>
#include <Adafruit_MPU6050.h>

//Definições da rede Wi-fi
const char* SSID = "Intengele_IOT";  
const char* Senha = "1Nt3n83!3";    

WiFiClient espClient;

//Definições do ThingSpeak/Broker
PubSubClient client(espClient);
unsigned long canal1 = 1; //DHT22 - Temperatura
unsigned long canal2 = 1; //DHT22 - Umidade
unsigned long canal3 = 1; //KY-037 - Decibéis
const char* apiKey = "POIKW298GF0E0Y8F";
const char* mqtt_server = "mqtt3.thingspeak.com";
const char* publishTopic ="channels/2259214/publish"; 
const char* subscribeTopicFor_Command_1="channels/2259214/subscribe/fields/field1";   //Temperatura 
const char* subscribeTopicFor_Command_2="channels/2259214/subscribe/fields/field2";   //Umidade
const char* subscribeTopicFor_Command_3="channels/2259214/subscribe/fields/field3";   //Decibéis
const char* subscribeTopicFor_Command_4="channels/2259214/subscribe/fields/field4";   //CO2
const char* subscribeTopicFor_Command_5="channels/2259214/subscribe/fields/field5";   //Vibração

//Definições do MQ-135
#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 34 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
double CO2 = (0);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

//Definições do MPU6050
Adafruit_MPU6050 mpu;

//Configurações do tempo de envio
const unsigned long intervaloDeEnvio = 20L * 1000L; //Intervalo de Envio = 20s
unsigned long ultimoEnvio = 0;

// Definições do DHT
#define DHTPIN 4    
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//Definições do KY-037
#define sensorDeSom 33 //GPIO26 é dedicada a rede, não usar!
#define JANELA 200
float potMedia;
float potMediaAnt;

//Definições do LED-RGB
#define pinoVermelho  13 
#define pinoVerde     12 
#define pinoAzul      14 

//Definições do Adaptador MicroSD
#define cs 5
File leituras;
File Temperatura;
File Umidade;
File Decibeis;
File aCO2;
File Vib;

//Mudar cor do LED-RGB
void mudarCor(int vM, int vR, int aZ){
  
  analogWrite(pinoVermelho, vM);
  analogWrite(pinoVerde, vR);
  analogWrite(pinoAzul, aZ);

}

//Configurações para aquisição do tempo de um NTP
const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;      // Fuso horário
const int   daylightOffset_sec = 0;  // Horário de verão

//Função para conectar o MQTT
void conectarMQTT() {

  while (!client.connected()) {
    Serial.print("Tentando Conectar com o dispositivo MQTT...");
    //Credenciais do Dispositivo MQTT
    if (client.connect("EzgxMjsoNhw1JDcLAg4QESk", "EzgxMjsoNhw1JDcLAg4QESk","yH+X7iPdlU+g36SeG/i02vap")) {  
      Serial.println("MQTT Conectado!");
      client.subscribe(subscribeTopicFor_Command_1);  //DHT22 - Temperatura
      client.subscribe(subscribeTopicFor_Command_2);  //DHT22 - Umidade
      client.subscribe(subscribeTopicFor_Command_3);  //KY-037 - Decibéis
      client.subscribe(subscribeTopicFor_Command_4);  //MQ-135 - CO2
      client.subscribe(subscribeTopicFor_Command_5);  //MPU-6050 - Vibração
      
    } else {
      Serial.print("Dispositivo MQTT não foi conectado, erro:");
      Serial.print(client.state());
      Serial.println(".");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];
  
  Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);
  
  //--- check the incomming message
    if( strcmp(topic,subscribeTopicFor_Command_1) == 0){
     if (incommingMessage.equals("1")) digitalWrite(BUILTIN_LED, LOW);  
     else digitalWrite(BUILTIN_LED, HIGH);  
  }
}

void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str()))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}

bool escrever(const char* arquivo, float valor, const char* tipoDeValor) {
    File file = SD.open(arquivo, FILE_APPEND);

    if (!file) {
        return false;
    }
    
    struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Falha ao obter o tempo atual");
  }

    file.print(&timeinfo, " %d/%B/%Y - %H:%M:%S ");
    file.print(tipoDeValor);
    file.print(": ");
    file.println(valor);
    file.close();

    return true;
}

void setup() {  
  Serial.begin(9600);
  while (!Serial) { }
  Serial.println(F("Sensor de Bem-Estar"));

  //Inicialização da Rede Wi-fi
  WiFi.mode(WIFI_STA);

  //inicialização do tempo
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //Inicialização do DHT22
  dht.begin();
  Serial.print("Sensor de Temperatura:");
  Serial.print(DHTTYPE);
  Serial.println("ativo");
  
  //Inicialização do MPU-6050
  if (!mpu.begin()) {
    Serial.println("Falha em ligar o MPU-6050");
    while (1) {
      delay(10);
    }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  //Inicialização do KY-037
  pinMode(sensorDeSom, INPUT); 
  potMedia = 0;

  Serial.print("Sensor de Decibeis:");
  Serial.print("KY-037");
  Serial.println("ativo"); 

  //Inicialização do MQ-135 
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
 
  MQ135.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}

  /*****************************  MQ CAlibration ********************************************/ 
  MQ135.serialDebug(false);
  
  //Inicialização das funções MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //Inicialização do LED
  pinMode(pinoVermelho, OUTPUT);
  pinMode(pinoVerde, OUTPUT);
  pinMode(pinoAzul, OUTPUT);

  //Inicialização do Adaptador MicroSD
  while (!SD.begin(cs)) {
    Serial.println(F("Cartão com defeito, mal ou não inserido"));
    delay(5000);
  }
  
  Serial.println(F("Cartão lido com sucesso!"));

 File leituras = SD.open("/leituras.txt", FILE_WRITE);
  if (leituras) {
    leituras.println("Iniciando Leituras");
    leituras.close();
    Serial.println("Comunicação com o cartão SD feita com sucesso");
  } else {
    Serial.println("Não foi possível escrever no arquivo");
  }
}


void loop() {
  
  //Conexão com a rede wi-fi
  if (WiFi.status() != WL_CONNECTED){
    mudarCor(0,255,255);
    Serial.print("Tentando conexão com a rede Wi-fi...");
    
    while(WiFi.status() != WL_CONNECTED){ 
      WiFi.begin(SSID, Senha);
      Serial.print(".");  
      delay(5000);  
    }
    Serial.println("");
    Serial.println("Rede Conectada!");
    Serial.print("Rede wi-fi: ");
    Serial.println(WiFi.SSID());
  }

  //Medição da temperatura e umidade
  //Tempo de medição
  mudarCor(255,0,255);
  delay(4000);
  mudarCor(255,153,255);
  delay(4000);
  mudarCor(255,0,255);
  delay(4000);
  mudarCor(255,153,255);
  delay(4000);
  mudarCor(255,0,255);
  delay(4000);
  
  //Medição do CO2
  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
 
  MQ135.setA(110.47); MQ135.setB(-2.862);
  CO2 = MQ135.readSensor();

  MQ135.setA(605.18); MQ135.setB(-3.937);
  float CO = MQ135.readSensor();

  MQ135.setA(77.255); MQ135.setB(-3.18);
  float Alcohol = MQ135.readSensor();

  MQ135.setA(44.947); MQ135.setB(-3.445);
  float Toluene = MQ135.readSensor();

  MQ135.setA(102.2 ); MQ135.setB(-2.473);
  float NH4 = MQ135.readSensor();

  MQ135.setA(34.668); MQ135.setB(-3.369);
  float Acetone = MQ135.readSensor();
  
  CO2 = CO2*12;

  Serial.print("CO2: ");
  Serial.println(CO2);

  //Medição da Vibraçao
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float vibr[JANELA];

  float acT = sqrt((a.acceleration.x)*(a.acceleration.x) + (a.acceleration.y)*(a.acceleration.y) + (a.acceleration.z)*(a.acceleration.z));
  float angT = sqrt((g.gyro.x)*(g.gyro.x) + (g.gyro.y)*(g.gyro.y) + (g.gyro.z)*(g.gyro.z));
  float vibT = 5*acT*angT;

  for(int i = 0; i < JANELA; i++){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  vibr[i] = vibT;
  }
  
  vibT = 0;

  for(int i = 0; i < JANELA; i++){
  vibT = vibT + vibr[i]; 
  }

  vibT = vibT/JANELA;

  Serial.print("Vibração = ");
  Serial.println(vibT);

  Serial.println("");

  //Medição do som
  int i;
  short som[JANELA];
  float media;
  short maximo;
  short minimo;

  for(i = 0; i < JANELA; i++){ 
    digitalWrite(13, HIGH); 
    som[i] = analogRead(sensorDeSom);
    digitalWrite(13, LOW); 
    delayMicroseconds(50);
  }
 
  media = 0;
  maximo = -32000;
  minimo = 32000;

  for(i = 0; i < JANELA; i++){
    media = media + som[i];

    if(som[i] > maximo) maximo = som[i];
    if(som[i] < minimo) minimo = som[i];
  }

  media = media/(float)JANELA;

  for(i = 0; i < JANELA; i++) som[i] = 10 * (som[i] - media);

   float potencia = 0;
  for(i = 0; i < JANELA; i++){
    if(som[i] < 0){  
      potencia += -som[i];
    }else{ 
      potencia += som[i];
    }
  }

  potMediaAnt = potMedia;
  potMedia = (potencia + 9 * potMedia)/10;
  

  if(potencia > 10 * potMediaAnt){
    Serial.println("----------------------------------------------------------------Evento!----------------------------------------------------------");
  }

  Serial.print("Media:");
  Serial.println(media);
  Serial.print("Maximo:");
  Serial.println(maximo);
  Serial.print("Minimo:");
  Serial.println(minimo);
  Serial.print("Potencia:");
  Serial.println(potencia);
  Serial.print("Potencia Media:");
  Serial.println(potMedia);
  Serial.print("Evento:");
  Serial.println(10*potMedia);
  Serial.println("");

  //Medição de temperatura e umidade
  float u = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(u) || isnan(t) ) {
    Serial.println(F("Falha na leitura do Sensor DHT22"));
    return;
  }
  mudarCor(0,255,0);
  Serial.print(F("Umidade: "));
  Serial.print(u);
  Serial.print("%");
  Serial.print(" | ");
  Serial.print(F("Temperatura: "));
  Serial.print(t);
  Serial.print(F("°C"));
  Serial.print(" | ");
  Serial.print(F("Intensidade Sonora Relativa: "));
  Serial.print(potencia);
  Serial.println("");

  mudarCor(0,0,255);
  if (!client.connected()) conectarMQTT();
  client.loop();

  if (millis() - ultimoEnvio > intervaloDeEnvio) { // The uploading interval must be > 15 seconds 
    
    String dataText = String("field1=" + String(t)+ "&field2=" + String(u) + "&field3=" + String(potencia) + "&field4=" + String(CO2) + "&field5=" + String(vibT) + "&status=MQTTPUBLISH");
    publishMessage(publishTopic,dataText,true);    
    ultimoEnvio = millis();
  }
  
  escrever("/Temperatura.txt", t, "Temperatura");
  escrever("/Umidade.txt", u, "Umidade");
  escrever("/Decibeis.txt", potencia, "Intensidade Sonora");
  escrever("/aCO2.txt", CO2, "CO2");
  escrever("/Vib.txt", vibT, "VCI");

 }
