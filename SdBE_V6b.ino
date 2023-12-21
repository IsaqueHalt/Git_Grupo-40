/*
|__________________________[Menu do programa]____________________________|
|       Nome      | Linha | tipo |              Propósito                |
|_________________|_______|______|_______________________________________|
|      wi-fi      |  35   |      | Declarar a rede wifi                  |
|    ThingSpeak   |  40   |      | Informações do canal/campos           |
|      tempo      |  50   |      | Definir marcas temporais              |
|       DHT       |  54   |      | Definir pinos e tipo do DHT           | 
|     LED-RGB     |  59   |      | Definir pinos do LED-RGB              |  
|     microSD     |  64   |      | Definir o pino CS                     |
|     mudarCor    |  68   | void | recebe as cores do LED-RGB            |
|   conectarMQTT  |  83   | void | estabelecer conecção com broker       |
|     callback    |  102  | void | receber mensagens do broker           |
|  publishMessage |  115  | void | publicar mensagens em um canal        |
|     escrever    |  120  | bool | Anexar um valor no arquivo do cartão  |
|      setup      |  146  | void | inicializar diversos aspectos         |
|      loop       |  191  | void | rodar o programa continuamente        |
|                                                                        |     
|                                                                        |
|________________________________________________________________________|

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

const char* SSID = "Eoq";  //Intengele_IOT
const char* Senha = "SenX123t";     //1Nt3n83!3

//Definições do Wi-Fi
WiFiClient espClient;

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
File Temperatura;
File Umidade;
File Decibeis;
File gCO2;
File VIC;

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

//Escrever Arquivo no Cartão SD
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

bool escreverData(const char* arquivo) {
    File file = SD.open(arquivo, FILE_APPEND);

    if (!file) {
        return false;
    }
    
    file.println("_______________[Novos Dados]_______________");
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
}

  Serial.println("");
  delay(100);

  //Inicialização do KY-037
  pinMode(sensorDeSom, INPUT); 
  potMedia = 0;

  Serial.print("Sensor de Decibeis:");
  Serial.print("KY-037");
  Serial.println("ativo");

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

  escreverData("/Temperatura.txt");
  escreverData("/Umidade.txt");
  escreverData("/Decibeis.txt");
  escreverData("/gCO2.txt"); 
  escreverData("/VIC.txt");
  
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

  //Medição da Vibraçao
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float vibr[JANELA];

  float acT = sqrt((a.acceleration.x)*(a.acceleration.x) + (a.acceleration.y)*(a.acceleration.y) + (a.acceleration.z)*(a.acceleration.z));
  float angT = sqrt((g.gyro.x)*(g.gyro.x) + (g.gyro.y)*(g.gyro.y) + (g.gyro.z)*(g.gyro.z));
  float vibT = acT*angT;

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

  //Medição de temperatura e umidade
  float u = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(u) || isnan(t) ) {
    Serial.println(F("Falha na leitura do Sensor DHT22"));
    return;
  }
  
  bool uF = 0;
  bool uFF = 0;
  bool tF = 0;
  bool tFF = 0;
  bool pF = 0; 
  bool pFF = 0;

  //Fatores
  if(40 > u || u > 60) uF = 1;
  if(15 > u || u > 85) uFF = 1;

  if(t > 23) tF = 1;
  if(t > 30) tFF = 1;

  if(potencia > 5000) pF = 1;
  if(potencia > 155000) pFF = 1;

  
  //
  

  Serial.println("______________________[Novo Ciclo]______________________");

  Serial.print("Umidade = ");
  Serial.print(u);
  Serial.println("%");

  Serial.print("Temperatura = ");
  Serial.print(t);
  Serial.println("°C");

  Serial.print("Potencia Sonora = ");
  Serial.print(potencia);
  Serial.println(" W");

  Serial.print("Vibração = ");
  Serial.print(vibT);
  Serial.println(" m.rad/s4");
  
  escrever("/Temperatura.txt", t, "Temperatura");
  escrever("/Umidade.txt", u, "Umidade");
  escrever("/Decibeis.txt", potencia, "Intensidade Sonora");
  escrever("/gCO2.txt", CO2, "CO2");
  escrever("/VIC.txt", vibT, "VCI");

  delay(2500);

}
