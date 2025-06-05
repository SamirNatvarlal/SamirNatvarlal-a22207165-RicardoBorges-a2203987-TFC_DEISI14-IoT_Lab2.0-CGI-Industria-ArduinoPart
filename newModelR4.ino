#include <WiFiS3.h>
#include <PubSubClient.h>
#include <Stepper.h>  // Inclui a biblioteca para o motor de passo
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DHT.h>

// Configurações de WiFi
const char* ssid = "Nsei"; // Substitua pelo SSID da sua rede WiFi
const char* password = "Tiger2015"; // Substitua pela senha da sua rede WiFi

// Configurações do Broker MQTT
const char* mqtt_server = "10.10.0.57"; // Substitua pelo IP do seu broker MQTT

WiFiClient espClient;
PubSubClient client(espClient);

// Configurações do motor de passo
const int stepsPerRevolution = 2048; // Número de passos por revolução (para 28BYJ-48)
const int motorPin1 = 8;
const int motorPin2 = 9;
const int motorPin3 = 10;
const int motorPin4 = 11;
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin3, motorPin2, motorPin4); // Inicializa a biblioteca Stepper

// Pino do microswitch
const int switchPin = 2;

// Pino do sensor SW-420
const int vibrationSensorPin = 3;

// Sensor de corrente e tensão INA219
Adafruit_INA219 ina219;

// Configuração do sensor DHT11
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Variável para monitorar o estado anterior do microswitch
bool lastSwitchState = HIGH;

// variáveis para uso nas KPIs
unsigned long startTime, uptime, motorActivationTime, ledActivationTime;
int mqttReconnects = 0, motorActivations = 0, switchActivations = 0;
int modeOKCount = 0, modeNOTOKCount = 0;
float totalEnergyConsumed = 0.0;


// Função para ligar ao WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando-se a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("Endereço IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Falha na conexão WiFi");
  }
}

// Função callback para lidar com mensagens recebidas
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Mensagem recebida no tópico: ");
  Serial.print(topic);
  Serial.print(". Mensagem: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (messageTemp == "1") {
    modoOK();
  } else if (messageTemp == "2") {
    modoNOTOK();
  }

  // Publicar uma mensagem de resposta
  client.publish("22205245/anawen/device/recebeu", "Mensagem recebida com sucesso");
}

// Função para acender e apagar os LEDs (modoOK) e mover o motor de passo
void modoOK() {
  modeOKCount++;
  unsigned long start = millis();
  delay(2000);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  delay(2000);  // Mantém os LEDs acesos por 2 segundos
  digitalWrite(7, HIGH);
  digitalWrite(6, LOW);

  // Movimentar o motor de passo para frente e para trás
  Serial.println("Movimentação do motor do modoOK");

  delay(3000); 
  // Define a velocidade do motor (15 RPM)
  myStepper.setSpeed(15);

  // Mover o motor -135 graus (768 passos)
  myStepper.step(-4000);  // Gira o motor -135 graus
  Serial.println("Motor move para frente");

  delay(5000);

   // Mover o motor -135 graus (768 passos)
  myStepper.step(4000);  // Gira o motor -135 graus

  motorActivationTime += millis() - start;
  motorActivations++;

  Serial.println("Motor move volta para trás");

  delay(500);
  // Verifica se o microswitch foi pressionado
  int currentSwitchState = digitalRead(switchPin);
  if (currentSwitchState == LOW) {
    switchActivations++;
    Serial.println("Microswitch pressionado - Motor para");
  }

  ledActivationTime += millis() - start;
  digitalWrite(5, LOW);
  digitalWrite(7, LOW);
}

// Função para acender e apagar os LEDs (modoNOTOK) sem mover o motor de passo
void modoNOTOK() {
  modeNOTOKCount++;
  unsigned long start = millis();
  delay(2000);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  delay(2000);  // Mantém os LEDs acesos por 2 segundos
  digitalWrite(7, LOW);
  digitalWrite(6, HIGH);

  // Simular a verificação do microswitch
  Serial.println("Simulação de verificação do microswitch para modoNOTOK");

 delay(1000);
  // Verifica se o microswitch foi pressionado
  int currentSwitchState = digitalRead(switchPin);
  if (currentSwitchState == LOW) {
    switchActivations++;
    Serial.println("Microswitch pressionado - Ação parada");
  }

  ledActivationTime += millis() - start;
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}

void reconnect() {
  // Loop até que o cliente esteja ligado com sucesso
  while (!client.connected()) {
    Serial.print("Tentando a conexão do MQTT... ||");
    // Tentando se conectar
    if (client.connect("ArduinoClient")) {
      Serial.println("... conectado ao MQTT");
      // Assinando o tópico
      client.subscribe("22205245/anawen/device/test"); 
    } else {
      Serial.print(" falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      mqttReconnects++;
      // Esperar 5 segundos antes de tentar novamente
      delay(5000);
    }
  }
}

/*
void checkVibration() {
  if (digitalRead(vibrationSensorPin) == HIGH) {
    unsigned long timestamp = millis();
    Serial.print("Vibração detectada! Timestamp: ");
    Serial.println(timestamp);
    char msg[50];
    sprintf(msg, "Vibração detectada - Timestamp: %lu", timestamp);
    client.publish("22205245/anawen/device/vibracao", msg);
  }
}
*/

void readPowerSensor() {
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  char msg[100];
  sprintf(msg, "Tensao: %.2fV, Corrente: %.2fmA, Potencia: %.2fmW", busvoltage, current_mA, power_mW);
  Serial.println(msg);
  client.publish("22205245/anawen/device/power", msg);
}

void readDHT11() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Falha na leitura do DHT11");
    return;
  }
  char msg[80];
  sprintf(msg, "Temperatura: %.2fC, Umidade: %.2f%%", temperature, humidity);
  Serial.println(msg);
  client.publish("22205245/anawen/device/dht11", msg);
}

void sendKPIs() {
  uptime = (millis() - startTime) / 1000;
  char msg[200];
  sprintf(msg, "Uptime: %lu s, MQTT reconexões: %d, Ativação motor: %d, Tempo motor ativo: %lu ms, Microswitch: %d, ModoOK: %d, ModoNOTOK: %d, LEDs ativos: %lu ms, Consumo: %.2f mWh", 
          uptime, mqttReconnects, motorActivations, motorActivationTime, switchActivations, modeOKCount, modeNOTOKCount, ledActivationTime, totalEnergyConsumed);
  client.publish("device/kpis", msg);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Espera a conexão da porta serial
  }

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield não encontrado");
    while (true); // Não continua se o shield não for encontrado
  }

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP); // Configura o pino do microswitch
  pinMode(vibrationSensorPin, INPUT);
  dht.begin();
  ina219.begin();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  reconnect();

  startTime = millis(); // var?
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //checkVibration();
  //readPowerSensor();
  //readDHT11();
  delay(2000);
}
