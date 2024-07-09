#include <Wire.h>              // Biblioteca para comunicação I2C
#include <VL53L0X.h>           // Biblioteca para lidar com o sensor VL53L0X
#include <NewPing.h>           // Biblioteca para lidar com o sensor ultrassônico HC-SR04
#include <Adafruit_MPU6050.h>  // Biblioteca para o sensor MPU6050
#include <Adafruit_Sensor.h>   // Biblioteca de sensor Adafruit
#include <cmath>               // Biblioteca para funções matemáticas como atan2

// Definindo os pinos do HC-SR04
#define TRIGGER_PIN 19    // Define o pino 19 do ESP32 como o pino TRIG do sensor HC-SR04
#define ECHO_PIN 18       // Define o pino 18 do ESP32 como o pino ECHO do sensor HC-SR04
#define MAX_DISTANCE 300  // Define a distância máxima de detecção do sensor em centímetros (3 metros)

// Definindo os pinos do buzzer e do controle de saída
#define VIBRACAL 13  // Define o pino GPIO 13 como pino de controle de saída
#define BUZZER 27    // Define o pino 27 como buzzer

// Cria uma instância do objeto NewPing chamado sonar, configurado 
// com os pinos TRIG e ECHO, e a distância máxima
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Declaração dos objetos dos sensores VL53L0X
VL53L0X sensor1;  // Cria uma instância do sensor VL53L0X chamada sensor1
VL53L0X sensor2;  // Cria uma instância do sensor VL53L0X chamada sensor2

// Declaração do objeto do sensor MPU6050
Adafruit_MPU6050 mpu;

// Definição dos pinos de controle dos sensores (XSHUT)
const int XSHUT1 = 16;  // Define o pino GPIO 16 para controle do sensor 1
const int XSHUT2 = 17;  // Define o pino GPIO 17 para controle do sensor 2

// Variáveis globais para armazenar os ângulos calculados
float AnguloX = 0;  // Armazena o ângulo calculado no eixo X
float AnguloY = 0;  // Armazena o ângulo calculado no eixo Y
float AnguloZ = 0;  // Armazena o ângulo calculado no eixo Z

// Variável para armazenar o tempo da última atualização
unsigned long lastUpdateTime = 0;  // Armazena o tempo da última atualização


// Variáveis para armazenar os offsets dos giroscópios
float gyroXOffset = 0;  // Armazena o offset do giroscópio no eixo X
float gyroYOffset = 0;  // Armazena o offset do giroscópio no eixo Y
float gyroZOffset = 0;  // Armazena o offset do giroscópio no eixo Z

// Variáveis para armazenar os offsets dos acelerômetros
float accelXOffset = 0;  // Armazena o offset do acelerômetro no eixo X
float accelYOffset = 0;  // Armazena o offset do acelerômetro no eixo Y
float accelZOffset = 0;  // Armazena o offset do acelerômetro no eixo Z

const int numReadings = 100;  // Número de leituras para a média

void setup() {
  Serial.begin(9600);  // Inicializa a comunicação serial com a taxa de 9600 bps
  Wire.begin(21, 22);  // Inicializa a comunicação I2C com SDA no GPIO 21 e SCL no GPIO 22

  // Inicialização dos pinos de controle dos sensores como saída
  pinMode(XSHUT1, OUTPUT);    // Define o pino XSHUT1 como saída
  pinMode(XSHUT2, OUTPUT);    // Define o pino XSHUT2 como saída
  pinMode(VIBRACAL, OUTPUT);  // Define o pino de controle como saída
  pinMode(BUZZER, OUTPUT);    // Define o pino do buzzer como saída

  // Desliga ambos os sensores VL53L0X inicialmente
  digitalWrite(XSHUT1, LOW);  // Mantém o sensor 1 desligado
  digitalWrite(XSHUT2, LOW);  // Mantém o sensor 2 desligado
  delay(10);                  // Aguarda 10 ms para garantir que os sensores estão desligados

  // Inicialização do primeiro sensor VL53L0X
  digitalWrite(XSHUT1, HIGH); // Liga o sensor 1
  delay(10);                  // Aguarda 10 ms para estabilização
  sensor1.setTimeout(500);    // Define o timeout para o sensor 1 em 500 ms
  if (!sensor1.init()) {      // Inicializa o sensor 1 e verifica se houve falha
    Serial.println("Falha ao iniciar o VL53L0X 1. Verifique as conexões!");  // Mensagem de erro
    while (1)
      ;  // Entra em loop infinito em caso de falha
  }
  sensor1.setAddress(0x30);  // Muda o endereço do sensor 1 para 0x30

  // Inicialização do segundo sensor VL53L0X
  digitalWrite(XSHUT2, HIGH);// Liga o sensor 2
  delay(10);                 // Aguarda 10 ms para estabilização
  sensor2.setTimeout(500);   // Define o timeout para o sensor 2 em 500 ms
  if (!sensor2.init()) {     // Inicializa o sensor 2 e verifica se houve falha
    Serial.println("Falha ao iniciar o VL53L0X 2. Verifique as conexões!");  // Mensagem de erro
    while (1)
      ;  // Entra em loop infinito em caso de falha
  }
  sensor2.setAddress(0x31);  // Muda o endereço do sensor 2 para 0x31

  // Configurações opcionais dos sensores VL53L0X
  sensor1.setMeasurementTimingBudget(20000);  // Define o tempo de medição do sensor 1 para 20 ms
  sensor2.setMeasurementTimingBudget(20000);  // Define o tempo de medição do sensor 2 para 20 ms

  Serial.println("VL53L0X 1 e 2 iniciados com sucesso");  // Mensagem de sucesso na inicialização

  // Inicializa o sensor MPU6050
  if (!mpu.begin()) {  // Inicializa o sensor MPU6050
    Serial.println("Não foi possível encontrar um sensor MPU6050, verificando conexão...");
    while (1) {
      delay(10);  // Loop infinito se o sensor não for encontrado
    }
  }
  Serial.println("MPU6050 encontrado!");

  // Configura os parâmetros do sensor MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // Configura o acelerômetro para o intervalo de 2G
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // Configura o giroscópio para o intervalo de 250 graus/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Configura a largura de banda do filtro para 21 Hz

  delay(100);  // Delay para estabilização

  // Variáveis para somar as leituras dos sensores para cálculo de média
  float gyroXSum = 0;  // Soma das leituras do giroscópio no eixo X
  float gyroYSum = 0;  // Soma das leituras do giroscópio no eixo Y
  float gyroZSum = 0;  // Soma das leituras do giroscópio no eixo Z

  float accelXSum = 0;  // Soma das leituras do acelerômetro no eixo X
  float accelYSum = 0;  // Soma das leituras do acelerômetro no eixo Y
  float accelZSum = 0;  // Soma das leituras do acelerômetro no eixo Z

  // Captura os valores iniciais como offset, calculando a média de várias leituras
  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;   // Declara eventos de sensor para acelerômetro, giroscópio e temperatura
    mpu.getEvent(&a, &g, &temp);  // Obtém eventos do sensor

    gyroXSum += g.gyro.x;  // Soma as leituras do giroscópio no eixo X
    gyroYSum += g.gyro.y;  // Soma as leituras do giroscópio no eixo Y
    gyroZSum += g.gyro.z;  // Soma as leituras do giroscópio no eixo Z

    accelXSum += a.acceleration.x;  // Soma as leituras do acelerômetro no eixo X
    accelYSum += a.acceleration.y;  // Soma as leituras do acelerômetro no eixo Y
    accelZSum += a.acceleration.z;  // Soma as leituras do acelerômetro no eixo Z

    delay(10);  // Pequeno delay entre as leituras
  }

  // Calcula os offsets (média das leituras iniciais)
  gyroXOffset = gyroXSum / numReadings;  // Calcula o offset do giroscópio no eixo X
  gyroYOffset = gyroYSum / numReadings;  // Calcula o offset do giroscópio no eixo Y
  gyroZOffset = gyroZSum / numReadings;  // Calcula o offset do giroscópio no eixo Z

  accelXOffset = accelXSum / numReadings;  // Calcula o offset do acelerômetro no eixo X
  accelYOffset = accelYSum / numReadings;  // Calcula o offset do acelerômetro no eixo Y
  accelZOffset = accelZSum / numReadings;  // Calcula o offset do acelerômetro no eixo Z

  lastUpdateTime = millis();  // Define o tempo da última atualização
}

void loop() {
  // Realiza uma medição de distância com o sensor HC-SR04 e armazena o resultado em centímetros
  unsigned int distanceHCSR04 = sonar.ping_cm();

  // Variável para controlar se a leitura do VL53L0X foi válida
  bool leituraValida = false;

  // Processa a leitura do sensor HC-SR04
  if (distanceHCSR04 > 0 && distanceHCSR04 <= MAX_DISTANCE) {  // Verifica se a leitura é válida
    Serial.print("Distância HC-SR04: ");
    Serial.print(distanceHCSR04);
    Serial.println(" cm");

    Serial.println("---------------------------------------------");

    if (distanceHCSR04 >= 100) {  // Se a distância for maior ou igual a 100 cm, aciona o vibrador
      digitalWrite(VIBRACAL, HIGH);
      delay(200);
      digitalWrite(VIBRACAL, LOW);
      delay(100);
    } else {  // Se a distância for menor que 100 cm, realiza a leitura dos sensores VL53L0X
      // Realiza uma medição de distância do sensor VL53L0X 1
      uint16_t distance1 = sensor1.readRangeSingleMillimeters();
      // Realiza uma medição de distância do sensor VL53L0X 2
      uint16_t distance2 = sensor2.readRangeSingleMillimeters();

      // Processa a leitura do sensor VL53L0X 1
      if (!sensor1.timeoutOccurred() && distance1 < 2000) {  // Verifica se a leitura é válida
        float distance1_cm = distance1 / 10.0;               // Converte a distância para centímetros
        Serial.print("Distância Sensor 1: ");
        Serial.print(distance1_cm);
        Serial.println(" cm");
        controlarPino(distance1_cm);  // Chama a função para controlar o pino
        leituraValida = true;

        Serial.println("---------------------------------------------");
      }

      // Processa a leitura do sensor VL53L0X 2
      if (!sensor2.timeoutOccurred() && distance2 < 2000) {  // Verifica se a leitura é válida
        float distance2_cm = (distance2 / 10.0) - 5.20;      // Converte a distância para centímetros e ajusta a leitura
        Serial.print("Distância Sensor 2: ");
        Serial.print(distance2_cm);
        Serial.println(" cm");
        controlarPino(distance2_cm);  // Chama a função para controlar o pino
        leituraValida = true;

        Serial.println("---------------------------------------------");
      }

      // Se nenhuma leitura dos sensores VL53L0X foi válida, desliga o pino de controle
      if (!leituraValida) {
        digitalWrite(VIBRACAL, LOW);
      }
    }
  }

  // Processa as leituras do sensor MPU6050
  sensors_event_t a, g, temp;   // Declara eventos de sensor para acelerômetro, giroscópio e temperatura
  mpu.getEvent(&a, &g, &temp);  // Obtém eventos do sensor

  unsigned long currentTime = millis();                         // Obtém o tempo atual
  float elapsedTime = (currentTime - lastUpdateTime) / 1000.0;  // Calcula o tempo decorrido em segundos
  lastUpdateTime = currentTime;                                 // Atualiza o tempo da última atualização

  // Subtrai os valores de offset das leituras do giroscópio e converte para graus
  float gyroX = (g.gyro.x - gyroXOffset) * 180 / PI;
  float gyroY = (g.gyro.y - gyroYOffset) * 180 / PI;
  float gyroZ = (g.gyro.z - gyroZOffset) * 180 / PI;

  // Subtrai os valores de offset das leituras do acelerômetro
  float accelX = a.acceleration.x - accelXOffset;
  float accelY = a.acceleration.y - accelYOffset;
  float accelZ = a.acceleration.z - accelZOffset;

  // Calcula os ângulos do acelerômetro em relação aos eixos X e Y
  float accelAngleX = atan2(accelY, accelZ) * 180 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  const float alpha = 0.96;  // Fator de suavização para o filtro complementar

  // Filtro complementar para combinar as leituras do acelerômetro e giroscópio
  AnguloX = alpha * (AnguloX + gyroX * elapsedTime) + (1.0 - alpha) * accelAngleX;
  AnguloY = alpha * (AnguloY + gyroY * elapsedTime) + (1.0 - alpha) * accelAngleY;
  AnguloZ += gyroZ * elapsedTime;  // O giroscópio é suficiente para o eixo Z

  // Imprime os ângulos no Monitor Serial
  Serial.print("Ângulo X: ");
  Serial.print(AnguloX);
  Serial.print(" °, Y: ");
  Serial.print(AnguloY);
  Serial.print(" °, Z: ");
  Serial.print(AnguloZ);
  Serial.println(" °");

  Serial.println("---------------------------------------------");


  // Condição para acionar/desligar o buzzer no pino 27 com base no ângulo X
  if (AnguloX > 70) {         // Se o ângulo X for maior que 70 graus, aciona o buzzer
    tone(BUZZER, 1450, 200);  // Aciona o buzzer com frequência de 1450 Hz e um delay de 200 ms
  } else {                    // Caso contrário, desliga o buzzer
    noTone(BUZZER);           // Desliga o buzzer
  }

  delay(1000);  // Delay para desacelerar o loop
}

// Função para controlar o pino conforme a distância
void controlarPino(float DISTANCIA) {
  if (DISTANCIA >= 0 && DISTANCIA < 10) {           // Se a distância estiver entre 0 e 10 cm
    digitalWrite(VIBRACAL, HIGH);                   // Liga o pino de controle
  } else if (DISTANCIA >= 10 && DISTANCIA < 20) {   // Se a distância estiver entre 10 e 20 cm
    digitalWrite(VIBRACAL, HIGH);                   // Liga o pino de controle
    delay(1000);                                    // Delay de 200 ms
    digitalWrite(VIBRACAL, LOW);                    // Desliga o pino de controle
    delay(500);                                     // Delay de 200 ms
  } else if (DISTANCIA >= 20 && DISTANCIA < 40) {   // Se a distância estiver entre 20 e 40 cm
    digitalWrite(VIBRACAL, HIGH);                   // Liga o pino de controle
    delay(800);                                     // Delay de 400 ms
    digitalWrite(VIBRACAL, LOW);                    // Desliga o pino de controle
    delay(400);                                     // Delay de 400 ms
  } else if (DISTANCIA >= 40 && DISTANCIA < 70) {   // Se a distância estiver entre 40 e 70 cm
    digitalWrite(VIBRACAL, HIGH);                   // Liga o pino de controle
    delay(600);                                     // Delay de 800 ms
    digitalWrite(VIBRACAL, LOW);                    // Desliga o pino de controle
    delay(300);                                     // Delay de 800 ms
  } else if (DISTANCIA >= 70 && DISTANCIA < 100) {  // Se a distância estiver entre 70 e 100 cm
    digitalWrite(VIBRACAL, HIGH);                   // Liga o pino de controle
    delay(400);                                     // Delay de 1000 ms
    digitalWrite(VIBRACAL, LOW);                    // Desliga o pino de controle
    delay(200);                                     // Delay de 1000 ms
  } else {                                          // Se a distância for maior ou igual a 100 cm
    digitalWrite(VIBRACAL, LOW);                    // Desliga o pino de controle
  }
}
