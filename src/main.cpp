/**********************************************************************************************************************
 * V1.0 - @Nabinho
 * 13/05/2023
 *
 * Codigo de mixagem de canais de receptor de radio controle com placas Arduino.
 * Baseado para o controle de motores DC com a DFRobot Romeo.
 **********************************************************************************************************************/

// --------------------------------------------------------------------------------------------------------------------

// Inclusao das bibliotecas do projeto
#include <Arduino.h>
#include <Joystick.h>

// Criacao do objeto para controle USB
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_MULTI_AXIS, 4, 0,
                   false, false, false, false, false, false,
                   false, false, true, true, true);

// Definicao para DEBUG
// Descomente para debug no monitor serial
//#define DEBUG

// Variaveis dos pinos para leitura dos canais
const int PINO_THR = 2;
const int PINO_STR = 3;

// Variaveis para o canal THR
unsigned long tempo_atual_THR = 0;
unsigned long tempo_antes_THR = 0;
long tempo_alta_THR = 0;
float tempo_alta_THR_float;

// Variaveis para o canal STR
unsigned long tempo_atual_STR = 0;
unsigned long tempo_antes_STR = 0;
long tempo_alta_STR = 0;
float tempo_alta_STR_float;

// Variaveis dos limites de sinais de PPM de saida
const int PPM_MAXIMO = 1023;
const int PPM_MINIMO = 0;

// Variaveis dos sinais de acionamento dos drivers
int PPM_throtle = 0;
int PPM_steer = 0;
int ultimo_PPM_throtle = 0;
int ultimo_PPM_steer = 0;
const long TEMPO_MINIMO_PPM = 900;
const long TEMPO_MAXIMO_PPM = 2100;

// Variaveis para temporizacao do Fail-Safe
unsigned long tempo_ultimo_dado = 0;
unsigned long tempo_failsafe = 0;
const int INTERVALO_FAILSAFE = 1000;

// Pino conectado ao LED BUILTIN da Vespa
const int PINO_LED = 13;

// --------------------------------------------------------------------------------------------------------------------

// Funcao para medir o tempo do sinal do canal "THR"
void mede_THR()
{

  // Atualiza a contagem de tempo atual
  tempo_atual_THR = micros();

  // Verifica se a mudança de sinal do canal foi de "HIGH" para "LOW"
  if (digitalRead(PINO_THR) == LOW)
  {
    // Calcula a duracao do pulso em nivel logico alto do canal
    tempo_alta_THR = tempo_atual_THR - tempo_antes_THR;
  }

  //  Atualiza a contagem de tempo anterior
  tempo_antes_THR = tempo_atual_THR;
}

// --------------------------------------------------------------------------------------------------------------------

// Funcao para medir o tempo do sinal do canal "STR"
void mede_STR()
{

  // Atualiza a contagem de tempo atual
  tempo_atual_STR = micros();

  // Verifica se a mudança de sinal do canal foi de "HIGH" para "LOW"
  if (digitalRead(PINO_STR) == LOW)
  {
    // Calcula a duracao do pulso em nivel logico alto do canal
    tempo_alta_STR = tempo_atual_STR - tempo_antes_STR;
  }

  //  Atualiza a contagem de tempo anterior
  tempo_antes_STR = tempo_atual_STR;
}

// --------------------------------------------------------------------------------------------------------------------

void setup()
{

  // Inicia comunicacao serial em 115200 bps
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("<------- VESPA HOBBY V1.0 ------->");
#endif

  // Inicia o controle USB
  Joystick.begin();

  // Configura o pino conectado ao LED BUILTIN  da Vespa como saída
  pinMode(PINO_LED, OUTPUT);

  // Configura os pinos de leitura dos canais como entradas
  pinMode(PINO_THR, INPUT);
  pinMode(PINO_STR, INPUT);

  // Configuracao das interrupcoes dos canais
  attachInterrupt(digitalPinToInterrupt(PINO_THR), mede_THR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_STR), mede_STR, CHANGE);
}

// --------------------------------------------------------------------------------------------------------------------

void loop()
{

  // Conversao da duracao dos pulsos de 'long' para 'float' e de 'us' para 'ms'
  tempo_alta_THR_float = float(tempo_alta_THR);
  tempo_alta_STR_float = float(tempo_alta_STR);
  // tempo_alta_THR_float = tempo_alta_THR_float / 1000.0;
  // tempo_alta_STR_float = tempo_alta_STR_float / 1000.0;

  // Verifica se a duracao do pulso de ambos os canais esta dentro da faixa limite
  if (tempo_alta_THR_float > TEMPO_MINIMO_PPM && tempo_alta_THR_float < TEMPO_MAXIMO_PPM && tempo_alta_STR_float > TEMPO_MINIMO_PPM && tempo_alta_STR_float < TEMPO_MAXIMO_PPM)
  {

    // Atualiza a contagem de tempo do ultimo pulso recebido
    tempo_ultimo_dado = millis();

    // Acende o LED BUILTIN para indicar o recebimento de dados do receptor
    digitalWrite(PINO_LED, HIGH);

    // Mapeia o resultado do calculo para os valores maximo e minimo de acionamento dos drivers
    PPM_steer = map(tempo_alta_STR_float, 1000, 2000, PPM_MINIMO, PPM_MAXIMO);
    if (tempo_alta_THR_float < 1450)
    {
      PPM_throtle = map(tempo_alta_THR_float, 1000, 1450, PPM_MAXIMO, PPM_MINIMO);
    }
    if (tempo_alta_THR_float > 1550)
    {
      PPM_throtle = map(tempo_alta_THR_float, 1550, 2000, PPM_MINIMO, PPM_MAXIMO);
    }

    // Limita os valores de controle dentro dos valores maximo e minimo
    PPM_steer = constrain(PPM_steer, PPM_MINIMO, PPM_MAXIMO);
    PPM_throtle = constrain(PPM_throtle, PPM_MINIMO, PPM_MAXIMO);

    // Verifica se o novo dado e diferente do ultimo recebido
    if (PPM_throtle != ultimo_PPM_throtle || PPM_steer != ultimo_PPM_steer)
    {
      // Aciona os canais com os sinais mapeados
      Joystick.setSteering(PPM_steer);
      if (tempo_alta_THR_float < 1450)
      {
        Joystick.setAccelerator(PPM_throtle);
        Joystick.setBrake(0);
      }
      else if (tempo_alta_THR_float > 1550)
      {
        Joystick.setAccelerator(0);
        Joystick.setBrake(PPM_throtle);
      }
      else
      {
        Joystick.setAccelerator(0);
        Joystick.setBrake(0);
      }
    }

    // Atualiza as variaveis dos ultimos valores recebidos
    ultimo_PPM_throtle = PPM_throtle;
    ultimo_PPM_steer = PPM_steer;

    // Se DEBUG definido
#ifdef DEBUG
    // Exibe informacoes dos canais e dos calculos para monitoramento
    Serial.println("------------ CANAL THR ------------");
    Serial.print("TEMPO EM ALTA (ms): ");
    Serial.print(tempo_alta_THR_float);
    Serial.println("");
    Serial.println("------------ CANAL STR ------------");
    Serial.print("TEMPO EM ALTA (ms): ");
    Serial.print(tempo_alta_STR_float);
    Serial.println("");
    Serial.println("------- SINAIS PPM MAPEADOS -------");
    Serial.print("THROTLE: ");
    Serial.print(PPM_throtle);
    Serial.print(" | STEER: ");
    Serial.print(PPM_steer);
    Serial.println("");
#endif
  }
  // Caso nao haja novos dados validos dos canais
  else {

    // Zera as variaveis que armazenam os tempos dos canais convertidas para 'float'
    tempo_alta_THR_float = 0;
    tempo_alta_STR_float = 0;

    // Apaga o LED BUILTIN para indicar que não há sinal do receptor
    digitalWrite(PINO_LED, LOW);
  }

  // Caso o sinal de um dos canais ezteja zerado
  if (tempo_alta_THR_float == 0.0 || tempo_alta_STR_float == 0.0) {
    // Atualiza contagem de tempo do Fail-Safe
    tempo_failsafe = millis();
    // Verifica se a diferenca entre o tempo de Fail-Safe o tempo do ultimo dado valido recebido e maior do que o intervalor de acionamento do Fail-Safe
    if ((tempo_failsafe - tempo_ultimo_dado) > INTERVALO_FAILSAFE) {

      // Mantem os motores parados ate o recebimento de sinais ser restaurado
      Joystick.setSteering(0);
      Joystick.setAccelerator(0);
      Joystick.setBrake(0);

      // Se DEBUG definido
#ifdef DEBUG
      // Alerta que o Fail-Safe foi ativado
      Serial.println("FAILSAFE ATIVADO!!!");
#endif
    }
  }
}

// --------------------------------------------------------------------------------------------------------------------