#include "Posicion.h"
#include "ControlPID.h"

// El equipo se inicia.
// Definicion de pines
#define PIN_PH1	8  // Control para el pin 1 del motor A // Baja al alimentar el IN1
#define PIN_PH2	9  // Control para el pin 2 del motor A // Sube el motor al alimentar IN2
#define PIN_PH_PWM 10 // Elegir un pin que admita PWM // 
#define PIN_EMERGENCIA 11 // Defino un pin para leer el boton de emergencia
#define PIN_OPTOACOPLADOR 12 //Elegir //Defino un pin para el optoacoplador
#define DELTA_TIEMPO = 50; // Intervalo entre muestras para control

// Constantes
#define INTERVALO_CONTROL 50 // en milisegundos
#define TOLERANCIA  // Define la tolerancia en centímetros
#define VELOCIDAD_MINIMA  // Define la velocidad mínima en cm/s
#define VEL_MAXIMA 35.6 // cm/s // Pasar a Posicion.h
//#define DISTANCIA_CAMBIO_ESTADO 0.287 // cm
#define VELOCIDAD_CONVERSION 5.93 // cm/V/s // Pasar a Posicion.h

// Instancia de la clase Posicion
// Posicion posicion;

// Instancia de la clase ControlPID
controlPID controlAscensor (0.7,0.0,0.5);

// Variables globales
float altura_objetivo = 0;
float altura_medida;
float accion_deseada_pwm;
estado_motor accion_deseada = DETENIDO;

// Se establece una variable ALTURA en 0 (la cual será el destino del ascensor)
// Definimos una variable MOTOR que nos indique 0 si esta quieto, 1 si sube y 2 si baja
// Se establece una variable ALT_OBJ en 0
// El ascensor estará en la planta baja.
// float ALTURA = 0.0;  // unificar unidades de altura y tipo de variable
//estado_motor accion_motor = DETENIDO;   // Estado inicial del motor
static unsigned long ultimo_tiempo = 0;
// unsigned long tiempo_actual = 0;

// Cambios de estados y desplazamiento:
// La polea tiene 46 cambios de estados en una vuelta.
// Diametro de polea = 42 mm => Recorrido en una vuelta pi*diametro = 131 mm
// Cada cambio de estado = 131 mm / 46 = 2,87 mm = 0,287 cm

// Declaración de funciones
void mostrar();
void detenerMaquina();
void controlar();
void actuar();

void setup() {
    inicializarPosicion(PIN_OPTOACOPLADOR);
    pinMode(PIN_PH1, OUTPUT);//Se inician los pines
    pinMode(PIN_PH2, OUTPUT); 
    pinMode(PIN_EMERGENCIA, INPUT);
    pinMode(PIN_PH_PWM, OUTPUT);
    // Definir PID (maximo y minimo)
    delay (3000); // Luego de 3 segundos arrancamos el programa.
    ESTADO = digitalRead(PIN_OPTOACOPLADOR); // verificar
      // inicialicen la conexión serial y manden encabezado de los datos
    Serial.begin(57600);  // poner velocidad mas rapida
    Serial.println("Encabezado de datos");
    // Agregar serial print con los datos de cada columna
    Serial.print("Tiempo\t");
    Serial.print("Altura actual (mm)\t");
    Serial.print("Altura objetivo (mm)\t");
    Serial.print("Acción Proporcional\t");
    Serial.print("Acción Integral\t");
    Serial.print("Acción Derivativa\t");
    Serial.println("Salida Controlador PID\t");
    ultimo_tiempo = millis();
    tiempo_actual = ultimo_tiempo;
    mostrar();
    
}

void loop() {
  tiempoActual = millis();
  
  // Verificamos el botón de emergencia (PIN_EMERGENCIA)
  if (digitalRead(PIN_EMERGENCIA) == HIGH) {
    detenerMaquina(); // Realizamos la parada de emergencia
  }
   
  // Obtenemos la altura medida utilizando la función de Posicion
  altura_medida = leerPosicion(accion_deseada);

  if (millis() - ultimo_tiempo >= DELTA_TIEMPO) {
    ultimo_tiempo += DELTA_TIEMPO;

    // Llamamos a la función de control
    accion_deseada_pwm = controlAscensor.Controlar(altura_objetivo, altura_medida); // Verificar parametros

    // Llamada a la función de actuación
    actuar(accion_deseada_pwm);

    // Mostramos información
    mostrar();
  }
}
//-------------------------------------------------------------------------------------------------
// Parada de emergencia
void detenerMaquina() {
  // Lógica para detener el ascensor de emergencia
  digitalWrite(PIN_PH1, LOW);
  digitalWrite(PIN_PH2, LOW);
  analogWrite(PIN_PH_PWM, 0);
}

float controlar(float objetivo, float altura) {

  const float kp = 0.7;  // Ganancia proporcional
  const float ti = 0;  // Ganancia integral (puedes ajustar según sea necesario)
  const float td = //No recuerdo cuanto era lo que habiamos calculado; // Ganancia derivativa

  static controlPID controlAscensor(kp, ti, td);

  // Calculamos el error (diferencia entre la altura objetivo y la altura actual)
  float error = objetivo - altura;

  // Calculamos la salida del control PID
  float voltaje_deseado = controlAscensor.Controlar(error);

  return voltaje_deseado;
}


void actuar(float accion_deseada_pwm) {

  int pwmValue = map(abs(accion_deseada_pwm), 0, 7, 0, 255);

  switch (accion_deseada)
​{
    case DETENIDO:
      // Ascensor Detenido
      if(altura_objetivo - altura_medida > TOLERANCIA){
        accion_deseada = SUBIENDO;
      }
      if(altura_objetivo - altura_medida < -TOLERANCIA){
        accion_deseada = BAJANDO;
      }
      break;

    case SUBIENDO:
      // Ascensor Subiendo
      if(accion_deseada_pwm <= 0){
        accion_deseada = DETENIENDO;
      }
      break;
    
    case BAJANDO:
      // Ascensor Bajando
      if(accion_deseada_pwm >= 0){
        accion_deseada = DETENIENDO;
      }
      break;

    case DETENIENDO:
      // Ascensor Deteniendose
      if(abs(leerVelocidad())< VELOCIDAD_MINIMA){
        accion_deseada = DETENIDO;
      }
      break;
  }

  switch (accion_deseada)
​{
    case DETENIDO:
      // Ascensor Detenido
      analogWrite(PIN_PH_PWM, 0);
      break;

    case SUBIENDO:
      // Ascensor Subiendo
      digitalWrite(PIN_PH1, LOW);
      digitalWrite(PIN_PH2, HIGH);
      analogWrite(PIN_PH_PWM, pwmValue); //Mapear 0 7 0 255 //Definir constante para el 7 
      break;
    
    case BAJANDO:
      // Ascensor Bajando
      digitalWrite(PIN_PH2, LOW);
      digitalWrite(PIN_PH1, HIGH);
      analogWrite(PIN_PH_PWM, pwmValue); //Mapear 0 7 0 255 //Definir constante para el 7 
    
      break;

    case DETENIENDO:
      // Ascensor Deteniendose
      analogWrite(PIN_PH_PWM, 0);
      break;
  }
}
void mostrar() {
    // Imprimimos los valores de las variables y del controlador PID
    Serial.print("Tiempo: ");
    Serial.print(ultimo_tiempo);
    Serial.print("\t");

    Serial.print("ALTURA actual: ");
    Serial.print(altura_medida);  // Cambiado de ALTURA a altura_medida
    Serial.print(" mm \t");

    Serial.print("ALTURA objetivo: ");
    Serial.print(altura_objetivo);
    Serial.print(" mm \t");

    Serial.print("Acción Proporcional: ");
    Serial.print(controlAscensor.ObtenerProporcional());
    Serial.print("\t");

    Serial.print("Acción Integral: ");
    Serial.print(controlAscensor.ObtenerIntegral());
    Serial.print("\t");

    Serial.print("Acción Derivativa: ");
    Serial.print(controlAscensor.ObtenerDerivativo());
    Serial.print("\t");

    Serial.print("Salida Controlador PID: ");
    Serial.print(controlAscensor.ObtenerSalida());
    Serial.print("\t");

    switch (MOTOR) {
    case SUBIENDO:
        Serial.println("SUB");
        break;
    case DETENIDO:
        Serial.println("DET");
        break;
    case BAJANDO:
        Serial.println("BAJ");
        break;
    }
}
