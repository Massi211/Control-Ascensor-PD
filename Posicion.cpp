#include "Posicion.h"

estado_motor ultima_accion = DETENIDO;
unsigned long tiempo_ultimo_cambio_0 = 0;
unsigned long tiempo_ultimo_cambio_1 = 0;
unsigned long tiempo_ultimo_cambio_2 = 0;
bool estadoOptoacoplador;
float altura = 0;
int pin_optoacoplador;

//**********************************************************************
void inicializarPosicion(int pin_opto) 
{
	pin_optoacoplador = pin_opto;
	// Configuramos el pin del optoacoplador como entrada
	pinMode(pin_optoacoplador, INPUT);
  estadoOptoacoplador = digitalRead(pin_optoacoplador);
  altura = 0;
}

//**********************************************************************
estado_motor leerEstadoMotor()
{
	return ultima_accion;
}

//**********************************************************************
float leerPosicion(estado_motor accion_deseada) 
{

	// Obtenemos el tiempo actual y leo estado de optoacoplador
	unsigned long tiempo_actual = millis();
  bool lectura_actual = digitalRead(pin_optoacoplador);

  // Verificamos si hubo cambio de estado
	if (estadoOptoacoplador() != lectura_actual) {
    
    // ANTIRREBOTE: verificamos si ha pasado un tiempo mínimo
    if (tiempo_actual - tiempo_ultimo_cambio_0 >= INTERVALO_MINIMO) {
      // Hubo un cambio de estado verificado!!!

      // Guardamos los tiempos de los dos últimos cambios de estado
      tiempo_ultimo_cambio_2 = tiempo_ultimo_cambio_1;
      tiempo_ultimo_cambio_1 = tiempo_ultimo_cambio_0;
      tiempo_ultimo_cambio_0 = tiempo_actual;

      // Actualizar la última acción si el motor estaba subiendo o bajando
      if (MOTOR == SUBIENDO) {
        ultima_accion = SUBIENDO;
      } 
      if (MOTOR == BAJANDO) {
        ultima_accion = BAJANDO;
      }

      // Calculamos la posición actual
      if (ultima_accion==SUBIENDO) {altura+=DISTANCIA;};
      if (ultima_accion==BAJANDO)  {altura-=DISTANCIA;};
    }    
  }

  //Devolvemos la altura
	return altura;
}

//**********************************************************************
float leerVelocidad()
{
  float medicion = 0;
  unsigned int delta_actual   = 0; // es el delta actual, que mide si reduce la velocidad o no
  unsigned int delta_anterior = 0; // es el delta anterior, medido entre los deltas de tiempo actual y anterior al anterior.
  
  if (0 == tiempo_ultimo_cambio_1){
    // Por el momento no hubo ninguna interrupciòn almacenada
    // Se supone una velocidad de 0 RMP.
    medicion = 0;
  } else {
    delta_actual = millis() - tiempo_ultimo_cambio_1;
    delta_anterior = tiempo_ultimo_cambio_0 - tiempo_ultimo_cambio_2;

    if ( 0 != tiempo_ultimo_cambio_2 ) {
      // Verifico que ya habìa sido actualizado tiempo_ultimo_cmbio_2 
      // y elijo la menor velocidad (== a el mayor tiempo): 
      delta_actual = max(delta_actual; delta_anterior);
    }
    
    medicion = 2 * DISTANCIA / (delta_actual);
  }

  return medicion;
}

estado_motor leerUltimoMovimiento()
{
  return ultima_accion;
}




