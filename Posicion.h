#ifndef ASCENSOR_H
#define ASCENSOR_H

#define DISTANCIA        0.287  // Distancia de un cambio de estado del optoacoplador, en cm.
#define INTERVALO_MINIMO 5      // Intervalo mínimo entre cambios de estados, en milisegundos. 

enum estado_motor { SUBIENDO, BAJANDO, DETENIENDO, DETENIDO };

estado_motor leerUltimoMovimiento();
void  inicializarPosicion (int pin_opto);
float leerPosicion (estado_motor accion_deseada);
float leerVelocidad ();

#endif
