/**************************************************************************************************
* Control PID
***************************************************************************************************
* Sistemas de Control Autom�tico (SCA)
* Universidad Nacional de Avellaneda (UNDAV)
*
* Archivo:    pid_sca.cpp
* Versi�n:    2.1.
* Fecha:      Noviembre 2023
* Novedades:  Reorganizaci�n de funciones en .h y .cpp
* Versi�n Anterior: 2.0, diciembre 2021.
**************************************************************************************************/

#include "ControlPID.h"
#include "Arduino.h"

/*************************************************************************************************/

controlPID::controlPID(float KP, float TI, float TD)
// Constructor: incluye configuraci�n inicial del PID y valores predeterminados.
{  // Kp puede ser negativo (esto �ltimo podr�a servir para controlar una planta cuya salida 
   //                        tienda a bajar cuando aumente la se�al de control. Ej.: heladera.) 
   // Si Ti=0, el PID no lo tomar� en cuenta
   // Si Td=0, el PID no lo tomar� en cuenta
   // Inicializa integraci�n e impone false en l�mites y compensaci�n.
    ConfigurarPID(KP, TI, TD);
    LimitaSalida = false;
    CompensaIntegral = false;
}
//-------------------------------------------------------------------------------------------------

void controlPID::ConfigurarPID(float KP, float TI, float TD)
// Configura las constantes b�sicas del control PID 
// Sirve para cambiar configuraci�n inicial, sin modificar l�mites y banderas.
{  // Kp puede ser negativo (esto �ltimo podr�a servir para controlar una planta cuya salida 
   //                        tienda a bajar cuando aumente la se�al de control. Ej.: heladera.) 
   // Si Ti=0, el PID no lo tomar� en cuenta
   // Si Td=0, el PID no lo tomar� en cuenta
    Kp = KP;
    Ti = TI;
    Td = TD;
    // Resetea valores de integraci�n.
    TiempoAnterior = 0;
    ErrorAnterior = 0;
    CompensacionAnterior = 0;
    Integral = 0;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::LimitarSalida()
// Devuelve el valor de la variable privada LimitaSalida
// que indica si nuestro PID est� configurado para limitar su salida.
{
    return LimitaSalida;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::LimitarSalida(bool RESPUESTA)
// Configura si limitar� la salida...
// No permite activar l�mites si antes no fueron establecidos.
{
    LimitaSalida = RESPUESTA;
    if (SalidaMax == 0 && SalidaMin == 0) {
        // ...no voy a limitar porque no tengo l�mites definidos.
        LimitaSalida = false;
    }
    return LimitaSalida;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::LimitarSalida(bool RESPUESTA, float SMIN, float SMAX)
// Configura si limitar� la salida entre SMAX y SMIN.
// Se puede establecer los l�mites pero no activarlos a�n.
// No activa con SMIN=SMAX.
// No activa si SMIN>SMAX.
{
    SalidaMax = SMAX;
    SalidaMin = SMIN;
    LimitaSalida = RESPUESTA;
    // La forma de desactivar este l�mite es:
    // 1) Volviendo a llamar esta funci�n con RESPUESTA=false
    // 2) Llamando a LimitarSalida(false)
    // Tambi�n se pueden poner l�mites muy grandes.
    if (SalidaMax == SalidaMin) {
        // ...no voy a limitar porque no tengo l�mites definidos.
        LimitaSalida = false;
    }
    if (SalidaMin > SalidaMax) {
        // ...no voy a limitar porque est�n mal configurados.
        LimitaSalida = false;
    }
    return LimitaSalida;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::CompensarIntegral()
// Devuelve el valor de CompensaIntegral.
{
    return CompensaIntegral;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::CompensarIntegral(bool RESPUESTA)
// Establece si debo compensar la integraci�n cuando la salida est� saturada.
// Deben haberse preestablecido los l�mites de salida.
{
    CompensaIntegral = RESPUESTA;
    if (!LimitaSalida) CompensaIntegral = false;
    return CompensaIntegral;
}
//-------------------------------------------------------------------------------------------------

float controlPID::Controlar(float ERROR)
// Calcula Salida en funci�n de la se�al error y los par�metros del PID
{
    unsigned long TiempoActual = micros(); // Tomo tiempo actual para comparar con anterior

    // PROPORCIONAL --------------------------------------------------------------------------------
    Proporcional = Kp * ERROR;

    // DERIVATIVO ----------------------------------------------------------------------------------
    if (TiempoAnterior > 0 && Td != 0) {
        // Dos condiciones para componente derivativa:
        // 1) Que no sea el primer c�lculo y 2) Td seteado
        float Cuenta = Kp * Td * (ERROR - ErrorAnterior) * MILLON / (TiempoActual - TiempoAnterior);
        Derivativo = COHEFICIENTE_FILTRO * Cuenta + (1-COHEFICIENTE_FILTRO) * Derivativo;
    }
    else {
        Derivativo = 0;
    }

    // �Debo compensar? ----------------------------------------------------------------------------
    Salida = Proporcional + Integral + Derivativo;
    Compensacion = 0;
    if (LimitaSalida && CompensaIntegral) {
        // Si no hay l�mite de saturaci�n a la salida, no hay nada que compensar...
        if (Salida > SalidaMax) {
            // Debo saturar la salida porque supera el m�ximo...
            Compensacion = Salida - SalidaMax;
        }
        if (Salida < SalidaMin) {
            // Debo saturar la salida porque est� por debajo del m�nimo...
            Compensacion = Salida - SalidaMin;
        }
    }

    // INTEGRAL ------------------------------------------------------------------------------------
    if (TiempoAnterior > 0 && Ti != 0) {
        // Cumplidas las condiciones para integrar: (Si Compensacion==0, no va a compensar nada...)
        Integral += (Kp * (ERROR + ErrorAnterior) - (Compensacion + CompensacionAnterior))
            * (TiempoActual - TiempoAnterior)
            / (2 * Ti * MILLON);
        if (LimitaSalida) {
            // Debo saturar la integral:
            // (se supone que esto s�lo podr�a pasar si cambio los par�metros de integraci�n)
            Integral = min(Integral, SalidaMax);
            Integral = max(Integral, SalidaMin);
        }
    }

    // Termina componente integral -----------------------------------------------------------------

    // C�culo final completo: 
    Salida = Proporcional + Integral + Derivativo;

    if (LimitaSalida) {
        // Debo saturar la salida:
        // (se supone que esto s�lo podr�a pasar si cambio los par�metros de integraci�n)
        Salida = min(Salida, SalidaMax);
        Salida = max(Salida, SalidaMin);
    }
    TiempoAnterior = TiempoActual;
    ErrorAnterior = ERROR;
    CompensacionAnterior = Compensacion;
    return Salida;
    // Termina funcion PID -------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerIntegral()
{
    return Integral;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerProporcional()
{
    return Proporcional;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerDerivativo()
{
    return Derivativo;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerSalida()
{
    return Salida;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerCompensacion()
{
    return Compensacion;
}
//-------------------------------------------------------------------------------------------------

void controlPID::Apagar()
{
    TiempoAnterior = 0;
    ErrorAnterior = 0;
    CompensacionAnterior = 0;
    Integral = 0;
    Proporcional = 0;
    Derivativo = 0;
}

/**************************************************************************************************
* FIN DE ARCHIVO pid_sca.cpp
**************************************************************************************************/
