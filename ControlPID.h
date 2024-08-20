/**************************************************************************************************
* Control PID
***************************************************************************************************
* Sistemas de Control Autom�tico (SCA)
* Universidad Nacional de Avellaneda (UNDAV)
*
* Archivo:    pid_sca.h
* Versi�n:    2.1.
* Fecha:      Noviembre 2023
* Novedades:  Reorganizaci�n de funciones en .h y .cpp
* Versi�n Anterior: 2.0, diciembre 2021.
**************************************************************************************************/

#define COHEFICIENTE_FILTRO 0.6

class controlPID            // Objeto para control Proporcional-Integral-Derivativo (PID)
{
private:

    float Salida;                  // La se�al de control que va al acuador
    // o potencia de salida (sin asignar unidades)
    float Proporcional;            // Componente proporcional de la salida 
    // (sin asignar unidades)
    float Integral;                // Componente integral 
    float Derivativo;              // Componente derivativa
    float Compensacion;            // Contiene la compensacion resultante ante saturaci�n
    float CompensacionAnterior;    // Como usamos aproximaci�n trapezoidal de la integral,
    // necesitamos conservar el valor anterior.
    float Kp;                      // Constante proporcional (sin asignar unidades)
    float Ti;                      // Tiempo de integraci�n (en segundos)
    float Td;                      // Tiempo para la componente derivativa (en segundos)
    unsigned long TiempoAnterior;  // Tiempo de la medici�n anterior utilizando micros
    float ErrorAnterior;           // Se�al de error anterior 
    bool LimitaSalida;             // Indica si establecimos l�mites superior e inferior
    bool CompensaIntegral;         // Indica si establecimos la compensacion de integral
    float SalidaMax;               // L�mite superior de la salida (y de la integral)
    float SalidaMin;               // L�mite inferior de la salida
    const float MILLON = 1e6;      // Constante para convertir micros() a segundos.

public:

    // Constructor con lo m�nimo:
    controlPID(float KP,          // KP: Constante de proporcionalidad (puede ser negativo)
        float TI,          // TI: Tiempo de integraci�n (si es 0, no integra)
        float TD);        // TD: Tiempo de derivaci�n (si es 0 no deriva)

    // Para cambiar configuraci�n inicial:
    void ConfigurarPID(float KP, float TI, float TD);  //Mismos par�metros que constructor.

    // Configura los l�mites de salida e indica si est�n activados:
    bool LimitarSalida(bool RESPUESTA, float SMIN, float SMAX);

    // Activa o desactiva los l�mites de salida e indica si el l�mite de salida est� activado:
    bool LimitarSalida(bool RESPUESTA);  // No permite activar l�mites si antes no fueron 
    // establecidos.

// Indica si el l�mite de salida est� activado:
    bool LimitarSalida();

    // Activa o desactiva la compansaci�n de integraci�n e indica si est� activado:
    bool CompensarIntegral(bool RESPUESTA);

    // Me indica si la compansaci�n est� activada:
    bool CompensarIntegral();

    // Calcula se�al de control (salida) en funci�n del error:
    float Controlar(float ERROR);

    // Apaga el PID manteniendo configuraci�n:
    void Apagar();                 // No se modifican los valores de KP, TI y TD.
    // Tampoco los l�mites pre establecidos.

    float ObtenerIntegral();
    float ObtenerProporcional();
    float ObtenerDerivativo();
    float ObtenerSalida();
    float ObtenerCompensacion();

};

/**************************************************************************************************
* FIN DE ARCHIVO pid_sca.h
**************************************************************************************************/
