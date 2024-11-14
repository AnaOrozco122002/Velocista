# Código para el Velocista Sollow
### Sobre el código
----------------------------------

- Este es un codigo creado para un robot velocista con turbina por estudiantes del Semillero de Robótica de la Universidad del Cauca.
- La versión del IDE de Arduino utilizada fue la 2.3.3
- La versión de la libreria de Espressif para programación es la 2.0.11
- El Link para programar tarjetas Esp32:
`<link>` : https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json


### Importante
----------------------------------

* Se utilizo la librera QTRSensor16 que es la versión modificada de la libreria para 8, esta se encuentra en la carpeta raiz del proyecto y se debe importar al IDE de arduino

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/add.jpg)

* La placa utiliza Bluetooth Low Energy por lo que es necesario la creación de UUID para cada caracteristica 

* La cadena para sintonización bluetooth debe ser de la siguiente forma: *Kp,Ti,Td,Vmax,Turbina

* Para modificar el Offset se debe entrar en la segunda caracteristica de la aplicacion y mandar valores entre 0 y 1.

### Pines Usados
----------------------------------

Se utilizan los siguientes pines

* Pin Analogico para la entrada del sensor: A2 
* Pines Digitales para la lectura del multiplexor del sensor: D9,D10,D0,D1
* Pines de control del PWM: Motor Derecho: D8, Motor Izquierdo: D6
* Pines de control de Dirección: Motor Derecho: D7, Motor Izquierdo: D5
* Pin Controlador de Turbina: D4
* Pin del modulo de inicio: D3



### Componentes Usados
----------------------------------

* ##### Microcontrolador

Para la logica del robot se utiliza la tarjeta XIAO ESP32C3 de Seeed Studio que tiene el siguiente diagrama de pines:

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/xiaopines.jpg)

* ##### Drivers

Se utilizarón 2 drivers DRV LFX9201 que tienen el siguiente diagrama de pines:

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/drvpin.jpg)

* ##### Regulador

Se utilizó el siguiente regulador a 5V ya que el sistema se alimenta con una bateria de 11.9V a 1500mA

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/regul.jpg)

* ##### Sensor

Se utilizó un sensor de 16 QTR de tamaño XL:

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/sensor.jpg)

