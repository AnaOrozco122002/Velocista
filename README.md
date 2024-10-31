# Código para el carro Sollow
### Sobre el código
----------------------------------
- Este es un codigo creado para un robot velocista sin turbina (aunque tiene lo necesario para el ingreso de la misma) por estudiantes del Semillero de Robótica de la Universidad del Cauca.
- La versión del IDE de Arduino utilizada fue la 2.3.3
- La versión de la libreria de Espressif para programación es la 2.0.11
- El Link para programar tarjetas Esp32:
`<link>` : https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

### Componentes Usados
----------------------------------

* ##### Microcontrolador
Para la logica del robot se utiliza la tarjeta XIAO ESP32C3 de Seeed Studio que tiene el siguiente diagrama de pines:

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/xiaopines.jpg)

* ##### Drivers

Se utilizarón drivers DRV LFX9201 que tiene el siguiente diagrama de pines:
![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/drvpin.jpg)

* ##### Regulador

Se utilizó el siguiente regulador a 5V debido a que utilizó una bateria de 11.9V a 1500mA

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/regul.jpg)

* ##### Sensor

Se utilizó un sensor de 16 QTR de tamaño XL:
![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/sensor.jpg)

### Pines Usados
----------------------------------

Se utilizan los siguientes pines

* Pin Analogico para la entrada del sensor: A2 
* Pines Digitales para la lectura del multiplexor del sensor: D9,D10,D0,D1
* Pines de control del PWM: Motor Derecho: D8, Motor Izquierdo: D4
* Pines de control de Dirección: Motor Derecho: D7, Motor Izquierdo: D3
* Pin Controlador de Turbina: D6
* Pin del modulo de inicio: D5


### Importante
----------------------------------

* Se utilizo la librera QTRSensor16 que es la versión modifica de la libreria para 8, esta se encuentra en la carpeta raiz del proyecto y se debe importar al IDE de arduino

![](https://github.com/AnaOrozco122002/Velocista/blob/master/images/add.jpg)

* La placa utiliza Bluetooth Low Energy por lo que es necesario la creación de UUID para cada caracteristica 

* La cadena para sintonización bluetooth debe ser de la siguiente forma: *Kp,Ti,Td,Vmax