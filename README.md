# Proyecto Electrónica Digital I  
## Sensor de turbidez de agua  

En este repositorio se explicará todo lo propuesto y desarrollado en el proyecto de electrónica digital paso a paso.

### Integrantes:
- Christian Camilo Cardenas
chcardenasr@unal.edu.co 
- Sebastián Tibaquirá Sánchez
stibaquira@unal.edu.co
- Anderson Camilo Rosero Yela
aroseroy@unal.edu.co 
- Daniel santiago Navarro Gil
dnavarrog@unal.edu.co


## Introducción

La calidad del agua es un factor determinante tanto en la salud humana como en la eficiencia de los cultivos agrícolas. Uno de los parámetros fundamentales para evaluar dicha calidad es la turbidez, que representa la cantidad de partículas suspendidas en el agua, como sedimentos, materia orgánica y microorganismos. Diversos estudios han demostrado que niveles elevados de turbidez están asociados a consecuencias negativas para la salud pública y la productividad agrícola.

En particular, un estudio publicado en el Journal of Water and Health señala que el uso de agua con turbidez muy superior a los estándares recomendados por la OMS puede implicar riesgos serios para la inocuidad de alimentos y la salud de los consumidores. En este estudio, el agua utilizada para riego en zonas urbanas presentaba turbidez hasta 17 veces mayor al límite permitido, generando un ambiente propicio para la proliferación de microorganismos patógenos y la contaminación cruzada de los cultivos  
Fuente: [Irrigation water quality and its impact on the microbial safety of irrigated vegetables in Kumasi, Ghana – IWA Publishing, 2021](https://iwaponline.com/jwh/article/19/3/488/80111/Irrigation-water-quality-and-its-impact-on-the)

Motivados por estas problemáticas, desarrollamos este proyecto con el objetivo de diseñar un sistema de monitoreo digital de turbidez basado en un sensor óptico y un microcontrolador ESP32, que permite detectar de forma continua los niveles de partículas suspendidas en el agua y emitir alertas si estos superan un umbral seguro. La idea es que este sistema pueda ser aplicado en contextos rurales o urbanos con acceso limitado a análisis de laboratorio, facilitando la toma de decisiones rápidas en cuanto al uso del agua para consumo o riego.

En la vida real, este dispositivo puede ser una herramienta económica, portátil y replicable para agricultores, comunidades rurales, plantas de tratamiento o investigadores que deseen evaluar la calidad del agua en tiempo real. Su uso puede prevenir enfermedades transmitidas por el agua, mejorar la seguridad alimentaria, y contribuir al uso más sostenible de los recursos hídricos.


### Descripción general
Este proyecto consiste en la medición de la turbidez de agua implementando un sensor de turbidez, una tarjeta de desarrollo ESP32 Wi-Fi, una FPGA y la transmisión de estos datos a un servidor vía Wi-Fi.

---

## Objetivo general
Diseñar un sistema de medición de turbidez de agua que obtenga datos en tiempo real, utilizando un sensor de turbidez SEN 0189, un microcontrolador ESP32, y una FPGA para procesar y analizar los datos obtenidos. Este diseño se realizó para ser escalable en las mediciones generales del agua.

## Objetivos específicos
- Diseñar un sistema con los tres componentes antes mencionados para solicitar al sensor los datos en tiempo real de la turbidez del agua y enviarlos a un servidor vía Wi-Fi.  
- Desarrollar un módulo en Verilog que implemente una máquina de estados finita para controlar la activación de una válvula según niveles de turbidez predefinidos.  
- Implementar una interfaz UART funcional en la FPGA para recibir datos de turbidez enviados por la ESP32.  
- Programar la ESP32 en MicroPython para leer valores del sensor de turbidez mediante su ADC y transmitirlos a la FPGA de forma serial.  
- Realizar pruebas funcionales y simulaciones del sistema digital completo para verificar la lógica de control, temporización y comunicación entre dispositivos.  
- Documentar detalladamente el diseño, desarrollo, pruebas y resultados del sistema, incluyendo diagramas, códigos fuente y procedimientos de carga/uso.  

---

## Descripción diagrama de caja negra
En el siguiente diagrama se muestran 4 componentes principales: un sensor de turbidez del agua, una tarjeta de desarrollo ESP32 Wi-Fi, una FPGA y un servidor receptor de datos.  

<p align="center">
  <img src="./diagrama_caja_negra.png" alt="Diagrama de caja negras" width="6000"/>
</p>

Todo inicia en la FPGA enviando una señal al ESP32; este le envía la instrucción de realizar la medición de turbidez al sensor, el ESP32 recibe el dato y lo envía a la FPGA. La FPGA convierte el dato para su lectura, análisis e interpretación, y posteriormente envía el resultado del análisis al ESP32, que finalmente transmite los datos vía Wi-Fi al servidor.

---

## Requerimientos Funcionales
- Lectura del nivel de turbidez mediante un sensor conectado a la ESP32.  
- Conversión del valor analógico del sensor a un valor digital entre 0 y 15, donde 0 representa agua clara (válvula cerrada) y 15 agua muy turbia (válvula totalmente abierta).  
- Envío del dato procesado desde la ESP32 a la FPGA utilizando comunicación UART.  
- Procesamiento del valor de turbidez en la FPGA para determinar el estado de apertura de la válvula.  
- Activación de una señal de control desde la FPGA que activa o desactiva la válvula según el nivel de turbidez.  
- Visualización del estado actual del sistema (valor de turbidez y estado de la válvula) mediante displays de 7 segmentos.  

## Requerimientos No Funcionales
- El sistema debe responder a los cambios en turbidez en un tiempo menor a 1 segundo.  
- El sistema debe ser confiable y funcionar de forma continua sin errores de comunicación ni fallas en la lógica.  
- El código debe estar documentado de forma clara para facilitar su comprensión y mantenimiento por parte de diferentes integrantes del equipo.  
- El diseño debe poder simularse digitalmente usando herramientas como Icarus Verilog y GTKWave para verificar su funcionamiento antes de ser cargado al hardware.  
- La implementación debe ser modular, permitiendo modificar o sustituir partes del sistema sin necesidad de reescribir todo el código.  

---

## Descripción diagrama de bloques

<p align="center">
  <img src="./diagrama_de_bloques.jpg" alt="Diagrama de bloques" width="6000"/>
</p>
---

## Descripción de los Módulos del Sistema y sus Conexiones

### Sensor de Turbidez – SEN0189
**Función:** Detecta la cantidad de partículas suspendidas en el agua, permitiendo medir su turbidez a través de una señal analógica.  

**Conexiones:**  
- `SEN0189_output → esp32_digital_pin_34`: Salida analógica del sensor que representa el nivel de turbidez.  
- `SEN0189_4.5V+ → Fuente de alimentación de 4.5V`: Alimentación del sensor.  
- `SEN0189_GND → esp32_GND`: Tierra común con el sistema.  

### ESP32 (Microcontrolador con Wi-Fi)
**Función:** Adquiere los datos del sensor SEN0189, actúa como convertidor ADC, envía los datos a la FPGA para procesamiento, y finalmente transmite la información procesada a través de Wi-Fi hacia un broker MQTT.  

**Conexiones:**  
- `esp32_digital_pin_34 ← SEN0189_output`: Entrada analógica para la señal de turbidez.  
- `esp32_tx_pin_17 → fpga_pin_D4`: Transmisión de datos hacia la FPGA mediante UART.  
- `esp32_rx_pin_16 ← fpga_pin_C4`: Recepción de datos procesados desde la FPGA.  
- `esp32_module_IEEE 802.11 → PC (Wi-Fi)`: Envío inalámbrico de datos usando el protocolo MQTT.  
- `esp32_GND → FPGA GND y SEN0189_GND`: Tierra común para todos los dispositivos conectados.  

### FPGA COLORLIGHT
**Función:** Procesa los datos digitales enviados por el ESP32. Además, genera la señal de control que indica cuándo se debe realizar la lectura del sensor.  

**Conexiones:**  
- `fpga_pin_D4 ← esp32_tx_pin_17`: UART Receiver (recibe los datos del ESP32).  
- `fpga_pin_C4 → esp32_rx_pin_16`: UART Transmitter (devuelve los datos al ESP32).  
- `fpga_jtag ↔ bridge_jtag_bitbang`: Interfaz de programación a través del puente JTAG.  
- `fpga_GND → esp32_GND y bridge_GND`: Tierra compartida con el resto del sistema.  

### BRIDGE (Interfaz USB–JTAG)
**Función:** Permite la programación de la FPGA desde el PC mediante la interfaz JTAG, utilizando una conexión USB.  

**Conexiones:**  
- `bridge_USB ↔ PC (USB)`: Canal de comunicación para cargar configuraciones desde el PC.  
- `bridge_jtag_bitbang ↔ fpga_jtag`: Interfaz de programación JTAG.  
- `bridge_GND → fpga_GND`: Conexión de tierra compartida.  

### PC (Servidor o Cliente MQTT)
**Función:** Recibe los datos de turbidez transmitidos por el ESP32 a través de Wi-Fi usando el protocolo MQTT. También se encarga de la energización de la FPGA a través del BRIDGE.  

**Conexiones:**  
- `pc_USB0 ↔ bridge_USB`: Conexión física para la programación del FPGA.  
- `pc_broker_MQTT`: Cliente o servidor MQTT encargado de recibir y gestionar los datos enviados por el ESP32.  

---

## Diagrama de flujo ESP32


<p align="center">
  <img src="./Diagrama_de_flujo_ESP32.jpg" alt="Diagrama de flujo ESP32" width="6000"/>
</p>

**Descripción del Diagrama de Flujo – ESP32**  
Este diagrama representa el flujo de funcionamiento lógico del microcontrolador ESP32 dentro del sistema de monitoreo de turbidez del agua.  

La ESP32 se encarga de recibir instrucciones de la FPGA, realizar lecturas del sensor de turbidez a través de su ADC, y enviar datos por UART o por Wi-Fi hacia un broker MQTT.

---

## Diagramas RTL del Proyecto

El sistema implementado fue sintetizado y verificado en niveles jerárquicos. A continuación se presentan los diagramas RTL (Register Transfer Level) de los principales módulos del diseño implementado en la FPGA.

---

### 1. RTL del SoC completo - `RTL_SoC.jpg`

Este diagrama representa el sistema completo integrado sobre la FPGA. El diseño es un SoC (System-on-Chip) que combina un procesador RISC-V (FemtoRV32) con múltiples periféricos personalizados conectados a un bus interno. Entre los periféricos se encuentra el controlador de riego, encargado de actuar según los niveles de turbidez del agua.

<p align="center">
  <img src="./Diagrama_de_flujo_ESP32.jpg" alt="Diagrama de flujo ESP32" width="6000"/>
</p>


Componentes principales:

- Módulo principal: femtorv32 SoC, que contiene:
  - CPU FemtoRV32: procesador ligero RISC-V que ejecuta el código cargado en memoria.
  - Memoria de instrucciones y datos: accedida directamente por el procesador.
  - Módulo de interconexión con periféricos: se observa lógica de multiplexado y chip select.
- Periféricos conectados:
  - peripheral_uart: controlador de comunicación serie UART.
  - periph_led_pwm: periférico que genera señales PWM.
  - periph_controladorriego: módulo de control de riego según turbidez.
- Señales de interconexión comunes: reloj (clk), reset, señales de dirección, lectura (rd) y escritura (wr).

Este diagrama permite visualizar cómo se integran todos los módulos funcionales en el sistema y cómo se conectan al procesador principal, permitiendo su control por software.

---

### 2. RTL del módulo controlador de riego - `RTL_controlador_riego.jpg`

Este RTL muestra la vista estructural del periférico encargado de la lógica de decisión sobre el riego, basado en los niveles de turbidez del agua.
<p align="center">
  <img src="./Diagrama_de_flujo_ESP32.jpg" alt="Diagrama de flujo ESP32" width="6000"/>
</p>

Entradas principales:

- clk: señal de reloj para sincronización.
- reset: señal de reinicio del módulo.
- turbidez: señal de entrada digitalizada que representa el nivel de turbidez del agua, recibida desde el ADC.
- ready_from_esp: señal que indica si el módulo ESP32 está listo para recibir comandos.

Salidas:

- enable_esp: señal que se activa cuando se detecta alta turbidez y se desea activar el sistema de comunicación con el ESP32.
- led_valvula: señal de control que enciende o apaga un LED o una electroválvula que representa el estado del sistema de riego.

La lógica del módulo se basa en comparar el valor de turbidez contra un umbral predefinido. Si el nivel de turbidez es alto, se activa la señal de riego. Adicionalmente, la señal enable_esp permite al sistema comunicarse con el microcontrolador ESP32 para registrar o reportar eventos.

---

### 3. RTL de la lógica interna del controlador de riego - `RTL_controlador_riego_interno.jpg`

Este diagrama corresponde al nivel más bajo de descripción estructural del módulo controlador de riego, mostrando el detalle de la lógica digital implementada.

<p align="center">
  <img src="./Diagrama_de_flujo_ESP32.jpg" alt="Diagrama de flujo ESP32" width="6000"/>
</p>

Componentes destacados:

- Comparadores lógicos: utilizados para comparar el valor de turbidez con umbrales fijos. Se observan comparaciones del tipo A > B, A == B, etc.
- Multiplexores (MUX): utilizados para seleccionar entre diferentes señales de entrada según las condiciones de control del sistema.
- Lógica combinacional: conformada por compuertas lógicas y redes de comparadores que permiten la toma de decisiones.
- Lógica secuencial: basada en registros y flip-flops que almacenan estados de control como ‘enable_esp’ o ‘led_valvula’ de un ciclo a otro.
- Constantes codificadas: valores como 0x00, 0xC, 0xEEB2B0, que son usados como referencias para comparación o para generar señales de control.
- Señales internas: incluyen señales temporales y de control intermedio generadas por FSMs (máquinas de estados finitos) o lógica combinacional.

Este diagrama es útil para validar que los componentes de control y decisión dentro del módulo están correctamente conectados y sincronizados con el sistema de reloj principal. También permite observar cómo el sistema responde de forma determinística a los datos de turbidez digitalizados.

---

---

## Adaptación de voltaje para el ADC del ESP32

La salida analógica del sensor (cable azul en la imagen) entrega una señal en el rango de 0 V a 4.5 V. Sin embargo, la entrada del ADC del ESP32 admite un máximo de 3.3 V, por lo cual conectar esta señal directamente podría dañar el microcontrolador o producir lecturas erróneas.

Para resolver este problema, se implementó un divisor resistivo compuesto por:

- Resistencia R1 = 100 kΩ conectada entre la salida del sensor y la entrada del ESP32.
- Resistencia R2 = 50 kΩ conectada entre la entrada del ESP32 y GND.

Este divisor reduce la tensión utilizando la siguiente ecuación:

![Ecuación divisor de tensión](./ecuacion_divisor_tension_ADC.png)


Con esto se asegura que el voltaje que llega al ESP32 no supere los 3.3 V, manteniendo así la integridad del sistema.

Este tipo de adaptación es crucial cuando se conectan sensores con salida superior a 3.3 V a microcontroladores modernos, que no toleran niveles TTL de 5 V.

Imagen del montaje del divisor de voltaje:

![RTL SoC](imagen_ADC.jpg) 

