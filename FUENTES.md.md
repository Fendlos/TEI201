# Declaración de Fuentes, Librerías y Atribución Técnica — TEI201

**Proyecto:** Automatización de Bodegas PYME mediante Robot Sigue Líneas  
**Curso:** Taller de Diseño en Ingeniería (TEI201) — Sección 1  
**Grupo:** Grupo 12 — Semestre 2026/2  
**Integrantes:** Vicente Correa Canals, Fernando Toledo, Maximiliano Basauri

## 1\. Librerías Externas de Firmware

A continuación se declaran las dependencias de software utilizadas en el microcontrolador ESP32-D0WDQ6 (NodeMCU-32S de 38 pines) para la lógica de navegación, temporización y telemetría:

|Librería / Dependencia|Versión / Entorno|Propósito en el Robot|Enlace / Desarrollador|
|-|:-:|-|-|
|`WiFi.h`|ESP32 Arduino Core|Gestión de enlace de radiofrecuencia a la red WiFi local (2.4 GHz) para enlace IoT.|[Espressif Systems](https://github.com/espressif/arduino-esp32)|
|`Arduino.h`|2.0+ / ESP32 Core|Mapeo de registros, control GPIO y framework central de ejecución.|[Arduino LLC](https://www.arduino.cc/)|
|`esp32-hal-ledc.h`|ESP32 Hardware Core|Generación de señales de modulación PWM por hardware (canales asignados a ENA/ENB).|[Espressif Systems](https://github.com/espressif/arduino-esp32)|
|`PubSubClient.h` / `HTTPClient.h`|2.8+|Transmisión de telemetría en tiempo real hacia el broker/dashboard remoto.|[Nick O'Leary / Espressif](https://github.com/knolleary/pubsubclient)|

\---

## 2\. Código de Terceros y Referencias Técnicas Adaptadas

* **Control PWM del Driver L298N mediante periférico LEDC:**

  * *Referencia:* Documentación oficial de Espressif Systems sobre el temporizador y generador LEDC del ESP32.
  * *Adaptación:* Se configuraron 2 canales independientes de modulación PWM a una frecuencia base de 5 kHz con resolución de 8 bits (0-255) vinculados a los pines `GPIO 14` (`ENA`) y `GPIO 32` (`ENB`). El puente H recibe además control lógico direccional en `IN1` (`GPIO 27`), `IN2` (`GPIO 26`), `IN3` (`GPIO 25`) e `IN4` (`GPIO 33`).
* **Adquisición del arreglo sensor infrarrojo BFD-1000 (TCRT5000):**

  * *Referencia:* Protocolo estándar de lectura digital paralelo para sensores de reflectancia óptica TCRT5000.
  * *Adaptación:* Lectura binaria síncrona en 5 pines digitales (`GPIO 13`, `GPIO 12`, `GPIO 15`, `GPIO 23` y `GPIO 22`) con cálculo del vector de desviación ponderada para el ajuste de velocidad diferencial entre motores.
* **Temporización del ciclo de muestreo:**

  * *Implementación:* Bucle de muestreo determinista mediante temporización no bloqueante (`millis()`) fijado a una frecuencia de muestreo de $100\\text{ Hz}$ ($10\\text{ ms}$ por ciclo), garantizando detección continua de curvas cerradas a la velocidad nominal de desplazamiento.

\---

## 3\. Herramientas de Diseño CAD, Simulación y Esquemas

* **Autodesk Fusion 360:** Modelado paramétrico 3D del chasis completo (`diseño-3d/modelo-3d.f3z`), diseño del gemelo digital con módulos reales embebidos, planos técnicos ortogonales (`diseño-3d/Plano.pdf`) y generación de vectores de corte láser 2D (`diseño-3d/corte laser.dxf` y `diseño-3d/corte laser.svg`).
* **Cirkit Designer:** Plataforma utilizada para la captura esquemática, cálculo de enrutamiento y documentación de conexionado eléctrico del sistema (`hardware/Diagrama de circuito.pdf`).

\---

## 4\. Declaración del Uso de Asistencia con Inteligencia Artificial (IA)

En concordancia con los estándares de integridad académica de la Facultad de Ingeniería y Ciencias (UAI) y la pauta de evaluación del curso TEI201, se transparenta el uso de herramientas de inteligencia artificial generativa:

1. **Modelos y plataformas empleadas:** Gemini (Google) y el asistente de cableado algorítmico de Cirkit Designer.
2. **Tareas específicas en las que se utilizó asistencia:**

   * **Hardware y Presupuesto:** Estructuración de parámetros técnicos, cuantificación de tolerancias eléctricas y generación consolidada de la lista de materiales (`hardware/BOM.xlsx`).
   * **Esquemático Eléctrico:** Cirkit Designer con la herramiente integrada se le describio el mapeo de pines, niveles de tensión lógica (3V3), potencia motriz (7,4V) y masa común (`GND`).
   * **Auditoría de Entregables:** Verificación de correspondencia entre el árbol de archivos del repositorio GitHub y la rúbrica formal del Avance 3.
3. **Validación y autoría del equipo humano:**  
Ninguna línea de código, conexión eléctrica o cota dimensional se implementó sin validación directa. El dimensionamiento de tolerancias mecánicas para corte láser en MDF de 3.0 mm, el cableado físico de componentes, el balanceo de masas, el cálculo de caída de tensión y las pruebas de tracción en pista fueron desarrollados, ensamblados y testeados experimentalmente por los integrantes del equipo.

