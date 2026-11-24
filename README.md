# TEI201
# Automatización de Bodegas PYME mediante Robot Sigue Líneas

![Render Del Robot](diseno_3d/renders/Render%20ISO.PNG)
## Integrantes
* Vicente Correa
* Fernando Toledo
* Maximiliano Basauri

# TEI201: Taller de Diseño en Ingeniería Sec. 1
# Universidad Adolfo Ibañez

## Descripcción del Proyecto
Este proyecto consiste en el diseño y desarrollo de un Vehículo de Guiado Automático (AGV) en formato de robot sigue líneas, diseñado específicamente para optimizar la intralogística en pequeñas y medianas empresas (PYMEs) del sector e-commerce en Chile. A diferencia de prototipos genéricos, este modelo cuenta con un chasis personalizado fabricado en madera mediante corte láser, una configuración de tracción delantera (dos motores frontales y una rueda de apoyo trasera) y un **array de sensores infrarrojos** para garantizar un seguimiento de trayectoria preciso mediante control de lazo cerrado.

## Problemática (Conexión ODS 9)
Actualmente en las bodegas PYME en Chile (menores de 500m^2 y con pasillos angostos) carecen de automatización. EL proceso de picking y traslado de carga se realiza manualmente. Los operarios caminan entre 10 y 15 km diarios, destinando hasta un 55% de su jornada laboral únicamente a desplazarse. Esto genera fatiga física, demoras significativas e ineficiencia logística. Nuestro proyecto busca eliminar este tiempo de traslado improductivo mediante tecnología accesible, modular y escalable.

## Componentes Utilizados (Hardware Principal)
* **Estructura:** Chasis de madera (MDF) diseñado en Fusion 360 y manufacturado mediante corte laser
* **Tracción:** 2x Motores DC con reductores (ruedas delanteras) + 1 rueda omnidireccional (apoyo trasero)
* **Controlador:** ESP32
* **Sensores:** Array de sensores infrarrojos para lectura de línea de alta resolución.
