<div align="center">

# Julian Stiven Cardona Martinez

**Estudiante de Ingenieria Mecatronica · Universidad ECCI, Bogota**

</div>

> Este es un resumen en espanol. La version completa y actualizada esta en [README.md](README.md).

## Sobre mi

Soy estudiante de ingenieria mecatronica en la Universidad ECCI, a punto de graduarme, enfocado en
accionamientos electricos, control, sistemas embebidos y robotica. Mi tesis estudia variadores de
velocidad (VFD) para motores de induccion, validados en un banco de maquinas electricas
LabVolt/FESTO: control escalar V/Hz, comportamiento par-velocidad y las limitaciones practicas de
operar un motor de induccion fuera de su frecuencia nominal.

Estoy buscando mi primera oportunidad como ingeniero en mecatronica/control/embebidos, donde
pueda combinar el entendimiento a nivel de circuito de las maquinas electricas, firmware embebido
en tiempo real y control aplicado.

## Habilidades

Python · C · ESP-IDF · FreeRTOS · OpenCV · TensorFlow Lite · Git · GitHub Actions

## Proyectos destacados

- **[induction-motor-vfd-sim](https://github.com/julianS-eng/induction-motor-vfd-sim)** — Control y simulacion de variador de frecuencia para un motor de induccion trifasico, extension del trabajo de tesis sobre el banco LabVolt/FESTO.
- **[esp32-freertos-espnow-mesh](https://github.com/julianS-eng/esp32-freertos-espnow-mesh)** — Red mesh sobre ESP-NOW con FreeRTOS en ESP32 para mensajeria de baja latencia entre sensores/actuadores sin router Wi-Fi.
- **[Control-lab-inverted-pendulum](https://github.com/julianS-eng/Control-lab-inverted-pendulum)** — Control clasico y en espacio de estados (PID / LQR) de un pendulo invertido, desde el modelo linealizado hasta la validacion en banco.
- **[vision-line-follower-sim](https://github.com/julianS-eng/vision-line-follower-sim)** — Pipeline de vision con OpenCV y simulador para un robot movil seguidor de linea.
- **[tinyml-vibration-classifier](https://github.com/julianS-eng/tinyml-vibration-classifier)** — Modelo TensorFlow Lite Micro para clasificar firmas de vibracion en el dispositivo, orientado a mantenimiento predictivo de maquinas rotativas.

> Proyectos construidos con desarrollo asistido por IA; las decisiones de diseno, la validacion y las pruebas en hardware son mias.

## Este repositorio: simulacion reproducible de motor de induccion / VFD

Este repositorio incluye un paquete Python tipado y con pruebas (`src/imvfd_demo/`) que implementa
el modelo de circuito equivalente en estado estacionario de un motor de induccion trifasico bajo
control escalar V/Hz en lazo abierto — la misma estrategia de control usada en el banco VFD
LabVolt/FESTO de la tesis. Todas las cifras y figuras del README en ingles se generaron ejecutando
`generate-report`; nada esta escrito a mano ni estimado. El detalle tecnico completo, las
decisiones de diseno y las alternativas descartadas estan en [docs/LEARNING.md](docs/LEARNING.md)
(en espanol).

Resultados clave a 60 Hz (motor de ejemplo ilustrativo, ver `src/imvfd_demo/motor_model.py`):

| f (Hz) | Par de arranque (N·m) | Par de ruptura (N·m) | Velocidad de ruptura (rpm) |
|---|---|---|---|
| 60 | 21.79 | 47.19 | 1437 |

Estudio de sensibilidad Monte Carlo (n=5000, semilla=42, tolerancia ±10% en R2/X2 del rotor):
par de ruptura promedio **47.198 N·m**, desviacion estandar **0.526 N·m** (coeficiente de
variacion 1.11%).

### Limitaciones conocidas / trabajo futuro

- Modelo puramente de estado estacionario (sin dinamica temporal, sin inercia, sin lazo de
  control cerrado).
- Efecto piel en el rotor no modelado (R2 constante con el deslizamiento).
- Sin perdidas en el nucleo, friccion ni ventilacion.
- Parametros ilustrativos de libro de texto, no los datos experimentales reales del banco de
  tesis.
- Solo alimentacion trifasica balanceada.
- Los repositorios de "proyectos destacados" son el portafolio planificado del autor; no todos
  estan necesariamente poblados aun.

## Contacto

- LinkedIn: `<LINKEDIN_PROFILE_URL>`
- Email: `<CONTACT_EMAIL>`
