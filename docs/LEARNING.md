# Notas de aprendizaje: motor de induccion y variador V/Hz

Este documento explica, paso a paso, la teoria detras del modelo implementado en
`src/imvfd_demo/`, las decisiones de diseno tomadas al construirlo (y las alternativas que se
descartaron y por que), y termina con 10 preguntas de entrevista tecnica con sus respuestas. Esta
escrito para alguien que ya sabe circuitos basicos pero quiere repasar la teoria de maquinas de
induccion y control escalar V/Hz con el mismo nivel de detalle que se usa en una entrevista tecnica.

## 1. Teoria paso a paso

### 1.1 Campo giratorio y deslizamiento

Un motor de induccion trifasico tiene un estator con tres devanados desfasados 120° electricos.
Al alimentarlos con corrientes trifasicas equilibradas de frecuencia `f` (Hz), se produce un campo
magnetico que gira a la **velocidad sincrona**:

```
n_sync (rpm) = 120 * f / p
```

donde `p` es el numero de polos. El rotor (tipo jaula de ardilla) nunca gira exactamente a esa
velocidad: si lo hiciera, no habria variacion de flujo relativa entre el campo del estator y las
barras del rotor, no se induciria corriente en el rotor, y por lo tanto no habria par. La
diferencia relativa entre la velocidad sincrona y la velocidad mecanica real `n_r` se llama
**deslizamiento**:

```
s = (n_sync - n_r) / n_sync
```

En `motor_model.py`, `synchronous_speed_rpm`, `slip_from_speed` y `speed_from_slip` implementan
exactamente estas tres relaciones (y son funciones inversas entre si, verificado en
`test_speed_and_slip_are_inverse_functions`).

### 1.2 Circuito equivalente por fase

El motor de induccion se modela, por fase y en estado estacionario senoidal, con el circuito
equivalente clasico (ver Chapman, *Electric Machinery Fundamentals*):

- `R1`, `X1`: resistencia y reactancia de dispersion del estator.
- `Xm`: reactancia de magnetizacion (rama en paralelo que modela el flujo mutuo entrehierro).
- `R2`, `X2`: resistencia y reactancia de dispersion del rotor, referidas al estator.

La rama del rotor se representa como `R2/s + jX2`: dividir `R2` entre el deslizamiento es lo que
convierte la potencia electrica transferida al rotor en la suma de la perdida cobre del rotor mas
la potencia mecanica desarrollada — es el truco algebraico estandar para poder resolver el
circuito con herramientas de corriente alterna en estado estacionario, sin tener que resolver
ecuaciones diferenciales del rotor en movimiento.

### 1.3 Equivalente de Thevenin

Para no tener que resolver el circuito completo cada vez que cambia el deslizamiento, se reduce
todo lo que esta a la izquierda de la rama del rotor (fuente, `R1+jX1`, `jXm`) a un equivalente de
Thevenin `(V_th, Z_th)`. Esto es exacto (no una aproximacion), porque el deslizamiento solo
aparece en la impedancia de la rama del rotor — el resto del circuito es lineal e independiente de
`s`. `thevenin_equivalent()` calcula esto con aritmetica compleja:

```
Z_th = (Z1 * Zm) / (Z1 + Zm)
V_th = V_fase * Zm / (Z1 + Zm)
```

### 1.4 Ecuacion de par

Con el equivalente de Thevenin, el par electromagnetico en funcion del deslizamiento es:

```
T(s) = 3 * |V_th|^2 * (R2/s) / (omega_sync * [(R_th + R2/s)^2 + (X_th + X2)^2])
```

`omega_sync` es la velocidad sincrona **mecanica** en rad/s. Esta es la formula implementada en
`torque_nm()`. Es continua y positiva para `0 < s <= 1` (region de motor), y vale 0 en `s = 0`
(no hay corriente de rotor a velocidad sincrona) — verificado directamente en
`test_torque_is_zero_at_synchronous_speed` y `test_torque_is_positive_for_motoring_slips`.

Un resultado util que se aprovecha en las pruebas: como `V_th` aparece al cuadrado, el par escala
con el **cuadrado del voltaje aplicado** — reducir el voltaje a la mitad reduce el par a un
cuarto, a igual deslizamiento (`test_torque_scales_with_voltage_squared`).

### 1.5 Par de arranque y par de ruptura

- **Par de arranque** (*starting/locked-rotor torque*): `T(s=1)`, el par disponible con el rotor
  detenido. Determina si el motor puede arrancar bajo la carga que tiene acoplada.
- **Par de ruptura** (*breakdown/pull-out torque*): el maximo de `T(s)` sobre todo el rango de
  deslizamiento. Tiene forma cerrada (derivando `T(s)` respecto a `s` e igualando a cero):

```
s_max = R2 / sqrt(R_th^2 + (X_th + X2)^2)
T_max = 3 * |V_th|^2 / (2 * omega_sync * (R_th + sqrt(R_th^2 + (X_th + X2)^2)))
```

`breakdown_torque_analytic()` implementa estas dos formulas cerradas. El test
`test_breakdown_torque_is_the_numeric_maximum_of_the_torque_curve` confirma que coinciden con una
busqueda numerica de grano fino sobre `T(s)`, y `test_breakdown_torque_reference_values` fija esos
valores como referencia de regresion (verificados a mano con aritmetica compleja independiente:
`R_th ~= 0.591 Ohm`, `X_th ~= 1.075 Ohm`, `V_th ~= 115.2 V` para el motor de ejemplo a 208 V/60 Hz).

Si operar mas alla del deslizamiento de ruptura (es decir, seguir cargando el motor mas alla de
`s_max`), el par **disminuye** en vez de aumentar: el motor se detiene ("se cala") en vez de
adaptarse a mas carga. Por eso el punto de operacion nominal siempre esta muy por debajo de
`s_max`, tipicamente `s < 0.05` en motores comerciales.

### 1.6 Control escalar V/Hz

El flujo en el entrehierro es aproximadamente proporcional a `V / f` (la tension inducida es
proporcional a `f * flujo`). Si se sube la frecuencia para aumentar la velocidad sin subir el
voltaje en la misma proporcion, el flujo cae y el par disponible cae con el (el motor se
"desmagnetiza"). El control escalar V/Hz simplemente mantiene `V/f` aproximadamente constante por
debajo de la frecuencia nominal, para mantener el flujo — y por lo tanto el par disponible —
aproximadamente constante en todo el rango de velocidad:

```
V(f) = V_boost + (V_nom - V_boost) / f_nom * f      si f <= f_nom
V(f) = V_nom                                         si f > f_nom
```

Esto es exactamente lo que implementa `VfControlLaw.output_voltage()`. Dos detalles importantes:

- **Boost de baja frecuencia** (`V_boost`, minusculo, p. ej. unos pocos voltios): a frecuencias
  muy bajas, `V/f` nominal da un voltaje pequeno, y una fraccion importante de ese voltaje se cae
  en la resistencia del estator `R1` (caida IR) en vez de producir flujo util. El boost compensa
  esa caida para no perder par de arranque a baja velocidad. Es una decision practica de diseno,
  no un efecto fisico del motor.
- **Debilitamiento de campo** (*field weakening*) por encima de `f_nom`: el variador no puede
  superar el voltaje nominal (limite de la fuente/inversor), asi que por encima de `f_nom` el
  voltaje se satura y el flujo — y el par disponible — caen con `1/f`. Esto se ve claramente en
  las curvas par-velocidad generadas: a 60 Hz (frecuencia nominal del motor de ejemplo) el par de
  ruptura es el mas alto de la familia de curvas.

### 1.7 Escalado de las reactancias con la frecuencia

`X = 2 * pi * f * L`, asi que toda reactancia (`X1`, `X2`, `Xm`) especificada a la frecuencia
nominal se reescala linealmente con la frecuencia de operacion: `X(f) = X_nominal * f / f_nominal`
(`_scaled_reactance()` en `motor_model.py`). Las resistencias, en este modelo, se tratan como
constantes con la frecuencia (ver limitaciones, seccion 3 del README, sobre el efecto piel).

## 2. Decisiones de diseno y alternativas descartadas

| Decision tomada | Alternativa considerada | Por que se descarto |
|---|---|---|
| Modelo de circuito equivalente en **estado estacionario** (algebraico, por deslizamiento) | Modelo dinamico d-q (ejes directo/cuadratura) con inercia y simulacion en el tiempo | El objetivo de este repositorio es producir figuras y metricas de par-velocidad reproducibles y verificables a mano; un modelo dinamico anade estados (velocidad, corrientes d-q), un integrador numerico y muchos mas parametros (inercia, friccion viscosa) sin cambiar la conclusion de diseno principal (como cae el par disponible con V/Hz escalar). Se documenta como trabajo futuro en el README en vez de omitirlo. |
| **Control escalar V/Hz** en lazo abierto | Control vectorial orientado a campo (FOC) | FOC requiere estimar o medir el flujo del rotor (con sensor de posicion/velocidad o un observador), y un lazo de corriente en ejes d-q con un microcontrolador dedicado. Es la tecnica que da mejor par a baja velocidad y mejor dinamica, pero esta fuera del alcance de un estudio de circuito equivalente en estado estacionario, y **no** es la tecnica usada en el variador LabVolt/FESTO del banco de tesis, que es escalar. Se deja como posible extension (ver README). |
| Parametros de motor **ilustrativos de libro de texto** (`EXAMPLE_MOTOR`) | Usar los datos experimentales reales del banco LabVolt/FESTO de la tesis | Los datos de la tesis son especificos del equipo de laboratorio y no estan preparados como un dataset limpio, versionado y libre de restricciones para un repositorio publico. Usar un ejemplo de circuito equivalente ampliamente citado en la literatura permite que cualquiera reproduzca exactamente las cifras de este README sin depender de datos propietarios, y evita presentar como "resultados de tesis" numeros que en realidad vienen de un modelo generico. Esto se declara explicitamente en el docstring de `EXAMPLE_MOTOR` y en la seccion de limitaciones del README. |
| Tolerancia de fabricacion del rotor modelada con **distribucion uniforme** `R2, X2 ~ U(0.9, 1.1) * nominal` | Distribucion normal/gaussiana centrada en el valor nominal | Las hojas de datos de fabricantes tipicamente especifican una tolerancia como un rango simetrico ("±10%"), no una desviacion estandar; una uniforme es la forma mas directa y menos supuesta de representar "el fabricante garantiza que el valor cae dentro de este rango", sin inventar una forma de campana que no esta documentada en ninguna hoja de datos real. |
| Generador aleatorio **`numpy.random.default_rng(seed=42)`**, semilla explicita como parametro de funcion | `numpy.random.seed()` global, o no fijar semilla | El generador moderno de NumPy (`Generator`, PCG64) es la API recomendada desde NumPy 1.17 y evita el estado aleatorio global mutable que puede filtrarse entre pruebas o llamadas. Fijar la semilla como argumento explicito (no como efecto global) hace que `monte_carlo_breakdown_torque()` sea una funcion pura y trivialmente reproducible, lo cual se verifica directamente con `test_monte_carlo_is_reproducible_with_same_seed`. |
| **`src/` layout** con `imvfd_demo` instalable (`pip install -e .`) | Scripts sueltos en la raiz del repositorio | El layout `src/` evita que las pruebas importen el paquete por accidente desde el directorio de trabajo en vez de la version instalada (un error clasico de empaquetado en Python), y refleja como se distribuiria un paquete real. |
| `MotorParameters` y `VfControlLaw` como **`@dataclass(frozen=True, slots=True)`** | Diccionarios sueltos, o clases mutables | Inmutabilidad evita que una funcion modifique por accidente los parametros de un motor que otra parte del codigo sigue usando (importante en el estudio Monte Carlo, donde se crean miles de variantes del motor de ejemplo); `slots=True` reduce el uso de memoria al generar miles de instancias en el bucle de Monte Carlo. |
| **mypy en modo `strict`** | mypy por defecto (sin `--strict`) | El paquete es pequeno; pagar el costo de anotar todo con precision desde el inicio es barato aqui y detecta errores de unidades (p. ej. mezclar fasores `complex` con magnitudes `float`) en tiempo de analisis estatico en vez de en tiempo de ejecucion. |
| **CI en GitHub Actions** con matriz Python 3.11 / 3.12, ejecutando lint + formato + tipos + pruebas + una regeneracion real del reporte | Solo pruebas unitarias | Ejecutar tambien `generate-report` en CI (con menos muestras Monte Carlo, para que sea rapido) actua como una prueba de humo de extremo a extremo: confirma que el CLI, matplotlib en modo sin pantalla (`Agg`) y la escritura de archivos funcionan igual en un entorno limpio, no solo en la maquina del autor. |

## 3. Preguntas de entrevista tecnica

**1. ¿Que es el deslizamiento en un motor de induccion y por que es necesario para que exista par?**

El deslizamiento `s = (n_sync - n_r) / n_sync` mide que tan atras va el rotor respecto al campo
giratorio del estator. Es necesario porque el par se produce por induccion electromagnetica: solo
si hay movimiento relativo entre el campo del estator y las barras del rotor se induce una fem, y
por lo tanto una corriente, en el rotor. Si el rotor girara exactamente a la velocidad sincrona
(`s = 0`), no habria variacion de flujo relativa, no se induciria corriente, y el par seria cero
— por eso un motor de induccion jamas alcanza exactamente la velocidad sincrona en vacio ideal ni
bajo carga.

**2. ¿Por que el par electromagnetico es proporcional al cuadrado del voltaje aplicado?**

Porque el par depende de la potencia transferida al entrehierro, que a su vez depende del
cuadrado de la corriente del rotor, y la corriente del rotor es proporcional al voltaje de
Thevenin aplicado (circuito lineal). Como `V_th` es proporcional al voltaje de alimentacion, y el
par depende de `|V_th|^2`, reducir el voltaje de alimentacion a la mitad reduce el par disponible
a un cuarto, a igual deslizamiento. Esta es la razon fisica por la que una caida de tension en la
red afecta mucho mas fuerte al par de un motor de induccion que a su corriente.

**3. ¿Que es el par de ruptura (breakdown torque) y por que ocurre a un deslizamiento
especifico y no en el arranque?**

Es el maximo valor posible de `T(s)` en toda la curva par-deslizamiento. Ocurre en
`s_max = R2 / sqrt(R_th^2 + (X_th+X2)^2)` porque la ecuacion de par es el resultado de dos efectos
que compiten: al aumentar el deslizamiento, `R2/s` disminuye (mas corriente de rotor, mas par),
pero tambien cambia el angulo de la impedancia total (menos factor de potencia efectivo). El
punto donde la derivada de `T` respecto a `s` se anula es un balance entre esos dos efectos, y en
general no coincide con `s=1` (arranque) salvo que `R2` sea inusualmente grande respecto a las
reactancias — por eso el par de arranque y el par de ruptura son, en general, numeros distintos.

**4. ¿Por que un variador escalar mantiene aproximadamente constante la relacion V/Hz?**

Porque el flujo en el entrehierro es aproximadamente proporcional a `V/f`. Si la frecuencia sube
sin subir el voltaje en la misma proporcion, el flujo cae, y como el par maximo disponible es
proporcional al cuadrado del flujo (via `V_th`), el motor pierde capacidad de par a medida que
sube de velocidad. Mantener `V/f` constante mantiene el flujo — y por lo tanto el par maximo
disponible — aproximadamente constante en todo el rango de velocidad por debajo de la frecuencia
nominal.

**5. ¿Que pasa con el par disponible por encima de la frecuencia nominal del motor?**

El variador no puede entregar mas voltaje que el nominal (limitado por el bus DC / la tension de
red rectificada), asi que por encima de la frecuencia nominal el voltaje se satura en su valor
maximo mientras la frecuencia sigue subiendo. Esto hace que `V/f` — y por lo tanto el flujo y el
par maximo disponible — caigan progresivamente. Esta region se llama de **debilitamiento de
campo** (*field weakening*) y es la razon por la que los variadores tienen un limite practico de
sobre-velocidad util.

**6. ¿Por que se agrega un "boost" de voltaje a baja frecuencia en el control V/Hz?**

Porque a frecuencias muy bajas el voltaje nominal segun `V/f` es muy pequeno, y una fraccion
proporcionalmente mayor de ese voltaje se pierde en la caida resistiva `I*R1` del devanado del
estator en vez de convertirse en flujo util. Sin compensacion, el motor pierde par de arranque
justamente a las frecuencias bajas donde mas se necesita (arranque bajo carga). El boost anade un
voltaje minimo fijo para compensar esa caida resistiva y recuperar par a baja velocidad, a costa
de un pequeno aumento de corriente/calentamiento en vacio.

**7. ¿Cual es la diferencia principal entre control escalar (V/Hz) y control vectorial (FOC) de
un motor de induccion, y cuando se prefiere cada uno?**

El control escalar regula solo la magnitud del voltaje en funcion de la frecuencia, en lazo
abierto, sin conocer el angulo instantaneo del flujo del rotor; es simple, no requiere sensor de
posicion/velocidad, y es adecuado para aplicaciones donde no se necesita buena dinamica ni par
pleno a muy baja velocidad (bombas, ventiladores, cintas transportadoras). El control vectorial
(orientado a campo) controla por separado, en tiempo real y en ejes d-q sincronizados con el
flujo del rotor, la componente de corriente que produce flujo y la que produce par, tipicamente
con un observador o sensor de velocidad; logra mejor respuesta dinamica y par pleno incluso a
velocidad cero, pero es mas complejo de implementar y de sintonizar. El banco LabVolt/FESTO usado
en la tesis de este autor implementa control escalar.

**8. ¿Que es el efecto piel en el rotor de un motor de induccion y por que este modelo lo
ignora?**

A altas frecuencias de la corriente inducida en el rotor (que es proporcional al deslizamiento:
`f_rotor = s * f_estator`), la corriente tiende a concentrarse cerca de la superficie de las
barras del rotor en vez de distribuirse uniformemente en su seccion transversal, lo cual aumenta
la resistencia efectiva `R2` a alto deslizamiento (por ejemplo, en el arranque, donde
`f_rotor = f_estator`). Este modelo trata `R2` como constante con el deslizamiento por
simplicidad — es una aproximacion razonable cerca del punto de operacion nominal (deslizamiento
bajo), pero subestima el efecto en el arranque; se documenta explicitamente como limitacion
conocida en el README.

**9. ¿Por que se usa el equivalente de Thevenin en vez de resolver el circuito completo del
motor para cada punto de la curva par-deslizamiento?**

Porque el deslizamiento solo afecta la impedancia de la rama del rotor (`R2/s + jX2`); el resto
del circuito (fuente, `R1+jX1`, rama de magnetizacion `jXm`) es lineal e independiente de `s`. Al
reducir ese resto a un equivalente de Thevenin una sola vez, calcular el par para cualquier
deslizamiento se convierte en evaluar una formula cerrada en vez de resolver un circuito completo
con numeros complejos en cada punto — mas simple, mas rapido, y ademas permite obtener una
formula cerrada para el par de ruptura (derivando respecto a `s`).

**10. En este repositorio se hizo un estudio Monte Carlo de sensibilidad del par de ruptura a la
tolerancia de fabricacion del rotor. ¿Como se planteo y que conclusion practica deja?**

Se modelaron `R2` y `X2` del rotor como variables aleatorias uniformes dentro de ±10% de su valor
nominal (tolerancia tipica de fabricacion/variacion termica), usando un generador de NumPy con
semilla fija (`seed=42`) para que el estudio sea exactamente reproducible. Se propagaron 5000
muestras a traves de la formula cerrada de par de ruptura a 60 Hz, obteniendo una media de
47.198 N·m con una desviacion estandar de 0.526 N·m (coeficiente de variacion ≈1.11%). La
conclusion practica es que, para este circuito equivalente, una tolerancia de fabricacion de
±10% en los parametros del rotor produce una variacion mucho menor (~1%) en el par de ruptura —
es decir, el par de ruptura es relativamente insensible a la incertidumbre tipica de fabricacion
del rotor, lo cual es una forma cuantitativa (no solo cualitativa) de justificar que un diseno de
variador no necesita margen adicional de par solo por esta fuente de incertidumbre.
