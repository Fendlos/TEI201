// =========================================================================
// SEGUIDOR DE LINEA PD - TOLERANCIA A HUECOS + MAX_SPEED AJUSTABLE
//   + MODULO DE TESTING Y TELEMETRIA
//   + COMUNICACION INALAMBRICA: WiFi (dashboard web) o Bluetooth
// S1 (Izq) = P13 | S2 = P33 | S3 (Centro) = P14 | S4 = P27 | S5 (Der) = P26
// Sensores: LOW (0) = Negro | HIGH (1) = Blanco
//
// MODOS DE COMUNICACION (elegir UNO en MODO_COM, mas abajo):
//   MODO_USB  : CSV y comandos por cable (Serial Monitor).
//   MODO_BT   : el mismo CSV y los mismos comandos por Bluetooth clasico
//               (app "Serial Bluetooth Terminal" o COM virtual en el PC).
//               Solo funciona en ESP32 clasico (no S3 / C3).
//   MODO_WIFI : el ESP32 crea su propia red WiFi y sirve un DASHBOARD WEB:
//               1) Conectar el celular/notebook a la red  Robot-TEI201
//                  (clave: tei201robot)
//               2) Abrir en el navegador  http://192.168.4.1
//               Muestra graficos en vivo, metricas, alertas, historial de
//               tests y permite descargar el CSV del ultimo test.
//
// COMANDOS (USB y Bluetooth: enviar una letra / WiFi: botones del dashboard):
//   g = iniciar un test nuevo      s = detener test y ver resumen
//   h = ayuda
//
// Las lineas que empiezan con '#' son comentarios/resumen (ignorar al cargar
// el CSV, ej. pandas.read_csv(..., comment='#')).
// =========================================================================
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
// --- MODO DE COMUNICACION ---
#define MODO_USB   0
#define MODO_BT    1
#define MODO_WIFI  2
#define MODO_COM   MODO_WIFI   // <--- CAMBIAR AQUI

#if MODO_COM == MODO_BT
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
#elif MODO_COM == MODO_WIFI
  #include <WiFi.h>
  #include <WebServer.h>
  WebServer server(80);
#endif

// --- SENSORES ---
const int SENSORS[5] = {13, 33, 14, 27, 26};
const float WEIGHTS[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};

// --- DRIVER L298N (Mapeo a ruedas físicas) ---
const int MOT_IZQ_FWD = 22;
const int MOT_IZQ_REV = 23;
const int MOT_DER_FWD = 18;
const int MOT_DER_REV = 19;

// --- CONFIGURACION PWM ---
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;
const int CH_IZQ_F = 0;
const int CH_IZQ_R = 1;
const int CH_DER_F = 2;
const int CH_DER_R = 3;

// =========================================================================
// --- PARAMETROS AJUSTABLES POR EL USUARIO (CONTROL) ---
// =========================================================================
const int BASE_SPEED = 170;  // Velocidad crucero en rectas
const int MAX_SPEED  = 200;  // <--- VELOCIDAD MAXIMA (Puedes cambiar este valor)
const int MAX_REV    = -120; // Tope de retroceso en giro cerrado

float Kp = 40.0;             // Fuerza de corrección proporcional
float Kd = 22.0;             // Amortiguación derivativa (antivadeo)

// =========================================================================
// --- PARAMETROS DE TESTING ---
// =========================================================================
const bool  AUTO_INICIO        = false;  // true: inicia un test solo al encender (util solo con USB)
const unsigned long CUENTA_REGRESIVA_MS = 3000; // Espera antes de partir (tras 'g' o al auto-iniciar)
const bool  LOG_ACTIVADO       = true;   // true: tambien imprime el CSV por USB/Bluetooth
const int   LOG_CADA_N         = 2;      // Registra 1 de cada N ciclos (2 -> ~50 Hz)

// Criterios de fin / falla de un test
const unsigned long LOST_TIMEOUT_MS = 1500;   // Blanco total > este tiempo = LINEA_PERDIDA (0 = desactivar)
const unsigned long TEST_MAX_MS     = 120000; // Duracion maxima de un test (0 = sin limite)

// Definiciones de las metricas
const float UMBRAL_CENTRADO    = 0.5;  // |error| <= 0.5 se considera "centrado en la linea"
const float BANDA_MUERTA_CRUCE = 0.25; // Banda para contar cruces de signo (zigzag)
const float LARGO_PISTA_CM     = 0.0;  // Largo del recorrido en cm (medir con huincha). 0 = no calcula cm/s

// Umbrales de ALERTA del dashboard (ajustar con los datos de sus pruebas)
const int ALERTA_PCT_CENTRADO_MIN = 80;  // % centrado minimo aceptable
const int ALERTA_PERIODO_MAX_MS   = 25;  // Periodo maximo del lazo aceptable (ms)
const int ALERTA_BLANCO_MS        = 500; // Blanco total maximo aceptable (ms)

// Credenciales de la red WiFi que crea el robot (MODO_WIFI)
#define WIFI_SSID "Robot-TEI201"
#define WIFI_PASS "tei201robot"          // minimo 8 caracteres
// Nombre Bluetooth (MODO_BT)
#define BT_NOMBRE "Robot-TEI201"
// =========================================================================

// =========================================================================
// TIPOS DE DATOS
// =========================================================================
enum Resultado { R_META, R_LINEA_PERDIDA, R_TIMEOUT, R_MANUAL };
const char* NOMBRE_RESULTADO[] = {"META", "LINEA_PERDIDA", "TIMEOUT", "MANUAL"};

// Metricas acumuladas durante un test
struct Metricas {
  uint32_t tInicio;          // millis() al iniciar el test
  uint32_t tFin;             // millis() al terminar el test
  uint32_t ciclos;           // Iteraciones totales del lazo de control
  uint32_t ciclosEnLinea;    // Ciclos con al menos 1 sensor sobre negro
  uint32_t ciclosCentrado;   // Ciclos en linea con |error| <= UMBRAL_CENTRADO
  float    sumAbsErr;        // Suma de |error| (para MAE)
  float    sumSqErr;         // Suma de error^2 (para RMS)
  float    maxAbsErr;        // Error maximo observado
  uint32_t cruces;           // Cruces de signo del error (oscilacion)
  uint32_t eventosBlanco;    // Cantidad de veces que se pasa a blanco total
  uint32_t msBlancoTotal;    // Tiempo acumulado en blanco total
  uint32_t msBlancoMax;      // Blanco total mas largo
  uint32_t activ[5];         // Ciclos en que cada sensor vio negro (salud de sensores)
  uint32_t ultimoLoopUs;     // micros() del ciclo anterior
  uint64_t sumPeriodoUs;     // Suma de periodos del lazo
  uint32_t maxPeriodoUs;     // Periodo maximo (peor latencia del lazo)
  float    sumVel;           // Suma de velocidad promedio de ruedas (PWM)
};

// Resumen calculado de un test (final o en vivo)
struct Resumen {
  uint16_t id;
  uint8_t  resultado;
  float    durS, periodoMs, maxPerMs, pctLinea, pctCentro;
  float    mae, rms, maxErr, crucesS, velPwm, velCmS;
  uint32_t eventosBlanco, msBlancoTotal, msBlancoMax;
  float    pctSensor[5];
};

// Una muestra de telemetria (para el grafico y el CSV descargable)
struct Muestra {
  uint32_t t;        // ms desde el inicio del test
  int16_t  err100;   // error * 100
  int16_t  corr;     // correccion PD
  int16_t  izq;      // velocidad rueda izquierda
  int16_t  der;      // velocidad rueda derecha
  uint16_t perUs;    // periodo del lazo (us)
  uint8_t  mask;     // bit i = sensor S(i+1) sobre negro
};

// Salida de texto: USB siempre, y Bluetooth si hay un cliente conectado
class Salida : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *buf, size_t n) override {
    Serial.write(buf, n);
    #if MODO_COM == MODO_BT
      if (SerialBT.hasClient()) SerialBT.write(buf, n);
    #endif
    return n;
  }
};
Salida out;

// =========================================================================
// ESTADO GLOBAL
// =========================================================================
// El lazo de control corre en el nucleo 1 y el servidor web en el nucleo 0:
// los datos compartidos se protegen con una seccion critica breve.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define LOCK()   portENTER_CRITICAL(&mux)
#define UNLOCK() portEXIT_CRITICAL(&mux)

// --- Control ---
float error = 0.0;
float lastError = 0.0;

// --- Test ---
Metricas m = {};
volatile bool     running = false;   // true mientras hay un test en curso
volatile uint16_t testId  = 0;
volatile char     cmdWeb  = 0;       // comando recibido desde el dashboard
bool     enBlanco     = false;       // true mientras los 5 sensores ven blanco
uint32_t inicioBlanco = 0;
int      ultimoSigno  = 0;           // Ultimo signo del error fuera de la banda muerta
uint32_t contadorLog  = 0;

// --- Historial de resultados (ultimos tests) ---
const int HIST_N = 10;
Resumen historial[HIST_N];
int histCount = 0;

// --- Buffer circular de muestras (solo WiFi) ---
volatile uint32_t totalMuestras = 0;   // muestras registradas en el test actual
#if MODO_COM == MODO_WIFI
const int BUF_N = 3000;                // ~60 s a 50 Hz
Muestra bufMuestras[BUF_N];
#endif

// =========================================================================
// MOTORES
// =========================================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // Limitamos estrictamente al MAX_SPEED definido arriba
  leftSpeed  = constrain(leftSpeed,  MAX_REV, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MAX_REV, MAX_SPEED);

  int izq_fwd = (leftSpeed >= 0)  ? leftSpeed  : 0;
  int izq_rev = (leftSpeed < 0)   ? -leftSpeed : 0;
  int der_fwd = (rightSpeed >= 0) ? rightSpeed : 0;
  int der_rev = (rightSpeed < 0)  ? -rightSpeed : 0;

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(MOT_IZQ_FWD, izq_fwd);
    ledcWrite(MOT_IZQ_REV, izq_rev);
    ledcWrite(MOT_DER_FWD, der_fwd);
    ledcWrite(MOT_DER_REV, der_rev);
  #else
    ledcWrite(CH_IZQ_F, izq_fwd);
    ledcWrite(CH_IZQ_R, izq_rev);
    ledcWrite(CH_DER_F, der_fwd);
    ledcWrite(CH_DER_R, der_rev);
  #endif
}

// =========================================================================
// METRICAS
// =========================================================================
// Convierte las metricas acumuladas en valores finales (sirve para el
// resumen del test y para los indicadores en vivo del dashboard)
void calcularResumen(const Metricas& mm, uint32_t durMs, Resumen& r) {
  float nCiclos  = (mm.ciclos > 0) ? (float)mm.ciclos : 1.0;
  float nEnLinea = (mm.ciclosEnLinea > 0) ? (float)mm.ciclosEnLinea : 1.0;

  r.durS      = durMs / 1000.0;
  r.periodoMs = (mm.ciclos > 1) ? (mm.sumPeriodoUs / (float)(mm.ciclos - 1)) / 1000.0 : 0.0;
  r.maxPerMs  = mm.maxPeriodoUs / 1000.0;
  r.pctLinea  = 100.0 * mm.ciclosEnLinea / nCiclos;
  r.pctCentro = 100.0 * mm.ciclosCentrado / nEnLinea;
  r.mae       = mm.sumAbsErr / nEnLinea;
  r.rms       = sqrt(mm.sumSqErr / nEnLinea);
  r.maxErr    = mm.maxAbsErr;
  r.crucesS   = (r.durS > 0) ? mm.cruces / r.durS : 0.0;
  r.velPwm    = mm.sumVel / nCiclos;
  r.velCmS    = 0.0;
  r.eventosBlanco = mm.eventosBlanco;
  r.msBlancoTotal = mm.msBlancoTotal;
  r.msBlancoMax   = mm.msBlancoMax;
  for (int i = 0; i < 5; i++) r.pctSensor[i] = 100.0 * mm.activ[i] / nCiclos;
}

// Actualiza las metricas de error con este ciclo (solo si la linea es visible)
void actualizarMetricas(float err, int nNegro) {
  if (nNegro == 0) return;  // En blanco total el error es "arrastrado": no se cuenta

  float absErr = fabs(err);
  m.ciclosEnLinea++;
  if (absErr <= UMBRAL_CENTRADO) m.ciclosCentrado++;
  m.sumAbsErr += absErr;
  m.sumSqErr  += err * err;
  if (absErr > m.maxAbsErr) m.maxAbsErr = absErr;

  // Cruces de signo con banda muerta: mide cuanto zigzaguea el robot
  int signo = (err > BANDA_MUERTA_CRUCE) ? 1 : (err < -BANDA_MUERTA_CRUCE) ? -1 : 0;
  if (signo != 0) {
    if (ultimoSigno != 0 && signo != ultimoSigno) m.cruces++;
    ultimoSigno = signo;
  }
}

// =========================================================================
// TESTING: inicio, fin y comandos
// =========================================================================
void imprimirAyuda() {
  out.println("# Comandos: g = iniciar test | s = detener y ver resumen | h = ayuda");
}

// Prepara y arranca un test nuevo (reinicia todas las metricas)
void iniciarTest() {
  LOCK();
  m = Metricas();          // Pone todas las metricas en cero
  error = 0.0;
  lastError = 0.0;
  enBlanco = false;
  ultimoSigno = 0;
  contadorLog = 0;
  totalMuestras = 0;
  testId = testId + 1;
  m.tInicio = millis();
  running = true;
  UNLOCK();
  out.printf("# TEST %u INICIO (BASE=%d MAX=%d Kp=%.1f Kd=%.1f)\n",
             (unsigned)testId, BASE_SPEED, MAX_SPEED, Kp, Kd);
  out.println("#COLUMNAS: test,t_ms,sensores,error,correccion,vel_izq,vel_der,periodo_us");
}

// Detiene los motores, cierra el test e imprime metricas
void finalizarTest(Resultado res) {
  driveMotors(0, 0);
  uint32_t ahora = millis();
  Resumen r;

  LOCK();
  running = false;
  // Si termino mientras estaba en blanco, cerrar ese evento
  if (enBlanco) {
    uint32_t dur = ahora - inicioBlanco;
    m.msBlancoTotal += dur;
    if (dur > m.msBlancoMax) m.msBlancoMax = dur;
    enBlanco = false;
  }
  m.tFin = ahora;
  calcularResumen(m, ahora - m.tInicio, r);
  r.id = testId;
  r.resultado = (uint8_t)res;
  if (LARGO_PISTA_CM > 0 && res == R_META && r.durS > 0) r.velCmS = LARGO_PISTA_CM / r.durS;

  // Guardar en el historial (los mas antiguos salen primero)
  if (histCount == HIST_N) {
    for (int i = 1; i < HIST_N; i++) historial[i - 1] = historial[i];
    histCount = HIST_N - 1;
  }
  historial[histCount++] = r;
  UNLOCK();

  out.println("# ---------------- RESUMEN DEL TEST ----------------");
  out.printf("# Test #%u  Resultado: %s\n", (unsigned)r.id, NOMBRE_RESULTADO[res]);
  out.printf("# Duracion: %.2f s\n", r.durS);
  out.printf("# Periodo del lazo: medio %.2f ms | maximo %.2f ms  (latencia de control)\n", r.periodoMs, r.maxPerMs);
  out.printf("# Tiempo sobre la linea: %.1f %%\n", r.pctLinea);
  out.printf("# Tiempo centrado (|e|<=%.1f): %.1f %% (de los ciclos en linea)\n", UMBRAL_CENTRADO, r.pctCentro);
  out.printf("# Error: MAE %.3f | RMS %.3f | maximo %.2f\n", r.mae, r.rms, r.maxErr);
  out.printf("# Oscilacion: %.2f cruces/s\n", r.crucesS);
  out.printf("# Blanco total: %lu eventos | acumulado %lu ms | maximo %lu ms\n",
             (unsigned long)r.eventosBlanco, (unsigned long)r.msBlancoTotal, (unsigned long)r.msBlancoMax);
  out.printf("# Velocidad media: %.1f PWM", r.velPwm);
  if (r.velCmS > 0) out.printf(" | %.1f cm/s", r.velCmS);
  out.println();
  out.print("# Activacion por sensor S1..S5 (%): ");
  for (int i = 0; i < 5; i++) {
    out.print(r.pctSensor[i], 1);
    out.print(i < 4 ? " | " : "\n");
  }
  // Linea de datos para pegar en Excel / cargar en Python
  out.printf("#RESUMEN,%u,%s,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.2f,%.2f,%lu,%lu,%lu,%.1f,%.1f\n",
             (unsigned)r.id, NOMBRE_RESULTADO[res], r.durS, r.periodoMs, r.maxPerMs, r.pctLinea, r.pctCentro,
             r.mae, r.rms, r.maxErr, r.crucesS, (unsigned long)r.eventosBlanco,
             (unsigned long)r.msBlancoTotal, (unsigned long)r.msBlancoMax, r.velPwm, r.velCmS);
  out.println("# --------------------------------------------------");
  out.println("# Listo. Envia 'g' para un nuevo test.");
}

// Cuenta regresiva (para dejar el robot en la pista) y luego parte
void cuentaRegresivaEIniciar() {
  if (CUENTA_REGRESIVA_MS > 0) {
    out.printf("# Partiendo en %lu ms...\n", CUENTA_REGRESIVA_MS);
    delay(CUENTA_REGRESIVA_MS);
  }
  iniciarTest();
}

// Devuelve el siguiente caracter recibido (dashboard, USB o Bluetooth), 0 si no hay
char leerCaracter() {
  if (cmdWeb) {
    char c = cmdWeb;
    cmdWeb = 0;
    return c;
  }
  if (Serial.available()) return Serial.read();
  #if MODO_COM == MODO_BT
    if (SerialBT.available()) return SerialBT.read();
  #endif
  return 0;
}

void leerComandos() {
  char c;
  while ((c = leerCaracter()) != 0) {
    if (c == 'g' || c == 'G') {
      if (!running) cuentaRegresivaEIniciar();
    } else if (c == 's' || c == 'S') {
      if (running) finalizarTest(R_MANUAL);
    } else if (c == 'h' || c == 'H') {
      imprimirAyuda();
    }
  }
}

// =========================================================================
// DASHBOARD WEB (solo MODO_WIFI)
// =========================================================================
#if MODO_COM == MODO_WIFI

const int MAX_ENVIO = 120;   // Maximo de muestras por respuesta del dashboard

// Pagina del dashboard (HTML + CSS + JS, sin librerias externas: funciona sin internet)
const char PAGINA[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot TEI201</title>
<style>
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,sans-serif;background:#f4f7f8;color:#12202b}
header{display:flex;justify-content:space-between;align-items:center;padding:12px 16px;background:#0f766e;color:#fff}
h1{font-size:18px;margin:0}
h2{font-size:15px;margin:0 0 8px}
.pill{padding:4px 12px;border-radius:99px;font-weight:600;background:#64748b}
.pill.on{background:#16a34a}
main{padding:12px;max-width:980px;margin:auto}
.barra{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
button,.btn{padding:10px 16px;border:0;border-radius:8px;font-size:15px;font-weight:600;cursor:pointer;color:#fff;text-decoration:none}
#bg{background:#16a34a}#bs{background:#dc2626}.btn{background:#0f766e}
.card{background:#fff;border-radius:10px;padding:12px;box-shadow:0 1px 3px #0002;margin-bottom:12px}
.sens{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.led{width:34px;height:34px;border-radius:50%;border:2px solid #12202b;background:#fff}
.led.n{background:#12202b}
.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-bottom:12px}
.kpi{background:#fff;border-radius:10px;padding:10px;box-shadow:0 1px 3px #0002}
.kpi b{display:block;font-size:22px}.kpi span{font-size:12px;color:#475569}
canvas{width:100%;height:170px;background:#fff;border-radius:10px;box-shadow:0 1px 3px #0002;margin-bottom:12px}
.al{padding:8px 12px;border-radius:8px;margin-bottom:6px;font-weight:600}
.bad{background:#fee2e2;color:#991b1b}.ok{background:#dcfce7;color:#166534}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:5px 6px;text-align:right;border-bottom:1px solid #e2e8f0}
th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){text-align:left}
.leyenda{font-size:12px;color:#475569;margin:-6px 0 12px}
</style></head><body>
<header><h1>Seguidor de línea · Dashboard</h1><span id="estado" class="pill">Conectando...</span></header>
<main>
<div class="barra">
<button id="bg">▶ Iniciar test</button>
<button id="bs">■ Detener</button>
<a class="btn" href="/api/csv" download="test_robot.csv">⬇ CSV último test</a>
</div>
<div class="card"><h2>Sensores (S1 izq ... S5 der) y error actual</h2>
<div class="sens"><div id="s0" class="led"></div><div id="s1" class="led"></div><div id="s2" class="led"></div><div id="s3" class="led"></div><div id="s4" class="led"></div>
<span>Error: <b id="err">-</b></span></div></div>
<div id="alertas"></div>
<div class="kpis">
<div class="kpi"><b id="k_dur">-</b><span>Duración</span></div>
<div class="kpi"><b id="k_pl">-</b><span>Tiempo sobre la línea</span></div>
<div class="kpi"><b id="k_pc">-</b><span>Tiempo centrado</span></div>
<div class="kpi"><b id="k_mae">-</b><span>Error medio (MAE)</span></div>
<div class="kpi"><b id="k_rms">-</b><span>Error RMS</span></div>
<div class="kpi"><b id="k_emax">-</b><span>Error máximo</span></div>
<div class="kpi"><b id="k_cr">-</b><span>Oscilación (cruces)</span></div>
<div class="kpi"><b id="k_eb">-</b><span>Eventos en blanco</span></div>
<div class="kpi"><b id="k_bmax">-</b><span>Blanco más largo</span></div>
<div class="kpi"><b id="k_per">-</b><span>Periodo lazo medio</span></div>
<div class="kpi"><b id="k_pmax">-</b><span>Periodo lazo máximo</span></div>
<div class="kpi"><b id="k_vel">-</b><span>Velocidad media</span></div>
</div>
<div class="leyenda">Activación por sensor S1..S5 (%): <b id="k_ps">-</b></div>
<h2>Error de posición (últimos 10 s) · banda verde = centrado</h2>
<canvas id="c1" width="800" height="240"></canvas>
<h2>Velocidad de ruedas (PWM): azul = izquierda, naranja = derecha</h2>
<canvas id="c2" width="800" height="240"></canvas>
<div class="card"><h2>Historial de tests (últimos 10) · <span id="tasa"></span></h2>
<table><thead><tr><th>Test</th><th>Resultado</th><th>Dur (s)</th><th>% centrado</th><th>MAE</th><th>Cruces/s</th><th>Blanco máx (ms)</th><th>Lazo máx (ms)</th></tr></thead>
<tbody id="hist"></tbody></table></div>
</main>
<script>
const $=id=>document.getElementById(id);
const VENT=10000;
let S=[],desde=0,tid=-1,cfg={uc:.5,vmin:-120,vmax:200,cm:80,pm:25,bm:500};
function f(v,d){return Number(v).toFixed(d);}
function cmd(c){fetch('/api/cmd?c='+c).catch(()=>{});}
$('bg').onclick=()=>cmd('g');
$('bs').onclick=()=>cmd('s');
function graf(id,idx,cols,ymin,ymax,bandas){
  const cv=$(id),c=cv.getContext('2d'),W=cv.width,H=cv.height;
  c.clearRect(0,0,W,H);
  const Y=v=>H-(v-ymin)/(ymax-ymin)*H;
  bandas.forEach(b=>{c.fillStyle='rgba(22,163,74,.15)';c.fillRect(0,Y(b[1]),W,Y(b[0])-Y(b[1]));});
  c.strokeStyle='#cbd5e1';c.lineWidth=1;c.beginPath();c.moveTo(0,Y(0));c.lineTo(W,Y(0));c.stroke();
  c.fillStyle='#64748b';c.font='18px sans-serif';c.fillText(ymax,4,18);c.fillText(ymin,4,H-6);
  if(S.length<2)return;
  const t1=S[S.length-1][0],t0=t1-VENT;
  idx.forEach((k,n)=>{
    c.strokeStyle=cols[n];c.lineWidth=2.5;c.beginPath();let ini=false;
    for(const s of S){
      if(s[0]<t0)continue;
      const x=(s[0]-t0)/VENT*W,y=Y(k===2?s[2]/100:s[k]);
      if(!ini){c.moveTo(x,y);ini=true;}else c.lineTo(x,y);
    }
    c.stroke();
  });
}
function pinta(j){
  const m=j.m,e=$('estado');
  e.textContent=j.run?('TEST #'+j.id+' EN CURSO'):'DETENIDO';
  e.className='pill'+(j.run?' on':'');
  const u=S.length?S[S.length-1]:null;
  for(let i=0;i<5;i++)$('s'+i).className='led'+((u&&((u[1]>>i)&1))?' n':'');
  $('err').textContent=u?f(u[2]/100,2):'-';
  const k={dur:f(m.dur,1)+' s',pl:f(m.pl,1)+' %',pc:f(m.pc,1)+' %',mae:f(m.mae,3),rms:f(m.rms,3),
    emax:f(m.emax,2),cr:f(m.cr,2)+' /s',eb:m.eb,bmax:m.bmax+' ms',per:f(m.per,1)+' ms',
    pmax:f(m.pmax,1)+' ms',vel:f(m.vel,0)+' PWM'};
  for(const x in k)$('k_'+x).textContent=k[x];
  $('k_ps').textContent=m.ps.map(v=>f(v,0)).join(' | ');
  graf('c1',[2],['#0f766e'],-2,2,[[-cfg.uc,cfg.uc]]);
  graf('c2',[3,4],['#2563eb','#ea580c'],cfg.vmin,cfg.vmax,[]);
  const A=[],h=j.h;
  if(m.dur>3&&m.pc<cfg.cm)A.push('Centrado bajo: '+f(m.pc,0)+' % (mínimo '+cfg.cm+' %)');
  if(m.pmax>cfg.pm)A.push('Periodo máximo del lazo alto: '+f(m.pmax,1)+' ms (límite '+cfg.pm+' ms)');
  if(m.bmax>cfg.bm)A.push('Blanco prolongado: '+m.bmax+' ms (límite '+cfg.bm+' ms)');
  if(!j.run&&h.length&&h[h.length-1][1]==='LINEA_PERDIDA')A.push('El último test terminó por LÍNEA PERDIDA');
  $('alertas').innerHTML=A.length?A.map(a=>'<div class="al bad">⚠ '+a+'</div>').join(''):'<div class="al ok">Sin alertas</div>';
  let ok=0;
  const filas=h.slice().reverse().map(r=>{
    if(r[1]==='META')ok++;
    return '<tr><td>#'+r[0]+'</td><td>'+r[1]+'</td><td>'+f(r[2],1)+'</td><td>'+f(r[3],0)+'</td><td>'+f(r[5],3)+'</td><td>'+f(r[7],2)+'</td><td>'+r[9]+'</td><td>'+f(r[11],1)+'</td></tr>';
  });
  $('hist').innerHTML=filas.join('');
  $('tasa').textContent=h.length?('Éxito (META): '+ok+'/'+h.length+' = '+f(100*ok/h.length,0)+' %'):'Aún sin tests';
}
async function tick(){
  try{
    const j=await(await fetch('/api/estado?desde='+desde)).json();
    cfg=j.cfg;
    if(j.id!==tid){tid=j.id;S=[];desde=0;return;}
    S=S.concat(j.s);
    if(S.length>600)S=S.slice(-600);
    desde=j.desde+j.s.length;
    pinta(j);
  }catch(e){
    $('estado').textContent='Sin conexión';$('estado').className='pill';
  }
}
setInterval(tick,300);
</script></body></html>)rawliteral";

// Agrega texto formateado a un String (para armar el JSON)
void agregar(String &s, const char *fmt, ...) {
  char b[300];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  s += b;
}

void maskATexto(uint8_t mask, char *txt) {
  for (int i = 0; i < 5; i++) txt[i] = ((mask >> i) & 1) ? '1' : '0';
  txt[5] = 0;
}

void handleRoot() {
  server.send_P(200, "text/html", PAGINA);
}

// Comando desde los botones del dashboard (g = iniciar, s = detener)
void handleCmd() {
  if (server.hasArg("c") && server.arg("c").length() > 0) {
    cmdWeb = server.arg("c")[0];
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}

// Estado en vivo: muestras nuevas, metricas, configuracion de alertas e historial (JSON)
void handleEstado() {
  uint32_t desde = server.hasArg("desde") ? (uint32_t)server.arg("desde").toInt() : 0;

  static Muestra tmp[MAX_ENVIO];   // solo lo usa la tarea web
  Metricas mc;
  Resumen  h[HIST_N];
  uint32_t total, durMs;
  bool corriendo;
  uint16_t id;
  int nh, n;

  LOCK();
  total = totalMuestras;
  corriendo = running;
  id = testId;
  mc = m;
  durMs = corriendo ? (millis() - m.tInicio) : (m.tFin - m.tInicio);
  nh = histCount;
  for (int i = 0; i < nh; i++) h[i] = historial[i];
  if (desde > total) desde = 0;
  if (total > (uint32_t)BUF_N && desde < total - BUF_N) desde = total - BUF_N;
  if (total - desde > (uint32_t)MAX_ENVIO) desde = total - MAX_ENVIO;
  n = (int)(total - desde);
  for (int k = 0; k < n; k++) tmp[k] = bufMuestras[(desde + k) % BUF_N];
  UNLOCK();

  Resumen rl;
  calcularResumen(mc, durMs, rl);

  String js;
  js.reserve(4096);
  agregar(js, "{\"run\":%d,\"id\":%u,\"total\":%lu,\"desde\":%lu,", corriendo ? 1 : 0, (unsigned)id,
          (unsigned long)total, (unsigned long)desde);
  agregar(js, "\"cfg\":{\"uc\":%.2f,\"vmin\":%d,\"vmax\":%d,\"cm\":%d,\"pm\":%d,\"bm\":%d},",
          UMBRAL_CENTRADO, MAX_REV, MAX_SPEED, ALERTA_PCT_CENTRADO_MIN, ALERTA_PERIODO_MAX_MS, ALERTA_BLANCO_MS);
  agregar(js, "\"m\":{\"dur\":%.2f,\"pl\":%.1f,\"pc\":%.1f,\"mae\":%.3f,\"rms\":%.3f,\"emax\":%.2f,\"cr\":%.2f,"
              "\"eb\":%lu,\"bmax\":%lu,\"per\":%.2f,\"pmax\":%.2f,\"vel\":%.1f,\"ps\":[%.1f,%.1f,%.1f,%.1f,%.1f]},",
          rl.durS, rl.pctLinea, rl.pctCentro, rl.mae, rl.rms, rl.maxErr, rl.crucesS,
          (unsigned long)rl.eventosBlanco, (unsigned long)rl.msBlancoMax, rl.periodoMs, rl.maxPerMs, rl.velPwm,
          rl.pctSensor[0], rl.pctSensor[1], rl.pctSensor[2], rl.pctSensor[3], rl.pctSensor[4]);

  js += "\"s\":[";
  for (int k = 0; k < n; k++) {
    agregar(js, "%s[%lu,%u,%d,%d,%d,%u]", k ? "," : "", (unsigned long)tmp[k].t, (unsigned)tmp[k].mask,
            (int)tmp[k].err100, (int)tmp[k].izq, (int)tmp[k].der, (unsigned)tmp[k].perUs);
  }
  js += "],\"h\":[";
  for (int i = 0; i < nh; i++) {
    agregar(js, "%s[%u,\"%s\",%.1f,%.1f,%.1f,%.3f,%.3f,%.2f,%lu,%lu,%.2f,%.2f,%.1f]", i ? "," : "",
            (unsigned)h[i].id, NOMBRE_RESULTADO[h[i].resultado], h[i].durS, h[i].pctCentro, h[i].pctLinea,
            h[i].mae, h[i].rms, h[i].crucesS, (unsigned long)h[i].eventosBlanco, (unsigned long)h[i].msBlancoMax,
            h[i].periodoMs, h[i].maxPerMs, h[i].velPwm);
  }
  js += "]}";

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", js);
}

// Descarga el CSV del ultimo test (en bloques, para no agotar la memoria)
void handleCSV() {
  server.sendHeader("Content-Disposition", "attachment; filename=test_robot.csv");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("test,t_ms,sensores,error,correccion,vel_izq,vel_der,periodo_us\n");

  uint32_t total;
  uint16_t id;
  LOCK();
  total = totalMuestras;
  id = testId;
  UNLOCK();

  const int BLOQUE = 40;
  static Muestra blk[BLOQUE];
  char linea[80], txt[6];
  uint32_t i = (total > (uint32_t)BUF_N) ? total - BUF_N : 0;
  while (i < total) {
    int n = (total - i > (uint32_t)BLOQUE) ? BLOQUE : (int)(total - i);
    LOCK();
    for (int k = 0; k < n; k++) blk[k] = bufMuestras[(i + k) % BUF_N];
    UNLOCK();
    String chunk;
    chunk.reserve(n * 48);
    for (int k = 0; k < n; k++) {
      maskATexto(blk[k].mask, txt);
      snprintf(linea, sizeof(linea), "%u,%lu,%s,%.2f,%d,%d,%d,%u\n", (unsigned)id, (unsigned long)blk[k].t, txt,
               blk[k].err100 / 100.0, (int)blk[k].corr, (int)blk[k].izq, (int)blk[k].der, (unsigned)blk[k].perUs);
      chunk += linea;
    }
    server.sendContent(chunk);
    i += n;
  }
  server.sendContent("");  // fin de la respuesta
}

// Tarea del servidor web: corre en el nucleo 0 para no perturbar el lazo de control (nucleo 1)
void tareaWeb(void *param) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void iniciarWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // Baja los picos de corriente a un tercio
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  server.on("/", handleRoot);
  server.on("/api/estado", handleEstado);
  server.on("/api/cmd", handleCmd);
  server.on("/api/csv", handleCSV);
  server.begin();
  xTaskCreatePinnedToCore(tareaWeb, "web", 10240, NULL, 1, NULL, 0);
  out.printf("# WiFi: red '%s' (clave '%s') -> abrir http://%s\n", WIFI_SSID, WIFI_PASS,
             WiFi.softAPIP().toString().c_str());
}

#endif  // MODO_WIFI

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Desactiva el reinicio por caída de voltaje
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    pinMode(SENSORS[i], INPUT);
  }

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOT_IZQ_FWD, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_IZQ_REV, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_DER_FWD, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_DER_REV, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(CH_IZQ_F, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_IZQ_FWD, CH_IZQ_F);
    ledcSetup(CH_IZQ_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_IZQ_REV, CH_IZQ_R);
    ledcSetup(CH_DER_F, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_DER_FWD, CH_DER_F);
    ledcSetup(CH_DER_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_DER_REV, CH_DER_R);
  #endif

  driveMotors(0, 0);
  delay(1000);

  // Comunicacion inalambrica
  #if MODO_COM == MODO_BT
    SerialBT.begin(BT_NOMBRE);
  #elif MODO_COM == MODO_WIFI
    iniciarWiFi();
  #endif

  // Encabezados (por USB; por Bluetooth/WiFi se reimprimen al iniciar cada test)
  Serial.println("# SEGUIDOR DE LINEA - MODO TESTING");
  Serial.println("#RESUMEN_COLS,test,resultado,dur_s,periodo_medio_ms,periodo_max_ms,pct_en_linea,pct_centrado,mae,rms,err_max,cruces_por_s,eventos_blanco,blanco_total_ms,blanco_max_ms,vel_pwm,vel_cm_s");
  Serial.println("test,t_ms,sensores,error,correccion,vel_izq,vel_der,periodo_us");
  imprimirAyuda();

  if (AUTO_INICIO) {
    cuentaRegresivaEIniciar();
  } else {
    out.println("# Envia 'g' (o usa el boton del dashboard) para iniciar un test.");
  }
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  leerComandos();

  // Sin test en curso: motores detenidos, a la espera de 'g'
  if (!running) {
    driveMotors(0, 0);
    delay(10);
    return;
  }

  uint32_t ahora   = millis();
  uint32_t ahoraUs = micros();

  // Periodo real del lazo (latencia sensor -> motor entre ciclos consecutivos)
  uint32_t periodoUs = (m.ciclos > 0) ? (ahoraUs - m.ultimoLoopUs) : 0;

  // Fin por tiempo maximo
  if (TEST_MAX_MS > 0 && (ahora - m.tInicio) > TEST_MAX_MS) {
    finalizarTest(R_TIMEOUT);
    return;
  }

  float sumWeighted = 0.0;
  int sensoresEnNegro = 0;
  uint8_t mask = 0;   // bit i = sensor S(i+1) sobre negro

  // 1. Leer sensores (LOW = 0 detecta línea negra)
  for (int i = 0; i < 5; i++) {
    if (digitalRead(SENSORS[i]) == LOW) {
      sumWeighted += WEIGHTS[i];
      sensoresEnNegro++;
      mask |= (1 << i);
    }
  }

  // 2. Si los 5 sensores ven negro (fin de pista o meta), frena y cierra el test
  if (sensoresEnNegro == 5) {
    finalizarTest(R_META);
    return;
  }

  bool eventoBlancoNuevo = false;
  uint32_t blancoCerradoMs = 0;

  // 3. Manejo de blanco total (espacio vacío o línea muy delgada entre sensores)
  if (sensoresEnNegro == 0) {
    // TOLERANCIA A HUECOS: Mantiene el último error para cruzar el espacio ciego
    error = lastError;

    // Registro del evento de blanco total
    if (!enBlanco) {
      enBlanco = true;
      inicioBlanco = ahora;
      eventoBlancoNuevo = true;
    }
    // Si el blanco dura demasiado, se considera que perdio la linea
    if (LOST_TIMEOUT_MS > 0 && (ahora - inicioBlanco) > LOST_TIMEOUT_MS) {
      finalizarTest(R_LINEA_PERDIDA);
      return;
    }
  } else {
    // Cálculo normal del centro de masa de la línea
    error = sumWeighted / sensoresEnNegro;

    // Cierra el evento de blanco si habia uno abierto
    if (enBlanco) {
      blancoCerradoMs = ahora - inicioBlanco;
      enBlanco = false;
    }
  }

  // 4. Cálculo PD
  float P = Kp * error;
  float D = Kd * (error - lastError);
  float correction = P + D;

  lastError = error;

  // 5. Aplicación diferencial a las ruedas
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;
  leftSpeed  = constrain(leftSpeed,  MAX_REV, MAX_SPEED);   // Valores reales aplicados
  rightSpeed = constrain(rightSpeed, MAX_REV, MAX_SPEED);

  driveMotors(leftSpeed, rightSpeed);

  // 6. Metricas y telemetria de este ciclo (seccion critica breve: el servidor web lee estos datos)
  bool registrar = (contadorLog++ % LOG_CADA_N == 0);

  LOCK();
  if (m.ciclos > 0) {
    m.sumPeriodoUs += periodoUs;
    if (periodoUs > m.maxPeriodoUs) m.maxPeriodoUs = periodoUs;
  }
  m.ultimoLoopUs = ahoraUs;
  for (int i = 0; i < 5; i++) {
    if ((mask >> i) & 1) m.activ[i]++;
  }
  if (eventoBlancoNuevo) m.eventosBlanco++;
  if (blancoCerradoMs > 0) {
    m.msBlancoTotal += blancoCerradoMs;
    if (blancoCerradoMs > m.msBlancoMax) m.msBlancoMax = blancoCerradoMs;
  }
  actualizarMetricas(error, sensoresEnNegro);
  m.sumVel += (leftSpeed + rightSpeed) / 2.0;
  m.ciclos++;
  #if MODO_COM == MODO_WIFI
    if (registrar) {
      Muestra &s = bufMuestras[totalMuestras % BUF_N];
      s.t     = ahora - m.tInicio;
      s.err100 = (int16_t)roundf(error * 100.0);
      s.corr  = (int16_t)constrain((int)correction, -32000, 32000);
      s.izq   = (int16_t)leftSpeed;
      s.der   = (int16_t)rightSpeed;
      s.perUs = (uint16_t)((periodoUs > 65535) ? 65535 : periodoUs);
      s.mask  = mask;
      totalMuestras = totalMuestras + 1;
    }
  #endif
  UNLOCK();

  // CSV por USB / Bluetooth: test,t_ms,sensores,error,correccion,vel_izq,vel_der,periodo_us
  if (LOG_ACTIVADO && registrar) {
    char txt[6];
    for (int i = 0; i < 5; i++) txt[i] = ((mask >> i) & 1) ? '1' : '0';
    txt[5] = 0;
    out.printf("%u,%lu,%s,%.2f,%d,%d,%d,%lu\n", (unsigned)testId, (unsigned long)(ahora - m.tInicio), txt,
               error, (int)correction, leftSpeed, rightSpeed, (unsigned long)periodoUs);
  }

  delay(10);
}
