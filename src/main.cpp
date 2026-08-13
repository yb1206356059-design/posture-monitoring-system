#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ==================================================
// Wi-Fi access point
// ==================================================
constexpr char AP_NAME[] = "ESP32C3_TEST";

WebServer server(80);

// ==================================================
// Hardware pins
// ==================================================
constexpr uint8_t PIN_I2C_SDA = 6;
constexpr uint8_t PIN_I2C_SCL = 5;

constexpr uint8_t PIN_MOTOR  = 3;
constexpr uint8_t PIN_BUZZER = 4;
constexpr uint8_t PIN_BUTTON = 7;

constexpr uint8_t MOTOR_ON_LEVEL  = HIGH;
constexpr uint8_t MOTOR_OFF_LEVEL = LOW;

constexpr uint8_t BUZZER_ON_LEVEL  = HIGH;
constexpr uint8_t BUZZER_OFF_LEVEL = LOW;

// ==================================================
// MPU6050
// ==================================================
constexpr uint8_t MPU_ADDRESS = 0x68;

Adafruit_MPU6050 mpu;

// ==================================================
// Detection parameters
// ==================================================
constexpr uint32_t MPU_INTERVAL_US = 10000; // 100Hz
constexpr float FILTER_ALPHA = 0.985f;

constexpr float ACCEL_MIN_VALID = 0.85f;
constexpr float ACCEL_MAX_VALID = 1.15f;

constexpr uint32_t ALARM_CONFIRM_MS = 100;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 20;

constexpr uint32_t MOTOR_ON_TIME_MS = 300;
constexpr uint32_t MOTOR_OFF_TIME_MS = 200;

constexpr uint32_t PRINT_INTERVAL_MS = 500;

// Adjustable from the web dashboard
float alarmAngle = 30.0f;

// ==================================================
// Sensor data
// ==================================================
struct MPUData
{
    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float temperature;
    float totalAcceleration;
};

MPUData sensorData;

// ==================================================
// Motor state
// ==================================================
enum class MotorPhase
{
    Stopped,
    Running,
    Quiet
};

MotorPhase motorPhase = MotorPhase::Stopped;

// ==================================================
// Posture state
// ==================================================
bool mpuConnected = false;
bool angleReferenceSet = false;

bool alarmState = false;
bool alarmConfirming = false;

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float filteredAngleX = 0.0f;
float referenceAngleX = 0.0f;
float relativeAngleX = 0.0f;

// ==================================================
// Statistics
// ==================================================
uint64_t poorPostureTimeMs = 0;
uint32_t reminderCount = 0;

String currentDate;

// ==================================================
// Timing variables
// ==================================================
uint32_t lastMpuTimeUs = 0;
uint32_t motorPhaseStartMs = 0;
uint32_t alarmConfirmStartMs = 0;
uint32_t lastPrintTimeMs = 0;
uint32_t lastStatisticsTimeMs = 0;

// ==================================================
// Button state
// ==================================================
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
uint32_t lastButtonChangeTimeMs = 0;

// ==================================================
// Web dashboard
// ==================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">

<head>
<meta charset="UTF-8">
<meta name="viewport"
      content="width=device-width,initial-scale=1">

<title>Posture Monitor</title>

<style>
:root {
    --bg: #eef2f7;
    --card: #ffffff;
    --text: #172033;
    --muted: #6b7280;
    --blue: #2563eb;
    --green: #16a34a;
    --red: #dc2626;
    --orange: #d97706;
    --border: #e5e7eb;
}

* {
    box-sizing: border-box;
}

body {
    margin: 0;
    padding: 22px;
    font-family: Arial, sans-serif;
    color: var(--text);
    background: var(--bg);
}

.container {
    width: min(1100px, 100%);
    margin: 0 auto;
}

header {
    margin-bottom: 20px;
}

h1 {
    margin: 0;
    font-size: 29px;
}

.subtitle {
    margin-top: 7px;
    color: var(--muted);
}

.connection {
    float: right;
    padding: 7px 12px;
    border-radius: 999px;
    color: white;
    background: var(--orange);
    font-size: 12px;
}

.grid {
    display: grid;
    grid-template-columns: repeat(12, 1fr);
    gap: 16px;
}

.card {
    padding: 20px;
    border-radius: 16px;
    background: var(--card);
    box-shadow: 0 5px 20px rgba(15, 23, 42, .07);
}

.status-card {
    grid-column: span 5;
}

.angle-card {
    grid-column: span 3;
}

.stats-card {
    grid-column: span 4;
}

.sensor-card {
    grid-column: span 6;
}

.chart-card {
    grid-column: span 6;
}

.settings-card {
    grid-column: span 12;
}

.label {
    margin-bottom: 10px;
    color: var(--muted);
    font-size: 14px;
}

.status {
    display: inline-block;
    padding: 9px 16px;
    border-radius: 999px;
    color: white;
    font-size: 18px;
    font-weight: bold;
}

.good {
    background: var(--green);
}

.bad {
    background: var(--red);
}

.waiting {
    background: var(--orange);
}

.big-number {
    font-size: 43px;
    font-weight: bold;
    line-height: 1;
}

.unit {
    color: var(--muted);
    font-size: 18px;
}

.stat-row {
    display: flex;
    justify-content: space-between;
    padding: 11px 0;
    border-bottom: 1px solid var(--border);
}

.stat-row:last-child {
    border-bottom: 0;
}

.data-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 11px;
}

.data-item {
    padding: 12px;
    border-radius: 10px;
    background: #f8fafc;
}

.data-name {
    color: var(--muted);
    font-size: 12px;
}

.data-value {
    margin-top: 6px;
    font-size: 17px;
    font-weight: bold;
}

canvas {
    display: block;
    width: 100%;
    height: 260px;
    border-radius: 10px;
    background: #fbfdff;
}

.controls {
    display: flex;
    flex-wrap: wrap;
    align-items: end;
    gap: 12px;
}

.control {
    min-width: 190px;
}

.control label {
    display: block;
    margin-bottom: 7px;
    color: var(--muted);
    font-size: 13px;
}

input {
    width: 100%;
    height: 42px;
    padding: 0 12px;
    border: 1px solid #cbd5e1;
    border-radius: 9px;
    font-size: 16px;
}

button {
    height: 42px;
    padding: 0 18px;
    border: 0;
    border-radius: 9px;
    color: white;
    background: var(--blue);
    font-size: 14px;
    cursor: pointer;
}

button.danger {
    background: var(--red);
}

.message {
    min-height: 20px;
    margin-top: 12px;
    color: var(--green);
    font-size: 14px;
}

@media (max-width: 800px) {
    body {
        padding: 13px;
    }

    .status-card,
    .angle-card,
    .stats-card,
    .sensor-card,
    .chart-card,
    .settings-card {
        grid-column: span 12;
    }

    .data-grid {
        grid-template-columns: repeat(2, 1fr);
    }
}
</style>
</head>

<body>

<div class="container">

<header>
    <span id="connection" class="connection">
        Connecting
    </span>

    <h1>Posture Monitor</h1>

    <div class="subtitle">
        Real-time posture monitoring and reminder system
    </div>
</header>

<main class="grid">

<section class="card status-card">
    <div class="label">Current Posture Status</div>

    <div id="postureStatus" class="status waiting">
        Not Calibrated
    </div>

    <div id="statusDescription" class="subtitle">
        Calibrate your current posture to begin.
    </div>
</section>

<section class="card angle-card">
    <div class="label">Real-Time Relative Angle</div>

    <span id="relativeAngle" class="big-number">
        0.0
    </span>

    <span class="unit">deg</span>

    <div class="subtitle">
        Threshold:
        <span id="thresholdText">30.0</span> deg
    </div>
</section>

<section class="card stats-card">
    <div class="label">Today's Statistics</div>

    <div class="stat-row">
        <span>Poor Posture Time</span>
        <strong id="poorTime">00:00:00</strong>
    </div>

    <div class="stat-row">
        <span>Reminder Count</span>
        <strong id="reminderCount">0</strong>
    </div>

    <div class="stat-row">
        <span>Date</span>
        <strong id="dateText">Waiting</strong>
    </div>

    <div class="stat-row">
        <span>Wi-Fi Clients</span>
        <strong id="clientCount">0</strong>
    </div>
</section>

<section class="card sensor-card">
    <div class="label">Real-Time Sensor Data</div>

    <div class="data-grid">

        <div class="data-item">
            <div class="data-name">Accelerometer X</div>
            <div class="data-value">
                <span id="ax">0.000</span> g
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Accelerometer Y</div>
            <div class="data-value">
                <span id="ay">0.000</span> g
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Accelerometer Z</div>
            <div class="data-value">
                <span id="az">0.000</span> g
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Gyroscope X</div>
            <div class="data-value">
                <span id="gx">0.00</span> deg/s
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Gyroscope Y</div>
            <div class="data-value">
                <span id="gy">0.00</span> deg/s
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Gyroscope Z</div>
            <div class="data-value">
                <span id="gz">0.00</span> deg/s
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Temperature</div>
            <div class="data-value">
                <span id="temperature">0.0</span> deg C
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Total Acceleration</div>
            <div class="data-value">
                <span id="totalAcceleration">0.000</span> g
            </div>
        </div>

        <div class="data-item">
            <div class="data-name">Absolute X Angle</div>
            <div class="data-value">
                <span id="absoluteAngle">0.0</span> deg
            </div>
        </div>

    </div>
</section>

<section class="card chart-card">
    <div class="label">Live Angle Curve</div>
    <canvas id="angleChart"></canvas>
</section>

<section class="card settings-card">
    <div class="label">
        Calibration & Threshold Settings
    </div>

    <div class="controls">

        <button onclick="calibratePosture()">
            Calibrate Current Posture
        </button>

        <div class="control">
            <label for="thresholdInput">
                Alarm Threshold (5 deg - 90 deg)
            </label>

            <input id="thresholdInput"
                   type="number"
                   min="5"
                   max="90"
                   step="1"
                   value="30">
        </div>

        <button onclick="saveThreshold()">
            Save Threshold
        </button>

        <button class="danger"
                onclick="resetStatistics()">
            Reset Today's Statistics
        </button>

    </div>

    <div id="message" class="message"></div>
</section>

</main>
</div>

<script>
const canvas =
    document.getElementById("angleChart");

const context =
    canvas.getContext("2d");

const historyLength = 240;
const angleHistory = [];

let currentThreshold = 30;
let requestBusy = false;

function setText(id, value) {
    document.getElementById(id).textContent = value;
}

function formatTime(totalSeconds) {
    totalSeconds = Math.max(
        0,
        Math.floor(totalSeconds)
    );

    const hours =
        Math.floor(totalSeconds / 3600);

    const minutes =
        Math.floor((totalSeconds % 3600) / 60);

    const seconds =
        totalSeconds % 60;

    return String(hours).padStart(2, "0") + ":" +
           String(minutes).padStart(2, "0") + ":" +
           String(seconds).padStart(2, "0");
}

function showMessage(text, error = false) {
    const message =
        document.getElementById("message");

    message.textContent = text;
    message.style.color =
        error ? "#dc2626" : "#16a34a";

    setTimeout(() => {
        message.textContent = "";
    }, 3000);
}

function resizeCanvas() {
    const ratio =
        window.devicePixelRatio || 1;

    const width =
        canvas.clientWidth;

    const height =
        canvas.clientHeight;

    canvas.width =
        Math.floor(width * ratio);

    canvas.height =
        Math.floor(height * ratio);

    context.setTransform(
        ratio, 0, 0, ratio, 0, 0
    );

    drawChart();
}

function angleToY(angle, height, maxAngle) {
    return height / 2 -
           angle / maxAngle *
           height * 0.43;
}

function drawChart() {
    const width =
        canvas.clientWidth;

    const height =
        canvas.clientHeight;

    const maxAngle =
        Math.max(60, currentThreshold + 15);

    context.clearRect(
        0, 0, width, height
    );

    context.strokeStyle = "#e2e8f0";
    context.lineWidth = 1;

    for (let i = 0; i <= 4; i++) {
        const y = height * i / 4;

        context.beginPath();
        context.moveTo(0, y);
        context.lineTo(width, y);
        context.stroke();
    }

    const centerY =
        angleToY(0, height, maxAngle);

    const upperY =
        angleToY(
            currentThreshold,
            height,
            maxAngle
        );

    const lowerY =
        angleToY(
            -currentThreshold,
            height,
            maxAngle
        );

    context.setLineDash([6, 5]);
    context.strokeStyle = "#dc2626";

    context.beginPath();
    context.moveTo(0, upperY);
    context.lineTo(width, upperY);
    context.stroke();

    context.beginPath();
    context.moveTo(0, lowerY);
    context.lineTo(width, lowerY);
    context.stroke();

    context.setLineDash([]);
    context.strokeStyle = "#94a3b8";

    context.beginPath();
    context.moveTo(0, centerY);
    context.lineTo(width, centerY);
    context.stroke();

    context.fillStyle = "#64748b";
    context.font = "12px Arial";

    context.fillText(
        "+" + currentThreshold.toFixed(0) + " deg",
        6,
        upperY - 5
    );

    context.fillText(
        "0 deg",
        6,
        centerY - 5
    );

    context.fillText(
        "-" + currentThreshold.toFixed(0) + " deg",
        6,
        lowerY - 5
    );

    if (angleHistory.length < 2) {
        return;
    }

    context.strokeStyle = "#2563eb";
    context.lineWidth = 2;
    context.beginPath();

    angleHistory.forEach((angle, index) => {
        const x =
            index * width /
            (historyLength - 1);

        const limited =
            Math.max(
                -maxAngle,
                Math.min(maxAngle, angle)
            );

        const y =
            angleToY(
                limited,
                height,
                maxAngle
            );

        if (index === 0) {
            context.moveTo(x, y);
        } else {
            context.lineTo(x, y);
        }
    });

    context.stroke();
}

function updateStatus(data) {
    const status =
        document.getElementById("postureStatus");

    const description =
        document.getElementById(
            "statusDescription"
        );

    if (!data.sensorConnected) {
        status.className = "status bad";
        status.textContent = "Sensor Error";

        description.textContent =
            "MPU6050 is not responding.";

    } else if (!data.calibrated) {
        status.className = "status waiting";
        status.textContent = "Not Calibrated";

        description.textContent =
            "Calibrate your current posture to begin.";

    } else if (data.poorPosture) {
        status.className = "status bad";
        status.textContent = "Poor Posture";

        description.textContent =
            data.alarm
            ? "Posture reminder is active."
            : "Angle threshold is being confirmed.";

    } else {
        status.className = "status good";
        status.textContent = "Good Posture";

        description.textContent =
            "The current posture is within the allowed range.";
    }
}

async function updateData() {
    if (requestBusy) {
        return;
    }

    requestBusy = true;

    try {
        const response = await fetch(
            "/api/data",
            { cache: "no-store" }
        );

        if (!response.ok) {
            throw new Error();
        }

        const data =
            await response.json();

        const connection =
            document.getElementById("connection");

        connection.textContent = "Connected";
        connection.style.background = "#16a34a";

        currentThreshold = data.threshold;

        updateStatus(data);

        setText(
            "relativeAngle",
            data.relativeAngle.toFixed(1)
        );

        setText(
            "thresholdText",
            data.threshold.toFixed(1)
        );

        setText(
            "absoluteAngle",
            data.absoluteAngle.toFixed(1)
        );

        setText("ax", data.ax.toFixed(3));
        setText("ay", data.ay.toFixed(3));
        setText("az", data.az.toFixed(3));

        setText("gx", data.gx.toFixed(2));
        setText("gy", data.gy.toFixed(2));
        setText("gz", data.gz.toFixed(2));

        setText(
            "temperature",
            data.temperature.toFixed(1)
        );

        setText(
            "totalAcceleration",
            data.totalAcceleration.toFixed(3)
        );

        setText(
            "poorTime",
            formatTime(data.poorSeconds)
        );

        setText(
            "reminderCount",
            data.reminderCount
        );

        setText(
            "dateText",
            data.date || "Waiting"
        );

        setText(
            "clientCount",
            data.clientCount
        );

        if (
            document.activeElement.id !==
            "thresholdInput")
        {
            document.getElementById(
                "thresholdInput"
            ).value =
                data.threshold.toFixed(0);
        }

        angleHistory.push(
            data.calibrated
            ? data.relativeAngle
            : 0
        );

        if (
            angleHistory.length >
            historyLength)
        {
            angleHistory.shift();
        }

        drawChart();

    } catch (error) {
        const connection =
            document.getElementById("connection");

        connection.textContent = "Disconnected";
        connection.style.background = "#dc2626";

    } finally {
        requestBusy = false;
    }
}

async function calibratePosture() {
    try {
        const response = await fetch(
            "/api/calibrate",
            { method: "POST" }
        );

        if (!response.ok) {
            throw new Error();
        }

        angleHistory.length = 0;

        showMessage(
            "Current posture calibrated successfully."
        );

    } catch (error) {
        showMessage(
            "Calibration failed.",
            true
        );
    }
}

async function saveThreshold() {
    const value = Number(
        document.getElementById(
            "thresholdInput"
        ).value
    );

    if (
        !Number.isFinite(value) ||
        value < 5 ||
        value > 90)
    {
        showMessage(
            "Threshold must be between 5 deg and 90 deg.",
            true
        );

        return;
    }

    try {
        const response = await fetch(
            "/api/settings?threshold=" +
            encodeURIComponent(value),
            { method: "POST" }
        );

        if (!response.ok) {
            throw new Error();
        }

        currentThreshold = value;

        showMessage(
            "Alarm threshold updated."
        );

    } catch (error) {
        showMessage(
            "Threshold update failed.",
            true
        );
    }
}

async function resetStatistics() {
    try {
        const response = await fetch(
            "/api/reset",
            { method: "POST" }
        );

        if (!response.ok) {
            throw new Error();
        }

        showMessage(
            "Today's statistics reset."
        );

    } catch (error) {
        showMessage(
            "Statistics reset failed.",
            true
        );
    }
}

async function syncDate() {
    const now = new Date();

    const date =
        now.getFullYear() + "-" +
        String(now.getMonth() + 1).padStart(2, "0") + "-" +
        String(now.getDate()).padStart(2, "0");

    try {
        await fetch(
            "/api/date?value=" +
            encodeURIComponent(date),
            { method: "POST" }
        );
    } catch (error) {
    }
}

window.addEventListener(
    "resize",
    resizeCanvas
);

resizeCanvas();
syncDate();
updateData();

setInterval(updateData, 250);
setInterval(syncDate, 60000);
</script>

</body>
</html>
)HTML";

// ==================================================
// Function declarations
// ==================================================
float normalizeAngle(float angle);

void scanI2C();
bool initializeMPU6050();
bool readMPU6050();
bool calibrateGyroscope();
bool initializeAngle();
bool updateAngle();

void startMotor();
void stopMotorTemporarily();
void setAlarm(bool state);
void updateMotor();
void checkAlarm();

void setCurrentAngleAsReference();
void updateButton();
void updateStatistics();

void setupWiFi();
void setupWebServer();

void handleData();
void handleCalibration();
void handleSettings();
void handleReset();
void handleDate();

void printDebugData();

// ==================================================
// Angle normalization
// ==================================================
float normalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

// ==================================================
// I2C scan
// ==================================================
void scanI2C()
{
    Serial.println();
    Serial.println("Scanning I2C devices...");

    uint8_t count = 0;

    for (uint8_t address = 1;
         address < 127;
         address++)
    {
        Wire.beginTransmission(address);

        if (Wire.endTransmission(true) == 0)
        {
            Serial.printf(
                "I2C device found: 0x%02X\n",
                address
            );

            count++;
        }
    }

    Serial.printf(
        "I2C scan complete. %u device(s) found.\n",
        count
    );
}

// ==================================================
// Initialise MPU6050
// ==================================================
bool initializeMPU6050()
{
    Serial.println();
    Serial.println(
        "Initialising MPU6050 using the Adafruit library..."
    );

    if (!mpu.begin(MPU_ADDRESS, &Wire))
    {
        Serial.println("MPU6050 initialisation failed!");
        return false;
    }

    mpu.setAccelerometerRange(
        MPU6050_RANGE_2_G
    );

    mpu.setGyroRange(
        MPU6050_RANGE_250_DEG
    );

    mpu.setFilterBandwidth(
        MPU6050_BAND_5_HZ
    );

    delay(100);

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    if (!mpu.getEvent(
            &acceleration,
            &gyro,
            &temperature))
    {
        Serial.println("MPU6050 read test failed!");
        return false;
    }

    Serial.println("MPU6050 initialised successfully!");
    return true;
}

// ==================================================
// Read MPU6050
// ==================================================
bool readMPU6050()
{
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    if (!mpu.getEvent(
            &acceleration,
            &gyro,
            &temperature))
    {
        return false;
    }

    constexpr float GRAVITY = 9.80665f;

    sensorData.accelX =
        acceleration.acceleration.x /
        GRAVITY;

    sensorData.accelY =
        acceleration.acceleration.y /
        GRAVITY;

    sensorData.accelZ =
        acceleration.acceleration.z /
        GRAVITY;

    sensorData.gyroX =
        gyro.gyro.x * RAD_TO_DEG -
        gyroOffsetX;

    sensorData.gyroY =
        gyro.gyro.y * RAD_TO_DEG -
        gyroOffsetY;

    sensorData.gyroZ =
        gyro.gyro.z * RAD_TO_DEG -
        gyroOffsetZ;

    sensorData.temperature =
        temperature.temperature;

    sensorData.totalAcceleration =
        sqrtf(
            sensorData.accelX *
            sensorData.accelX +

            sensorData.accelY *
            sensorData.accelY +

            sensorData.accelZ *
            sensorData.accelZ
        );

    return true;
}

// ==================================================
// Gyroscope zero-offset calibration
// ==================================================
bool calibrateGyroscope()
{
    constexpr uint16_t SAMPLE_COUNT = 500;

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    uint16_t validSamples = 0;

    gyroOffsetX = 0.0f;
    gyroOffsetY = 0.0f;
    gyroOffsetZ = 0.0f;

    Serial.println();
    Serial.println(
        "Starting gyroscope calibration. Keep the device still..."
    );

    delay(500);

    for (uint16_t i = 0;
         i < SAMPLE_COUNT;
         i++)
    {
        sensors_event_t acceleration;
        sensors_event_t gyro;
        sensors_event_t temperature;

        if (mpu.getEvent(
                &acceleration,
                &gyro,
                &temperature))
        {
            sumX +=
                gyro.gyro.x * RAD_TO_DEG;

            sumY +=
                gyro.gyro.y * RAD_TO_DEG;

            sumZ +=
                gyro.gyro.z * RAD_TO_DEG;

            validSamples++;
        }

        delay(5);
    }

    if (validSamples <
        SAMPLE_COUNT / 2)
    {
        Serial.println("Gyroscope calibration failed!");
        return false;
    }

    gyroOffsetX =
        sumX / validSamples;

    gyroOffsetY =
        sumY / validSamples;

    gyroOffsetZ =
        sumZ / validSamples;

    Serial.printf(
        "Gyroscope offset X:%.3f Y:%.3f Z:%.3f deg/s\n",
        gyroOffsetX,
        gyroOffsetY,
        gyroOffsetZ
    );

    return true;
}

// ==================================================
// Initialise angle
// ==================================================
bool initializeAngle()
{
    if (!readMPU6050())
    {
        return false;
    }

    filteredAngleX =
        atan2f(
            sensorData.accelY,
            sensorData.accelZ
        ) * RAD_TO_DEG;

    filteredAngleX =
        normalizeAngle(filteredAngleX);

    lastMpuTimeUs = micros();

    Serial.printf(
        "Initial X-axis angle: %.2f deg\n",
        filteredAngleX
    );

    return true;
}

// ==================================================
// Continuous angle estimation
// ==================================================
bool updateAngle()
{
    uint32_t nowUs = micros();

    if (
        nowUs - lastMpuTimeUs <
        MPU_INTERVAL_US)
    {
        return false;
    }

    float deltaTime =
        (nowUs - lastMpuTimeUs) /
        1000000.0f;

    lastMpuTimeUs = nowUs;

    if (
        deltaTime <= 0.0f ||
        deltaTime > 0.05f)
    {
        deltaTime = 0.01f;
    }

    if (!readMPU6050())
    {
        return false;
    }

    float accelAngleX =
        atan2f(
            sensorData.accelY,
            sensorData.accelZ
        ) * RAD_TO_DEG;

    float gyroAngleX =
        filteredAngleX +
        sensorData.gyroX *
        deltaTime;

    bool motorRunning =
        alarmState &&
        motorPhase ==
        MotorPhase::Running;

    bool accelerationValid =
        sensorData.totalAcceleration >=
            ACCEL_MIN_VALID &&
        sensorData.totalAcceleration <=
            ACCEL_MAX_VALID;

    if (motorRunning)
    {
        // Use gyroscope only while the motor is running
        filteredAngleX =
            gyroAngleX;
    }
    else if (accelerationValid)
    {
        float correction =
            normalizeAngle(
                accelAngleX -
                gyroAngleX
            );

        filteredAngleX =
            gyroAngleX +
            (1.0f - FILTER_ALPHA) *
            correction;
    }
    else
    {
        filteredAngleX =
            gyroAngleX;
    }

    filteredAngleX =
        normalizeAngle(
            filteredAngleX
        );

    if (angleReferenceSet)
    {
        relativeAngleX =
            normalizeAngle(
                filteredAngleX -
                referenceAngleX
            );
    }
    else
    {
        relativeAngleX = 0.0f;
    }

    return true;
}

// ==================================================
// Alarm output
// ==================================================
void startMotor()
{
    digitalWrite(
        PIN_MOTOR,
        MOTOR_ON_LEVEL
    );

    motorPhase =
        MotorPhase::Running;

    motorPhaseStartMs =
        millis();
}

void stopMotorTemporarily()
{
    digitalWrite(
        PIN_MOTOR,
        MOTOR_OFF_LEVEL
    );

    motorPhase =
        MotorPhase::Quiet;

    motorPhaseStartMs =
        millis();
}

void setAlarm(bool state)
{
    if (alarmState == state)
    {
        return;
    }

    alarmState = state;
    alarmConfirming = false;

    if (state)
    {
        digitalWrite(
            PIN_BUZZER,
            BUZZER_ON_LEVEL
        );

        startMotor();

        reminderCount++;

        Serial.println(
            "Alarm: posture angle exceeds the threshold!"
        );
    }
    else
    {
        digitalWrite(
            PIN_MOTOR,
            MOTOR_OFF_LEVEL
        );

        digitalWrite(
            PIN_BUZZER,
            BUZZER_OFF_LEVEL
        );

        motorPhase =
            MotorPhase::Stopped;

        Serial.println(
            "Posture returned to normal. Alarm disabled."
        );
    }
}

void updateMotor()
{
    if (!alarmState)
    {
        return;
    }

    uint32_t elapsed =
        millis() -
        motorPhaseStartMs;

    if (
        motorPhase ==
        MotorPhase::Running)
    {
        if (
            elapsed >=
            MOTOR_ON_TIME_MS)
        {
            stopMotorTemporarily();
        }
    }
    else if (
        motorPhase ==
        MotorPhase::Quiet)
    {
        if (
            elapsed >=
            MOTOR_OFF_TIME_MS)
        {
            startMotor();
        }
    }
}

void checkAlarm()
{
    if (!angleReferenceSet)
    {
        return;
    }

    float absoluteAngle =
        fabsf(relativeAngleX);

    if (alarmState)
    {
        if (
            absoluteAngle <
            alarmAngle)
        {
            setAlarm(false);
        }

        return;
    }

    if (
        absoluteAngle >=
        alarmAngle)
    {
        if (!alarmConfirming)
        {
            alarmConfirming = true;

            alarmConfirmStartMs =
                millis();
        }
        else if (
            millis() -
            alarmConfirmStartMs >=
            ALARM_CONFIRM_MS)
        {
            setAlarm(true);
        }
    }
    else
    {
        alarmConfirming = false;
    }
}

// ==================================================
// Set current posture as the reference
// ==================================================
void setCurrentAngleAsReference()
{
    if (!mpuConnected)
    {
        return;
    }

    if (alarmState)
    {
        setAlarm(false);
    }

    referenceAngleX =
        filteredAngleX;

    relativeAngleX = 0.0f;

    angleReferenceSet = true;
    alarmConfirming = false;

    Serial.printf(
        "Reference angle set: %.2f deg\n",
        referenceAngleX
    );
}

// ==================================================
// Physical button
// ==================================================
void updateButton()
{
    bool reading =
        digitalRead(PIN_BUTTON);

    if (
        reading !=
        lastButtonReading)
    {
        lastButtonReading =
            reading;

        lastButtonChangeTimeMs =
            millis();
    }

    if (
        millis() -
        lastButtonChangeTimeMs >=
        BUTTON_DEBOUNCE_MS)
    {
        if (
            reading !=
            stableButtonState)
        {
            stableButtonState =
                reading;

            if (
                stableButtonState ==
                LOW)
            {
                setCurrentAngleAsReference();
            }
        }
    }
}

// ==================================================
// Statistics
// ==================================================
void updateStatistics()
{
    uint32_t nowMs = millis();

    uint32_t elapsed =
        nowMs -
        lastStatisticsTimeMs;

    lastStatisticsTimeMs =
        nowMs;

    if (elapsed > 1000)
    {
        elapsed = 0;
    }

    bool poorPosture =
        mpuConnected &&
        angleReferenceSet &&
        fabsf(relativeAngleX) >=
        alarmAngle;

    if (poorPosture)
    {
        poorPostureTimeMs +=
            elapsed;
    }
}

// ==================================================
// Web data API
// ==================================================
void handleData()
{
    bool poorPosture =
        mpuConnected &&
        angleReferenceSet &&
        fabsf(relativeAngleX) >=
        alarmAngle;

    String json;
    json.reserve(600);

    json += "{";

    json += "\"sensorConnected\":";
    json += mpuConnected ?
        "true" : "false";

    json += ",\"calibrated\":";
    json += angleReferenceSet ?
        "true" : "false";

    json += ",\"poorPosture\":";
    json += poorPosture ?
        "true" : "false";

    json += ",\"alarm\":";
    json += alarmState ?
        "true" : "false";

    json += ",\"absoluteAngle\":";
    json += String(
        filteredAngleX,
        2
    );

    json += ",\"relativeAngle\":";
    json += String(
        relativeAngleX,
        2
    );

    json += ",\"threshold\":";
    json += String(
        alarmAngle,
        1
    );

    json += ",\"ax\":";
    json += String(
        sensorData.accelX,
        4
    );

    json += ",\"ay\":";
    json += String(
        sensorData.accelY,
        4
    );

    json += ",\"az\":";
    json += String(
        sensorData.accelZ,
        4
    );

    json += ",\"gx\":";
    json += String(
        sensorData.gyroX,
        3
    );

    json += ",\"gy\":";
    json += String(
        sensorData.gyroY,
        3
    );

    json += ",\"gz\":";
    json += String(
        sensorData.gyroZ,
        3
    );

    json += ",\"temperature\":";
    json += String(
        sensorData.temperature,
        2
    );

    json += ",\"totalAcceleration\":";
    json += String(
        sensorData.totalAcceleration,
        4
    );

    json += ",\"poorSeconds\":";
    json += String(
        static_cast<uint32_t>(
            poorPostureTimeMs /
            1000ULL
        )
    );

    json += ",\"reminderCount\":";
    json += String(
        reminderCount
    );

    json += ",\"date\":\"";
    json += currentDate;
    json += "\"";

    json += ",\"clientCount\":";
    json += String(
        WiFi.softAPgetStationNum()
    );

    json += "}";

    server.sendHeader(
        "Cache-Control",
        "no-store"
    );

    server.send(
        200,
        "application/json",
        json
    );
}

void handleCalibration()
{
    if (!mpuConnected)
    {
        server.send(
            503,
            "application/json",
            "{\"ok\":false}"
        );

        return;
    }

    setCurrentAngleAsReference();

    server.send(
        200,
        "application/json",
        "{\"ok\":true}"
    );
}

void handleSettings()
{
    if (!server.hasArg("threshold"))
    {
        server.send(
            400,
            "application/json",
            "{\"ok\":false}"
        );

        return;
    }

    float value =
        server.arg(
            "threshold"
        ).toFloat();

    if (
        value < 5.0f ||
        value > 90.0f)
    {
        server.send(
            400,
            "application/json",
            "{\"ok\":false}"
        );

        return;
    }

    alarmAngle = value;
    alarmConfirming = false;

    checkAlarm();

    server.send(
        200,
        "application/json",
        "{\"ok\":true}"
    );
}

void handleReset()
{
    poorPostureTimeMs = 0;
    reminderCount = 0;

    server.send(
        200,
        "application/json",
        "{\"ok\":true}"
    );
}

void handleDate()
{
    if (!server.hasArg("value"))
    {
        server.send(
            400,
            "application/json",
            "{\"ok\":false}"
        );

        return;
    }

    String newDate =
        server.arg("value");

    if (newDate.length() != 10)
    {
        server.send(
            400,
            "application/json",
            "{\"ok\":false}"
        );

        return;
    }

    if (currentDate.length() == 0)
    {
        currentDate = newDate;
    }
    else if (newDate != currentDate)
    {
        currentDate = newDate;
        poorPostureTimeMs = 0;
        reminderCount = 0;
    }

    server.send(
        200,
        "application/json",
        "{\"ok\":true}"
    );
}

// ==================================================
// Wi-Fi initialisation
// Use the previously verified start-up sequence
// ==================================================
void setupWiFi()
{
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    delay(1000);

    WiFi.mode(WIFI_AP);

    delay(500);

    bool result =
        WiFi.softAP(
            AP_NAME,
            nullptr,
            1,
            false,
            4
        );

    if (!result)
    {
        Serial.println(
            "Wi-Fi access point creation failed!"
        );

        return;
    }

    delay(500);

    Serial.println();
    Serial.println(
        "Wi-Fi access point created successfully!"
    );

    Serial.printf(
        "Access point name: %s\n",
        AP_NAME
    );

    Serial.println(
        "Access point password: none"
    );

    Serial.print(
        "Web page address: http://"
    );

    Serial.println(
        WiFi.softAPIP()
    );
}

// ==================================================
// Web server
// ==================================================
void setupWebServer()
{
    server.on(
        "/",
        HTTP_GET,
        []()
        {
            server.send_P(
                200,
                "text/html",
                INDEX_HTML
            );
        }
    );

    server.on(
        "/api/data",
        HTTP_GET,
        handleData
    );

    server.on(
        "/api/calibrate",
        HTTP_POST,
        handleCalibration
    );

    server.on(
        "/api/settings",
        HTTP_POST,
        handleSettings
    );

    server.on(
        "/api/reset",
        HTTP_POST,
        handleReset
    );

    server.on(
        "/api/date",
        HTTP_POST,
        handleDate
    );

    server.onNotFound(
        []()
        {
            server.sendHeader(
                "Location",
                "/"
            );

            server.send(
                302,
                "text/plain",
                ""
            );
        }
    );

    server.begin();

    Serial.println(
        "Web server started."
    );
}

// ==================================================
// Serial debug output
// ==================================================
void printDebugData()
{
    if (
        millis() -
        lastPrintTimeMs <
        PRINT_INTERVAL_MS)
    {
        return;
    }

    lastPrintTimeMs =
        millis();

    Serial.printf(
        "Current:%7.2f deg | "
        "Reference:%7.2f deg | "
        "Relative:%7.2f deg | "
        "ACC:%5.2fg | "
        "Alarm:%d | "
        "WiFi:%u\n",
        filteredAngleX,
        referenceAngleX,
        relativeAngleX,
        sensorData.totalAcceleration,
        alarmState,
        WiFi.softAPgetStationNum()
    );
}

// ==================================================
// setup
// ==================================================
void setup()
{
    pinMode(PIN_MOTOR, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    digitalWrite(
        PIN_MOTOR,
        MOTOR_OFF_LEVEL
    );

    digitalWrite(
        PIN_BUZZER,
        BUZZER_OFF_LEVEL
    );

    Serial.begin(115200);

    uint32_t waitStart = millis();

    while (
        !Serial &&
        millis() - waitStart < 3000)
    {
        delay(10);
    }

    Serial.println();
    Serial.println(
        "=================================="
    );
    Serial.println(
        "ESP32-C3 Posture Monitor"
    );
    Serial.println(
        "=================================="
    );

    // Start the verified Wi-Fi access point first
    setupWiFi();
    setupWebServer();

    // Then initialise the MPU6050
    Wire.begin(
        PIN_I2C_SDA,
        PIN_I2C_SCL
    );

    Wire.setClock(100000);

    delay(100);

    scanI2C();

    mpuConnected =
        initializeMPU6050();

    if (mpuConnected)
    {
        calibrateGyroscope();
        initializeAngle();

        Serial.println(
            "MPU6050 is ready."
        );

        Serial.println(
            "Press GPIO7 or the dashboard calibration button to set the reference."
        );
    }
    else
    {
        Serial.println(
            "MPU6050 initialisation failed, but the web dashboard is still accessible."
        );
    }

    lastStatisticsTimeMs =
        millis();
}

// ==================================================
// loop
// ==================================================
void loop()
{
    // Run posture-related functions first
    updateButton();
    updateMotor();

    if (mpuConnected)
    {
        updateAngle();
        checkAlarm();
    }

    updateStatistics();

    // Handle web requests
    server.handleClient();

    printDebugData();
}
