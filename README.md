# Robot auto-balanceado con ESP32 (ESP-IDF + PlatformIO)

## Libreria MPU6050

![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange?style=flat&logo=platformio)
![C++](https://img.shields.io/badge/C%2B%2B-MPU6050-blue?style=flat&logo=c%2B%2B)

Este proyecto proporciona una librería personalizada para leer datos del sensor **MPU6050** usando el framework **ESP-IDF** dentro de **PlatformIO**, diseñada específicamente para la aplicacion de un **robot auto-balanceado**.

---

### 🚀 Características

- Conexión I2C y test de conectividad (`WHO_AM_I`)
- Lectura de datos crudos de acelerómetro y giroscopio
- Conversión de datos a unidades físicas
- Cálculo de **yaw, pitch y roll** en grados
- Calibración de offset inicial (ideal para compensar drift)
- Estructura modular con separación de lógica (`src/`, `include/`)

---

### 🗂️ Estructura del Proyecto

```bash
📁 your_project/
├── include/
│   └── MPU6050.h           # Declaración de la clase y funciones
├── src/
│   ├── main.cpp            # Ejemplo de uso de la librería
│   └── MPU6050.cpp         # Implementación de la lógica
├── platformio.ini          # Configuración de PlatformIO
```

---

### 📌 Requisitos

- ESP32 (cualquier modelo compatible con ESP-IDF)
- Sensor MPU6050 (conectado vía I2C)
- PlatformIO con entorno ESP-IDF configurado

---

### 🔧 Conexiones I2C

| MPU6050 | ESP32       |
|---------|-------------|
| VCC     | 3.3V        |
| GND     | GND         |
| SDA     | GPIO21 (ejemplo) |
| SCL     | GPIO22 (ejemplo) |

---

### 📘 API de la Librería

### 🛠 Inicialización

```cpp
MPU6050 mpu;
mpu.begin();             // Despierta el sensor
bool conectado = mpu.testConnection();  // Test de WHO_AM_I
```

---

### 🎯 Calibración

```cpp
mpu.calibrate(3000);     // Toma como referencia 0° durante 3 segundos en posición estable
```

---

### 📡 Lectura de Datos

```cpp
mpu.update();            // Llama constantemente en el loop

float yaw = mpu.getYaw();
float pitch = mpu.getPitch();
float roll = mpu.getRoll();
```

---

### 📐 Cálculo de Ángulos

| Ángulo  | Cómo se calcula                 | Fuente de datos     |
|---------|----------------------------------|----------------------|
| Yaw     | Integración del giroscopio Z     | gyroZ                |
| Pitch   | Trigonometría entre accX y accZ  | acelerómetro         |
| Roll    | Trigonometría entre accY y accZ  | acelerómetro         |

> ℹ️ Para mayor precisión, considera aplicar un filtro complementario o Kalman en futuras versiones.

---

### 💡 Ejemplo Básico (`main.cpp`)

```cpp
#include "MPU6050.h"

MPU6050 mpu;

extern "C" void app_main(void) {
    mpu.begin();

    if (mpu.testConnection()) {
        printf("MPU6050 conectado correctamente.\n");
    }

    mpu.calibrate(3000); // Calibración por 3 segundos

    while (true) {
        mpu.update();
        printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n",
            mpu.getYaw(),
            mpu.getPitch(),
            mpu.getRoll()
        );
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
```

---

### 🧠 Notas Técnicas

- Acelerómetro convertido con escala ±2g → divisor `16384.0`
- Giroscopio convertido con escala ±250°/s → divisor `131.0`
- Registros I2C usados:
  - `WHO_AM_I`: 0x75
  - `ACCEL_XOUT_H`: 0x3B
  - `GYRO_XOUT_H`: 0x43
  - `PWR_MGMT_1`: 0x6B

---

### 📈 Mejoras Futuras

- Añadir filtro complementario o Kalman
- Soporte para quaterniones y orientación 3D total
- Compatibilidad con magnetómetro externo para corrección de yaw

---

## 🤝 Créditos

Desarrollado por [Edgar Mendez (@EdGiochi)](https://github.com/EdGiochi)  
Inspirado en proyectos de robótica auto-balanceada y aprendizaje con ESP32

---

## 🪪 Licencia

MIT License – Libre uso con atribución.
