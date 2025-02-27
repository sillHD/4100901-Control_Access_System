# Sistema de Control de Acceso Inteligente

Este repositorio contiene la implementación base para el proyecto final del curso de Estructuras Computacionales en la Universidad Nacional de Colombia - Sede Manizales, utilizando la placa Nucleo_L476RG y componentes externos.

## Descripción General

El proyecto implementa un sistema de control de acceso inteligente que permite desbloquear una puerta (emulada por un LED externo) mediante diferentes interfaces:

- Teclado hexadecimal
- Comandos desde internet con ESP01(UART3)
- Comandos desde PC host (UART2)
- Botón físico (simulando estar dentro de la habitación)
- Comandos especiales para depuración (#*O*# y #*C*#)

## Componentes de Hardware

- Placa Nucleo_L476RG (STM32L476RG)
- Pantalla OLED SSD1306
- Teclado hexadecimal
- LED externo (simula cerradura)
- LED LD2 integrado en la placa (heartbeat)
- Módulo ESP01 (WiFi)
- Botón integrado en la placa (para abrir desde el interior)
- Botón externo opcional (timbre)

## Diagrama de Conexiones

```
+-------------------+         +------------+
|                   |         |            |
|  NUCLEO_L476RG    |-------->| SSD1306    |
|                   |   I2C   | (OLED)     |
+-------------------+         +------------+
    |    |    |    |
    |    |    |    |          +------------+
    |    |    |    +--------->|            |
    |    |    |         UART2 | PC Host    |
    |    |    |               | (ST-Link)  |
    |    |    |               +------------+
    |    |    |
    |    |    |               +------------+
    |    |    +-----------+-->|            |
    |    |            GPIO|   | LED LD2    |
    |    |                |   | (Heartbeat)|
    |    |                |   +------------+
    |    |                |
    |    |                |   +------------+
    |    |                |   |            |
    |    |                +-->| LED Externo|
    |    |                |   | (Cerradura)|
    |    |                |   +------------+
    |    |                |
    |    |                |   +------------+
    |    |                |   |            |
    |    |                +-->| Botón B1   |
    |    |                |   | (Interior) |
    |    |                |   +------------+
    |    |                |
    |    |                |   +------------+
    |    |                |   |            |
    |    |                +-->| Botón Ext. |
    |    |                    | (Timbre)   |
    |    |                    +------------+
    |    |
    |    |                    +------------+
    |    |                    |            |
    |    +------------------->| Teclado    |
    |                 GPIO    | Hexadecimal|
    |                         +------------+
    |
    |                         +------------+
    +------------------------>|            |
                       UART3  | ESP01      |
                              | (WIFI)     |
                              +------------+
```

## Estructura del Software

El proyecto se organiza siguiendo una arquitectura modular y utiliza las siguientes tecnologías:

- HAL (Hardware Abstraction Layer) de STM32
- Drivers para periféricos
- Controladores para componentes externos
- Sistema de manejo de eventos

### Módulos Implementados

1. **Gestión de Periféricos**
   - Inicialización y configuración de GPIO, UART, I2C
   - Manejo de interrupciones
   - Control de modos de bajo consumo

2. **Drivers de Componentes**
   - Driver SSD1306 (pantalla OLED)
   - Driver de teclado hexadecimal
   - Control de LED (heartbeat y cerradura)

3. **Comunicación**
   - Implementación UART para comunicación con PC host
   - Integración ESP01 para conectividad WiFi
   - Ring buffer para manejo eficiente de datos

4. **Lógica de Aplicación**
   - Sistema de autenticación
   - Manejo de comandos
   - Rutinas de eventos

## Funcionalidades Implementadas

### 1. Autenticación
- Validación de contraseña ingresada por teclado
- Autenticación por comandos remotos
- Registros de intentos fallidos

### 2. Indicadores
- LD2 como heartbeat (indicador de sistema en funcionamiento)
- LED externo como indicador de estado de la cerradura
- Pantalla OLED para mensajes de estado y configuración

### 3. Comunicación
- Protocolo UART para depuración y control desde PC
- Comandos a través de WiFi
- Comandos especiales de diagnóstico
- Detección de timbre (botón externo opcional)

### 4. Gestión de Energía
- Implementación de modo de bajo consumo (sleep mode)
- Despertar por eventos (teclado, UART, botón)

### 5. Máquina de Estados para Control de Puerta
- **CERRADO**: Estado por defecto, puerta cerrada
- **ABIERTO_TEMPORAL**: Puerta abierta temporalmente (después de introducir clave correcta o presionar botón una vez)
- **ABIERTO_PERMANENTE**: Puerta abierta permanentemente (después de presionar botón dos veces consecutivas)
- **TRANSICIÓN**: La puerta regresa a estado CERRADO desde ABIERTO_PERMANENTE cuando se presiona el botón nuevamente

```
     +------------+
     |            |
     |   CERRADO  <-----------------+
     |            |                 |
     +-----+------+                 |
           |                        |
   Botón   |  Clave correcta        |
   simple  |                        |
           v                        |
     +------------+                 |
     |  ABIERTO   |   Timeout       |
     | TEMPORAL   +---------------->+
     |            |                 |
     +-----+------+                 |
           |                        |
           | Doble                  |
  Botón    | presión                |
           v                        |
     +------------+                 |
     |  ABIERTO   |                 |
     | PERMANENTE +------+          |
     |            |      |          |
     +------------+      |          |
                         | Botón    |
                         +----------+
```

## Comandos del Sistema

| Comando | Descripción |
|---------|-------------|
| #*O*#   | Abrir cerradura (depuración) |
| #*C*#   | Cerrar cerradura (depuración) |
| [Clave] | Validar clave y abrir temporalmente si es correcta |
| Botón (1 presión) | Abrir temporalmente desde el interior |
| Botón (2 presiones rápidas) | Abrir permanentemente |
| Botón (en estado abierto permanente) | Cerrar |
| Botón Timbre | Enviar notificación de timbre |

## Cómo Completar el Proyecto

Para completar la implementación, será necesario:

1. Revisar la lógica de autenticación
2. Implementar funcionalidades faltantes
3. Corregir posibles errores
4. Agregar características adicionales (para proyecto final)
5. Documentar el código
6. Preparar presentación y video explicativo

## Herramientas de Desarrollo

- VS Code IDE
- STM32CubeMX para configuración inicial
- YAT o Teraterm para COM o TCP/IP

## Recursos Adicionales

La implementación actual hace uso de conceptos aprendidos durante el curso:
- Diseño de procesadores
- Programación en ASM (para entender el funcionamiento del procesador)
- Programación en C con acceso directo a registros (para entender el funcionamiento de los perifericos)
- Uso de HAL para abstraer hardware
- Implementación de drivers propios
- Manejo de periféricos (GPIO, UART, I2C)
- Integración de componentes externos
- Técnicas de bajo consumo energético

---

*Este proyecto es parte del curso de Estructuras Computacionales en la Universidad Nacional de Colombia - Sede Manizales.*