# Apolo2 con FreeRTOS

El archivo `apolo2.c` separa el programa en tareas de muestreo EMG,
control de la mano y registro USB. El PWM de los servos sigue funcionando por
hardware.

## Dependencias

Este proyecto requiere el Pico SDK 2.3.1 y el repositorio oficial
`FreeRTOS-Kernel` con el port RP2040. El kernel está en
`FreeRTOS-Kernel/` dentro de la carpeta del proyecto. El helper encuentra esa
ruta local automáticamente; si se mueve, configurá `FREERTOS_KERNEL_PATH` para
que apunte a la nueva ubicación.

La carpeta debe tener estos archivos de proyecto:

```text
apolo2/
  CMakeLists.txt
  FreeRTOS_Kernel_import.cmake
  FreeRTOSConfig.h
  apolo2.c
```

`pico_sdk_import.cmake` es el que ya tiene el proyecto original.

En VS Code, abrí esta carpeta como proyecto Pico y compilá el target
`apolo2_freertos`. La configuración fija FreeRTOS a un núcleo; el código no
usa el segundo núcleo del RP2040.

## Tareas

- `EMG`: calibración inicial con 100 muestras y luego muestreo periódico cada
  5 ms con `vTaskDelayUntil()`.
- `Control`: consume la muestra más reciente desde una cola FreeRTOS y actualiza
  los servos al detectar una activación.
- `Log`: imprime lecturas por USB cada 250 ms.

Los umbrales `EMG_THRESHOLD` (50) y `EMG_RELEASE_THRESHOLD` (25) se conservaron
del archivo recibido; hay que calibrarlos con la señal real.
