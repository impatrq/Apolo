# Apolo
<h1 align="center">Proyecto Apolo</h1>

<p align="center"><strong>Prótesis de mano controlada por señales musculares</strong></p>

<p align="center">
  <img width="512" height="512" alt="ApoLogo" src="https://github.com/user-attachments/assets/bad67410-fced-46a5-939e-827018866c3f" />
</p>

## Sobre el proyecto

Proyecto Apolo busca desarrollar un prototipo de mano protésica que se pueda abrir y cerrar mediante contracciones musculares voluntarias. Para lograrlo, el sistema capta las pequeñas señales eléctricas que generan los músculos y las convierte en órdenes de movimiento.

## ¿Cómo funciona?

1. **Captación.** Los electrodos colocados sobre la piel registran la actividad muscular.
2. **Acondicionamiento.** Un circuito amplifica la señal y filtra el ruido.
3. **Control.** Un microcontrolador compara el nivel de la señal con un umbral para detectar una contracción.
4. **Movimiento.** El motor acciona el mecanismo que abre o cierra la mano.

## Diseño electrónico

| Módulo | Función |
| --- | --- |
| [AmpyFil](./idk/AmpyFil.kicad_pro) | Amplificación y filtrado de la señal muscular. |
| [ServosyADC](./idk/ServosyADC/ServosyADC.kicad_pro) | Conversión analógica a digital y control de servos. |

Los esquemas y diseños de las placas se pueden consultar en KiCad.


## Redes sociales

- [Instagram](https://www.instagram.com/proyecto.apolo2026/)
- [TikTok](https://www.tiktok.com/@proyecto.apolo5)
