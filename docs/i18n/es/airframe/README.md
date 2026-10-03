# Estructura del Astro-Cargo: modelo y archivos para imprimir

> 🌐 Esta página es una traducción del [original en ruso](../../../../airframe/README.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

Aquí está el avión en sí: el proyecto de Fusion 360 y los archivos STL para la impresión 3D. La electrónica y la placa del controlador de vuelo se describen en [FC_BOARD.md](../FC_BOARD.md), y el montaje y el primer vuelo, en la [guía del piloto](../PILOT_GUIDE.md).

## Versión del modelo: v2

En esta carpeta está el **Astro-Cargo v2**. No habrá v1 en el repositorio: el primer modelo no se publica, así que el segundo pasó a ser el primero en salir a la luz.

- [Proyecto de Fusion 360](../../../../airframe/fusion360/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.f3d), 14 MB;
- [Archivo STL para imprimir](../../../../airframe/stl/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.stl), 6 MB.

Los archivos se llaman así para que el nombre deje claro de inmediato qué falla en esta versión (detalles más abajo).

> [!WARNING]
> **Se han encontrado fallos críticos de diseño en la v2:**
>
> 1. **La fijación del tren de aterrizaje al fuselaje es demasiado débil.** No aguanta el peso del avión y el fuselaje se desgarra en el punto de anclaje.
> 2. **No hay soporte para la cinta de velcro que sujeta la batería.**
>
> Ambos fallos se corregirán en el siguiente prototipo, el **Astro-Cargo v3**. Su modelo aparecerá en esta carpeta en cuanto esté listo. Hasta entonces, no vueles con la v2 sin modificarla: refuerza tú mismo la fijación del tren de aterrizaje y prepara un lugar para la cinta de velcro.

## Qué hay en cada carpeta

| Carpeta | Contenido |
|---|---|
| [`fusion360/`](../../../../airframe/fusion360/) | El proyecto fuente: un archivo `.f3d` (o un archivo comprimido `.f3z` si el proyecto tiene varios archivos). Permite cambiar las dimensiones y volver a exportar las piezas |
| [`stl/`](../../../../airframe/stl/) | Piezas listas para imprimir, en formato STL |

## Cómo nombrar los archivos

- Los nombres van en letras latinas y llevan el número de versión. Los archivos de la v2 se llaman así para que el nombre hable de sus fallos, pero en adelante conviene evitar espacios y paréntesis: `astro-cargo_v3.f3d`, `astro-cargo_v3.stl`. Así es más fácil enlazar el archivo desde la documentación. Si hay varias piezas, cada una va en su propio archivo: `fuselage_v3.stl`, `wing_left_v3.stl`.
- Los archivos de la v3 estarán al lado (`astro-cargo_v3.f3d`) y los de la v2 se conservarán: así se ve qué se ha corregido exactamente.
- Las unidades son milímetros. Si el proyecto usa otras, indícalo junto al archivo.

## Si un archivo es demasiado grande

GitHub no admite archivos de más de 100 MB y avisa ya a partir de 50 MB. Por eso, comprueba el tamaño del archivo antes de hacer el commit. Esos archivos no deben ir en el repositorio; colócalos en su lugar:

- en la sección **Releases** de GitHub: el archivo se puede adjuntar a una versión publicada y puede pesar hasta 2 GB;
- en [Git LFS](https://git-lfs.com), si el archivo debe estar en el propio repositorio y cambiar junto con el código;
- en un alojamiento externo, añadiendo el enlace a este README.

Git trata los archivos `.stl`, `.f3d`, `.f3z`, `.step`, `.stp` y `.3mf` de esta carpeta como binarios (véase [`.gitattributes`](../../../../.gitattributes)): no modifica sus saltos de línea ni muestra diferencias línea por línea.

## Licencia

El modelo se distribuye en los mismos términos que todo el proyecto: la [OpenPlane License](../LICENSE.md), es decir, MIT con atribución obligatoria al autor, prohibición del uso militar y prohibición de dañar intencionadamente a personas o bienes sin su consentimiento. Puedes imprimirlo, modificarlo y mejorarlo dentro de estos términos, pero debes mencionar al autor, Damir Lebedev (Damn / Проклятый).
