# Astro-Cargo airframe: model and print files

> 🌐 This page is a translation of the [Russian original](../../../../airframe/README.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

This is the airplane itself: the Fusion 360 project and the STL files for 3D printing. The electronics and the flight controller board are described in [FC_BOARD.md](../FC_BOARD.md), and the build and first flight in the [pilot's guide](../PILOT_GUIDE.md).

## Model version: v2

This folder contains **Astro-Cargo v2**. There will be no v1 in the repository: the first model is not being published, so the second one became the first to be released.

- [Fusion 360 project](../../../../airframe/fusion360/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.f3d), 14 MB;
- [STL file for printing](../../../../airframe/stl/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.stl), 6 MB.

The files are named so that the name shows right away what is wrong with this version (details below).

> [!WARNING]
> **Critical design flaws were found in v2:**
>
> 1. **The landing gear is attached to the fuselage too weakly.** The mount cannot carry the weight of the airplane, and the fuselage tears at the attachment point.
> 2. **There is no mount for the Velcro strap that holds the battery in place.**
>
> Both flaws will be fixed in the next prototype, **Astro-Cargo v3**. Its model will appear in this folder as soon as it is ready. Until then, do not fly v2 without modifying it: strengthen the landing gear mount and make room for the Velcro strap yourself.

## What is where

| Folder | What it holds |
|---|---|
| [`fusion360/`](../../../../airframe/fusion360/) | The source project: an `.f3d` file (or an `.f3z` archive if the project has several files). You can change dimensions in it and export the parts again |
| [`stl/`](../../../../airframe/stl/) | Print-ready parts in STL format |

## How to name files

- Names use Latin letters and include the version number. The v2 files are named so that the name tells you about the flaws, but from now on it is better to avoid spaces and parentheses: `astro-cargo_v3.f3d`, `astro-cargo_v3.stl`. That makes it easier to link to a file from the documentation. If there are several parts, each one gets its own file: `fuselage_v3.stl`, `wing_left_v3.stl`.
- The v3 files will sit next to them (`astro-cargo_v3.f3d`), and the v2 files will stay, so you can see exactly what was fixed.
- Units are millimeters. If the project uses different ones, say so next to the file.

## If a file is too large

GitHub does not accept files larger than 100 MB and starts warning at 50 MB. So check the file size before committing. Do not put such files in the repository; use one of these instead:

- the **Releases** section on GitHub: a file can be attached to a release and may weigh up to 2 GB;
- [Git LFS](https://git-lfs.com), if the file should live in the repository itself and change together with the code;
- external hosting, with a link to it added to this README.

Git treats `.stl`, `.f3d`, `.f3z`, `.step`, `.stp` and `.3mf` files in this folder as binary (see [`.gitattributes`](../../../../.gitattributes)): it does not change their line endings and does not show line-by-line differences.

## License

The model is distributed under the same terms as the whole project: the [OpenPlane License](../../../../LICENSE), that is, MIT with mandatory credit to the author, a ban on military use, and a ban on intentionally harming people or property without their consent. You may print, modify and improve it within these terms, but you must credit the author, Damir Lebedev (Damn / Проклятый).
