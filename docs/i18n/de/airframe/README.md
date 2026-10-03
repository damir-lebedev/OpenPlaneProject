# Astro-Cargo-Flugzeugzelle: Modell und Druckdateien

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../../airframe/README.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert.

Hier liegt das Flugzeug selbst: das Fusion-360-Projekt und die STL-Dateien für den 3D-Druck. Die Elektronik und die Flugsteuerungsplatine sind in [FC_BOARD.md](../FC_BOARD.md) beschrieben, Aufbau und Erstflug im [Pilotenleitfaden](../PILOT_GUIDE.md).

## Modellversion: v2

In diesem Ordner liegt die **Astro-Cargo v2**. Eine v1 wird es im Repository nicht geben: Das erste Modell wird nicht veröffentlicht, deshalb ist die zweite Version die erste veröffentlichte geworden.

- [Fusion-360-Projekt](../../../../airframe/fusion360/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.f3d), 14 MB;
- [STL-Datei zum Drucken](../../../../airframe/stl/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.stl), 6 MB.

Die Dateien sind so benannt, dass der Name sofort zeigt, was an dieser Version nicht stimmt (Details unten).

> [!WARNING]
> **In der v2 wurden kritische Konstruktionsmängel gefunden:**
>
> 1. **Das Fahrwerk ist zu schwach am Rumpf befestigt.** Die Befestigung hält das Gewicht des Flugzeugs nicht aus, und der Rumpf reißt an der Befestigungsstelle ein.
> 2. **Es gibt keine Halterung für das Klettband, das den Akku fixiert.**
>
> Beide Mängel werden im nächsten Prototyp, der **Astro-Cargo v3**, behoben. Ihr Modell erscheint in diesem Ordner, sobald es fertig ist. Bis dahin sollten Sie die v2 nicht unverändert fliegen: Verstärken Sie die Fahrwerksbefestigung selbst und schaffen Sie Platz für das Klettband.

## Was wo liegt

| Ordner | Inhalt |
|---|---|
| [`fusion360/`](../../../../airframe/fusion360/) | Das Quellprojekt: eine `.f3d`-Datei (oder ein `.f3z`-Archiv, wenn das Projekt aus mehreren Dateien besteht). Daraus lassen sich Maße ändern und Teile erneut exportieren |
| [`stl/`](../../../../airframe/stl/) | Druckfertige Teile im STL-Format |

## Wie Dateien benannt werden

- Namen bestehen aus lateinischen Buchstaben und tragen die Versionsnummer. Die Dateien der v2 sind so benannt, dass der Name auf die Mängel hinweist; künftig sollten Leerzeichen und Klammern aber vermieden werden: `astro-cargo_v3.f3d`, `astro-cargo_v3.stl`. So lässt sich in der Dokumentation leichter auf die Datei verweisen. Bei mehreren Teilen hat jedes Teil seine eigene Datei: `fuselage_v3.stl`, `wing_left_v3.stl`.
- Die Dateien der v3 liegen daneben (`astro-cargo_v3.f3d`), die der v2 bleiben erhalten: So sieht man, was genau behoben wurde.
- Die Einheit ist Millimeter. Falls das Projekt andere Einheiten verwendet, vermerken Sie das neben der Datei.

## Wenn eine Datei zu groß ist

GitHub nimmt keine Dateien über 100 MB an und warnt schon ab 50 MB. Prüfen Sie daher vor dem Commit die Dateigröße. Solche Dateien gehören nicht ins Repository, sondern:

- in den Bereich **Releases** auf GitHub: Die Datei lässt sich an ein Release anhängen und darf bis zu 2 GB groß sein;
- in [Git LFS](https://git-lfs.com), wenn die Datei direkt im Repository liegen und sich zusammen mit dem Code ändern soll;
- auf einen externen Hoster, wobei Sie den Link in dieser README ergänzen.

Git behandelt die Dateien `.stl`, `.f3d`, `.f3z`, `.step`, `.stp` und `.3mf` in diesem Ordner als Binärdateien (siehe [`.gitattributes`](../../../../.gitattributes)): Es ändert ihre Zeilenenden nicht und zeigt keine zeilenweisen Unterschiede an.

## Lizenz

Das Modell wird zu denselben Bedingungen verbreitet wie das gesamte Projekt: unter der [OpenPlane License](../LICENSE.md), also MIT mit verpflichtender Nennung des Urhebers, einem Verbot der militärischen Nutzung und einem Verbot, Menschen oder Sachen ohne ihre Einwilligung vorsätzlich zu schädigen. Sie dürfen es in diesem Rahmen drucken, verändern und verbessern, müssen aber den Urheber nennen: Damir Lebedev (Damn / Проклятый).
