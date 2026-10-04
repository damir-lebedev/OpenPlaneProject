# Cellule Astro-Cargo : modèle et fichiers d’impression

> 🌐 Cette page est la traduction de l’[original en russe](../../../../airframe/README.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

Voici l’avion lui-même : le projet Fusion 360 et les fichiers STL pour l’impression 3D. L’électronique et la carte du contrôleur de vol sont décrites dans [FC_BOARD.md](../FC_BOARD.md), le montage et le premier vol dans le [guide du pilote](../PILOT_GUIDE.md).

## Version du modèle : v2

Ce dossier contient l’**Astro-Cargo v2**. Il n’y aura pas de v1 dans le dépôt : le premier modèle n’est pas publié, c’est donc la deuxième version qui est devenue la première à être mise en ligne.

- [Projet Fusion 360](../../../../airframe/fusion360/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.f3d), 14 Mo ;
- [Fichier STL à imprimer](../../../../airframe/stl/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.stl), 6 Mo.

Les fichiers portent ces noms pour que le nom indique d’emblée ce qui ne va pas dans cette version (détails ci-dessous).

> [!WARNING]
> **Des défauts de conception critiques ont été trouvés dans la v2 :**
>
> 1. **La fixation du train d’atterrissage au fuselage est trop faible.** Elle ne supporte pas le poids de l’avion, et le fuselage se déchire au niveau de la fixation.
> 2. **Il n’y a aucun support pour la sangle Velcro qui retient la batterie.**
>
> Les deux défauts seront corrigés dans le prochain prototype, l’**Astro-Cargo v3**. Son modèle apparaîtra dans ce dossier dès qu’il sera prêt. D’ici là, ne faites pas voler la v2 sans la modifier : renforcez vous-même la fixation du train d’atterrissage et prévoyez un emplacement pour la sangle Velcro.

## Ce qui se trouve où

| Dossier | Contenu |
|---|---|
| [`fusion360/`](../../../../airframe/fusion360/) | Le projet source : un fichier `.f3d` (ou une archive `.f3z` si le projet comporte plusieurs fichiers). On peut y modifier les dimensions et réexporter les pièces |
| [`stl/`](../../../../airframe/stl/) | Les pièces prêtes à imprimer, au format STL |

## Comment nommer les fichiers

- Les noms s’écrivent en lettres latines et comportent le numéro de version. Les fichiers de la v2 sont nommés de façon que leur nom signale les défauts, mais il vaut mieux désormais éviter les espaces et les parenthèses : `astro-cargo_v3.f3d`, `astro-cargo_v3.stl`. Il est ainsi plus facile de renvoyer au fichier depuis la documentation. S’il y a plusieurs pièces, chacune a son propre fichier : `fuselage_v3.stl`, `wing_left_v3.stl`.
- Les fichiers de la v3 seront placés à côté (`astro-cargo_v3.f3d`) et ceux de la v2 resteront : on voit ainsi ce qui a été corrigé exactement.
- Les unités sont des millimètres. Si le projet en utilise d’autres, indiquez-le à côté du fichier.

## Si un fichier est trop volumineux

GitHub n’accepte pas les fichiers de plus de 100 Mo et avertit dès 50 Mo. Vérifiez donc la taille du fichier avant de valider (commit). Ces fichiers ne doivent pas aller dans le dépôt ; placez-les plutôt :

- dans la section **Releases** de GitHub : le fichier peut être joint à une version publiée et peser jusqu’à 2 Go ;
- dans [Git LFS](https://git-lfs.com), si le fichier doit se trouver dans le dépôt même et évoluer avec le code ;
- sur un hébergement externe, en ajoutant le lien à ce README.

Git traite les fichiers `.stl`, `.f3d`, `.f3z`, `.step`, `.stp` et `.3mf` de ce dossier comme des fichiers binaires (voir [`.gitattributes`](../../../../.gitattributes)) : il ne modifie pas leurs fins de ligne et n’affiche pas de différences ligne par ligne.

## Licence

Le modèle est distribué aux mêmes conditions que l’ensemble du projet : la [OpenPlane License](../LICENSE.md), c’est-à-dire la licence MIT avec mention obligatoire de l’auteur, interdiction de l’usage militaire et interdiction de nuire intentionnellement aux personnes ou aux biens sans leur consentement. Vous pouvez l’imprimer, le modifier et l’améliorer dans ce cadre, mais vous devez mentionner l’auteur, Damir Lebedev (Damn / Проклятый).
