# Autonomiczne Roboty Mobilne
Podstawy estymacji stanu robotów autonomicznych i wprowadzenie do filtru Kalmana.

### Prowadzący: dr inż. Tomasz Gawron

## Autor
* Klaudia Sagat
* Michał heit

![diagram](https://github.com/jumpincrane/arm_gt_v2/blob/main/estimationdiagram.PNG)

Funkcje poszczególnych bloków diagramu:
* Input generator - generator wejść (trajektorii);
* Robot node - model kinematyczny pojazdu;
* Noise generator node - generator zaszumień (wejść i modelu kinamatycznego);
* RVIZ - wizualizacja ścieżki;
* Sate estimator node - węzeł publikujący zaszumione wejścia.  
