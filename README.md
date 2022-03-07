# Autonomiczne Roboty Mobilne
Podstawy estymacji stanu robotów autonomicznych i wprowadzenie do filtru Kalmana.

### Prowadzący: dr inż. Tomasz Gawron

## Autor
* Klaudia Sagat
* Michał heit

## Część I: Zamodelowanie modelu robota i wizualizacja przebytej ścieżki

![diagram](https://github.com/jumpincrane/arm_gt_v2/blob/main/estimationdiagram.PNG)

Funkcje poszczególnych bloków diagramu:
* Input generator - generator wejść (trajektorii);
* Robot node - model kinematyczny pojazdu;
* Noise generator node - generator zaszumień (wejść i modelu kinamatycznego);
* RVIZ - wizualizacja ścieżki;
* Sate estimator node - węzeł publikujący zaszumione wejścia.  

## Uruchomienie
Jest to paczka ROS1:
* Tworzymy workspace dla ROS
* Budujemy workspace
* W folderze source klonujemy paczke
* z poziomu workspace'u możemy włączyć paczkę za pomocą launch'a
`roslaunch ARM_robot_state_estimation arm_gt.launch`
