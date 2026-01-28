# Camera Subscriber (ROS 2)

## Ogólny opis pakietu
Pakiet camera_subscriber jest pakietem systemu ROS 2, który demonstruje możliwości integracji środowiska ROS z biblioteką OpenCV oraz prostego sterowania manipulatorem robota na podstawie interakcji użytkownika.

Pakiet subskrybuje strumień wideo z kamery i wykorzystuje go do utworzenia okna graficznego, w którym użytkownik może wykonywać kliknięcia myszą. Położenie kliknięcia (powyżej lub poniżej środka ekranu) interpretowane jest jako polecenie ruchu manipulatora w określonym kierunku.

Rozwiązanie to stanowi prosty przykład połączenia paczki obslługi kamery USB, OpenCV oraz sterowania ruchem robota w systemie ROS 2.

## Zakres funkcjonalny
Pakiet camera_subscriber realizuje następujące funkcje:
- subskrypcja strumienia wideo z kamery,
- interakcja użytkownika poprzez kliknięcia myszą,
- sterowanie manipulatorem robota,
- publikacja trajektorii ruchu.

## Instalacja
Aby zainstalować pakiet camera_subscriber, należy wykonać poniższe kroki:

Umieścić pakiet w katalogu src roboczego środowiska ROS 2:
```bash
cd ~/ros2_ws/src
```
Zbudować workspace przy użyciu narzędzia colcon:
```bash
colcon build
```
Po zakończeniu budowania załadować środowisko ROS 2:
```bash
source install/setup.bash
```
Po wykonaniu powyższych kroków pakiet jest gotowy do uruchomienia.

## Uruchomienie
Uruchomienie pakietu odbywa się z wykorzystaniem pliku launch:
```bash
ros2 launch camera_subscriber camera_node.launch.py
```

## Uruchomienie w kontenerze
Pobieramy obraz ROS:
```bash
docker pull ros:humble
```
Wyłączamy kontrolę dostępu do X11:
```bash
xhost +
```
Uruchomienie kontenera:
```bash
docker run -it --privileged \
-v ~"sciezka do pakietu na hoscie":"sciezka do pakietu w kontenerze" \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=$DISPLAY \
--net=host \
--user root \
--name=ros2_docker ros:humble
```
W pseudoterminalu instaluemy paczke usb-cam:
```bash
apt update
apt-get install ros-humble-usb-cam
```
Uruchomienie paczki wewnatrz kontenera jest analogiczne do powyzszego.

