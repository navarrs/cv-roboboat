## Computer Vision and A.I. - VantTEC 

# Install required modules

Tip: When you're ready to deploy the application to other computers, you can create a requirements.txt file with the command 
```bash
pip freeze > requirements.txt 
```
(pip3 on macOS/Linux). The requirements file describes the packages you've installed in your virtual environment. With only this file, you or other developers can restore those packages using
```bash
pip install -r requirements.txt 
```
(or, again, pip3 on macOS/Linux). **By using a requirements file, you need not commit the virtual environment itself to source control**.

From: [MicrosoftVSCode Docs](https://code.visualstudio.com/docs/python/environments#_manually-specify-an-interpreter)

# Detection and Tracking on Aquatic Surfaces

Referencias
```
@article{redmon2016yolo9000,
  title={YOLO9000: Better, Faster, Stronger},
  author={Redmon, Joseph and Farhadi, Ali},
  journal={arXiv preprint arXiv:1612.08242},
  year={2016}
}

@article{yolov3,
  title={YOLOv3: An Incremental Improvement},
  author={Redmon, Joseph and Farhadi, Ali},
  journal = {arXiv},
  year={2018}
}
```

## Detector + Tracker

NOTA: Por el momento solo se tiene el CentroidTracker. 

Dentro de la carpeta boat-vision, ejecutar: 
```

python3 main.py --config vanttec/config/yolo3-vantec.cfg --weights vanttec/weights/yolo3-vantec.weights --classes vanttec/obj.names --video vanttec/video.mp4

```
Ejemplo de resultado:
<p align="center"><img src="./readme/det-track.png" /> </p>

## TO DO:
- [x] Deteccion de objetos
- [ ] Buscar un metodo mas preciso para realizar rastreo de objetos (Tracking) 
- [ ] Optimizacion de sistema de deteccion - rastreo
- [ ] Reconocimiento de colores
- [ ] Estimacion de distancias con respecto a las camaras (Stereo)
- [ ] Planeacion de trayectorias
- [ ] Calibracion de camaras
- [ ] Deteccion de numeros
- [ ] Planeacion de retos de RoboBoat


# Retos RoboBoat

## Introductorios 

### 1. Maintain Heading 

<p align="center"><img src="./readme/intro-heading.png" /> </p>

### 2. Slalom Maneuver

<p align="center"><img src="./readme/intro-slalom.png" /> </p>

## Navegacion autonoma

<p align="center"><img src="./readme/autonomous.png" /> </p>

## Misiones

### 1. Speed Challenge

<p align="center"><img src="./readme/mission-speed.png" /> </p>

### 2. Raise the flag

<p align="center"><img src="./readme/mission-flag.png" /> </p>

### 3. Find the path 

<p align="center"><img src="./readme/mission-path.png" /> </p>

### 4. Follow the leader

<p align="center"><img src="./readme/mission-leader.png" /> </p>