## Computer Vision and A.I. - VantTEC 

# VANTEC-ROS
# Detection on Aquatic Surfaces 

## Requerimientos
- OpenCV 3.4+
- Python2.7 o Python3.5
- ROS Kinetic
- Ubuntu 16.04 

## Referencias
``
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
## Configurar ambiente en ROS (bash)
```
source /opt/ros/kinetic/setup.bash
mkdir -p ros_vanttec/src
cd ros_vanttec
catkin_make
source devel/setup.bash
```

## Crear paquete en ambiente
```
roscd ~/ros_vantec/src
catkin_create_pkg boat roscpp rospy std_msg cv_bridge
```

## En el CMakeLists.txt que se encuentra en el paquete boat agregar lo siguiente:
```
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
```

## Finalmente, ejecutar:
```
cd ~/ros_vanttec
catkin_make
```

## Detector + Tracker

Dentro de la carpeta ros_vanttec, ejecutar: 
```
roscore
```

En otra terminal, si es necesario re-compilar ejecutar:

Dentro de la carpeta boat-vision, ejecutar (bash): 
```
catkin_make
source devel/setup.bash
```

En la terminal del publisher de detecciones, ejecutar:
```
rosrun boat detector_node.py --config vanttec/config/tiny3-vantec.cfg --weights vanttec/weights/tiny3-vantec.weights --classes vanttec/obj.names --video vanttec/video.mp4
```

Ejemplo de resultado:
<p align="center"><img src="./readme/det-track.png" /> </p>

## TO DO:
- [ ] Nodo de deteccion 
- [ ] Nodo de tracking
- [ ] Nodo para webcams
- [ ] Nodo para reconocimiento de color
- [ ] Nodo de estimacion de distancias
- [ ] Nodo de deteccion de numeros