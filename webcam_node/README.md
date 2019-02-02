# Nodo suscrito a 2 camaras

### Prerequisitos

Instalar los paquetes de image_view, usb_cam, cv_bridge, opencv para ros

```
sudo apt-get install ros-kinetic-opencv3
sudo apt-get install ros-kinetic-usb-cam
sudo apt-get install ros-kinetic-image-view
sudo apt-get install ros-kinetic-cv-bridge

```

### Ejecutar Nodo

1.Se ejecuta el archivo launch en la carpeta launch


```
roslaunch camara_launch.launch

```

2.Se ejecuta el nodo nodo_camaras_ros_cv.py en la carpeta scripts

```
python nodo_camaras_ros_cv.py

```


