## Computer Vision and A.I. - VantTEC 

# Detection and Tracking on Aquatic Surfaces

This repository contains:


References
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


