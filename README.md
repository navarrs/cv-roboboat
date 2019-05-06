# Path planning 

This repository contains:
- A C++ implementation of the A* algorithm for path planning. 

## Prerequisites

```
	- Eigen3
	- Boost
	- OpenCV 3.4 
```

## Process (example)

### 1. Ground Annotator

Compile

```
	mkdir & cd build
	cmake ..
	make 
```

To test if it works, run the following command:
```
	cd $ROBOT_ROOT/build
	./astar
```

#### Testing

First, provide input map (default is $ROBOT_ROOT/maps/map1.png):
<p align="center"> <img src="./readme/input_map.png" /> </p>

Then, the algorithm converts the map to grayscale: 
<p align="center"><img src="./readme/gray_map.png" /> </p>

Then, the grayscale image is binarized:
<p align="center"><img src="./readme/binary_map.png" /> </p>

And morphological filters are applied to reduce risk of obstacle collision: 
<p align="center"><img src="./readme/dilated_map.png" /> </p>

Next, an ocuppancy grid is generated and user sets source and destination:
<p align="center"><img src="./readme/obstacle_map.png" /> </p>

Finally, the algorithm computes a trajectory:
<p align="center"><img src="./readme/final_map.png" /> </p>
