# Local Costmap Generator Node (ROS, C++)

## Intro
Generate Cost Map using Point Cloud Height Map algorithm

## Running
After building this ROS package using 'catkin_make' in your catkin workspace, launch the heightmap_node with heightmap_costmap_node:

`roslaunch local_costmap_generator run.launch`

## TODO (in heightmap.cpp)
TODO: find and change the topic name of the point cloud data
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap.cpp#L58-L62

TODO: convert the position of the points from the camera to the local coordinate
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap.cpp#L79-L84
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap.cpp#L102-L107
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap.cpp#L149-L154
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap.cpp#L174-L178

## Parameters (in heightmap.cpp)
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap.cpp#L43-L46

## Parameters (in heightmap_to_costmap.cpp)
https://github.com/hynkis/local_costmap_generator/blob/fc3f4a86131d87b010f53c9b48512b4afb0942b3/src/heightmap_to_costmap.cpp#L29-L38

## Topics to subscribe

* /points <-- change to correct topic name

## Topics to publish

* /points/velodyne_obstacles
* /points/velodyne_clear
* /map/local_map/obstacle
