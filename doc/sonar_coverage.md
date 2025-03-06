# Multibeam Sonar Coverage

How do we get a marine robot to autonomously map an area of the seafloor?

## Coverage Tracker

The coverage tracker supplies a list of polygons needing coverage (polygons may have priorities assigned maybe?)

### coverage tracker inputs

#### desired coverage polygon(s)

This represents the operator's input. We can assume this is static for the whole mission, but we can think about making it modifiable if we want.

#### gridded data

 This would represent the metadata we need to determine if a location is adequately covered. This could be a multi-layered grid with layers representing uncertainty, ping count, etc. We can start with a single grid with uniform resolution, but we shouldn't preclude tiles and multi-resolution grids.

### output

#### list of polygons

May have metadata such as priority. Note that when starting with no data collected, this output would essentially be the input coverage polygon(s).

## Survey Planner

The survey planner takes a list of polygons, and optionally costmaps, and generates a path for the robot to follow. This could be split up in multiple levels of planning, like a higher level planner determining the order of the  polygons and a low level path planner that simply plans a path to multiple points.

### survey planner inputs

#### list of polygons with optional priority metadata

#### environment representation

Can be costmaps or other representations of dynamic obstacles

#### robot starting location (and optional ending location?)

### survey planner output

#### path for the robot to drive

As data is collected, the loop repeats as coverage increases and the path for the robot is updates as needed.

In a real time scenario, this could run in a loop every second perhaps, and the result might be very similar to running Damian's planner.

A few consideration to make this usable with ROS or without ROS:

- make the algorithms access the data via an api instead of prescribing a data format. So, for use with ROS, the algorithm could work with  costmaps or gridmaps without needing to copy to an internal format. With a stand-alone system, we can make up a simple format if necessary. This allows us to efficiently use ROS types in our environment without needing to depend on ROS libraries in a stand-alone.

- same with the low level path planner. We can simply use dubins to connect points for a simple stand alone version, but in ROS, we can use existing ROS Navigation planners in addition to dubins.
