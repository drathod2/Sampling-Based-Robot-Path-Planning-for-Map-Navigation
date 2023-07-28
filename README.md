# Sampling-Based-Robot-Path-Planning-for-Map-Navigation

This project provides an implementation of sampling-based robot path planning for map navigation. The project includes a PRM (Probabilistic Roadmap) class with four different sampling methods. The goal is to find a collision-free path for a robot navigating through a given map.

## Files Included

1. `ASU_map`: This binary ASU map image represents the map with school buildings. You can replace it with other maps of your preference.
2. `PRM.py`: This file contains the implementation of the PRM class, which includes the four different sampling methods.
3. `main.py`: This file includes the load map function, the PRM class instantiation, and functions to run and visualize the different samplings (random, uniform, Gaussian, and bridge).

## Instructions to Run the Code

To run the code and perform sampling-based robot path planning, follow these steps:

1. Ensure you have Python installed on your system.
2. Replace the `ASU_map` file with the map of your choice. The map should be a binary image with obstacles represented as black pixels and free space as white pixels.
3. Open the `main.py` file and run it. This will execute the sampling methods and visualize the results.

## Usage

After running the code, you will see the map displayed with the obstacles marked. The program will then perform sampling-based robot path planning using the PRM class and the four different sampling methods: random, uniform, Gaussian, and bridge.

The output will show the sampled points on the map, the constructed graph, the number of nodes and edges as well as the collision-free path found using each sampling method. The visualized paths will be marked on the map.

Feel free to experiment with different maps and compare the performance of the different sampling methods in terms of path quality and computation time.

## Results

After running the code, the terminal will display information about the sampling methods' performance and the collision-free paths found for the robot on the map.

**Terminal Output:**
![Terminal Output](results/terminal.png)

**Sampling Methods Plot:**
(results/Uniform.png)
(results/Random.png)
(results/Gaussian.png)
(results/Bridge.png)

The above screenshot shows the output of the terminal, displaying the sampled points on the map, the constructed graph, the number of nodes and edges, and the collision-free paths found using each sampling method.

The plot illustrates the comparison of the four sampling methods: random, uniform, Gaussian, and bridge. Each method's visualized path on the map is marked with different colors.

## Conclusion

This project provides an implementation of sampling-based robot path planning for map navigation. By running the code and modifying the map, you can observe the behavior and performance of different sampling methods in finding a collision-free path for a robot.
