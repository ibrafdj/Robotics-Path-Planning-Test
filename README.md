# About

This is a python program that can test and visualize the A* path planning algorithnm to navigate a robot between two points in a 2D map.

# Dependencies
* matplotlib

# How-to-run
* Install python from [here](https://www.python.org/downloads/)
* Install matplotlib from [here](https://matplotlib.org/stable/users/installing.html)
* Open the program directory in terminal or command prompt and run the program with the following command:
```bash
python path_planning_test.py
```

# Scenario

The program will initialize the following maze as a 2D map:

| ![image](https://user-images.githubusercontent.com/72083779/118756724-c0402800-b895-11eb-85b3-66b815376a2c.png) |
|:--:| 
|*Figure 1: A 2D map of a maze*|

The red dot shows the start point and the magenta dot shows the goal point of the robot. The A* algorithm will search the maze 

# Description

## A* Algorithm
### Definition
A* is a pathfinding algorithm that can find the shortest path between two nodes in a graph. 

A* chooses the path with the minimum value of the function f(n) as defined as follows:

> **f(n) = g(n) + h(n)**

Where:
* **n** is the next node (Neighbouring node) on the path
* **g(n)** is the actual cost of the path from the start node to *n*
* **h(n)** is the estimated cost of the path from **n** to 

| ![image](https://user-images.githubusercontent.com/72083779/118751071-e1e7e200-b88a-11eb-84d7-9824e27d575f.png) |
|:--:| 
|*Figure 1: An example of a weighted graph*| 
|*Source: [isaaccomputerscience.org](https://isaaccomputerscience.org/concepts/dsa_search_a_star)*|
  
### How it works
The A* algorithm executes the following steps to determine the minimum path between two nodes:

1. Insert all nodes into a set of unvisited nodes called **open set**

2. Initialize the stored f(n) values of all nodes as **Infinity** and the address of the previous node of all nodes as **None**

3. Set the start node as the currently visited node

4. From the current node, choose a neighboring node with the minimum value of f(n) = g(n) + h(n)
    * The value of **g(n)** is determined by the total cost of path from the start node to reach the neighbouring node (Denoted by the black numbers in Figure 1)  
    * The value of **h(n)** is determined by the value of a heuristic function that estimates the cost between the neighboring node to the goal node (Denoted by the orange numbers in Figure 1)
        * Example of heuristic functions **h(n)** that can be used are [Euclidean distance](https://en.wikipedia.org/wiki/Great-circle_distance), [Manhattan distance](https://en.wikipedia.org/wiki/Taxicab_geometry), and [Great-circle distance](https://en.wikipedia.org/wiki/Great-circle_distance)

5. Consider the values of f(n) and the address of the previous node for each neighboring nodes
    * If the current f(n) value of a neighboring node is lower than the stored f(n) value of that node store the value f(n) and the previous node of **n**
    * Else, ignore the node **n**

6. Once all neighbouring nodes **n** of the current node has been considered, remove the current node from the **open set** and insert it into a set of visited nodes called **closed set**

7. Repeat steps 3 to 6 until the current considered node is the goal node
    * If there are no available path from the start node to the goal node, then the algorithm cannot find the minimum path

8. The minimum path to reach the goal node from the start node can be determined by:
    * Taking the address of the previous node of the goal node and storing it in a list
    * Taking the address of the following previous nodes and storing it in a list until we reach the start node
    * The completed list contains the ordered minimum path from the start node to the goal node


## Results

# To-do

# Additional References to Learn A*

Additional video references that can be accessed to learn A*:
* [A* Search - John Levine](https://www.youtube.com/watch?v=ySN5Wnu88nE)
* [A* (A Star) Search Algorithm - Computerphile](https://www.youtube.com/watch?v=ySN5Wnu88nE)

