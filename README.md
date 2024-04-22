# Minimum-Spanning-Tree-and-Shortest-Path-Algorithms
This repository contains Java implementations of two fundamental graph algorithms: Kruskal's algorithm for finding the minimum spanning tree (MST) and Dijkstra's algorithm for finding the shortest paths from a source vertex to all other vertices in a weighted graph.

The project provides a menu-driven interface that allows users to select the desired algorithm and input the graph data from a file. The program reads the graph information, including the number of vertices, edges, and their weights, from the specified input file.

# Key Features:

### Kruskal's Algorithm:

* Finds the minimum spanning tree of a weighted undirected graph.
* Uses a depth-first search (DFS) approach to detect cycles and ensure a valid MST.
* Prints the total weight of the MST and the edges included in the MST.
* Measures and displays the running time of the algorithm.

### Dijkstra's Algorithm:

* Finds the shortest paths from a source vertex to all other vertices in a weighted graph.
* Utilizes a priority queue to efficiently select the vertex with the minimum distance.
* Prints the shortest paths from the source vertex to all other vertices and their corresponding lengths.
* Measures and displays the running time of the algorithm.
The program provides a user-friendly interface that guides the user through the process of selecting the algorithm, entering the source vertex (for Dijkstra's algorithm), and displaying the results.

# Input Format:
The input files should follow a specific format:

### For Kruskal's algorithm (MST):

The first line contains the number of vertices in the graph.
Each subsequent line represents an edge, with the format: source target weight.

### For Dijkstra's algorithm (shortest paths):

The first line contains the number of vertices and edges in the graph.
Each subsequent line represents an edge, with the format: source target weight.

## Output Format:
The program generates output in a readable format, displaying the results of the selected algorithm. For Kruskal's algorithm, it shows the total weight of the MST and the edges included in the MST. For Dijkstra's algorithm, it presents the shortest paths from the source vertex to all other vertices and their corresponding lengths. The running time of each algorithm is also displayed.

This project serves as a practical implementation of Kruskal's and Dijkstra's algorithms, demonstrating their usage in finding the minimum spanning tree and shortest paths in weighted graphs. It provides a solid foundation for understanding and applying these fundamental graph algorithms in various problem-solving scenarios.

Feel free to explore the code, run the program with different input files, and enhance the functionality as needed. Contributions and suggestions are welcome!
