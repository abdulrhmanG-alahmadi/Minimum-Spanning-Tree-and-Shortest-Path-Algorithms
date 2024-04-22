/*
Abdulrhman Alahmadi
2036026
Osama Khalid Bayunus
1936825
 */
import java.io.*;
import java.util.*;

public class GraphAlgorithms {
    static class Edge {
        int source;
        int target;
        double weight;

        Edge(int source, int target, double weight) {
            this.source = source;
            this.target = target;
            this.weight = weight;
        }
    }

    static class Graph {
        int vertices;
        ArrayList<Edge>[] adjacencyList;

        Graph(int vertices) {
            this.vertices = vertices;
            adjacencyList = new ArrayList[vertices];
            for (int i = 0; i < vertices; i++) {
                adjacencyList[i] = new ArrayList<>();
            }
        }

        void addEdge(int source, int target, double weight) {
            Edge edge = new Edge(source, target, weight);
            adjacencyList[source].add(edge);
        }
    }

    // Function to find MST using Prim's algorithm
    static void primMST(double[][] adjacencyMatrix) {
        int vertices = adjacencyMatrix.length;
        boolean[] visited = new boolean[vertices];
        double[] key = new double[vertices];
        int[] parent = new int[vertices];

        // Initialize keys to infinity and visited to false
        for (int i = 0; i < vertices; i++) {
            key[i] = Double.MAX_VALUE;
            visited[i] = false;
        }

        // Start with vertex 0
        key[0] = 0;
        parent[0] = -1;

        // Find MST using unordered Min-Priority Queue
        for (int i = 0; i < vertices; i++) {
            int minIndex = findMinKey(key, visited);
            visited[minIndex] = true;

            for (int j = 0; j < vertices; j++) {
                if (adjacencyMatrix[minIndex][j] != 0 && !visited[j] && adjacencyMatrix[minIndex][j] < key[j]) {
                    key[j] = adjacencyMatrix[minIndex][j];
                    parent[j] = minIndex;
                }
            }
        }

        // Print MST weight and edges
        double totalWeight = 0;
        ArrayList<int[]> mstEdges = new ArrayList<>();
        for (int i = 1; i < vertices; i++) {
            int source = parent[i];
            int target = i;
            double weight = adjacencyMatrix[source][target];
            mstEdges.add(new int[]{source, target, (int) weight});
            totalWeight += weight;
        }
        System.out.println("Total weight of MST by Prim's algorithm (Using unordered Min-Priority queue): " + totalWeight);

        System.out.println("The edges in the MST are:");
        mstEdges.sort((a, b) -> {
            if (a[0] != b[0]) {
                return Integer.compare(a[0], b[0]);
            } else {
                return Integer.compare(a[2], b[2]);
            }
        });
        for (int[] edge : mstEdges) {
            System.out.println("Edge from " + edge[0] + " to " + edge[1] + " has weight " + edge[2] + ".0");
        }
    }

    // Function to find the vertex with minimum key value
    static int findMinKey(double[] key, boolean[] visited) {
        double minValue = Double.MAX_VALUE;
        int minIndex = -1;

        for (int i = 0; i < key.length; i++) {
            if (!visited[i] && key[i] < minValue) {
                minValue = key[i];
                minIndex = i;
            }
        }
        return minIndex;
    }

    // Function to find MST using Kruskal's algorithm
    static void kruskalMST(ArrayList<Edge> edges, int vertices) {
        ArrayList<Edge> mst = new ArrayList<>();
        int[] parent = new int[vertices];

        // Initialize parent array for Union-Find
        for (int i = 0; i < vertices; i++) {
            parent[i] = i;
        }

        // Sort edges in non-decreasing order of weight
        Collections.sort(edges, Comparator.comparingDouble(e -> e.weight));

        // Find MST using Union-Find approach
        for (Edge edge : edges) {
            int sourceParent = find(parent, edge.source);
            int targetParent = find(parent, edge.target);

            if (sourceParent != targetParent) {
                mst.add(edge);
                union(parent, sourceParent, targetParent);
            }
        }

        // Print MST edges and weight
        double totalWeight = 0;
        for (Edge edge : mst) {
            totalWeight += edge.weight;
        }
        System.out.println("Total weight of MST by Kruskal's algorithm: " + totalWeight);
        totalWeight = 0;
        System.out.println("The edges in the MST are:");
        for (Edge edge : mst) {
            System.out.println("Edge from " + edge.source + " to " + edge.target + " has weight " + edge.weight);
            totalWeight += edge.weight;
        }
    }

    // Find operation for Union-Find
    static int find(int[] parent, int vertex) {
        if (parent[vertex] != vertex) {
            parent[vertex] = find(parent, parent[vertex]);
        }
        return parent[vertex];
    }

    // Union operation for Union-Find
    static void union(int[] parent, int sourceParent, int targetParent) {
        parent[sourceParent] = targetParent;
    }
    // Function to find the shortest path using Dijkstra's algorithm
    static void dijkstra(double[][] adjacencyMatrix, int source) {
        int vertices = adjacencyMatrix.length;
        double[] distance = new double[vertices];
        boolean[] visited = new boolean[vertices];
        int[] prev = new int[vertices];

        // Initialize distances to infinity, visited to false, and prev to -1
        for (int i = 0; i < vertices; i++) {
            distance[i] = Double.MAX_VALUE;
            visited[i] = false;
            prev[i] = -1;
        }

        // Distance from source to itself is 0
        distance[source] = 0;

        // Find shortest path using Min-Priority Queue
        for (int i = 0; i < vertices - 1; i++) {
            int minIndex = findMinDistance(distance, visited);
            visited[minIndex] = true;

            for (int j = 0; j < vertices; j++) {
                if (!visited[j] && adjacencyMatrix[minIndex][j] != 0 && distance[minIndex] != Double.MAX_VALUE
                        && distance[minIndex] + adjacencyMatrix[minIndex][j] < distance[j]) {
                    distance[j] = distance[minIndex] + adjacencyMatrix[minIndex][j];
                    prev[j] = minIndex;
                }
            }
        }

        // Print shortest paths from the source vertex
        System.out.println("Shortest paths from vertex " + source + " are:");
        for (int i = 0; i < vertices; i++) {
            System.out.print("A path from " + source + " to " + i + ": ");
            printPath(i, prev, source);
            System.out.printf("  (Length: %.1f)\n", distance[i]);
        }
    }
    // Function to find the vertex with the minimum distance
    static int findMinDistance(double[] distance, boolean[] visited) {
        double minValue = Double.MAX_VALUE;
        int minIndex = -1;

        for (int i = 0; i < distance.length; i++) {
            if (!visited[i] && distance[i] < minValue) {
                minValue = distance[i];
                minIndex = i;
            }
        }
        return minIndex;
    }

    // Function to print the shortest path from source to destination
    static void printPath(int destination, int[] prev, int source) {
        if (destination == source) {
            System.out.print(source);
        } else if (prev[destination] == -1) {
            System.out.print("No path from " + source + " to " + destination);
        } else {
            printPath(prev[destination], prev, source);
            System.out.print(" " + destination);
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        int choice;

        do {
            System.out.println("Menu:");
            System.out.println("1. Finding minimum spanning tree using Prim's algorithm");
            System.out.println("2. Finding minimum spanning tree using Kruskal's algorithm");
            System.out.println("3. Finding shortest path using Dijkstra's algorithm");
            System.out.println("4. Quit");
            System.out.print("Enter your choice: ");
            choice = scanner.nextInt();

            switch (choice) {
                case 1:
                    try {
                        File file = new File("input1.txt");
                        Scanner fileScanner = new Scanner(file);
                        int vertices = fileScanner.nextInt();
                        int edges = fileScanner.nextInt();

                        double[][] adjacencyMatrix = new double[vertices][vertices];

                        while (fileScanner.hasNextInt()) {
                            int source = fileScanner.nextInt();
                            int target = fileScanner.nextInt();
                            double weight = fileScanner.nextDouble();
                            adjacencyMatrix[source][target] = weight;
                            adjacencyMatrix[target][source] = weight;
                        }

                        long startTime = System.nanoTime();
                        primMST(adjacencyMatrix);
                        long endTime = System.nanoTime();
                        long executionTime = endTime - startTime;
                        System.out.println("Running time of Prim's algorithm using unordered Min-Priority Queue is " + executionTime + " Nano seconds");

                        fileScanner.close();
                    } catch (FileNotFoundException e) {
                        System.out.println("File not found: input1.txt");
                    }
                    break;

                case 2:
                    try {
                        File file = new File("input1.txt");
                        Scanner fileScanner = new Scanner(file);
                        int vertices = fileScanner.nextInt();
                        int numEdges = fileScanner.nextInt();

                        ArrayList<Edge> edges = new ArrayList<>();

                        while (fileScanner.hasNextInt()) {
                            int source = fileScanner.nextInt();
                            int target = fileScanner.nextInt();
                            double weight = fileScanner.nextDouble();
                            edges.add(new Edge(source, target, weight));
                        }

                        long startTime = System.nanoTime();
                        kruskalMST(edges, vertices);
                        long endTime = System.nanoTime();
                        long executionTime = endTime - startTime;
                        System.out.println("Running Time of Kruskal's algorithm using Union-Find approach is " + executionTime + " Nano seconds.");

                        fileScanner.close();
                    } catch (FileNotFoundException e) {
                        System.out.println("File not found: input1.txt");
                    }
                    break;

                case 3:
                    try {
                        File file = new File("input2.txt");
                        Scanner fileScanner = new Scanner(file);
                        int vertices = fileScanner.nextInt();
                        int edges = fileScanner.nextInt();

                        double[][] adjacencyMatrix = new double[vertices][vertices];

                        while (fileScanner.hasNextInt()) {
                            int source = fileScanner.nextInt();
                            int target = fileScanner.nextInt();
                            double weight = fileScanner.nextDouble();
                            adjacencyMatrix[source][target] = weight;
                        }

                        // Print weight matrix
                        System.out.println("Weight  Matrix: ");
                        System.out.println();
                        System.out.print("     ");
                        System.out.printf("%3d", 0);
                        for (int i = 1; i < vertices; i++) {
                            System.out.printf(" %6d", i);
                        }
                        System.out.println();
                        for (int i = 0; i < vertices; i++) {
                            System.out.printf("%d", i);
                            for (int j = 0; j < vertices; j++) {
                                System.out.printf(" %6.0f", adjacencyMatrix[i][j]);
                            }
                            System.out.println();
                        }
                        System.out.println();
                        // Print number of vertices and edges
                        int edgeCount = 0;
                        for (int i = 0; i < vertices; i++) {
                            for (int j = 0; j < vertices; j++) {
                                if (adjacencyMatrix[i][j] != 0) {
                                    edgeCount++;
                                }
                            }
                        }
                        System.out.println("# of vertices is: " + vertices + ", # of edges is: " + edgeCount);

                        // Print vertex-edge list
                        for (int i = 0; i < vertices; i++) {
                            System.out.printf("%4d: ", i);
                            for (int j = 0; j < vertices; j++) {
                                if (adjacencyMatrix[i][j] != 0) {
                                    System.out.printf("%4d-%d %.0f ", i, j, adjacencyMatrix[i][j]);
                                }
                            }
                            System.out.println();
                        }
                        System.out.println();

                        System.out.print("Enter Source vertex: ");
                        int source = scanner.nextInt();

                        System.out.println();
                        System.out.println("Dijkstra using priority queue:");
                        long startTime = System.nanoTime();
                        dijkstra(adjacencyMatrix, source);
                        long endTime = System.nanoTime();
                        long executionTime = endTime - startTime;
                        System.out.println();
                        System.out.println("Running time of Dijkstra using priority queue is: " + executionTime + " nano seconds");

                        fileScanner.close();
                    } catch (FileNotFoundException e) {
                        System.out.println("File not found: input2.txt");
                    }
                    break;

                case 4:
                    System.out.println("Exiting the program...");
                    break;

                default:
                    System.out.println("Invalid choice. Please try again.");
                    break;
            }

            System.out.println();
        } while (choice != 4);

        scanner.close();
    }
}