#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <set>
#include <queue>
#include <unordered_map>

using namespace std;

// Struct to represent a Point in 2D space
struct Point {
    int id;
    double x, y;

    Point(int id, double x, double y) : id(id), x(x), y(y) {}
};

// Struct to represent an Edge in the graph
struct Edge {
    int u, v;   // Indices of the points connected by this edge
    double weight;

    Edge(int u, int v, double weight) : u(u), v(v), weight(weight) {}
};

// Struct to represent a graph
struct Graph {
    vector<Point> vertices;
    vector<Edge> edges;
};


// Function to calculate Manhattan Distance
double manhattanDistance(const Point &a, const Point &b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// Helper function to print the priority queue
void printPriorityQueue(priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> &pq) {
    cout << "Priority Queue contents: ";
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> temp_pq = pq;
    
    // Extract elements and print them
    while (!temp_pq.empty()) {
        auto [weight, vertex] = temp_pq.top();
        cout << "(" << weight << ", " << vertex << ") ";
        temp_pq.pop();
    }
    cout << endl;
}

// Function to calculate MST given a complete graph
Graph generateMST(const Graph graph){
    vector<Edge> all_edges = graph.edges;
    vector<Point> all_vertices = graph.vertices;

    // Using Prim's algorithm
    int n = all_vertices.size();

    // Priority queue to store the edges with their weights
    using WeightedEdge = pair<double, int>; // {weight, destination_vertex}
    priority_queue<WeightedEdge, vector<WeightedEdge>, greater<>> pq;

    printPriorityQueue(pq);

    unordered_map<int, bool> inMST; // Tracks if a vertex is in the MST
    vector<Edge> mst_edges;      // Store edges in the MST

    // Start from an arbitrary vertex (first sink)
    int start_vertex_id = all_vertices[0].id;
    inMST[start_vertex_id] = true;

    // Push all edges from the start vertex to the priority queue
    for (const auto& edge : all_edges) {
        if (edge.u == start_vertex_id || edge.v == start_vertex_id) {
            int dest = (edge.u == start_vertex_id) ? edge.v : edge.u;
            pq.push({edge.weight, dest});
        }
    }

    printPriorityQueue(pq);

    // Process until MST contains all vertices
    while (!pq.empty() && mst_edges.size() < all_vertices.size() - 1) {
        auto [weight, dest_id] = pq.top();
        pq.pop();
        cout << "Processing edge to " << dest_id << " with weight " << weight << endl;

        if (inMST[dest_id]) {
            cout << "Vertex " << dest_id << " is already in MST, skipping this edge." << endl;
            continue; // Skip if the vertex is already in the MST
        }
        
        inMST[dest_id] = true;
        cout << "Added vertex " << dest_id << " to MST." << endl;

        // Add the corresponding edge to the MST
        for (const auto& edge : all_edges) {
            if (edge.weight == weight && ((edge.u == dest_id && inMST[edge.v]) || (edge.v == dest_id && inMST[edge.u]))) {
                mst_edges.push_back(edge);
                cout << "Added edge (" << edge.u << ", " << edge.v << ") with weight " << edge.weight << " to MST." << endl;
                break;
            }
        }
        printPriorityQueue(pq);
        // Add new edges from the current vertex to the queue
        cout << "Adding edges connected to vertex " << dest_id << " to the priority queue:" << endl;
        for (const auto& edge : all_edges) {
            if (edge.u == dest_id || edge.v == dest_id) {
                int neighbor_id = (edge.u == dest_id) ? edge.v : edge.u;
                if (!inMST[neighbor_id]) {
                    pq.push({edge.weight, neighbor_id});
                    cout << "Pushed edge (" << edge.u << ", " << edge.v << ") with weight " << edge.weight << " to the queue." << endl;
                    printPriorityQueue(pq);
                }
            }
        }
        printPriorityQueue(pq);
    }

    // End of the MST generation
    cout << "\nMST generation complete. MST contains the following edges:" << endl;
    for (const auto& edge : mst_edges) {
        cout << "Edge (" << edge.u << ", " << edge.v << ") with weight " << edge.weight << endl;
    }

    // Construct the MST graph
    Graph mst;
    mst.vertices = all_vertices;
    mst.edges = mst_edges;

    return mst;

}

// Function to generate Hanan grid points

// vector<Point> generateHananGrid(const vector<Point> &sinks) {
//     set<double> x_coords, y_coords;
//     for (const auto &sink : sinks) {
//         x_coords.insert(sink.x);
//         y_coords.insert(sink.y);
//     }

//     vector<Point> hananGrid;
//     int id = 1000; // Starting ID for Steiner points
//     for (const auto &x : x_coords) {
//         for (const auto &y : y_coords) {
//             hananGrid.emplace_back(id++, x, y);
//         }
//     }
//     return hananGrid;
// }



vector<Point> generateHananGrid(const vector<Point>& sinks) {
    set<double> uniqueX, uniqueY;

    // Collect unique x and y coordinates
    for (const auto& sink : sinks) {
        uniqueX.insert(sink.x);
        uniqueY.insert(sink.y);
    }

    // Generate Steiner points at intersections
    vector<Point> steinerPoints;
    int id = 1000;

    for (double x : uniqueX) {
        for (double y : uniqueY) {
            bool isSink = false;
            for (const auto& sink : sinks) {
                if (sink.x == x && sink.y == y) {
                    isSink = true;
                    break;
                }
            }
            if (!isSink) {
                steinerPoints.push_back({id++, x, y});
            }
        }
    }

    return steinerPoints;
}

// Function to create edges between points
vector<Edge> createEdges(const vector<Point> &points) {
    vector<Edge> edges;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            double weight = manhattanDistance(points[i], points[j]);
            edges.emplace_back(points[i].id, points[j].id, weight);
        }
    }
    return edges;
}

// Function to find the total path length of edges
double calculatePathLength(const vector<Edge> &edges) {
    double total = 0.0;
    for (const auto &edge : edges) {
        total += edge.weight;
    }
    return total;
}

// Function to write output to a file
// void writeOutput(const string &outputFile, const vector<Point> &sinks, const vector<Point> &hananGrid, const vector<Edge> &edges, double pathLength) {
//     ofstream out(outputFile);

//     // Write sinks
//     out << "number_of_sinks " << sinks.size() << "\n\n";
//     for (const auto &sink : sinks) {
//         out << "sink " << sink.id << " " << fixed << setprecision(1) << sink.x << " " << sink.y << "\n";
//     }

//     // Write Hanan grid points
//     out << "\n";
//     for (const auto &point : hananGrid) {
//         out << "points " << point.id << " " << fixed << setprecision(1) << point.x << " " << point.y << "\n";
//     }

//     // Write edges
//     out << "\n";
//     for (const auto &edge : edges) {
//         out << "edge " << edge.u << " " << edge.v << "\n";
//     }

//     // Write total path length
//     out << "\nPath length = " << fixed << setprecision(1) << pathLength << "\n";

//     out.close();
//     cout << "Output written to " << outputFile << endl;
// }

// Main function
int main(int argc, char* argv[]) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " <input_file> <output_file>" << endl;
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = argv[2];

    ifstream in(inputFile);
    if (!in) {
        cerr << "Error: Unable to open input file." << endl;
        return 1;
    }

    // Read input file
    vector<Point> sinks;
    string line;
    while (getline(in, line)) {
        stringstream ss(line);
        string type;
        ss >> type;
        if (type == "sink") {
            int id;
            double x, y;
            ss >> id >> x >> y;
            sinks.emplace_back(id, x, y);
        }
    }
    in.close();

    // Compute the initial MST of T using Manhattan distance
    Graph curr_graph;
    curr_graph.vertices = sinks;
    curr_graph.edges = createEdges(sinks);

    Graph initial_MST = generateMST(curr_graph);

    vector<Edge> selectedEdges = initial_MST.edges;
    double pathLength = calculatePathLength(initial_MST.edges);

    // Print to the output file (for initial MST)
    ofstream out(outputFile);
    if (!out) {
        cerr << "Error: Unable to open output file." << endl;
        return 1;
    }
    out << "Initial MST without steiner points" << endl;
    out << "Length: " << fixed << setprecision(1) << pathLength << endl;

    // Write sinks
    out << "number_of_sinks " << sinks.size() << "\n\n";
    for (const auto &sink : sinks) {
        out << "sink " << sink.id << " " << fixed << setprecision(1) << sink.x << " " << sink.y << "\n";
    }

    // Write edges
    for (const auto &edge : selectedEdges) {
        out << "edge " << edge.u << " " << edge.v << " Weight:" << edge.weight << "\n";
    }

    out << endl;

    // Write all edges
    for (const auto &edge : curr_graph.edges) {
        out << "edge " << edge.u << " " << edge.v << " Weight:" << edge.weight << "\n";
    }
    
    out.close();

    // Generate Hanan grid points
    vector<Point> hananGrid = generateHananGrid(sinks);


    //----------------------------------------
    // reference code
    // // Combine sinks and Hanan grid for edge generation
    // vector<Point> allPoints = sinks;
    // allPoints.insert(allPoints.end(), hananGrid.begin(), hananGrid.end());

    // // Create edges
    // vector<Edge> edges = createEdges(allPoints);

    // // Select some edges for visualization (minimal spanning set or heuristic)
    // // For now, just a placeholder subset of edges
    // vector<Edge> selectedEdges = edges; // Use all edges for demonstration

    // // Calculate total path length
    // double pathLength = calculatePathLength(selectedEdges);

    // // Write to output file
    // // writeOutput(outputFile, sinks, hananGrid, selectedEdges, pathLength);

    return 0;
}