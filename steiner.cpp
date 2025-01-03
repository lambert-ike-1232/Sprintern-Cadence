#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <set>

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

// Function to calculate Manhattan Distance
double manhattanDistance(const Point &a, const Point &b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}


// Add vector of point(graph structure)

// Function to generate Hanan grid points
vector<Point> generateHananGrid(const vector<Point> &sinks) {
    set<double> x_coords, y_coords;
    for (const auto &sink : sinks) {
        x_coords.insert(sink.x);
        y_coords.insert(sink.y);
    }

    vector<Point> hananGrid;
    int id = 1000; // Starting ID for Steiner points
    for (const auto &x : x_coords) {
        for (const auto &y : y_coords) {
            hananGrid.emplace_back(id++, x, y);
        }
    }
    return hananGrid;
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
void writeOutput(const string &outputFile, const vector<Point> &sinks, const vector<Point> &hananGrid, const vector<Edge> &edges, double pathLength) {
    ofstream out(outputFile);

    // Write sinks
    out << "number_of_sinks " << sinks.size() << "\n\n";
    for (const auto &sink : sinks) {
        out << "sink " << sink.id << " " << fixed << setprecision(1) << sink.x << " " << sink.y << "\n";
    }

    // Write Hanan grid points
    out << "\n";
    for (const auto &point : hananGrid) {
        out << "points " << point.id << " " << fixed << setprecision(1) << point.x << " " << point.y << "\n";
    }

    // Write edges
    out << "\n";
    for (const auto &edge : edges) {
        out << "edge " << edge.u << " " << edge.v << "\n";
    }

    // Write total path length
    out << "\nPath length = " << fixed << setprecision(1) << pathLength << "\n";

    out.close();
    cout << "Output written to " << outputFile << endl;
}

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

    // Generate Hanan grid points
    vector<Point> hananGrid = generateHananGrid(sinks);

    // Combine sinks and Hanan grid for edge generation
    vector<Point> allPoints = sinks;
    allPoints.insert(allPoints.end(), hananGrid.begin(), hananGrid.end());

    // Create edges
    vector<Edge> edges = createEdges(allPoints);

    // Select some edges for visualization (minimal spanning set or heuristic)
    // For now, just a placeholder subset of edges
    vector<Edge> selectedEdges = edges; // Use all edges for demonstration

    // Calculate total path length
    double pathLength = calculatePathLength(selectedEdges);

    // Write to output file
    writeOutput(outputFile, sinks, hananGrid, selectedEdges, pathLength);

    return 0;
}