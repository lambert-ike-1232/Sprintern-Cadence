#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <set>
#include <algorithm> // For std::sort

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

// Comparator for sorting edges by weight
bool compareEdgeWeight(const Edge &a, const Edge &b) {
    return a.weight < b.weight;
}

// Union-Find (Disjoint Set) for Kruskal's Algorithm
class UnionFind {
    vector<int> parent, rank;

public:
    UnionFind(int n) {
        parent.resize(n);
        rank.resize(n, 0);
        for (int i = 0; i < n; ++i)
            parent[i] = i;
    }

    int find(int u) {
        if (parent[u] != u)
            parent[u] = find(parent[u]);
        return parent[u];
    }

    bool unite(int u, int v) {
        int rootU = find(u);
        int rootV = find(v);
        if (rootU != rootV) {
            if (rank[rootU] > rank[rootV]) {
                parent[rootV] = rootU;
            } else if (rank[rootU] < rank[rootV]) {
                parent[rootU] = rootV;
            } else {
                parent[rootV] = rootU;
                rank[rootU]++;
            }
            return true;
        }
        return false;
    }
};

// Function to calculate Manhattan Distance
double manhattanDistance(const Point &a, const Point &b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

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
vector<Edge> createEdges(const vector<Point> &points, ofstream &out) {
    vector<Edge> edges;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            double weight = manhattanDistance(points[i], points[j]);
            edges.emplace_back(points[i].id, points[j].id, weight);

            // Output edge information to file
            out << "Edge (" << points[i].id << ", " << points[j].id << ") weight: " << weight << endl;
        }
    }
    return edges;
}

// Function to compute the Minimum Spanning Tree using Kruskal's Algorithm
vector<Edge> computeMST(int n, vector<Edge> &edges, ofstream &out) {
    sort(edges.begin(), edges.end(), compareEdgeWeight);
    UnionFind uf(n);

    vector<Edge> mst;
    double totalWeight = 0.0;

    // Output all edges before MST computation
    out << "Edges before MST computation:" << endl;
    for (const auto &edge : edges) {
        out << "(" << edge.u << ", " << edge.v << ") weight: " << edge.weight << endl;
    }

    for (const auto &edge : edges) {
        if (uf.unite(edge.u, edge.v)) {
            mst.push_back(edge);
            totalWeight += edge.weight;

            // Output MST edge being added
            out << "Adding edge to MST: (" << edge.u << ", " << edge.v << ") weight: " << edge.weight << endl;
        }

        // Stop when we've added (n-1) edges
        if (mst.size() == n - 1)
            break;
    }

    out << "Total MST Weight: " << totalWeight << endl;
    return mst;
}

// Function to calculate the total weight of edges in the MST
double calculatePathLength(const vector<Edge> &edges, ofstream &out) {
    double total = 0.0;
    for (const auto &edge : edges) {
        total += edge.weight;
    }

    // Output total path length
    out << "Total Path Length: " << total << endl;

    return total;
}

// Function to write output to a file
void writeOutput(const string &outputFile, const vector<Point> &sinks, const vector<Point> &hananGrid, const vector<Edge> &mst, double pathLength) {
    ofstream out(outputFile);

    if (!out) {
        cerr << "Error: Unable to open output file " << outputFile << endl;
        return;
    }

    // Write sinks
    out << "number_of_sinks " << sinks.size() << "\n\n";
    for (const auto &sink : sinks) {
        out << "sink " << sink.id << " " << fixed << setprecision(1) << sink.x << " " << sink.y << "\n";
    }

    // Write Steiner points used in the MST
    set<int> usedPoints;
    for (const auto &edge : mst) {
        usedPoints.insert(edge.u);
        usedPoints.insert(edge.v);
    }
    out << "\n";
    for (const auto &point : hananGrid) {
        if (usedPoints.count(point.id)) {
            out << "point " << point.id << " " << fixed << setprecision(1) << point.x << " " << point.y << "\n";
        }
    }

    // Write edges
    out << "\n";
    for (const auto &edge : mst) {
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
        cerr << "Usage: " << argv[0] << " <input_file> <output_file>" << endl;
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = argv[2];

    ifstream in(inputFile);
    if (!in) {
        cerr << "Error: Unable to open input file " << inputFile << endl;
        return 1;
    }

    // Read sinks from input file
    vector<Point> sinks;
    string line;
    while (getline(in, line)) {
        stringstream ss(line);
        string type;
        ss >> type;
        if (type == "sink") {
            int id;
            double x, y;
            if (!(ss >> id >> x >> y)) {
                cerr << "Error: Invalid sink format in input file." << endl;
                return 1;
            }
            sinks.emplace_back(id, x, y);
        }
    }
    in.close();

    if (sinks.empty()) {
        cerr << "Error: No sinks found in input file." << endl;
        return 1;
    }

    // Open output file for writing debug information
    ofstream out(outputFile);

    // Generate Hanan grid points
    vector<Point> hananGrid = generateHananGrid(sinks);

    // Combine sinks and Hanan grid for edge generation
    vector<Point> allPoints = sinks;
    allPoints.insert(allPoints.end(), hananGrid.begin(), hananGrid.end());

    // Create edges and output debug info
    vector<Edge> edges = createEdges(allPoints, out);

    // Compute MST and output debug info
    vector<Edge> mst = computeMST(allPoints.size(), edges, out);

    // Calculate total path length and output debug info
    double pathLength = calculatePathLength(mst, out);

    // Write final output to file
    writeOutput(outputFile, sinks, hananGrid, mst, pathLength);

    return 0;
}