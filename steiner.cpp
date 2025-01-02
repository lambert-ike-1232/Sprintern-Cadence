#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>
#include <map>
#include <set>
#include <iomanip> // for setting precision

using namespace std;

// Structure for a point (sink or Steiner point)
struct Point {
    int id;
    double x, y;
};

// Structure for an edge
struct Edge {
    int start, end; // IDs of the points
    double weight;  // Manhattan distance between points
};

// Global variables to store input data
vector<Point> sinks;
vector<Point> points;
vector<Edge> edges;
double path_length;

// Function to calculate Manhattan distance
double manhattanDistance(const Point& p1, const Point& p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

// Function to read input file
void readInputFile(const string& filename) {
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "Error: Unable to open input file: " << filename << endl;
        exit(1);
    }

    string line;
    while (getline(infile, line)) {
        stringstream ss(line);
        string keyword;
        ss >> keyword;

        if (keyword == "number_of_sinks") {
            int num_sinks;
            ss >> num_sinks;
            sinks.reserve(num_sinks);
        } else if (keyword == "sink") {
            Point sink;
            ss >> sink.id >> sink.x >> sink.y;
            sinks.push_back(sink);
        } else if (keyword == "points") {
            Point point;
            ss >> point.id >> point.x >> point.y;
            points.push_back(point);
        } else if (keyword == "edge") {
            Edge edge;
            ss >> edge.start >> edge.end;
            // Compute and store the weight (Manhattan distance)
            Point p1, p2;
            for (const auto& sink : sinks) {
                if (sink.id == edge.start) p1 = sink;
                if (sink.id == edge.end) p2 = sink;
            }
            for (const auto& point : points) {
                if (point.id == edge.start) p1 = point;
                if (point.id == edge.end) p2 = point;
            }
            edge.weight = manhattanDistance(p1, p2);
            edges.push_back(edge);
        } else if (keyword == "Path") {
            string temp;
            ss >> temp >> temp >> path_length; // Skip "Path" and "length ="
        }
    }
    infile.close();
}

// Function to write output file
void writeOutputFile(const string& filename) {
    ofstream outfile(filename);
    if (!outfile.is_open()) {
        cerr << "Error: Unable to open output file: " << filename << endl;
        exit(1);
    }

    // Write number of sinks
    outfile << "number_of_sinks " << sinks.size() << endl;

    // Write sinks
    for (const auto& sink : sinks) {
        outfile << "sink " << sink.id << " " << fixed << setprecision(2) << sink.x << " " << sink.y << endl;
    }

    // Write points
    for (const auto& point : points) {
        outfile << "points " << point.id << " " << fixed << setprecision(2) << point.x << " " << point.y << endl;
    }

    // Write edges
    for (const auto& edge : edges) {
        outfile << "edge " << edge.start << " " << edge.end << endl;
    }

    // Write path length
    outfile << "Path length = " << fixed << setprecision(1) << path_length << endl;

    outfile.close();
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_file> <output_file>" << endl;
        return 1;
    }

    string input_file = argv[1];
    string output_file = argv[2];

    // Read input data
    readInputFile(input_file);

    // Process data (logic for Steiner tree construction goes here)

    // Write output data
    writeOutputFile(output_file);

    return 0;
}