#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <limits>  // for std::numeric_limits<double>::infinity()
#include <map>

// A struct to represent a point (sink or Steiner)
struct Point {
    int id;
    double x, y;
    bool sink;  // true if originally from input, false if added as a Steiner point
};

struct Edge {
    int node1, node2;   // The IDs of the endpoints
    double weight;
};

bool operator==(const Point& p1, const Point& p2) {
    return (p1.id == p2.id &&
            p1.x  == p2.x  &&
            p1.y  == p2.y  &&
            p1.sink == p2.sink);
}

// =================== SteinerTree Class ===================

class SteinerTree {
private:
    std::string filePath;

    // We store all points in this vector (including sinks + Steiner points)
    std::vector<Point> points;

    // The edges that represent the MST on our current set of points
    std::vector<Edge> edges;

    // The total number of points changes when we add Steiner points
    int numberOfPoints;

    // Keep track of how many sinks we started with
    int originalSinkCount;

    // DP Table: 
    //   Key: Subset of points (represented as a bitmask)
    //   Value: Pair (min MST length, Steiner point ID that achieved it)
    std::map<unsigned long long, std::pair<double, int>> dpTable;

public:
    // Constructor
    SteinerTree(const std::string& path)
        : filePath(path), numberOfPoints(0), originalSinkCount(0)
    {
        readInputFile();
        originalSinkCount = numberOfPoints; // store the initial sink count
    }

    // Basic getters
    const std::vector<Point>& getPoints() const {
        return points;
    }
    const std::vector<Edge>& getEdges() const {
        return edges;
    }

    // =============== Reading Input ===============
    bool readInputFile() {
        std::ifstream inputFile(filePath);
        if (!inputFile.is_open()) {
            throw std::runtime_error("Error opening file: " + filePath);
        }

        std::string line;
        while (std::getline(inputFile, line)) {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;

            if (keyword == "number_of_sinks") {
                iss >> numberOfPoints;
            }
            else if (keyword == "sink") {
                Point sink;
                iss >> sink.id >> sink.x >> sink.y;
                sink.sink = true;
                points.push_back(sink);
            }
        }
        inputFile.close();
        return true;
    }

    // Just print how many sinks and their info
    void printSinks() {
        std::cout << "Number of sinks: " << numberOfPoints << std::endl;
        for (const auto& p : points) {
            if (p.sink) {
                std::cout << "Sink ID: " << p.id
                          << ", X: " << p.x
                          << ", Y: " << p.y << std::endl;
            }
        }
    }

    // A helper to compute Manhattan distance
    double mhDistance(const Point& a, const Point& b) {
        return std::fabs(a.x - b.x) + std::fabs(a.y - b.y);
    }

    // ================== Prim's Algorithm ==================
    // This function computes the MST (using Prim’s) among a given vector of points
    // Returns a vector of MST Edges. We also store the MST length in *outMstLen
    std::vector<Edge> computePrimMST(const std::vector<Point>& pts,
                                     double* outMstLen = nullptr) 
    {
        // If no points or just one point, MST is empty
        if (pts.size() < 2) {
            if (outMstLen) *outMstLen = 0.0;
            return {};
        }

        int n = (int)pts.size();

        // We'll map point IDs to [0..n-1]
        std::unordered_map<int,int> idToIndex;
        idToIndex.reserve(n);
        for (int i = 0; i < n; i++) {
            idToIndex[pts[i].id] = i;
        }

        // We'll store the MST edges in this vector
        std::vector<Edge> mstEdges;
        mstEdges.reserve(n - 1);

        // dist[i] = minimal distance from node i to the MST so far
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        // parent[i] = the index that connects i to MST (-1 if root)
        std::vector<int> parent(n, -1);
        // inMST[i] = whether node i is in MST
        std::vector<bool> inMST(n, false);

        // Start from node 0
        dist[0] = 0.0;

        for (int i = 0; i < n; i++) {
            // Pick the unvisited node with smallest dist
            int u = -1;
            double best = std::numeric_limits<double>::infinity();
            for (int v = 0; v < n; v++) {
                if (!inMST[v] && dist[v] < best) {
                    best = dist[v];
                    u = v;
                }
            }

            inMST[u] = true;
            // If u is not root, add edge to MST
            if (parent[u] != -1) {
                Edge e;
                int pu = parent[u];
                e.node1  = pts[u].id;
                e.node2  = pts[pu].id;
                e.weight = mhDistance(pts[u], pts[pu]);
                mstEdges.push_back(e);
            }

            // Update neighbors
            for (int v = 0; v < n; v++) {
                if (!inMST[v]) {
                    double cost = mhDistance(pts[u], pts[v]);
                    if (cost < dist[v]) {
                        dist[v]   = cost;
                        parent[v] = u;
                    }
                }
            }
        }

        if (outMstLen) {
            double totalLen = 0.0;
            for (const auto& e : mstEdges) {
                totalLen += e.weight;
            }
            *outMstLen = totalLen;
        }
        return mstEdges;
    }

    // A convenience function to sum MST edges (Prim’s result)
    double sumEdges(const std::vector<Edge>& edges) {
        double total = 0.0;
        for (auto& e : edges) total += e.weight;
        return total;
    }

    // ================ Hanan Grid ================
    // Get unique X or Y coords from a set of points
    std::vector<double> getUniqueCoords(const std::vector<Point>& pts, bool isX) {
        std::set<double> coords;
        for (auto& p : pts) {
            coords.insert(isX ? p.x : p.y);
        }
        return std::vector<double>(coords.begin(), coords.end());
    }

    // Return all Hanan grid points that do not coincide with any existing points
    std::vector<Point> getHananGridPoints(const std::vector<Point>& pts) {
        auto xVals = getUniqueCoords(pts, true);
        auto yVals = getUniqueCoords(pts, false);

        std::vector<Point> candidates;
        int startId = 1000;

        for (double x : xVals) {
            for (double y : yVals) {
                bool alreadyExists = false;
                for (auto& p : pts) {
                    // Using a small epsilon check if needed, or direct equality
                    if (std::fabs(p.x - x) < 1e-9 && std::fabs(p.y - y) < 1e-9) {
                        alreadyExists = true;
                        break;
                    }
                }
                if (!alreadyExists) {
                    Point sp;
                    sp.id   = startId++;
                    sp.x    = x;
                    sp.y    = y;
                    sp.sink = false; // Steiner
                    candidates.push_back(sp);
                }
            }
        }

        // Debug Print: Show Hanan grid points
        std::cout << "\n[DEBUG] Hanan Grid Points Generated:\n";
        for (auto& c : candidates) {
            std::cout << "   ID=" << c.id
                      << "  (x=" << c.x << ", y=" << c.y << ")\n";
        }
        std::cout << "[DEBUG] Total candidate Hanan points: " 
                  << candidates.size() << "\n\n";

        return candidates;
    }

    // =================== ADD STEINER POINTS ===================

    // Function to compute MST length for a subset of points
    // using dynamic programming and optionally adding a Steiner point.
    double steinerMSTRecursive(unsigned long long subset, int numSteiner, 
                                const std::vector<Point>& hanan) {
        
        // Base Cases:
        if (dpTable.count(subset) > 0) {
            return dpTable[subset].first; // Subproblem already solved
        }
        if (numSteiner == 0) { 
            // No more Steiner points allowed, compute MST directly
            std::vector<Point> subPoints;
            for (size_t i = 0; i < points.size(); ++i) {
                if (subset & (1ULL << i)) {
                    subPoints.push_back(points[i]);
                }
            }
            double len;
            computePrimMST(subPoints, &len); 
            dpTable[subset] = std::make_pair(len, -1); // No Steiner added
            return len;
        }

        double minLength = std::numeric_limits<double>::infinity();
        int bestSteiner = -1;

        // 1. Try without adding a new Steiner point
        double withoutSteiner = steinerMSTRecursive(subset, numSteiner - 1, hanan);
        minLength = withoutSteiner;

        // 2. Try adding each potential Steiner point
        for (const Point& sp : hanan) {
            // Create a new subset with the Steiner point included
            unsigned long long newSubset = subset;
            newSubset |= (1ULL << (points.size() + sp.id - 1000)); // Assuming sp.id starts from 1000

            // Recursively compute MST with the added Steiner point
            double withSteiner = steinerMSTRecursive(newSubset, numSteiner - 1, hanan);

            // Update minLength and bestSteiner if improvement found
            if (withSteiner < minLength) {
                minLength = withSteiner;
                bestSteiner = sp.id;
            }
        }

        dpTable[subset] = std::make_pair(minLength, bestSteiner);
        return minLength;
    }

    // Function to add Steiner points using dynamic programming
    void addSteinerPointsDP() {
        // 1. Generate Hanan grid points
        std::vector<Point> hanan = getHananGridPoints(points);

        // 2. Initialize DP table for the initial set of sinks
        unsigned long long initialSubset = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            initialSubset |= (1ULL << i); 
        }
        steinerMSTRecursive(initialSubset, hanan.size(), hanan); // Precompute

        // 3. Trace back DP table to add Steiner points
        unsigned long long currentSubset = initialSubset;
        while (dpTable[currentSubset].second != -1) {
            int steinerID = dpTable[currentSubset].second;
            // Find the Steiner point in 'hanan' by ID 
            // and add it to 'points'
            for (const Point& sp : hanan) {
                if (sp.id == steinerID) {
                    points.push_back(sp);
                    break;
                }
            }

            // Update the current subset 
            currentSubset |= (1ULL << (points.size() + steinerID - 1000));
        }

        // 4. Compute the final MST (optional, for visualization or verification)
        double finalMSTLen;
        this->edges = computePrimMST(points, &finalMSTLen);
    }

    // =============== Blockage Handling (To be implemented) ===============
    // (Research and add logic to handle blockages in the grid)

    // Helper to find a point by ID
    const Point* findPointByID(int id) const {
        for (auto &p : points) {
            if (p.id == id) return &p;
        }
        return nullptr;
    }

    // =============== Output File ===============
    void writeOutputToFile(const std::string& outFile, 
                           const std::vector<Edge>& mstEdges) 
    {
        std::ofstream ofs(outFile);
        if (!ofs.is_open()) {
            throw std::runtime_error("Error opening output file: " + outFile);
        }

        // We only want to show the original number of sinks in the file
        ofs << "number_of_sinks " << originalSinkCount << "\n\n";

        // Print the points, labeling original sinks vs. Steiner
        // We'll insert a blank line before listing Steiner points for clarity
        bool onceSpace = true;
        for (auto& p : points) {
            if (p.sink) {
                ofs << "sink "  << p.id << " " << p.x << " " << p.y << "\n";
            } else {
                if (onceSpace) {
                    ofs << "\n";
                    onceSpace = false;
                }
                ofs << "point " << p.id << " " << p.x << " " << p.y << "\n";
            }
        }
        ofs << "\n";

        // Print MST edges
        for (auto& e : mstEdges) {
            const Point* p1 = findPointByID(e.node1);
            const Point* p2 = findPointByID(e.node2);
            if (!p1 || !p2) continue;
            // If they share x or y, it's already rectilinear
            if (std::fabs(p1->x - p2->x) < 1e-9 ||
                std::fabs(p1->y - p2->y) < 1e-9) {
                ofs << "edge " << e.node1 << " " << e.node2 << "\n";
            } else {
                // Make an intermediate Steiner point at (p1->x, p2->y)
                // (Or choose (p2->x, p1->y) – either is fine)
                int midID = 20000 + e.node1; // any unique ID scheme
                ofs << "point " << midID << " " << p1->x << " " << p2->y << "\n";
                ofs << "edge " << e.node1 << " " << midID << "\n";
                ofs << "edge " << midID   << " " << e.node2 << "\n";
            }
        }
        ofs << "\n";

        // Print MST length
        double totalLen = sumEdges(mstEdges);
        ofs << "Path length = " << totalLen << "\n";
        ofs.close();
    }

    // =============== Debug Helper ===============
    void printEdges(const std::vector<Edge>& edgeList) {
        std::cout << "Total number of edges: " << edgeList.size() << "\n";
        for (auto& e : edgeList) {
            std::cout << "Edge between " << e.node1
                      << " and " << e.node2
                      << " has Manhattan distance: " << e.weight << "\n";
        }
    }
};


void reorderOutputFile(const std::string &filename) {
    std::ifstream in(filename);
    if (!in) return;

    std::string line, numberOfSinks, pathLength;
    std::vector<std::string> sinks, points, edges;

    while (std::getline(in, line)) {
        if (line.rfind("number_of_sinks", 0) == 0) {
            numberOfSinks = line;
        } else if (line.rfind("sink", 0) == 0) {
            sinks.push_back(line);
        } else if (line.rfind("point", 0) == 0) {
            points.push_back(line);
        } else if (line.rfind("edge", 0) == 0) {
            edges.push_back(line);
        } else if (line.rfind("Path length", 0) == 0) {
            pathLength = line;
        }
    }
    in.close();

    std::ofstream out(filename);
    if (!out) return;

    out << numberOfSinks << "\n\n";
    for (auto &s : sinks) out << s << "\n";
    out << "\n";
    for (auto &p : points) out << p << "\n";
    out << "\n";
    for (auto &e : edges) out << e << "\n";
    out << "\n";
    out << pathLength << std::endl;
}



// ================= MAIN =================

int main() {
    try {
        // 1) Initialize
        SteinerTree steiner("data/r53.in");

        // 2) Print sinks that we read in
        steiner.printSinks();

        // 3) Compute Prim MST for the initial sinks (baseline)
        auto initPoints = steiner.getPoints();
        double initMSTLen;
        auto initMST = steiner.computePrimMST(initPoints, &initMSTLen);
        
        // Print edges
        std::cout << "\nInitial MST with only sinks:\n";
        steiner.printEdges(initMST);
        std::cout << "Initial MST length = " << initMSTLen << "\n\n";

        // 4) Add Steiner points using DP
        std::cout << "Adding Steiner Points using Hanan Grid and DP...\n";
        steiner.addSteinerPointsDP();

        // 5) Retrieve the final MST edges after adding Steiner points (DP)
        auto finalEdgesDP = steiner.getEdges();
        double finalMSTLenDP = 0.0;
        for (auto& e : finalEdgesDP) {
            finalMSTLenDP += e.weight;
        }

        // Print final MST edges (DP)
        std::cout << "\nFinal MST Edges with Steiner Points (DP):\n";
        steiner.printEdges(finalEdgesDP);
        std::cout << "Final MST length (DP) = " << finalMSTLenDP << "\n\n";

        // 6) Write to output file
        steiner.writeOutputToFile("steiner_tree_output_dp.txt", finalEdgesDP);
        std::cout << "Output (DP) written to file: steiner_tree_output_dp.txt\n";

        reorderOutputFile("steiner_tree_output_dp.txt");

        return 0;
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}