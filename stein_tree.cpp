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

    // Function to find a Point by its ID
    Point* findPointById(int id) {
        for (auto& p : points) {
            if (p.id == id) {
                return &p;
            }
        }
        return nullptr; // Not found
    }
    

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
// =================== Generate Rectilinear Steiner Points ===================
std::vector<Point> generateRectilinearSteinerPoints(const std::vector<Point>& sinks, double interval = 1.0) {
    // Determine bounding rectangle based on sink points
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    for (const auto& sink : sinks) {
        minX = std::min(minX, sink.x);
        maxX = std::max(maxX, sink.x);
        minY = std::min(minY, sink.y);
        maxY = std::max(maxY, sink.y);
    }

    // Generate candidate points at fixed intervals within the bounding rectangle
    std::vector<Point> candidates;
    int startId = 1000;  // ID for Steiner points

    for (double x = minX; x <= maxX; x += interval) {
        for (double y = minY; y <= maxY; y += interval) {
            bool alreadyExists = false;
            for (const auto& sink : sinks) {
                if (std::fabs(sink.x - x) < 1e-9 && std::fabs(sink.y - y) < 1e-9) {
                    alreadyExists = true;
                    break;
                }
            }

            if (!alreadyExists) {
                Point sp;
                sp.id = startId++;
                sp.x = x;
                sp.y = y;
                sp.sink = false;  // This is a Steiner point
                candidates.push_back(sp);
            }
        }
    }

    // Debug Print: Rectilinear Steiner Points
    std::cout << "\n[DEBUG] Generated Rectilinear Steiner Points within bounding box:\n";
    for (const auto& c : candidates) {
        std::cout << "   ID=" << c.id << "  (x=" << c.x << ", y=" << c.y << ")\n";
    }
    std::cout << "[DEBUG] Total Rectilinear Steiner points: " << candidates.size() << "\n\n";

    return candidates;
}

// Add this helper function inside the `SteinerTree` class

// Generate diagonal intersection points dynamically
std::vector<Point> getDiagonalIntersectionPoints(const std::vector<Point>& steinerPoints) {
    std::vector<Point> intersectionPoints;
    int startId = 2000;  // Start IDs for diagonal intersection points to avoid conflict

    for (size_t i = 0; i < steinerPoints.size(); i++) {
        for (size_t j = i + 1; j < steinerPoints.size(); j++) {
            const Point& p1 = steinerPoints[i];
            const Point& p2 = steinerPoints[j];

            // Check if the points form a diagonal (x1 ≠ x2 and y1 ≠ y2)
            if (p1.x != p2.x && p1.y != p2.y) {
                // Compute the potential intersection points
                Point intersect1 = {startId++, p1.x, p2.y, false}; // (x1, y2)
                Point intersect2 = {startId++, p2.x, p1.y, false}; // (x2, y1)

                // Ensure these points are unique and not already in steinerPoints
                auto isUnique = [&](const Point& pt) {
                    for (const auto& existing : steinerPoints) {
                        if (std::fabs(existing.x - pt.x) < 1e-9 && std::fabs(existing.y - pt.y) < 1e-9) {
                            return false; // Point already exists
                        }
                    }
                    for (const auto& existing : intersectionPoints) {
                        if (std::fabs(existing.x - pt.x) < 1e-9 && std::fabs(existing.y - pt.y) < 1e-9) {
                            return false; // Point already exists
                        }
                    }
                    return true;
                };

                if (isUnique(intersect1)) {
                    intersectionPoints.push_back(intersect1);
                }
                if (isUnique(intersect2)) {
                    intersectionPoints.push_back(intersect2);
                }
            }
        }
    }

    // Debug print
    std::cout << "[DEBUG] Diagonal intersection points generated:\n";
    for (const auto& pt : intersectionPoints) {
        std::cout << "   ID=" << pt.id << " (x=" << pt.x << ", y=" << pt.y << ")\n";
    }

    return intersectionPoints;
}

// Function to enforce diagonal Steiner point generation
void enforceDiagonalSteinerPoints(std::vector<Point>& allPoints) {
    std::vector<Point> newSteinerPoints;
    int startId = 3000; // Start IDs for dynamically added diagonal Steiner points

    for (size_t i = 0; i < allPoints.size(); i++) {
        for (size_t j = i + 1; j < allPoints.size(); j++) {
            const Point& p1 = allPoints[i];
            const Point& p2 = allPoints[j];

            // Check if the points form a diagonal (x1 ≠ x2 and y1 ≠ y2)
            if (p1.x != p2.x && p1.y != p2.y) {
                // Compute intersection points
                Point intersect1 = {startId++, p1.x, p2.y, false}; // (x1, y2)
                Point intersect2 = {startId++, p2.x, p1.y, false}; // (x2, y1)

                // Check if these intersection points already exist
                auto isUnique = [&](const Point& pt) {
                    for (const auto& existing : allPoints) {
                        if (std::fabs(existing.x - pt.x) < 1e-9 && std::fabs(existing.y - pt.y) < 1e-9) {
                            return false; // Point already exists
                        }
                    }
                    for (const auto& existing : newSteinerPoints) {
                        if (std::fabs(existing.x - pt.x) < 1e-9 && std::fabs(existing.y - pt.y) < 1e-9) {
                            return false; // Point already exists
                        }
                    }
                    return true;
                };

                if (isUnique(intersect1)) {
                    newSteinerPoints.push_back(intersect1);
                }
                if (isUnique(intersect2)) {
                    newSteinerPoints.push_back(intersect2);
                }
            }
        }
    }

    // Add all new Steiner points to the main list
    std::cout << "[DEBUG] New diagonal Steiner points added:\n";
    for (const auto& pt : newSteinerPoints) {
        std::cout << "   ID=" << pt.id << " (x=" << pt.x << ", y=" << pt.y << ")\n";
        allPoints.push_back(pt);
    }
}

// Update addSteinerPoints() to always enforce diagonal Steiner point generation
void addSteinerPoints() {
    // Step 1: Start with the current set of points (sinks + initial Steiner points)
    double baseLength;
    auto baseMST = computePrimMST(points, &baseLength);
    std::cout << "[DEBUG] Initial baseline MST length = " << baseLength << "\n";

    // Step 2: Generate all Hanan grid points
    auto hanan = getHananGridPoints(points);

    // Step 3: Enforce diagonal Steiner points for ALL points
    // Combine Hanan points and current points, then enforce diagonal Steiner generation
    std::vector<Point> allPoints = points;
    allPoints.insert(allPoints.end(), hanan.begin(), hanan.end());

    enforceDiagonalSteinerPoints(allPoints);

    // Step 4: Iteratively add the best-improving Steiner point
    bool improved = true;
    while (improved) {
        improved = false;
        double bestImprovement = 0.0;
        Point bestSteiner;

        std::cout << "[DEBUG] --- New iteration over all candidate points. "
                  << "Current #points=" << points.size()
                  << ", Baseline MST length=" << baseLength << "\n";

        for (auto& candidate : allPoints) {
            // Build a temp set: current points + candidate
            std::vector<Point> tempPoints = points;
            tempPoints.push_back(candidate);

            double newLength;
            auto candidateMST = computePrimMST(tempPoints, &newLength);

            double improvement = baseLength - newLength;
            // Print each candidate test
            std::cout << "   [DEBUG] Test candidate ID=" << candidate.id
                      << " (x=" << candidate.x << ", y=" << candidate.y << ")"
                      << ", new MST len=" << newLength
                      << ", improvement=" << improvement << "\n";

            if (improvement > bestImprovement) {
                bestImprovement = improvement;
                bestSteiner = candidate;
                improved = true;
            }
        }

        if (improved) {
            // Actually add that bestSteiner to points
            points.push_back(bestSteiner);
            numberOfPoints++;

            double newBaseLen;
            auto newMST = computePrimMST(points, &newBaseLen);

            std::cout << "[DEBUG] >>> Adding Steiner ID=" << bestSteiner.id
                      << " => MST length now=" << newBaseLen
                      << ", improvement=" << bestImprovement << "\n";

            baseLength = newBaseLen; // update baseline
        }
    }

    // Step 5: Attempt removal: if removing a Steiner doesn’t worsen the MST, remove it
    std::cout << "\n[DEBUG] Checking if we can remove any Steiner points...\n";
    auto it = points.begin();
    while (it != points.end()) {
        if (!it->sink) {
            // Temporarily remove this Steiner point
            std::vector<Point> tempPoints = points;
            tempPoints.erase(std::remove(tempPoints.begin(), tempPoints.end(), *it), tempPoints.end());

            double newLen;
            auto newMST = computePrimMST(tempPoints, &newLen);

            double curLen;
            auto curMST = computePrimMST(points, &curLen);

            std::cout << "   [DEBUG] Removal check => Steiner ID=" << it->id
                      << ": MST if removed=" << newLen
                      << ", current MST=" << curLen << "\n";

            if (newLen <= curLen) {
                std::cout << "   [DEBUG] >>> Removing Steiner ID=" << it->id << "\n";
                it = points.erase(it);
                numberOfPoints--;
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    // Step 6: Finally, computePrimMST for the final set and store that in edges
    double finalLen;
    this->edges = computePrimMST(points, &finalLen);
    std::cout << "[DEBUG] Final MST length after add/remove = "
              << finalLen << "\n\n";
}
    void enforceRectilinearConnections() {
        std::vector<Edge> newEdges;
        int steinerId = 4000; // Starting ID for new Steiner points

        for (const Edge& edge : edges) {
            Point* p1 = findPointById(edge.node1);
            Point* p2 = findPointById(edge.node2);

            if (!p1 || !p2) {
                std::cerr << "Error: Invalid edge endpoints!\n";
                continue; // Skip to the next edge
            }

            if (!isRectilinear(edge, points)) {
                // Create a new Steiner point at the intersection
                Point newSteiner = {steinerId++, p1->x, p2->y, false}; 
                points.push_back(newSteiner);

                // Add two new rectilinear edges
                newEdges.push_back({p1->id, newSteiner.id, mhDistance(*p1, newSteiner)});
                newEdges.push_back({newSteiner.id, p2->id, mhDistance(newSteiner, *p2)});
            } else {
                // Keep the original rectilinear edge
                newEdges.push_back(edge);
            }
        }

        // Replace the old edges with the new edges
        edges = newEdges;
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
            ofs << "edge " << e.node1 << " " << e.node2 << "\n";
        }
        ofs << "\n";

        // Print MST length
        double totalLen = sumEdges(mstEdges);
        ofs << "Path length = " << totalLen << "\n";
        ofs.close();
    }

bool isRectilinear(const Edge& edge, const std::vector<Point>& points) {
    int id1 = edge.node1;
    int id2 = edge.node2;

    // Find the points corresponding to the edge's endpoints
    Point p1, p2;
    for (const auto& p : points) {
        if (p.id == id1) p1 = p;
        if (p.id == id2) p2 = p;
    }

    // Check if either x-coordinates or y-coordinates are the same
    return (std::fabs(p1.x - p2.x) < 1e-9 || std::fabs(p1.y - p2.y) < 1e-9);
}

    // =============== Debug Helper ===============
    void printEdges(const std::vector<Edge>& edgeList) {
    std::cout << "Total number of edges: " << edgeList.size() << "\n";
    for (auto& e : edgeList) {
        std::cout << "Edge between " << e.node1
                  << " and " << e.node2
                  << " has Manhattan distance: " << e.weight;

        // Check and print if the edge is rectilinear
        if (isRectilinear(e, points)) {
            std::cout << " (Rectilinear)";
        } else {
            std::cout << " (Non-Rectilinear)";
        }
        std::cout << "\n";
    }
}
};


// ================= MAIN =================

int main() {
    try {
        // 1) Initialize
        SteinerTree steiner("data/r31.in");

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

        // 4) Add Steiner points
        std::cout << "Adding Steiner Points using Hanan Grid...\n";
        steiner.addSteinerPoints();
        
        steiner.enforceRectilinearConnections();

        // 5) Retrieve the final MST edges after adding/removing Steiner points
        auto finalEdges = steiner.getEdges(); // these are from Prim on the final points
        double finalMSTLen = 0.0;
        for (auto& e : finalEdges) {
            finalMSTLen += e.weight;
        }

        // Print final MST edges
        std::cout << "\nFinal MST Edges with Steiner Points:\n";
        steiner.printEdges(finalEdges);
        std::cout << "Final MST length = " << finalMSTLen << "\n\n";

        // 6) Write to output file
        steiner.writeOutputToFile("stein_tree.txt", finalEdges);
        std::cout << "Output written to file: stein_tree.txt\n";

        return 0;
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}