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
#include <iomanip> // for std::fixed and std::setprecision

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

// A struct to represent a blockage
struct Block {
    int id;
    double x1, y1;
    double x2, y2;
};

bool operator==(const Point& p1, const Point& p2) {
    return (p1.id == p2.id &&
            p1.x  == p2.x  &&
            p1.y  == p2.y  &&
            p1.sink == p2.sink);
}

bool operator==(const Edge& e1, const Edge& e2) {
    return (e1.node1 == e2.node1 &&
            e1.node2  == e2.node2  &&
            e1.weight  == e2.weight);
}

// =================== SteinerTree Class ===================

class SteinerTree {
private:
    std::string filePath;

    // We store all points in this vector (including sinks + Steiner points)
    std::vector<Point> points;

    // The edges that represent the MST on our current set of points
    std::vector<Edge> edges;

    //The blockages in the MST
    std::vector<Block> blockages;

    // The total number of points changes when we add Steiner points
    int numberOfPoints;

    // Keep track of how many sinks we started with
    int originalSinkCount;

    // Number of blockages in the plot
    int numberOfBlocks;

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
    const std::vector<Block>& getBlockages() const {
       return blockages; 
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
            else if (keyword == "number_of_blockages") {
                iss >> numberOfBlocks;
                // blockages.reserve(numberOfBlockages);
            } 
            else if (keyword == "blockage") {
                Block block;
                iss >> block.id >> block.x1 >> block.y1 >> block.x2 >> block.y2;
                blockages.push_back(block);
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

    // Prints the blockages and their info
    void displayBlockages() const {
        std::cout << "Blockages:\n";
        for (const auto& block : blockages) {
            std::cout << "Blockage ID: " << block.id
                      << ", Point 1: (" << block.x1 << ", " << block.y1 << ")"
                      << ", Point 2: (" << block.x2 << ", " << block.y2 << ")\n";
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

        // Check for nearby sink points and create branches
        for (const auto& p : pts) {
            if (p.sink) {
                for (const auto& q : pts) {
                    if (q.sink && p.id != q.id && mhDistance(p, q) < 1.0) { // Adjust the distance threshold as needed
                        Edge branchEdge = {p.id, q.id, mhDistance(p, q)};
                        if (!isEdgeBlocked(branchEdge, pts)) {
                            mstEdges.push_back(branchEdge);
                        }
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

        // Extended hanan grid points
        std::set<double> xVals_blockage;
        std::set<double> yVals_blockage;
        for (const auto& b: blockages){
            xVals_blockage.insert(b.x1);
            xVals_blockage.insert(b.x2);
            yVals_blockage.insert(b.y1);
            yVals_blockage.insert(b.y2);
        }
    
        // //Iterate through x and y sets and add the x and y values
        for (const auto& x: xVals_blockage){
            xVals.push_back(x);
        }

        for (const auto& y: yVals_blockage){
                yVals.push_back(y);
        }

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

        // Filter out blocked points from the hanan grid
        candidates = getUnblockedPoints(candidates);

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

    // Function to check if a point is blocked or not
    bool isBlocked(Point pt){
        bool blocked = false;
        for (const auto& block: blockages){
            if (block.x1 < pt.x && pt.x < block.x2 && block.y1 < pt.y && pt.y < block.y2){
                 blocked = true;
            }
        }
        return blocked;
    }

    // Function that gets all the unblocked points by filtering a vector of given points
    std::vector<Point> getUnblockedPoints(const std::vector<Point>& pts) {
        std::vector<Point> unblockedPts;
        for (const auto& pt : pts) {
            bool blocked = isBlocked(pt);
            if (!blocked){
                unblockedPts.push_back(pt);
            }
        }
        return unblockedPts;
    }

// // Helper function to check the orientation of three points
// int orientation(const Point& p, const Point& q, const Point& r) {
//     double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
//     if (val == 0) return 0; // Collinear
//     return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
// }

// // Helper function to check if point q lies on segment pr
// bool onSegment(const Point& p, const Point& q, const Point& r) {
//     return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
//             q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
// }

// // Function to check if two line segments (p1, q1) and (p2, q2) intersect
// bool doIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
//     int o1 = orientation(p1, q1, p2);
//     int o2 = orientation(p1, q1, q2);
//     int o3 = orientation(p2, q2, p1);
//     int o4 = orientation(p2, q2, q1);

//     // General case: segments intersect
//     if (o1 != o2 && o3 != o4) return true;

//     // Special cases: check if they are collinear and overlap
//     if (o1 == 0 && onSegment(p1, p2, q1)) return true;
//     if (o2 == 0 && onSegment(p1, q2, q1)) return true;
//     if (o3 == 0 && onSegment(p2, p1, q2)) return true;
//     if (o4 == 0 && onSegment(p2, q1, q2)) return true;

//     return false;
// }

// // Function to check if an edge intersects with any blockage
// bool isEdgeBlocked(const Edge& edge, const std::vector<Point>& points) {
//     // Retrieve the endpoints of the edge
//     const Point* node1 = nullptr;
//     const Point* node2 = nullptr;
//     for (const auto& point : points) {
//         if (point.id == edge.node1) node1 = &point;
//         if (point.id == edge.node2) node2 = &point;
//     }

//     // Ensure both points exist
//     if (!node1 || !node2) return false;

//     for (const auto& bloc : blockages) {
//         Point bl = {0, bloc.x1, bloc.y1}; // Bottom-left
//         Point tr = {0, bloc.x2, bloc.y2}; // Top-right
//         Point br = {0, bloc.x2, bloc.y1}; // Bottom-right
//         Point tl = {0, bloc.x1, bloc.y2}; // Top-left

//         // Check for intersection with any side of the blockage
//         bool intersects =
//             doIntersect(*node1, *node2, bl, br) ||
//             doIntersect(*node1, *node2, br, tr) ||
//             doIntersect(*node1, *node2, tr, tl) ||
//             doIntersect(*node1, *node2, tl, bl);

//         if (intersects) {
//             // Check if the edge lies entirely on the boundary
//             if ((onSegment(bl, *node1, br) && onSegment(bl, *node2, br)) || // Bottom edge
//                 (onSegment(br, *node1, tr) && onSegment(br, *node2, tr)) || // Right edge
//                 (onSegment(tr, *node1, tl) && onSegment(tr, *node2, tl)) || // Top edge
//                 (onSegment(tl, *node1, bl) && onSegment(tl, *node2, bl))) {
//                 continue; // Consider as not blocked
//             }
//             return true; // Edge is blocked
//         }
//     }

//     return false; // Edge is unblocked
// }

    // Function to check the orientation of the triplet (p, q, r)
    int orientation(const Point& p, const Point& q, const Point& r) {
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (val == 0) return 0; // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }

    // Function to check if point q lies on segment pr
    bool onSegment(const Point& p, const Point& q, const Point& r) {
        if (q.x < std::max(p.x, r.x) && q.x > std::min(p.x, r.x) &&
            q.y < std::max(p.y, r.y) && q.y > std::min(p.y, r.y))
            return true;
        return false;
    }

    // Function to check if two line segments (p1, q1) and (p2, q2) intersect
    bool doIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;
    }

    //Function to find point by ID in a given list of points
    const Point* findPointByID_list(int id, std::vector<Point> pt_list) {
        for (auto &p : pt_list) {
            if (p.id == id) return &p;
        }
        return nullptr;
    }


    // Function that checks if an edge is crossing any blockages
    // bool isEdgeBlocked(Edge edge, std::vector<Point> tempPoints) {
    //     Point node1_ptr = *findPointByID_list(edge.node1, tempPoints);
    //     Point node2_ptr = *findPointByID_list(edge.node2, tempPoints);
    //     std::cout << "IN FUNCTION:" << edge.weight << "\n";
    //     // Make exception for diagonal edges
    //     if ((node1_ptr.x != node2_ptr.x) && (node1_ptr.y != node2_ptr.y)) {
    //         std::cout << "diagonal case: " << node1_ptr.x << "  "
    //         << node2_ptr.x << "  " << node1_ptr.y << "  " << node2_ptr.y << "\n";
    //         return false;
    //     }

    //     std::cout << "Cords: " << node1_ptr.x << "  "
    //         << node2_ptr.x << "  " << node1_ptr.y << "  " << node2_ptr.y << "\n";
        
    //     for (const auto& bloc : blockages) {
    //         Point bl = {1, bloc.x1, bloc.y1}; // bottom-left
    //         Point tr = {2, bloc.x2, bloc.y2}; // top-right
    //         Point br = {3, bloc.x2, bloc.y1}; // bottom-right
    //         Point tl = {4, bloc.x1, bloc.y2}; // top-left
    //         std::cout << "IN LOOP:" << edge.weight << "\n";
    //         // Check if the edge intersects any side of the blockage rectangle
    //         if (doIntersect(node1_ptr, node2_ptr, bl, br) || 
    //             doIntersect(node1_ptr, node2_ptr, br, tr) || 
    //             doIntersect(node1_ptr, node2_ptr, tr, tl) || 
    //             doIntersect(node1_ptr, node2_ptr, tl, bl)) {
    //             std::cout << "DO INTERSECT:" << edge.weight << "\n";
    //             // Additional checks
    //             // To see if the edge lies on the boundary
    //             if ((onSegment(bl, node1_ptr, br) && onSegment(bl, node2_ptr, br)) || // Bottom edge
    //                 (onSegment(br, node1_ptr, tr) && onSegment(br, node2_ptr, tr)) || // Right edge
    //                 (onSegment(tr, node1_ptr, tl) && onSegment(tr, node2_ptr, tl)) || // Top edge
    //                 (onSegment(tl, node1_ptr, bl) && onSegment(tl, node2_ptr, bl))) { // Left edge
    //                 return false; // Edge lies on the boundary
    //             }
    //             // To see if edge nodes lie on blockage corners
    //             int cornerCount = 0;
    //             if ((node1_ptr.x == bl.x && node1_ptr.y == bl.y) || 
    //                 (node2_ptr.x == bl.x && node2_ptr.y == bl.y)) {
    //                 cornerCount++;
    //             }
    //             if ((node1_ptr.x == br.x && node1_ptr.y == br.y) || 
    //                 (node2_ptr.x == br.x && node2_ptr.y == br.y)) {
    //                 cornerCount++;
    //             }
    //             if ((node1_ptr.x == tr.x && node1_ptr.y == tr.y) || 
    //                 (node2_ptr.x == tr.x && node2_ptr.y == tr.y)) {
    //                 cornerCount++;
    //             }
    //             if ((node1_ptr.x == tl.x && node1_ptr.y == tl.y) || 
    //                 (node2_ptr.x == tl.x && node2_ptr.y == tl.y)) {
    //                 cornerCount++;
    //             }
    //             std::cout << "Corner Count = " << cornerCount << "\n";
    //             if (cornerCount == 1) {
    //                 return false; // Edge touches only two corners
    //             }   

    //             return true; // Edge intersects and does not lie on the boundary
    //         }
    //     }

    //     return false; // Edge is unblocked
    // }


    bool isEdgeBlocked(Edge edge, const std::vector<Point>& points) {
        // Find the start and end points of the edge
        const Point* p1 = findPointByID(edge.node1);
        const Point* p2 = findPointByID(edge.node2);

        if (!p1 || !p2) {
            // If the points don't exist, return false (edge is not blocked)
            return false;
        }

        // Ensure p1 is the smaller point in terms of coordinates for consistent checking
        if (p1->x > p2->x || (p1->x == p2->x && p1->y > p2->y)) {
            std::swap(p1, p2);
        }

        // Iterate over all blockages
        for (const auto& block : blockages) {
            // Define the bounds of the blockage
            double minX = std::min(block.x1, block.x2);
            double maxX = std::max(block.x1, block.x2);
            double minY = std::min(block.y1, block.y2);
            double maxY = std::max(block.y1, block.y2);

            // Check if the edge is vertical
            if (std::fabs(p1->x - p2->x) < 1e-9) {
                double x = p1->x; // The x-coordinate of the vertical edge
                if (x > minX && x < maxX && p1->y < maxY && p2->y > minY) {
                    // Edge crosses the blockage vertically
                    return true;
                }
            }
            // Check if the edge is horizontal
            else if (std::fabs(p1->y - p2->y) < 1e-9) {
                double y = p1->y; // The y-coordinate of the horizontal edge
                if (y > minY && y < maxY && p1->x < maxX && p2->x > minX) {
                    // Edge crosses the blockage horizontally
                    return true;
                }
            }
        }

        // No blockage blocks the edge
        return false;
    }

    // bool isEdgeBlocked(Edge edge, std::vector<Point> tempPoints){
    //     std::cout << "top of isEdgeBlocked()\n";
    //     const Point* node1_ptr = findPointByID_list(edge.node1, tempPoints);
    //     const Point* node2_ptr = findPointByID_list(edge.node2, tempPoints);
    //     // std::cout << "Points read\n";
    //     // std::cout << "Any null : " << (edge.node1) << "\n";
    //     //Make exception for diagonal edges
    //     if(node1_ptr->x != node2_ptr->x && node1_ptr->y != node2_ptr->y){
    //         std::cout << "Diagonal exception" << edge.weight << "\n";
    //         return false;
    //     }
        
    //     for(const auto& bloc: blockages){
    //         Point bl = {1, bloc.x1, bloc.y1}; // bottom-left 
    //         Point tr = {2, bloc.x2, bloc.y2}; // top-right 
    //         Point br = {3, bloc.x2, bloc.y1}; // bottom-right 
    //         Point tl = {4, bloc.x1, bloc.y2}; // top-left

    //         // Check if the edge intersects any side of the blockage rectangle 
    //         if (doIntersect(*node1_ptr, *node2_ptr, bl, br) || 
    //             doIntersect(*node1_ptr, *node2_ptr, br, tr) || 
    //             doIntersect(*node1_ptr, *node2_ptr, tr, tl) || 
    //             doIntersect(*node1_ptr, *node2_ptr, tl, bl)) { 
    //                 return true; // Edge is blocked 
    //         }
    //     }

    //     return false; // Edge is unblocked 
    // }

    // Function to get the number of edges crossing blockages 
    double getNumberOfEdgeBlockages (std::vector<Edge> MST, std::vector<Point> tempPoints){
        
        double numberBlocks = 0;
        // printEdges(MST);
        for(const auto& edge: MST){
            std::cout << "here" << edge.weight << "\n";
            if(isEdgeBlocked(edge, tempPoints)){
                
                numberBlocks+=1;
                std::cout << "numBlocks" << numberBlocks <<"\n";
            }
            std::cout << "loop cycle\n";
        }
        std::cout << "end";
        return numberBlocks;
    }

    // Function to calculate the blockage cost based on the blocked edges in the MST
    double getBlockageCost(const std::vector<Edge>& candidateMST) {
        double cost = 0;

        // Iterate over the edges of the candidate MST
        for (const auto& edge : candidateMST) {
            // Check if the edge crosses any blockages
            if (isEdgeBlocked(edge, points)) {
                // Add the weight of the blocked edge to the cost (you can modify this logic)
                cost += edge.weight;
            }
        }

        // Return the total blockage cost
        return cost;
    }

    // Function to check if a point on the edge of some blockage
    bool isPointOnBlockageEdge(Point p){
        bool output = false;
        for(const auto& bloc: blockages){

            // Get the min and max coordinates of the blockage
            double x_min = std::min(bloc.x1, bloc.x2), x_max = std::max(bloc.x1, bloc.x2);
            double y_min = std::min(bloc.y1, bloc.y2), y_max = std::max(bloc.y1, bloc.y2);

            // Check if the point is on the vertical or horizontal edges
            bool on_vertical_edge = (p.x == x_min || p.x == x_max) && (p.y >= y_min && p.y <= y_max);
            bool on_horizontal_edge = (p.y == y_min || p.y == y_max) && (p.x >= x_min && p.x <= x_max);

            output = output || (on_vertical_edge || on_horizontal_edge);
        }
        return output;
    }

    // Function to determine the other 3 points around the blockage
    std::vector<Point> findPathAroundBlockage(Point edgePt, Block block){
        
        // Get the min and max coordinates of the blockage
        double x_min = std::min(block.x1, block.x2), x_max = std::max(block.x1, block.x2);
        double y_min = std::min(block.y1, block.y2), y_max = std::max(block.y1, block.y2);

        std::vector<Point> path;
        
        int ID = 3000; // temp ID for new points
        // Check where the edgePoint lies and add points accordingly
        if (edgePt.x == x_min) {
            // Left vertical edge
            path.push_back({ID + 1, x_min, edgePt.y});       // Starting point
            path.push_back({ID + 2, x_min, y_min});   // Bottom-left corner
            path.push_back({ID + 3, x_max, y_min});   // Bottom-right corner
            path.push_back({ID + 4, x_max, edgePt.y});      // End point on the right vertical edge
        } else if (edgePt.x == x_max) {
            // Right vertical edge
            path.push_back({ID + 5, x_max, edgePt.y});       // Starting point
            path.push_back({ID + 6, x_max, y_max});   // Top-right corner
            path.push_back({ID + 7, x_min, y_max});   // Top-left corner
            path.push_back({ID + 8, x_min, edgePt.y});      // End point on the left vertical edge
        } else if (edgePt.y == y_min) {
            // Bottom horizontal edge
            path.push_back({ID + 9, edgePt.x, y_min});      // Starting point
            path.push_back({ID + 10, x_max, y_min});   // Bottom-right corner
            path.push_back({ID + 11, x_max, y_max});   // Top-right corner
            path.push_back({ID + 12, edgePt.x, y_max});      // End point on the top horizontal edge
        } else if (edgePt.y == y_max) {
            // Top horizontal edge
            path.push_back({ID + 13, edgePt.x, y_max});      // Starting point
            path.push_back({ID + 14, x_min, y_max});   // Top-left corner
            path.push_back({ID + 15, x_min, y_min});   // Bottom-left corner
            path.push_back({ID + 16, edgePt.x, y_min});      // End point on the bottom horizontal edge
        }

        return path;
    }

    // Helper func to get Block that contains point (point on  block's edge is considered to be in block)
    Block getBlockagebyPoint(Point pt){
        Block block_output;
        for (const auto& b: blockages){
            if(b.x1 <= pt.x && pt.x <= b.x2 && b.y1 <= pt.y && pt.y <= b.y2){
                block_output = b;
                break;
            }
        }
        return block_output;
    }


    // =================== ADD STEINER POINTS ===================
    void addSteinerPoints() {

        // 1) Compute baseline MST with the current official set of points
        double baseLength;
        auto baseMST = computePrimMST(points, &baseLength);
        std::cout << "[DEBUG] Initial baseline MST length = " << baseLength << "\n";

        // 2) Generate all possible Hanan points
        auto hanan = getHananGridPoints(points);

        // 3) Iteratively add the best-improving Steiner point
        bool improved = true;
        while (improved) {
            improved = false;
            double bestImprovement = 0.0;
            Point bestSteiner;
            
            std::cout << "[DEBUG] --- New iteration over Hanan points. "
                      << "Current #points=" << points.size()
                      << ", Baseline MST length=" << baseLength << "\n";

            for (auto& candidate : hanan) {
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
                    bestSteiner     = candidate;
                    improved        = true;
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
                // // Recompute Hanan grid with newly added point
                // hanan = getHananGridPoints(points);
                
                // Remove added Steiner point from Hanan points(potential steiner points)
                hanan.erase(std::remove_if(hanan.begin(), hanan.end(), 
                               [&bestSteiner](const Point& p) {
                                   return p.id == bestSteiner.id;
                               }), hanan.end());
            }
        }

        // 4) Attempt removal: if removing a Steiner doesn’t worsen the MST, remove it
        // std::cout << "\n[DEBUG] Checking if we can remove any Steiner points...\n";
        // auto it = points.begin();
        // while (it != points.end()) {
        //     if (!it->sink) {
        //         // Temporarily remove this Steiner point
        //         std::vector<Point> tempPoints = points;
        //         tempPoints.erase(std::remove(tempPoints.begin(),
        //                                      tempPoints.end(), *it),
        //                          tempPoints.end());

        //         double newLen;
        //         auto newMST = computePrimMST(tempPoints, &newLen);

        //         double curLen;
        //         auto curMST = computePrimMST(points, &curLen);

        //         std::cout << "   [DEBUG] Removal check => Steiner ID=" << it->id 
        //                   << ": MST if removed=" << newLen
        //                   << ", current MST=" << curLen << "\n";

        //         if (newLen < curLen) {
        //             std::cout << "   [DEBUG] >>> Removing Steiner ID=" 
        //                       << it->id << "\n";
        //             it = points.erase(it);
        //             numberOfPoints--;
        //         } else {
        //             ++it;
        //         }
        //     } else {
        //         ++it;
        //     }
        // }

        // Finally, computePrimMST for the final set and store that in `edges`
        double finalLen;
        this->edges = computePrimMST(points, &finalLen);
        std::cout << "[DEBUG] Final MST length after add/remove = "
                  << finalLen << "\n\n";
    }

    // Post-process the edges to "L" points and edges
    void postProcess() {
        std::vector<Edge> updatedEdges;
        for (const auto& e : edges) {
            const Point* p1 = findPointByID(e.node1);
            const Point* p2 = findPointByID(e.node2);
            if (!p1 || !p2) continue;

            // If already rectilinear and unblocked, keep the edge as is
            if (std::fabs(p1->x - p2->x) < 1e-9 || std::fabs(p1->y - p2->y) < 1e-9) {
                if (isEdgeBlocked(e, points)) {
                    std::cout << "RECTILINEAR PROCESS" << e.weight << "\n";
                    double edgePt_ID = isPointOnBlockageEdge(*findPointByID(e.node1)) ? 
                                        e.node1 : e.node2;
                    double nonEdgePt_ID = isPointOnBlockageEdge(*findPointByID(e.node1)) ? 
                                        e.node2 : e.node1;
                    Block curr_block = getBlockagebyPoint(*findPointByID(edgePt_ID));
                    
                    std::vector<Point> path = findPathAroundBlockage(*findPointByID(edgePt_ID), curr_block);
                    for (const auto& pathPt : path) {
                        points.push_back(pathPt);
                    }
                    path.push_back(*findPointByID(nonEdgePt_ID));

                    // Push the path edges
                    bool swch = false;
                    Point prev;
                    for (const auto& P : path) {
                        if (swch) {
                            updatedEdges.push_back({prev.id, P.id, mhDistance(prev, P)});
                        }
                        prev = P;
                        swch = true;
                    }
                } else {
                    updatedEdges.push_back(e);
                }
            } else {
                // Add intermediate Steiner point
                int midID = 20000 + e.node1; // any unique ID scheme
                Point p_option1; // L-point 1
                p_option1.x = p1->x;
                p_option1.y = p2->y;
                Point p_option2; // L-point 2
                p_option2.x = p2->x;
                p_option2.y = p1->y;
                Point intermediatePt;
                if (isBlocked(p_option1)) {
                    intermediatePt = {midID, p2->x, p1->y}; // option 2
                } else {
                    intermediatePt = {midID, p1->x, p2->y}; // option 1
                }

                points.push_back(intermediatePt);

                // Create two new edges
                Edge newEdge1 = {e.node1, midID, mhDistance(*findPointByID(e.node1), intermediatePt)};
                Edge newEdge2 = {midID, e.node2, mhDistance(intermediatePt, *findPointByID(e.node2))};

                // DEBUG
                std::vector<Edge> debug_e;
                debug_e.push_back(newEdge1);
                debug_e.push_back(newEdge2);
                std::cout << "New Edges\n";
                printEdges(debug_e);
                std::cout << "NewEdge1 blocked= " << isEdgeBlocked(newEdge1, points) << "\n"
                          << "NewEdge2 blocked= " << isEdgeBlocked(newEdge2, points) << "\n";

                if (!isEdgeBlocked(newEdge1, points) && !isEdgeBlocked(newEdge2, points)) {
                    updatedEdges.push_back(newEdge1);
                    updatedEdges.push_back(newEdge2);
                    std::cout << "NORMAL CASE\n";
                } else if (isEdgeBlocked(newEdge1, points) && !isEdgeBlocked(newEdge2, points)) {
                    std::cout << "CASE 2" << "\n";
                    updatedEdges.push_back(newEdge2); // push the unblocked edge
                    // Here newEdge1 is blocked
                    double edgePt_ID = isPointOnBlockageEdge(*findPointByID(newEdge1.node1)) ? 
                                        newEdge1.node1 : newEdge1.node2;
                    double nonEdgePt_ID = isPointOnBlockageEdge(*findPointByID(newEdge1.node1)) ? 
                                        newEdge1.node2 : newEdge1.node1;
                    Point debug = *findPointByID(edgePt_ID);
                    
                    std::cout << "EdgePt x= " << debug.x << ", y= " << debug.y << "\n";
                    Block curr_block = getBlockagebyPoint(*findPointByID(edgePt_ID));
                    std::cout << "Block ID= " << curr_block.id << "\n";
                    std::vector<Point> path = findPathAroundBlockage(*findPointByID(edgePt_ID), curr_block);
                    for (const auto& pathPt : path) {
                        points.push_back(pathPt);
                    }
                    path.push_back(*findPointByID(nonEdgePt_ID));

                    // Push the path edges
                    bool swch = false;
                    Point prev;
                    for (const auto& P : path) {
                        if (swch) {
                            updatedEdges.push_back({prev.id, P.id, mhDistance(prev, P)});
                        }
                        prev = P;
                        swch = true;
                    }

                } else if (!isEdgeBlocked(newEdge1, points) && isEdgeBlocked(newEdge2, points)) {
                    std::cout << "CASE 3" << "\n";
                    updatedEdges.push_back(newEdge1); // push the unblocked edge
                    // Here newEdge2 is blocked
                    double edgePt_ID = isPointOnBlockageEdge(*findPointByID(newEdge2.node1)) ? 
                                        newEdge2.node1 : newEdge2.node2;
                    double nonEdgePt_ID = isPointOnBlockageEdge(*findPointByID(newEdge2.node1)) ? 
                                        newEdge2.node2 : newEdge2.node1;
                    Block curr_block = getBlockagebyPoint(*findPointByID(edgePt_ID));
                    
                    std::vector<Point> path = findPathAroundBlockage(*findPointByID(edgePt_ID), curr_block);
                    for (const auto& pathPt : path) {
                        points.push_back(pathPt);
                    }
                    path.push_back(*findPointByID(nonEdgePt_ID));

                    // Push the path edges
                    bool swch = false;
                    Point prev;
                    for (const auto& P : path) {
                        if (swch) {
                            updatedEdges.push_back({prev.id, P.id, mhDistance(prev, P)});
                        }
                        prev = P;
                        swch = true;
                    }

                } else {
                    std::cout << "CASE 4" << "\n";
                    // Both edges are blocked
                    
                    // Handle first blocked edge (newEdge1)
                    double edgePt1_ID = isPointOnBlockageEdge(*findPointByID(newEdge1.node1)) ? 
                                        newEdge1.node1 : newEdge1.node2;
                    double nonEdgePt1_ID = isPointOnBlockageEdge(*findPointByID(newEdge1.node1)) ? 
                                        newEdge1.node2 : newEdge1.node1;
                    Block block1 = getBlockagebyPoint(*findPointByID(edgePt1_ID));
                    std::vector<Point> path1 = findPathAroundBlockage(*findPointByID(edgePt1_ID), block1);
                    for (const auto& pathPt : path1) {
                        points.push_back(pathPt);
                    }
                    path1.push_back(*findPointByID(nonEdgePt1_ID));

                    // Handle second blocked edge (newEdge2)
                    double edgePt2_ID = isPointOnBlockageEdge(*findPointByID(newEdge2.node1)) ? 
                                        newEdge2.node1 : newEdge2.node2;
                    double nonEdgePt2_ID = isPointOnBlockageEdge(*findPointByID(newEdge2.node1)) ? 
                                        newEdge2.node2 : newEdge2.node1;
                    Block block2 = getBlockagebyPoint(*findPointByID(edgePt2_ID));
                    std::vector<Point> path2 = findPathAroundBlockage(*findPointByID(edgePt2_ID), block2);
                    for (const auto& pathPt : path2) {
                        points.push_back(pathPt);
                    }
                    path2.push_back(*findPointByID(nonEdgePt2_ID));

                    // Add path1 edges
                    bool swch = false;
                    Point prev;
                    for (const auto& P : path1) {
                        if (swch) {
                            updatedEdges.push_back({prev.id, P.id, mhDistance(prev, P)});
                        }
                        prev = P;
                        swch = true;
                    }

                    // Add path2 edges
                    swch = false;
                    for (const auto& P : path2) {
                        if (swch) {
                            updatedEdges.push_back({prev.id, P.id, mhDistance(prev, P)});
                        }
                        prev = P;
                        swch = true;
                    }
                }
            }
        }

        // Check for nearby sink points and create branches
        for (const auto& p : points) {
            if (p.sink) {
                for (const auto& q : points) {
                    if (q.sink && p.id != q.id && mhDistance(p, q) < 1.0) { // Adjust the distance threshold as needed
                        Edge branchEdge = {p.id, q.id, mhDistance(p, q)};
                        if (!isEdgeBlocked(branchEdge, points)) {
                            updatedEdges.push_back(branchEdge);
                        }
                    }
                }
            }
        }
        // Replace edges with updatedEdges
        edges = std::move(updatedEdges);
    }

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

        // Print blockages
        ofs << "number_of_blockages " << blockages.size() << "\n";
        ofs << std::fixed << std::setprecision(1); // Set fixed-point notation and precision to 1
        for (const auto& block : blockages) {
            ofs << "blockage " << block.id << " " << block.x1 << " " << block.y1
                << " " << block.x2 << " " << block.y2 << "\n";
        }
        ofs << "\n";

        // Print MST edges
        for (auto& e : mstEdges) {
            const Point* p1 = findPointByID(e.node1);
            const Point* p2 = findPointByID(e.node2);
            if (!p1 || !p2) continue;
            // If they share x or y, it's already rectilinear
            // if (std::fabs(p1->x - p2->x) < 1e-9 ||
            //     std::fabs(p1->y - p2->y) < 1e-9) {
                ofs << "edge " << e.node1 << " " << e.node2 << "\n";
            // } else {
            //     // Make an intermediate Steiner point at (p1->x, p2->y)
            //     // (Or choose (p2->x, p1->y) – either is fine)
            //     int midID = 20000 + e.node1; // any unique ID scheme
            //     ofs << "point " << midID << " " << p1->x << " " << p2->y << "\n";
            //     ofs << "edge " << e.node1 << " " << midID << "\n";
            //     ofs << "edge " << midID   << " " << e.node2 << "\n";
            // }
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
            Point p1 = *findPointByID(e.node1);
            Point p2 = *findPointByID(e.node2);
            std::cout << "Edge between " << e.node1 <<" (" << p1.x << ", " << p1.y << ")"
                      << " and " << e.node2 <<" (" << p2.x << ", " << p2.y << ")"
                      << " has Manhattan distance: " << e.weight << "\n";
        }
    }
};


void reorderOutputFile(const std::string &filename) {
     std::ifstream in(filename);
    if (!in) return;

    std::string line, numberOfSinks, numberOfBlockages, pathLength;
    std::vector<std::string> sinks, points, edges, blockages;

    while (std::getline(in, line)) {
        if (line.rfind("number_of_sinks", 0) == 0) {
            numberOfSinks = line;
        } else if (line.rfind("number_of_blockages", 0) == 0) {
            numberOfBlockages = line;
        } else if (line.rfind("sink", 0) == 0) {
            sinks.push_back(line);
        } else if (line.rfind("blockage", 0) == 0) {
            blockages.push_back(line);
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
    out << numberOfBlockages << "\n";
    for (auto &b : blockages) out << b << "\n";
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
        SteinerTree steiner("data_b/r31b.in");

        // 2) Print sinks that we read in
        steiner.printSinks();
        steiner.displayBlockages();

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
        steiner.postProcess();

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
        steiner.writeOutputToFile("steiner_tree_output.txt", finalEdges);
        std::cout << "Output written to file: output/steiner_tree_output.txt\n";

        reorderOutputFile("steiner_tree_output.txt");

        return 0;
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
