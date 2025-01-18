#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <set>
#include <limits>
#include <iomanip>

// A struct to represent a point (sink or Steiner)
struct Point
{
    int id;
    double x, y;
    bool sink; // true if originally from input, false if added as a Steiner point

    Point() : id(-1), x(0), y(0), sink(false) {} // Default constructor
    Point(int id, double x, double y, bool sink = false) : id(id), x(x), y(y), sink(sink) {}
    Point(double x, double y, bool sink = false) : x(x), y(y), sink(sink) {}
};

// A struct to represent an edge
struct Edge
{
    int node1, node2; // The IDs of the endpoints
    double weight;

    Edge(int node1, int node2, double weight) : node1(node1), node2(node2), weight(weight) {}
};

// A struct to represent a blockage
struct Block
{
    int id;
    double x1, y1, x2, y2;
};

// =================== SteinerTree Class ===================

class SteinerTree
{
private:
    std::string filePath;
    std::vector<Point> points;
    std::vector<Edge> edges;
    std::vector<Block> blockages;
    int numberOfPoints;
    int originalSinkCount;
    int numberOfBlocks;

    // Helper function: Check if two line segments intersect
    bool doLinesIntersect(const std::pair<double, double> &p1, const std::pair<double, double> &q1,
                          const std::pair<double, double> &p2, const std::pair<double, double> &q2)
    {
        auto orientation = [](const std::pair<double, double> &p,
                              const std::pair<double, double> &q,
                              const std::pair<double, double> &r)
        {
            double val = (q.second - p.second) * (r.first - q.first) -
                         (q.first - p.first) * (r.second - q.second);
            if (fabs(val) < 1e-9)
                return 0;             // Collinear
            return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
        };

        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case: intersect if orientations differ
        if (o1 != o2 && o3 != o4)
            return true;

        // Special cases: Check for collinear points lying on the segment
        auto onSegment = [](const std::pair<double, double> &p,
                            const std::pair<double, double> &q,
                            const std::pair<double, double> &r)
        {
            return q.first <= std::max(p.first, r.first) && q.first >= std::min(p.first, r.first) &&
                   q.second <= std::max(p.second, r.second) && q.second >= std::min(p.second, r.second);
        };

        if (o1 == 0 && onSegment(p1, p2, q1))
            return true;
        if (o2 == 0 && onSegment(p1, q2, q1))
            return true;
        if (o3 == 0 && onSegment(p2, p1, q2))
            return true;
        if (o4 == 0 && onSegment(p2, q1, q2))
            return true;

        return false;
    }

    // Function to check if an edge intersects a block
    bool doesEdgeIntersectBlock(const std::pair<double, double> &start,
                                const std::pair<double, double> &end,
                                const Block &block)
    {
        // Check if either endpoint is inside the block
        auto isInsideBlock = [&](const std::pair<double, double> &point)
        {
            return point.first >= block.x1 && point.first <= block.x2 &&
                   point.second >= block.y1 && point.second <= block.y2;
        };

        if (isInsideBlock(start) || isInsideBlock(end))
        {
            std::cout << "Edge from (" << start.first << ", " << start.second << ") to ("
                      << end.first << ", " << end.second << ") is inside block " << block.id << "\n";
            return true;
        }

        // Check intersection with each side of the block
        std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> blockEdges = {
            {{block.x1, block.y1}, {block.x1, block.y2}}, // Left edge
            {{block.x1, block.y2}, {block.x2, block.y2}}, // Top edge
            {{block.x2, block.y2}, {block.x2, block.y1}}, // Right edge
            {{block.x2, block.y1}, {block.x1, block.y1}}  // Bottom edge
        };

        for (const auto &blockEdge : blockEdges)
        {
            if (doLinesIntersect(start, end, blockEdge.first, blockEdge.second))
            {
                std::cout << "Edge from (" << start.first << ", " << start.second << ") to ("
                          << end.first << ", " << end.second << ") intersects with block " << block.id << "\n";
                return true;
            }
        }

        return false;
    }

    // Function to check if an edge is blocked by any obstacle
    bool isEdgeBlocked(const std::pair<double, double> &start,
                       const std::pair<double, double> &end,
                       const std::vector<Block> &blockages)
    {
        for (const auto &block : blockages)
        {
            if (doesEdgeIntersectBlock(start, end, block))
            {
                std::cout << "Edge from (" << start.first << ", " << start.second << ") to ("
                          << end.first << ", " << end.second << ") is blocked by block " << block.id << "\n";
                return true;
            }
        }
        return false;
    }

    // Function to find a path around a blockage
    std::vector<Point> findPathAroundBlockage(const std::pair<double, double> &start,
                                              const std::pair<double, double> &end,
                                              const std::vector<Block> &blockages)
    {
        std::vector<Point> path;
        path.push_back(Point(start.first, start.second));

        // Find paths with better detours
        for (const auto &block : blockages)
        {
            if (doesEdgeIntersectBlock(start, end, block))
            {
                // Add intermediate points to go around the block
                Point intermediate1(block.x1 - 0.2, block.y1 - 0.2); // Use geometry to make detours optimal
                Point intermediate2(block.x2 + 0.2, block.y2 + 0.2);
                path.push_back(intermediate1);
                path.push_back(intermediate2);
                std::cout << "Adding detour points around block " << block.id << ": ("
                          << intermediate1.x << ", " << intermediate1.y << ") and ("
                          << intermediate2.x << ", " << intermediate2.y << ")\n";
                break;
            }
        }

        path.push_back(Point(end.first, end.second)); // Add endpoint
        return path;
    }

    // Function to construct obstacle-avoiding spanning graph
    std::vector<Edge> obstacleAvoidingSpanningGraph(const std::vector<Point> &points,
                                                    const std::vector<Block> &blockages)
    {
        std::vector<Edge> oasgEdges;

        for (size_t i = 0; i < points.size(); ++i)
        {
            for (size_t j = i + 1; j < points.size(); ++j)
            {
                std::pair<double, double> start = {points[i].x, points[i].y};
                std::pair<double, double> end = {points[j].x, points[j].y};

                if (!isEdgeBlocked(start, end, blockages))
                {
                    oasgEdges.emplace_back(points[i].id, points[j].id, mhDistance(points[i], points[j]));
                    std::cout << "Adding edge between points " << points[i].id << " and " << points[j].id << "\n";
                }
                else
                {
                    // Compute obstacle-avoiding path
                    std::vector<Point> path = findPathAroundBlockage(start, end, blockages);
                    double pathLength = 0;

                    // Add points in the path to the main points vector
                    for (const auto &p : path)
                    {
                        addUniqueSteinerPoint(p); // Ensure no duplicates are added
                    }

                    for (size_t k = 1; k < path.size(); ++k)
                    {
                        pathLength += mhDistance(path[k - 1], path[k]);
                    }
                    oasgEdges.emplace_back(points[i].id, points[j].id, pathLength);
                    std::cout << "Adding edge with detour between points " << points[i].id << " and " << points[j].id << " with length " << pathLength << "\n";
                }
            }
        }
        return oasgEdges;
    }

    // Compute MST using Prim's algorithm
    std::vector<Edge> computePrimMST(const std::vector<Point> &pts, double *outMstLen = nullptr)
    {
        if (pts.size() < 2)
        {
            if (outMstLen)
                *outMstLen = 0.0;
            return {};
        }

        int n = (int)pts.size();
        std::unordered_map<int, int> idToIndex;
        idToIndex.reserve(n);
        for (int i = 0; i < n; i++)
        {
            idToIndex[pts[i].id] = i;
        }

        std::vector<Edge> mstEdges;
        mstEdges.reserve(n - 1);
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> parent(n, -1);
        std::vector<bool> inMST(n, false);

        dist[0] = 0.0;

        for (int i = 0; i < n - 1; i++)
        {
            int u = -1;
            double best = std::numeric_limits<double>::infinity();
            for (int v = 0; v < n; v++)
            {
                if (!inMST[v] && dist[v] < best)
                {
                    best = dist[v];
                    u = v;
                }
            }

            if (u == -1) // If no vertex is found, break
                break;

            inMST[u] = true;
            if (parent[u] != -1)
            {
                Edge e(pts[u].id, pts[parent[u]].id, mhDistance(pts[u], pts[parent[u]]));
                mstEdges.push_back(e);
                std::cout << "Adding edge to MST: " << e.node1 << " - " << e.node2 << " with weight " << e.weight << "\n";
            }

            for (int v = 0; v < n; v++)
            {
                if (!inMST[v])
                {
                    double cost = mhDistance(pts[u], pts[v]);
                    if (cost < dist[v])
                    {
                        dist[v] = cost;
                        parent[v] = u;
                        std::cout << "Updating distance for point " << pts[v].id << " to " << cost << "\n";
                    }
                }
            }
        }

        if (outMstLen)
        {
            double totalLen = 0.0;
            for (const auto &e : mstEdges)
            {
                totalLen += e.weight;
            }
            *outMstLen = totalLen;
            std::cout << "Total MST length: " << totalLen << "\n";
        }
        return mstEdges;
    }

public:
    // Constructor
    SteinerTree(const std::string &path) : filePath(path), numberOfPoints(0), originalSinkCount(0)
    {
        readInputFile();
        originalSinkCount = numberOfPoints; // store the initial sink count
    }

    // Post-process and check for violations
    void postProcessAndCheckForViolations(std::vector<Edge> &mstEdges)
    {
        std::cout << "Starting post-processing to check for violations...\n";

        // Check all points against blockages
        for (const auto &point : points)
        {
            for (const auto &block : blockages)
            {
                if (point.x >= block.x1 && point.x <= block.x2 && point.y >= block.y1 && point.y <= block.y2)
                {
                    std::cout << "Violation: Point (" << point.x << ", " << point.y << ") overlaps with blockage ID " << block.id << "\n";
                    // Trigger recalculation
                    std::cout << "Recalculating obstacle-avoiding spanning graph and MST...\n";
                    auto oasgEdges = obstacleAvoidingSpanningGraph(points, blockages);
                    mstEdges = computePrimMST(points);
                    return; // Exit after recalculation
                }
            }
        }

        // Check all edges against blockages
        for (const auto &e : mstEdges)
        {
            const Point *p1 = findPointByID(e.node1);
            const Point *p2 = findPointByID(e.node2);
            if (p1 && p2)
            {
                std::pair<double, double> start = {p1->x, p1->y};
                std::pair<double, double> end = {p2->x, p2->y};

                for (const auto &block : blockages)
                {
                    if (doesEdgeIntersectBlock(start, end, block))
                    {
                        std::cout << "Violation: Edge from (" << start.first << ", " << start.second << ") to ("
                                  << end.first << ", " << end.second << ") intersects with blockage ID " << block.id << "\n";
                        // Trigger recalculation
                        std::cout << "Recalculating obstacle-avoiding spanning graph and MST...\n";
                        auto oasgEdges = obstacleAvoidingSpanningGraph(points, blockages);
                        mstEdges = computePrimMST(points);
                        return; // Exit after recalculation
                    }
                }
            }
        }

        // Check if all points are connected
        std::unordered_set<int> connectedPoints;
        for (const auto &e : mstEdges)
        {
            connectedPoints.insert(e.node1);
            connectedPoints.insert(e.node2);
        }

        for (const auto &point : points)
        {
            if (connectedPoints.find(point.id) == connectedPoints.end())
            {
                std::cout << "Violation: Point ID " << point.id << " is not connected to any edges.\n";
                // Trigger recalculation
                std::cout << "Recalculating obstacle-avoiding spanning graph and MST...\n";
                auto oasgEdges = obstacleAvoidingSpanningGraph(points, blockages);
                mstEdges = computePrimMST(points);
                return; // Exit after recalculation
            }
        }

        std::cout << "Post-processing completed. No violations found.\n";
    }

    // Read input file
    bool readInputFile()
    {
        std::ifstream inputFile(filePath);
        if (!inputFile.is_open())
        {
            throw std::runtime_error("Error opening file: " + filePath);
        }

        std::string line;
        while (std::getline(inputFile, line))
        {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;

            if (keyword == "number_of_sinks")
            {
                iss >> numberOfPoints;
                std::cout << "Number of sinks read: " << numberOfPoints << "\n";
            }
            else if (keyword == "sink")
            {
                Point sink;
                iss >> sink.id >> sink.x >> sink.y;
                sink.sink = true;
                points.push_back(sink);
                std::cout << "Sink added: ID " << sink.id << " at (" << sink.x << ", " << sink.y << ")\n";
            }
            else if (keyword == "number_of_blockages")
            {
                iss >> numberOfBlocks;
                std::cout << "Number of blockages read: " << numberOfBlocks << "\n";
            }
            else if (keyword == "blockage")
            {
                Block block;
                iss >> block.id >> block.x1 >> block.y1 >> block.x2 >> block.y2;
                blockages.push_back(block);
                std::cout << "Blockage added: ID " << block.id << " from (" << block.x1 << ", " << block.y1 << ") to ("
                          << block.x2 << ", " << block.y2 << ")\n";
            }
        }
        inputFile.close();
        return true;
    }

    // Just print how many sinks and their info
    void printSinks()
    {
        std::cout << "Number of sinks: " << numberOfPoints << std::endl;
        for (const auto &p : points)
        {
            if (p.sink)
            {
                std::cout << "Sink ID: " << p.id
                          << ", X: " << p.x
                          << ", Y: " << p.y << std::endl;
            }
        }
    }

    // Prints the blockages and their info
    void displayBlockages() const
    {
        std::cout << "Blockages:\n";
        for (const auto &block : blockages)
        {
            std::cout << "Blockage ID: " << block.id
                      << ", Point 1: (" << block.x1 << ", " << block.y1 << ")"
                      << ", Point 2: (" << block.x2 << ", " << block.y2 << ")\n";
        }
    }

    // Compute Manhattan distance
    double mhDistance(const Point &a, const Point &b)
    {
        return std::fabs(a.x - b.x) + std::fabs(a.y - b.y);
    }

    // Find point by ID
    const Point *findPointByID(int id) const
    {
        for (const auto &p : points)
        {
            if (p.id == id)
                return &p;
        }
        return nullptr;
    }

    // Construct obstacle-avoiding rectilinear Steiner tree
    std::vector<Edge> obstacleAvoidingRectilinearSteinerTree()
    {
        auto oasgEdges = obstacleAvoidingSpanningGraph(points, blockages);
        return computePrimMST(points);
    }

    // Add Steiner points to MST
    std::vector<Edge> addSteinerPointsToMST(const std::vector<Edge> &mstEdges)
    {
        std::vector<Edge> updatedEdges = mstEdges; // Start with existing MST edges
        std::unordered_set<int> existingNodes;     // To track existing nodes in the MST

        // Add existing nodes to the set
        for (const auto &e : mstEdges)
        {
            existingNodes.insert(e.node1);
            existingNodes.insert(e.node2);
        }

        // Iterate through the points to add Steiner points
        for (const auto &point : points)
        {
            if (!point.sink) // Only consider Steiner points
            {
                double minDist = std::numeric_limits<double>::infinity();
                int closestNode = -1;

                // Find the closest existing node in the MST
                for (const auto &node : existingNodes)
                {
                    double dist = mhDistance(point, *findPointByID(node));
                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestNode = node;
                    }
                }

                // If a closest node is found, add an edge to the MST
                if (closestNode != -1 && minDist > 0) // Ensure there's a valid distance
                {
                    updatedEdges.emplace_back(point.id, closestNode, minDist);
                    std::cout << "Connecting Steiner point " << point.id << " to existing node " << closestNode << " with distance " << minDist << "\n";
                }
            }
        }

        return updatedEdges;
    }

    // Post-process MST edges
    std::vector<Edge> postProcess(std::vector<Edge> mstEdges)
    {
        std::vector<Edge> updatedEdges;
        int ID_COUNTER = 1000; // Or whatever starting value you want
        for (const auto &e : mstEdges)
        {
            const Point *p1 = findPointByID(e.node1);
            const Point *p2 = findPointByID(e.node2);
            if (!p1 || !p2)
                continue;

            if (std::fabs(p1->x - p2->x) < 1e-9 || std::fabs(p1->y - p2->y) < 1e-9)
            {
                // Directly connect if they are already on the same horizontal/vertical line
                updatedEdges.push_back(e);
            }
            else
            {
                // If not on the same horizontal/vertical line, add an intermediate point
                Point intermediatePt = {ID_COUNTER++, p2->x, p1->y, false};
                addUniqueSteinerPoint(intermediatePt); // Add unique point

                updatedEdges.emplace_back(e.node1, intermediatePt.id, mhDistance(*p1, intermediatePt));
                updatedEdges.emplace_back(intermediatePt.id, e.node2, mhDistance(intermediatePt, *p2));
                std::cout << "Adding intermediate point " << intermediatePt.id << " between " << e.node1 << " and " << e.node2 << "\n";
            }
        }
        return updatedEdges;
    }

    // Function to add unique Steiner point
    void addUniqueSteinerPoint(const Point &newPoint)
    {
        for (const auto &p : points)
        {
            if (p.x == newPoint.x && p.y == newPoint.y)
            {
                std::cout << "Duplicate point found: (" << newPoint.x << ", " << newPoint.y << "), not adding.\n";
                return; // Duplicate found, do not add
            }
        }
        points.push_back(newPoint); // Add unique Steiner point
        std::cout << "Unique Steiner point added: ID " << newPoint.id << " at (" << newPoint.x << ", " << newPoint.y << ")\n";
    }

    // Write output to file
    void writeOutputToFile(const std::string &outFile, const std::vector<Edge> &mstEdges)
    {
        std::ofstream ofs(outFile);
        if (!ofs.is_open())
        {
            throw std::runtime_error("Error opening output file: " + outFile);
        }

        // Output number of sinks
        ofs << "number_of_sinks " << originalSinkCount << "\n\n";

        // Output sinks
        for (const auto &p : points)
        {
            if (p.sink)
            {
                ofs << "sink " << p.id << " " << p.x << " " << p.y << "\n";
            }
        }
        ofs << "\n";

        // Output number of blockages
        ofs << "number_of_blockages " << blockages.size() << "\n";

        // Output blockages
        for (const auto &block : blockages)
        {
            ofs << "blockage " << block.id << " " << block.x1 << " " << block.y1
                << " " << block.x2 << " " << block.y2 << "\n";
        }
        ofs << "\n";

        // Output points (Steiner points)
        for (const auto &p : points)
        {
            if (!p.sink)
            {
                ofs << "points " << p.id << " " << p.x << " " << p.y << "\n";
            }
        }
        ofs << "\n";

        // Output edges
        for (const auto &e : mstEdges)
        {
            ofs << "edge " << e.node1 << " " << e.node2 << "\n";
        }
        ofs << "\n";

        // Calculate and output total path length
        double totalLen = 0.0;
        for (const auto &e : mstEdges)
        {
            totalLen += e.weight;
        }
        ofs << "Path length = " << totalLen << "\n";
        ofs.close();
        std::cout << "Output written to file: " << outFile << "\n";
    }
};

// ================= MAIN =================

int main()
{
    try
    {
        SteinerTree steiner("r62b in.txt");

        steiner.printSinks();
        steiner.displayBlockages();

        auto mstEdges = steiner.obstacleAvoidingRectilinearSteinerTree();
        mstEdges = steiner.addSteinerPointsToMST(mstEdges);
        mstEdges = steiner.postProcess(mstEdges);

        // Call the new post-processing function
        steiner.postProcessAndCheckForViolations(mstEdges);

        steiner.writeOutputToFile("steiner_tree_output.txt", mstEdges);
        std::cout << "Output written to file: steiner_tree_output.txt\n";

        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
