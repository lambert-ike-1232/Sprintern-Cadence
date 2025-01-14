
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#define DEBUG_MODE true

struct Point {
    int id;
    double x, y;
    bool sink;
};

struct Edge {
    int node1, node2;
    double weight;
};

struct Block {
    int id;
    double x1, y1, x2, y2;
};

bool operator==(const Point& p1, const Point& p2) {
    return p1.id == p2.id && p1.x == p2.x && p1.y == p2.y && p1.sink == p2.sink;
}

bool operator==(const Edge& e1, const Edge& e2) {
    return e1.node1 == e2.node1 && e1.node2 == e2.node2 && e1.weight == e2.weight;
}

class UnionFind {
public:
    UnionFind(int n) : parent(n), rank(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }

    int find(int u) {
        if (u < 0 || u >= parent.size()) throw std::out_of_range("Index out of range");
        if (parent[u] != u) parent[u] = find(parent[u]);
        return parent[u];
    }

    void unite(int u, int v) {
        int rootU = find(u), rootV = find(v);
        if (rootU != rootV) {
            if (rank[rootU] > rank[rootV]) parent[rootV] = rootU;
            else if (rank[rootU] < rank[rootV]) parent[rootU] = rootV;
            else parent[rootV] = rootU, rank[rootU]++;
        }
    }

private:
    std::vector<int> parent, rank;
};

class SteinerTree {
private:
    std::string filePath;
    std::vector<Point> points;
    std::vector<Edge> edges;
    std::vector<Block> blockages;
    int numberOfPoints, originalSinkCount, numberOfBlocks, nextPointId = 30000;
    double threshold = 100.0;

public:
    SteinerTree(const std::string& path) : filePath(path), numberOfPoints(0), originalSinkCount(0) {
        readInputFile();
        originalSinkCount = numberOfPoints;
        generateSpanningGraphOptimized();
    }

    const std::vector<Point>& getPoints() const { return points; }
    const std::vector<Edge>& getEdges() const { return edges; }
    const std::vector<Block>& getBlockages() const { return blockages; }

    bool readInputFile() {
        std::ifstream inputFile(filePath);
        if (!inputFile.is_open()) throw std::runtime_error("Error opening file: " + filePath);

        std::string line;
        while (std::getline(inputFile, line)) {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;

            if (keyword == "number_of_sinks") iss >> numberOfPoints;
            else if (keyword == "sink") {
                Point sink;
                iss >> sink.id >> sink.x >> sink.y;
                sink.sink = true;
                points.push_back(sink);
                if (DEBUG_MODE) std::cout << "Reading sink: ID=" << sink.id << " x=" << sink.x << " y=" << sink.y << "\n";
            } else if (keyword == "number_of_blockages") iss >> numberOfBlocks;
            else if (keyword == "blockage") {
                Block block;
                iss >> block.id >> block.x1 >> block.y1 >> block.x2 >> block.y2;
                blockages.push_back(block);
                if (DEBUG_MODE) std::cout << "Reading blockage: ID=" << block.id << " x1=" << block.x1 << " y1=" << block.y1 << " x2=" << block.x2 << " y2=" << block.y2 << "\n";
            }
        }
        inputFile.close();
        return true;
    }

    void printSinks() {
        std::cout << "Number of sinks: " << numberOfPoints << std::endl;
        for (const auto& p : points) if (p.sink) std::cout << "Sink ID: " << p.id << ", X: " << p.x << ", Y: " << p.y << std::endl;
    }

    void displayBlockages() const {
        std::cout << "Blockages:\n";
        for (const auto& block : blockages) std::cout << "Blockage ID: " << block.id << ", Point 1: (" << block.x1 << ", " << block.y1 << "), Point 2: (" << block.x2 << ", " << block.y2 << ")\n";
    }

    double mhDistance(const Point& a, const Point& b) {
        return std::fabs(a.x - b.x) + std::fabs(a.y - b.y);
    }

    std::vector<Edge> constructMST() {
        std::vector<Edge> mstEdges;
        std::sort(edges.begin(), edges.end(), [](const Edge& e1, const Edge& e2) { return e1.weight < e2.weight; });

        UnionFind uf(numberOfPoints);
        for (const auto& edge : edges) {
            int u = edge.node1, v = edge.node2;
            if (uf.find(u) != uf.find(v)) {
                mstEdges.push_back(edge);
                uf.unite(u, v);
                if (mstEdges.size() == points.size() - 1) break;
            }
        }
        return mstEdges;
    }

    std::vector<double> getUniqueCoords(const std::vector<Point>& pts, bool isX) {
        std::set<double> coords;
        for (auto& p : pts) coords.insert(isX ? p.x : p.y);
        return std::vector<double>(coords.begin(), coords.end());
    }

    std::vector<Point> getHananGridPoints(const std::vector<Point>& pts) {
        auto xVals = getUniqueCoords(pts, true), yVals = getUniqueCoords(pts, false);
        std::set<double> xVals_blockage, yVals_blockage;
        for (const auto& b : blockages) {
            xVals_blockage.insert(b.x1);
            xVals_blockage.insert(b.x2);
            yVals_blockage.insert(b.y1);
            yVals_blockage.insert(b.y2);
        }
        for (const auto& x : xVals_blockage) xVals.push_back(x);
        for (const auto& y : yVals_blockage) yVals.push_back(y);

        std::vector<Point> candidates;
        int startId = 1000;
        for (double x : xVals) {
            for (double y : yVals) {
                bool alreadyExists = false;
                for (auto& p : pts) if (std::fabs(p.x - x) < 1e-9 && std::fabs(p.y - y) < 1e-9) { alreadyExists = true; break; }
                if (!alreadyExists) candidates.push_back({startId++, x, y, false});
            }
        }
        return getUnblockedPoints(candidates);
    }

    bool isBlocked(Point pt) {
        for (const auto& block : blockages) if (block.x1 < pt.x && pt.x < block.x2 && block.y1 < pt.y && pt.y < block.y2) return true;
        return false;
    }

    std::vector<Point> getUnblockedPoints(const std::vector<Point>& pts) {
        std::vector<Point> unblockedPts;
        for (const auto& pt : pts) if (!isBlocked(pt)) unblockedPts.push_back(pt);
        return unblockedPts;
    }

    int orientation(const Point& p, const Point& q, const Point& r) {
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        return (val == 0) ? 0 : (val > 0) ? 1 : 2;
    }

    bool onSegment(const Point& p, const Point& q, const Point& r) {
        return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
    }

    bool doIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
        int o1 = orientation(p1, q1, p2), o2 = orientation(p1, q1, q2), o3 = orientation(p2, q2, p1), o4 = orientation(p2, q2, q1);
        return (o1 != o2 && o3 != o4) || (o1 == 0 && onSegment(p1, p2, q1)) || (o2 == 0 && onSegment(p1, q2, q1)) || (o3 == 0 && onSegment(p2, p1, q2)) || (o4 == 0 && onSegment(p2, q1, q2));
    }

    const Point* findPointByID_list(int id, std::vector<Point> pt_list) {
        for (auto& p : pt_list) if (p.id == id) return &p;
        return nullptr;
    }

    bool isEdgeBlocked(Edge edge, std::vector<Point> tempPoints) {
        const Point* node1Ptr = findPointByID_list(edge.node1, tempPoints);
        const Point* node2Ptr = findPointByID_list(edge.node2, tempPoints);
        if (!node1Ptr || !node2Ptr) return true;

        Point node1 = *node1Ptr, node2 = *node2Ptr;
        if (isBlocked(node1) || isBlocked(node2)) return true;

        if (std::fabs(node1.y - node2.y) < 1e-9) {
            double minX = std::min(node1.x, node2.x), maxX = std::max(node1.x, node2.x), y = node1.y;
            for (const auto& block : blockages) if (y >= block.y1 && y <= block.y2 && maxX >= block.x1 && minX <= block.x2) return true;
        }

        if (std::fabs(node1.x - node2.x) < 1e-9) {
            double minY = std::min(node1.y, node2.y), maxY = std::max(node1.y, node2.y), x = node1.x;
            for (const auto& block : blockages) if (x >= block.x1 && x <= block.x2 && maxY >= block.y1 && minY <= block.y2) return true;
        }

        for (const auto& block : blockages) if (doIntersectWithBlockage(node1, node2, block)) return true;
        return false;
    }

    double getNumberOfEdgeBlockages(std::vector<Edge> MST, std::vector<Point> tempPoints) {
        double numberBlocks = 0;
        for (const auto& edge : MST) if (isEdgeBlocked(edge, tempPoints)) numberBlocks += 1;
        return numberBlocks;
    }

    double getBlockageCost(const std::vector<Edge>& candidateMST) {
        double cost = 0;
        for (const auto& edge : candidateMST) if (isEdgeBlocked(edge, points)) cost += edge.weight;
        return cost;
    }

    bool isPointOnBlockageEdge(Point p) {
        for (const auto& bloc : blockages) {
            double x_min = std::min(bloc.x1, bloc.x2), x_max = std::max(bloc.x1, bloc.x2), y_min = std::min(bloc.y1, bloc.y2), y_max = std::max(bloc.y1, bloc.y2);
            if ((p.x == x_min || p.x == x_max) && (p.y >= y_min && p.y <= y_max)) return true;
            if ((p.y == y_min || p.y == y_max) && (p.x >= x_min && p.x <= x_max)) return true;
        }
        return false;
    }

    std::vector<Point> findPathAroundBlockage(Point edgePt, Block block) {
        double x_min = std::min(block.x1, block.x2), x_max = std::max(block.x1, block.x2), y_min = std::min(block.y1, block.y2), y_max = std::max(block.y1, block.y2);
        std::vector<Point> path;
        if (edgePt.x == x_min) path = {{getUniquePointId(), x_min, edgePt.y}, {getUniquePointId(), x_min, y_min}, {getUniquePointId(), x_max, y_min}, {getUniquePointId(), x_max, edgePt.y}};
        else if (edgePt.x == x_max) path = {{getUniquePointId(), x_max, edgePt.y}, {getUniquePointId(), x_max, y_max}, {getUniquePointId(), x_min, y_max}, {getUniquePointId(), x_min, edgePt.y}};
        else if (edgePt.y == y_min) path = {{getUniquePointId(), edgePt.x, y_min}, {getUniquePointId(), x_max, y_min}, {getUniquePointId(), x_max, y_max}, {getUniquePointId(), edgePt.x, y_max}};
        else if (edgePt.y == y_max) path = {{getUniquePointId(), edgePt.x, y_max}, {getUniquePointId(), x_min, y_max}, {getUniquePointId(), x_min, y_min}, {getUniquePointId(), edgePt.x, y_min}};
        if (DEBUG_MODE) std::cout << "Finding path around blockage: x_min=" << x_min << " x_max=" << x_max << " y_min=" << y_min << " y_max=" << y_max << "\n";
        return path;
    }

    Block getBlockagebyPoint(Point pt) {
        for (const auto& b : blockages) if (b.x1 <= pt.x && pt.x <= b.x2 && b.y1 <= pt.y && pt.y <= b.y2) return b;
        return {};
    }

    int getUniquePointId() { return nextPointId++; }

    bool doIntersectWithBlockage(const Point& p1, const Point& p2, const Block& block) {
        Point bl{0, block.x1, block.y1}, br{0, block.x2, block.y1}, tr{0, block.x2, block.y2}, tl{0, block.x1, block.y2};
        return doIntersect(p1, p2, bl, br) || doIntersect(p1, p2, br, tr) || doIntersect(p1, p2, tr, tl) || doIntersect(p1, p2, tl, bl);
    }

    bool hasBlockageIntersections(const std::vector<Point>& path) {
        for (size_t i = 0; i < path.size() - 1; i++) if (isEdgeBlocked({path[i].id, path[i + 1].id, 0}, points)) return true;
        return false;
    }

    void addSteinerPoints() {
        edges = constructMST();
        std::vector<Edge> finalEdges;
        for (const auto& edge : edges) {
            const Point* p1 = findPointByID(edge.node1);
            const Point* p2 = findPointByID(edge.node2);
            if (!p1 || !p2) continue;
            if ((std::fabs(p1->x - p2->x) < 1e-9 || std::fabs(p1->y - p2->y) < 1e-9) && !isEdgeBlocked(edge, points)) {
                finalEdges.push_back(edge);
                continue;
            }
            bool pathFound = false;
            double bestLength = std::numeric_limits<double>::infinity();
            std::vector<Edge> bestPath;
            std::vector<Point> bestSteinerPoints;
            Point steiner1 = {getUniquePointId(), p1->x, p2->y, false};
            Edge e1 = {p1->id, steiner1.id, mhDistance(*p1, steiner1)}, e2 = {steiner1.id, p2->id, mhDistance(steiner1, *p2)};
            if (!isEdgeBlocked(e1, points) && !isEdgeBlocked(e2, points)) {
                double length = e1.weight + e2.weight;
                if (length < bestLength) bestLength = length, bestPath = {e1, e2}, bestSteinerPoints = {steiner1}, pathFound = true;
            }
            Point steiner2 = {getUniquePointId(), p2->x, p1->y, false};
            Edge e3 = {p1->id, steiner2.id, mhDistance(*p1, steiner2)}, e4 = {steiner2.id, p2->id, mhDistance(steiner2, *p2)};
            if (!isEdgeBlocked(e3, points) && !isEdgeBlocked(e4, points)) {
                double length = e3.weight + e4.weight;
                if (length < bestLength) bestLength = length, bestPath = {e3, e4}, bestSteinerPoints = {steiner2}, pathFound = true;
            }
            if (pathFound) {
                for (const auto& sp : bestSteinerPoints) points.push_back(sp);
                for (const auto& e : bestPath) finalEdges.push_back(e);
            } else finalEdges.push_back(edge);
        }
        edges = finalEdges;
        if (DEBUG_MODE) std::cout << "Final number of edges: " << edges.size() << "\n", printEdges(edges);
    }

    void postProcess() {
        std::vector<Edge> updatedEdges;
        std::set<std::pair<int, int>> processedEdges;
        for (const auto& e : edges) {
            auto edgeId = std::make_pair(std::min(e.node1, e.node2), std::max(e.node1, e.node2));
            if (processedEdges.count(edgeId) > 0) continue;
            processedEdges.insert(edgeId);
            const Point* p1 = findPointByID(e.node1);
            const Point* p2 = findPointByID(e.node2);
            if (!p1 || !p2) continue;
            if (std::fabs(p1->x - p2->x) < 1e-9 || std::fabs(p1->y - p2->y) < 1e-9) {
                updatedEdges.push_back(e);
            } else {
                Point intermediatePt = isBlocked({0, p1->x, p2->y}) ? Point{0, p2->x, p1->y} : Point{0, p1->x, p2->y};
                points.push_back(intermediatePt);
                Edge newEdge1 = {e.node1, intermediatePt.id, mhDistance(*p1, intermediatePt)};
                Edge newEdge2 = {intermediatePt.id, e.node2, mhDistance(intermediatePt, *p2)};
                if (!isEdgeBlocked(newEdge1, points) && !isEdgeBlocked(newEdge2, points)) {
                    updatedEdges.push_back(newEdge1);
                    updatedEdges.push_back(newEdge2);
                } else {
                    updatedEdges.push_back(e);
                }
            }
        }
        edges = std::move(updatedEdges);
    }

    const Point* findPointByID(int id) const {
        for (const auto& p : points) {
            if (p.id == id) return &p;
        }
        return nullptr;
    }

    void writeOutputToFile(const std::string& outFile, const std::vector<Edge>& mstEdges) {
        std::ofstream ofs(outFile);
        if (!ofs.is_open()) throw std::runtime_error("Error opening output file: " + outFile);
        ofs << "number_of_sinks " << originalSinkCount << "\n\n";
        bool onceSpace = true;
        for (auto& p : points) {
            if (p.sink) {
                ofs << "sink " << p.id << " " << p.x << " " << p.y << "\n";
            } else {
                if (onceSpace) {
                    ofs << "\n";
                    onceSpace = false;
                }
                ofs << "point " << p.id << " " << p.x << " " << p.y << "\n";
            }
        }
        ofs << "\nnumber_of_blockages " << blockages.size() << "\n";
        ofs << std::fixed << std::setprecision(1);
        for (const auto& block : blockages) {
            ofs << "blockage " << block.id << " " << block.x1 << " " << block.y1 << " " << block.x2 << " " << block.y2 << "\n";
        }
        ofs << "\n";
        double totalLen = 0.0;
        for (auto& e : mstEdges) {
            const Point* p1 = findPointByID(e.node1);
            const Point* p2 = findPointByID(e.node2);
            if (!p1 || !p2) continue;
            ofs << "edge " << e.node1 << " " << e.node2 << "\n";
            totalLen += e.weight;
        }
        ofs << "\nPath length = " << totalLen << "\n";
        ofs.close();
    }

    void outputRST(const std::string& outFile) {
        std::ofstream ofs(outFile);
        if (!ofs.is_open()) throw std::runtime_error("Error opening output file: " + outFile);
        ofs << "number_of_sinks " << originalSinkCount << "\n\n";
        bool onceSpace = true;
        for (const auto& p : points) {
            if (p.sink) {
                ofs << "sink " << p.id << " " << p.x << " " << p.y << "\n";
            } else {
                if (onceSpace) {
                    ofs << "\n";
                    onceSpace = false;
                }
                ofs << "point " << p.id << " " << p.x << " " << p.y << "\n";
            }
        }
        ofs << "\nnumber_of_blockages " << blockages.size() << "\n";
        ofs << std::fixed << std::setprecision(1);
        for (const auto& block : blockages) {
            ofs << "blockage " << block.id << " " << block.x1 << " " << block.y1 << " " << block.x2 << " " << block.y2 << "\n";
        }
        ofs << "\n";
        double totalLen = 0.0;
        for (const auto& e : edges) {
            const Point* p1 = findPointByID(e.node1);
            const Point* p2 = findPointByID(e.node2);
            if (!p1 || !p2) continue;
            ofs << "edge " << e.node1 << " " << e.node2 << "\n";
            totalLen += e.weight;
        }
        ofs << "\nPath length = " << totalLen << "\n";
        ofs.close();
    }

    void printEdges(const std::vector<Edge>& edgeList) {
        std::cout << "Total number of edges: " << edgeList.size() << "\n";
        for (auto& e : edgeList) {
            Point p1 = *findPointByID(e.node1);
            Point p2 = *findPointByID(e.node2);
            std::cout << "Edge between " << e.node1 << " (" << p1.x << ", " << p1.y << ") and " << e.node2 << " (" << p2.x << ", " << p2.y << ") has Manhattan distance: " << e.weight << "\n";
        }
    }

    void generateSpanningGraphOptimized() {
        edges.clear();
        std::vector<Point> sortedPoints = points;
        std::sort(sortedPoints.begin(), sortedPoints.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
        std::set<Point, bool(*)(const Point&, const Point&)> activePoints([](const Point& a, const Point& b) { return a.y < b.y; });
        for (const auto& p : sortedPoints) {
            auto it = activePoints.begin();
            while (it != activePoints.end() && p.x - it->x > threshold) it = activePoints.erase(it);
            for (const auto& q : activePoints) {
                if (std::abs(p.y - q.y) <= threshold) edges.push_back({p.id, q.id, mhDistance(p, q)});
            }
            activePoints.insert(p);
        }
    }

    void dynamicProgrammingEdgeFlipping() {
        std::vector<Edge> updatedEdges;
        double totalWeight = 0.0;
        for (const auto& edge : edges) {
            const Point* p1 = findPointByID(edge.node1);
            const Point* p2 = findPointByID(edge.node2);
            if (!p1 || !p2) continue;
            if (std::fabs(p1->x - p2->x) < 1e-9 || std::fabs(p1->y - p2->y) < 1e-9) {
                updatedEdges.push_back(edge);
                totalWeight += edge.weight;
                continue;
            }
            Point steinerPoint = {getUniquePointId(), p1->x, p2->y, false};
            if (isBlocked(steinerPoint)) steinerPoint = {getUniquePointId(), p2->x, p1->y, false};
            points.push_back(steinerPoint);
            Edge newEdge1 = {edge.node1, steinerPoint.id, mhDistance(*p1, steinerPoint)};
            Edge newEdge2 = {steinerPoint.id, edge.node2, mhDistance(steinerPoint, *p2)};
            updatedEdges.push_back(newEdge1);
            updatedEdges.push_back(newEdge2);
            totalWeight += newEdge1.weight + newEdge2.weight;
        }
        edges = std::move(updatedEdges);
        if (DEBUG_MODE) {
            std::cout << "Total weight after edge flipping: " << totalWeight << "\n";
            printEdges(edges);
        }
    }
};

void reorderOutputFile(const std::string &filename) {
    std::ifstream in(filename);
    if (!in) return;
    std::string line, numberOfSinks, numberOfBlockages, pathLength;
    std::vector<std::string> sinks, points, edges, blockages;
    while (std::getline(in, line)) {
        if (line.rfind("number_of_sinks", 0) == 0) numberOfSinks = line;
        else if (line.rfind("number_of_blockages", 0) == 0) numberOfBlockages = line;
        else if (line.rfind("sink", 0) == 0) sinks.push_back(line);
        else if (line.rfind("blockage", 0) == 0) blockages.push_back(line);
        else if (line.rfind("point", 0) == 0) points.push_back(line);
        else if (line.rfind("edge", 0) == 0) edges.push_back(line);
        else if (line.rfind("Path length", 0) == 0) pathLength = line;
    }
    in.close();
    std::ofstream out(filename);
    if (!out) return;
    out << numberOfSinks << "\n\n";
    for (auto &s : sinks) out << s << "\n";
    out << "\n" << numberOfBlockages << "\n";
    for (auto &b : blockages) out << b << "\n";
    out << "\n";
    for (auto &p : points) out << p << "\n";
    out << "\n";
    for (auto &e : edges) out << e << "\n";
    out << "\n" << pathLength << std::endl;
}

// ================= MAIN =================

int main() {
    try {
        SteinerTree steiner("data_b/r31b.in");
        steiner.printSinks();
        steiner.displayBlockages();
        auto initMST = steiner.constructMST();
        double initMSTLen = 0.0;
        for (auto& e : initMST) initMSTLen += e.weight;
        std::cout << "\nInitial MST with only sinks:\n";
        steiner.printEdges(initMST);
        std::cout << "Initial MST length = " << initMSTLen << "\n\n";
        std::cout << "Adding Steiner Points using Hanan Grid...\n";
        steiner.addSteinerPoints();
        steiner.postProcess();
        std::cout << "Performing Dynamic Programming-Based Edge Flipping...\n";
        steiner.dynamicProgrammingEdgeFlipping();
        auto finalEdges = steiner.getEdges();
        double finalMSTLen = 0.0;
        for (auto& e : finalEdges) finalMSTLen += e.weight;
        std::cout << "\nFinal MST Edges with Steiner Points:\n";
        steiner.printEdges(finalEdges);
        std::cout << "Final MST length = " << finalMSTLen << "\n\n";
        steiner.writeOutputToFile("steiner_tree_output.txt", finalEdges);
        std::cout << "Output written to file: output/steiner_tree_output.txt\n";
        std::cout << "Outputting the RST...\n";
        steiner.outputRST("rst_output.txt");
        std::cout << "RST output written to file: rst_output.txt\n";
        reorderOutputFile("steiner_tree_output.txt");
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}