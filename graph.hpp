
#pragma once
#include <vector>
#include <limits>
#include <cstddef>
#include <cmath>

struct Edge {
    int to;
    float w; // weight = travel time (distance)
};

struct Node {
    float x, y; // for drawing
};

struct Graph {
    std::vector<Node> nodes;
    std::vector<std::vector<Edge>> adj;

    explicit Graph(size_t n = 0) : nodes(n), adj(n) {}

    void add_edge(int u, int v, float w) {
        if (u < 0 || v < 0 || u >= (int)adj.size() || v >= (int)adj.size()) return;
        adj[u].push_back({v, w});
        adj[v].push_back({u, w}); // undirected road
    }

    static float dist(const Node& a, const Node& b) {
        float dx = a.x - b.x, dy = a.y - b.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};
