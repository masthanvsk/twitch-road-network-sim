#pragma once
#include "graph.hpp"
#include <queue>
#include <functional>
#include <utility>

struct DijkstraResult {
    std::vector<float> dist;
    std::vector<int> parent;
};

// weight_factor(u,v) returns a multiplier for edge weight (>=1.0 typical)
inline DijkstraResult dijkstra(const Graph& g, int src,
                               std::function<float(int,int)> weight_factor = nullptr) {
    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> d(g.adj.size(), INF);
    std::vector<int> parent(g.adj.size(), -1);

    using P = std::pair<float,int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    d[src] = 0.f;
    pq.push({0.f, src});

    while(!pq.empty()) {
        auto [du,u] = pq.top(); pq.pop();
        if (du > d[u]) continue;
        for (const auto& e : g.adj[u]) {
            float mult = 1.0f;
            if (weight_factor) mult = weight_factor(u, e.to);
            float w = e.w * mult;
            if (d[e.to] > d[u] + w) {
                d[e.to] = d[u] + w;
                parent[e.to] = u;
                pq.push({d[e.to], e.to});
            }
        }
    }
    return {std::move(d), std::move(parent)};
}
