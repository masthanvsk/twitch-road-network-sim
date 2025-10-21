#include "simulation.hpp"
#include <limits>
#include <cmath>

static std::vector<int> reconstruct_path(int src, int dst, const std::vector<int>& parent) {
    std::vector<int> p;
    for (int v = dst; v != -1; v = parent[v]) p.push_back(v);
    std::reverse(p.begin(), p.end());
    if (p.empty() || p.front() != src) return {};
    return p;
}

void Simulation::build_demo_graph() {
    g = Graph(12);
    g.nodes[0] = {-0.8f, -0.5f};
    g.nodes[1] = {-0.4f, -0.5f};
    g.nodes[2] = { 0.0f, -0.5f};
    g.nodes[3] = { 0.4f, -0.5f};
    g.nodes[4] = { 0.8f, -0.5f};

    g.nodes[5] = {-0.8f,  0.0f};
    g.nodes[6] = {-0.4f,  0.0f};
    g.nodes[7] = { 0.0f,  0.0f};
    g.nodes[8] = { 0.4f,  0.0f};
    g.nodes[9] = { 0.8f,  0.0f};

    g.nodes[10] = {-0.4f,  0.5f};
    g.nodes[11] = { 0.4f,  0.5f};

    auto add = [&](int a, int b) {
        float d = Graph::dist(g.nodes[a], g.nodes[b]);
        g.add_edge(a,b, d);
    };

    // horizontal
    add(0,1); add(1,2); add(2,3); add(3,4);
    add(5,6); add(6,7); add(7,8); add(8,9);

    // vertical connectors
    add(1,6); add(2,7); add(3,8);

    // diagonals/top
    add(6,10); add(8,11); add(10,11);

    // mid cross
    add(6,7); add(7,8);
}

void Simulation::spawn_cars(int n) {
    std::uniform_int_distribution<int> node_dist(0, (int)g.nodes.size()-1);
    std::uniform_real_distribution<float> speed_dist(0.35f, 0.65f);

    auto weight_factor = [&](int u, int v) {
        return cfg.traffic_bias * edge_incident_multiplier(u,v);
    };

    for (int i=0; i<n; ++i) {
        int s = node_dist(rng), d = node_dist(rng);
        while (d==s) d = node_dist(rng);

        auto dj = dijkstra(g, s, weight_factor);
        auto path = reconstruct_path(s, d, dj.parent);
        if (path.size() < 2) { --i; continue; }

        Car c;
        c.path = std::move(path);
        c.segment_idx = 0;
        c.t_along = 0.f;
        c.base_speed = speed_dist(rng);
        c.speed = c.base_speed;
        c.src = s; c.dst = d;
        cars.push_back(std::move(c));
    }
}

void Simulation::recompute_routes() {
    auto weight_factor = [&](int u, int v) {
        return cfg.traffic_bias * edge_incident_multiplier(u,v);
    };
    for (auto& c : cars) {
        auto dj = dijkstra(g, c.src, weight_factor);
        auto path = reconstruct_path(c.src, c.dst, dj.parent);
        if (path.size() >= 2) {
            c.path = std::move(path);
            c.segment_idx = 0;
            c.t_along = 0.f;
        }
    }
}

int Simulation::edge_density(int u, int v) const {
    int a = std::min(u,v), b = std::max(u,v);
    int count = 0;
    for (auto& c : cars) {
        if (!c.alive || c.segment_idx >= (int)c.path.size()-1) continue;
        int cu = c.path[c.segment_idx], cv = c.path[c.segment_idx+1];
        int aa = std::min(cu,cv), bb = std::max(cu,cv);
        if (aa==a && bb==b) ++count;
    }
    return count;
}

float Simulation::edge_incident_multiplier(int u, int v) const {
    int a = std::min(u,v), b = std::max(u,v);
    float mult = 1.0f;
    for (const auto& inc : incidents) {
        if (inc.t_remaining > 0.f) {
            int ia = std::min(inc.a, inc.b), ib = std::max(inc.a, inc.b);
            if (ia==a && ib==b) mult = std::max(mult, inc.mult);
        }
    }
    return mult;
}

void Simulation::add_random_incident() {
    std::vector<std::pair<int,int>> edges;
    for (int u=0; u<(int)g.adj.size(); ++u) {
        for (const auto& e : g.adj[u]) if (u < e.to) edges.push_back({u, e.to});
    }
    if (edges.empty()) return;

    std::uniform_int_distribution<size_t> pick(0, edges.size()-1);
    auto [a,b] = edges[pick(rng)];

    Incident inc;
    inc.a = a; inc.b = b;
    inc.mult = cfg.incident_multiplier;
    inc.t_remaining = cfg.incident_duration;
    incidents.push_back(inc);
}

void Simulation::decay_incidents(float dt) {
    for (auto& inc : incidents) {
        if (inc.t_remaining > 0.f) inc.t_remaining -= dt;
    }
    incidents.erase(std::remove_if(incidents.begin(), incidents.end(),
        [](const Incident& i){ return i.t_remaining <= 0.f; }), incidents.end());
}

void Simulation::step() {
    // timers
    t_accum_reroute   += cfg.dt;
    t_accum_incident  += cfg.dt;

    // auto incidents
    if (t_accum_incident >= cfg.incident_interval) {
        t_accum_incident = 0.f;
        add_random_incident();
    }

    // adaptive speeds & collision avoidance
    for (size_t i=0; i<cars.size(); ++i) {
        auto& c = cars[i];
        if (!c.alive || c.segment_idx >= (int)c.path.size()-1) { c.alive=false; continue; }

        int u = c.path[c.segment_idx];
        int v = c.path[c.segment_idx+1];

        int density = edge_density(u, v);
        float slow_factor = 1.0f / (1.0f + 0.15f * (density - 1));
        c.speed = std::max(0.15f, c.base_speed * slow_factor);

        float inc_mult = edge_incident_multiplier(u,v);
        c.speed = c.speed / std::max(1.0f, inc_mult - 0.5f);

        float my_pos = c.t_along;
        for (size_t j=0; j<cars.size(); ++j) if (j!=i) {
            auto& o = cars[j];
            if (!o.alive) continue;
            if (o.segment_idx == c.segment_idx && o.path == c.path) {
                float gap = (o.t_along - my_pos);
                if (gap > 0.f && gap < c.min_gap) c.speed = std::min(c.speed, 0.1f);
            }
        }

        c.t_along += c.speed * cfg.dt;

        if (c.t_along >= 1.0f) {
            c.t_along = 0.0f;
            c.segment_idx++;
            if (c.segment_idx >= (int)c.path.size()-1) c.alive = false;
        }
    }

    int dead = 0;
    for (auto& c: cars) if(!c.alive) dead++;
    if (dead > 0) {
        cars.erase(std::remove_if(cars.begin(), cars.end(), [](const Car& c){return !c.alive;}), cars.end());
        spawn_cars(dead);
    }

    if (t_accum_reroute >= cfg.reroute_interval) {
        t_accum_reroute = 0.f;
        recompute_routes();
    }

    decay_incidents(cfg.dt);
}
