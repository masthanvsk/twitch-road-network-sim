#pragma once
#include "graph.hpp"
#include "dijkstra.hpp"
#include <vector>
#include <random>
#include <algorithm>

struct Car {
    std::vector<int> path;
    int segment_idx = 0;
    float t_along = 0.0f;
    float speed = 0.3f;
    float base_speed = 0.5f;
    float min_gap = 0.025f;
    int src = -1, dst = -1;
    bool alive = true;
};

struct Incident {
    int a=-1, b=-1;        // edge (unordered pair)
    float mult=1.0f;       // weight multiplier
    float t_remaining=0.f; // seconds
};

struct SimConfig {
    int car_count = 15;
    float dt = 0.016f;            // ~60 FPS
    float traffic_bias = 1.0f;    // global bias
    float reroute_interval = 3.0f;
    float incident_interval = 8.0f;
    float incident_duration = 10.0f;
    float incident_multiplier = 2.5f;
};

struct Simulation {
    Graph g;
    std::vector<Car> cars;
    SimConfig cfg;
    std::mt19937 rng{1337};
    std::vector<Incident> incidents;

    float t_accum_reroute = 0.f;
    float t_accum_incident = 0.f;

    void build_demo_graph();
    void spawn_cars(int n);
    void recompute_routes();
    void step();

    // incidents
    void add_random_incident();
    void decay_incidents(float dt);

    // helpers
    int edge_density(int u, int v) const;
    float edge_incident_multiplier(int u, int v) const;
};
