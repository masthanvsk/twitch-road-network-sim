#include <GLFW/glfw3.h>
#include <cstdio>
#include <algorithm>
#include <cmath>

#include "simulation.hpp"

static void drawCircle(float x, float y, float r) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x,y);
    const int N=24;
    for(int i=0;i<=N;i++){
        float a = (float)i/N * 2.f * 3.1415926f;
        glVertex2f(x + r*std::cos(a), y + r*std::sin(a));
    }
    glEnd();
}

static void drawLine(float x1,float y1,float x2,float y2,float w){
    glLineWidth(w);
    glBegin(GL_LINES);
    glVertex2f(x1,y1); glVertex2f(x2,y2);
    glEnd();
}

static void setHeatColor(int den, int maxDen, bool incident) {
    if (incident) { glColor3f(1.0f, 0.55f, 0.1f); return; }
    float t = (maxDen > 0) ? (float)den / (float)maxDen : 0.f;
    float r = 0.4f + 0.6f*t;
    float g = 0.4f * (1.0f - t) + 0.2f;
    float b = 0.5f * (1.0f - t) + 0.2f;
    glColor3f(r,g,b);
}

int main() {
    if(!glfwInit()){
        std::fprintf(stderr,"Failed to init GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    GLFWwindow* win = glfwCreateWindow(1100, 750, "Alternative Routes — Dijkstra / Incidents / Heatmap / Re-routing", nullptr, nullptr);
    if(!win){ std::fprintf(stderr,"Failed to create window\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    Simulation sim;
    sim.cfg.car_count = 24;
    sim.cfg.dt = 1.f/60.f;
    sim.cfg.traffic_bias = 1.0f;
    sim.cfg.reroute_interval = 3.0f;
    sim.cfg.incident_interval = 8.0f;
    sim.cfg.incident_duration = 10.0f;
    sim.cfg.incident_multiplier = 2.5f;

    sim.build_demo_graph();
    sim.spawn_cars(sim.cfg.car_count);

    bool paused = false;
    bool show_path_hint = false;

    while(!glfwWindowShouldClose(win)){
        glfwPollEvents();

        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(win, 1);

        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS) paused = true;
        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_RELEASE) paused = false;

        if (glfwGetKey(win, GLFW_KEY_R) == GLFW_PRESS) sim.recompute_routes();
        if (glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS) show_path_hint = true;
        if (glfwGetKey(win, GLFW_KEY_P) == GLFW_RELEASE) show_path_hint = false;

        if (glfwGetKey(win, GLFW_KEY_KP_ADD) == GLFW_PRESS || glfwGetKey(win, GLFW_KEY_EQUAL) == GLFW_PRESS) sim.spawn_cars(1);
        if (glfwGetKey(win, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS || glfwGetKey(win, GLFW_KEY_MINUS) == GLFW_PRESS) {
            if(!sim.cars.empty()) sim.cars.pop_back();
        }

        if (glfwGetKey(win, GLFW_KEY_I) == GLFW_PRESS) sim.add_random_incident();

        if(!paused) sim.step();

        int maxDen = 1;
        for (int u=0; u<(int)sim.g.adj.size(); ++u)
            for (auto& e: sim.g.adj[u]) if (u < e.to)
                maxDen = std::max(maxDen, sim.edge_density(u,e.to));

        int w,h; glfwGetFramebufferSize(win,&w,&h);
        glViewport(0,0,w,h);
        glClearColor(0.08f,0.09f,0.12f,1);
        glClear(GL_COLOR_BUFFER_BIT);

        // edges with heat/incident coloring
        for (int u=0; u<(int)sim.g.adj.size(); ++u) {
            for (auto& e: sim.g.adj[u]) if (u < e.to) {
                auto a = sim.g.nodes[u];
                auto b = sim.g.nodes[e.to];
                int den = sim.edge_density(u, e.to);
                bool incident = (sim.edge_incident_multiplier(u,e.to) > 1.0001f);
                setHeatColor(den, maxDen, incident);
                drawLine(a.x, a.y, b.x, b.y, incident ? 5.0f : 3.0f);
            }
        }

        // nodes
        glColor3f(0.92f,0.93f,0.97f);
        for (auto& n : sim.g.nodes) drawCircle(n.x,n.y, 0.015f);

        // optional path highlight
        if (show_path_hint && !sim.cars.empty()) {
            const auto& p = sim.cars.front().path;
            glColor3f(0.2f,0.85f,0.35f);
            for (size_t i=1;i<p.size();++i){
                auto a = sim.g.nodes[p[i-1]];
                auto b = sim.g.nodes[p[i]];
                drawLine(a.x, a.y, b.x, b.y, 6.0f);
            }
        }

        // cars (color by speed)
        for (const auto& c : sim.cars) {
            if (!c.alive || c.segment_idx >= (int)c.path.size()-1) continue;
            auto A = sim.g.nodes[c.path[c.segment_idx]];
            auto B = sim.g.nodes[c.path[c.segment_idx+1]];
            float x = A.x + (B.x - A.x) * c.t_along;
            float y = A.y + (B.y - A.y) * c.t_along;

            float v = std::clamp((c.speed - 0.15f) / 0.5f, 0.0f, 1.0f);
            glColor3f(1.0f - v, 0.25f + 0.6f*v, 0.25f);
            drawCircle(x,y, 0.02f);
        }

        glfwSwapBuffers(win);
    }

    glfwTerminate();
    return 0;
}
