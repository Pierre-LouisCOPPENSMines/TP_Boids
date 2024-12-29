//
//  main.cpp
//  tp_boids_app
//
//  Created by Pierre-Louis Coppens on 17/12/2024.
//
//compile with: g++ -o nouveau_fichier main.cpp -std=c++17 -F/Library/Frameworks -framework SDL2 -Wl,-rpath,/Library/Frameworks
//run with: ./nouveau_fichier
//install SDL2 with: brew install sdl2



#include <stdio.h>
#include <SDL2/SDL.h>
#include <stdlib.h>

#include "it_s_work.h"

#include <iostream>
#include <random>
#include <vector>
#include <cmath>

//Paramètres de la simulation
const int n_boids = 100;
const int delay = 10;
const float radius_separation = 5;
const float radius_alignment = 20;
const float f_cohesion = 0.01;
const float f_separation = 0.5;
const float f_alignment = 0.1;
const float max_speed = 2.0f;

struct global_t {
    SDL_Window * window = NULL;
    SDL_Renderer * renderer = NULL;
    
    float brightness;
    std::random_device rd;
    std::default_random_engine eng;
    std::uniform_real_distribution<float> rand;

};

global_t g;

struct boid {
    std::vector<float> position;
    std::vector<float> velocity;
    float max_speed;
    boid(float x, float y, float max_speed){
        position = {x, y};
        velocity = {0, 0};
        this->max_speed = max_speed;
    }
};

struct flock {
    std::vector<boid> boids;
    float max_speed;
    float max_force;
    flock(int n, float max_speed, float max_force){
        this->max_speed = max_speed;
        this->max_force = max_force;
        for (int i = 0; i < n; ++i){
            boids.push_back(boid(g.rand(g.eng)*800, g.rand(g.eng)*600, max_speed));
        }
    }
    void update(float f_co, float f_sep, float f_al);
};

struct rule {
    std::vector<float> acceleration = {0.0f, 0.0f};
    float factor;
    std::vector<int> get_boids_by_radius (int boid_id, flock flock, float radius) {
        std::vector<int> boids_in_radius;
        for (int i = 0; i < flock.boids.size(); ++i) {
            if (i != boid_id) {
                float distance = std::sqrt(std::pow(flock.boids[i].position[0] - flock.boids[boid_id].position[0], 2) + std::pow(flock.boids[i].position[1] - flock.boids[boid_id].position[1], 2));
                if (distance < radius) {
                    boids_in_radius.push_back(i);
                }
            }
            if (boids_in_radius.empty()) {
                float min_distance = std::numeric_limits<float>::max();
                int closest_boid = -1;
                for (int j = 0; j < flock.boids.size(); ++j) {
                if (j != boid_id) {
                    float distance = std::sqrt(std::pow(flock.boids[j].position[0] - flock.boids[boid_id].position[0], 2) + std::pow(flock.boids[j].position[1] - flock.boids[boid_id].position[1], 2));
                    if (distance < min_distance) {
                    min_distance = distance;
                    closest_boid = j;
                    }
                }
                }
                if (closest_boid != -1) {
                boids_in_radius.push_back(closest_boid);
                }
            }
        }
        return boids_in_radius;
    }
};

struct co_rule : public rule {
    co_rule (int boid_id, flock& flock) { 
        std::vector<float> mean_position = {0.0f, 0.0f};
        for (const auto& b : flock.boids) {
            mean_position[0] += b.position[0];
            mean_position[1] += b.position[1];
        }
        mean_position[0] /= flock.boids.size();
        mean_position[1] /= flock.boids.size();
        acceleration[0] = mean_position[0] - flock.boids[boid_id].position[0];
        acceleration[1] = mean_position[1] - flock.boids[boid_id].position[1];
    }
};

struct sep_rule : public rule {
    sep_rule (int boid_id, flock& flock, float radius) { 
        std::vector<float> vector_sum = {0.0f, 0.0f};
        std::vector<int> closer_boids = get_boids_by_radius(boid_id, flock, radius);
        for (const auto& b : closer_boids) {
            vector_sum[0] += flock.boids[b].position[0] - flock.boids[boid_id].position[0];
            vector_sum[1] += flock.boids[b].position[1] - flock.boids[boid_id].position[1];
        }
        if (!closer_boids.empty()) {
            float norm = std::sqrt(vector_sum[0] * vector_sum[0] + vector_sum[1] * vector_sum[1]);
            if (norm > 0) {
                acceleration[0] = -vector_sum[0] / norm;
                acceleration[1] = -vector_sum[1] / norm;
            }
        }
    }
};

struct align_rule : public rule {
    align_rule (int boid_id, flock& flock, float radius) {
        std::vector<float> mean_speed = {0.0f, 0.0f};
        std::vector<int> closer_boids = get_boids_by_radius(boid_id, flock, radius);
        for (const auto& b : closer_boids) {
            mean_speed[0] += flock.boids[b].velocity[0];
            mean_speed[1] += flock.boids[b].velocity[1];
        }
        if (!closer_boids.empty()) {
            acceleration[0] = mean_speed[0] / float(closer_boids.size());
            acceleration[1] = mean_speed[1] / float(closer_boids.size());
        }
    }
};

/*
struct barrier_rule : public rule {
    align_rule (int boid_id, flock& flock, float radius) {
        std::vector<float> mean_speed = {0.0f, 0.0f};
        std::vector<int> closer_boids = get_boids_by_radius(boid_id, flock, radius);
        for (const auto& b : closer_boids) {
            mean_speed[0] += flock.boids[b].velocity[0];
            mean_speed[1] += flock.boids[b].velocity[1];
        }
        if (!closer_boids.empty()) {
            acceleration[0] = mean_speed[0] / float(closer_boids.size());
            acceleration[1] = mean_speed[1] / float(closer_boids.size());
        }
    }
};
*/
void flock::update(float f_co, float f_sep, float f_al) {
    for (size_t i = 0; i < boids.size(); ++i) {
        co_rule cohesion_rule(i, *this);
        sep_rule separation_rule(i, *this, radius_separation);
        align_rule alignment_rule(i, *this, radius_alignment);
    
        std::vector<float> acceleration = {0.0f, 0.0f};
        acceleration[0] += cohesion_rule.acceleration[0] * f_co; 
        acceleration[1] += cohesion_rule.acceleration[1] * f_co;
         acceleration[0] += separation_rule.acceleration[0] * f_sep;
        acceleration[1] += separation_rule.acceleration[1] * f_sep;
        acceleration[0] += alignment_rule.acceleration[0] * f_al; 
        acceleration[1] += alignment_rule.acceleration[1] * f_al;
        boids[i].velocity[0] += acceleration[0];
        boids[i].velocity[1] += acceleration[1];
    }
    for (size_t i = 0; i < boids.size(); ++i) {
        float speed = std::sqrt(boids[i].velocity[0] * boids[i].velocity[0] + boids[i].velocity[1] * boids[i].velocity[1]);
        if (speed > boids[i].max_speed) {
            boids[i].velocity[0] = (boids[i].velocity[0] / speed) * boids[i].max_speed;
            boids[i].velocity[1] = (boids[i].velocity[1] / speed) * boids[i].max_speed;
        }
        boids[i].position[0] += boids[i].velocity[0];
        boids[i].position[1] += boids[i].velocity[1];
        if (boids[i].position[0] < 0) boids[i].position[0] = 0;
        else if (boids[i].position[0] > 800) boids[i].position[0] = 799;
        if (boids[i].position[1] < 0) boids[i].position[1] = 0;
        else if (boids[i].position[1] > 600) boids[i].position[1] = 599;
    }
}

void paint_boids(flock my_flock, int scale = 20) {
    for (const auto& b : my_flock.boids) {
        SDL_SetRenderDrawColor(g.renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_Rect r = { int(b.position[0]), int(b.position[1]), 5, 5 };
        SDL_RenderFillRect(g.renderer, &r);
    }
}

void do_render(flock& my_flock){
    SDL_SetRenderDrawColor(g.renderer, int(g.brightness*255u), int(g.brightness*255u), int(g.brightness*255u), SDL_ALPHA_OPAQUE);
    SDL_RenderClear(g.renderer);
    paint_boids(my_flock, 20);
    SDL_RenderPresent(g.renderer);
}


void do_update(flock& my_flock) {
    my_flock.update(f_cohesion, f_separation, f_alignment);
}

//Le main sert uniquement à l'affichage, toute modification autre se fera par le biais du "do_update"
int main(int argc, const char *argv[]) {
    try {
        flock my_flock(n_boids, max_speed, 1.0f);
        if (SDL_Init(SDL_INIT_EVERYTHING) < 0){
            throw std::runtime_error(SDL_GetError());
        }
        g.window = SDL_CreateWindow("Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            800,
            600,
            SDL_WINDOW_SHOWN
        );
        if(!g.window){
            throw std::runtime_error(SDL_GetError());
        }
        g.renderer = SDL_CreateRenderer(g.window, -1, 0);
        if (!g.renderer) {
            throw std::runtime_error(SDL_GetError());
        }
        bool end = false;
        while (!end) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    end = true;
                }
            }
            SDL_Delay(delay);
            do_update(my_flock);
            do_render(my_flock);
        }
        SDL_DestroyRenderer(g.renderer);
        SDL_DestroyWindow(g.window);
        SDL_Quit();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        if (g.renderer) SDL_DestroyRenderer(g.renderer);
        if (g.window) SDL_DestroyWindow(g.window);
        SDL_Quit();
        return 1;
    }
    return 0;
}


