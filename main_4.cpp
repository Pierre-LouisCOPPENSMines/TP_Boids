//
//  main.cpp
//  tp_boids_app
//
//  Created by Pierre-Louis Coppens on 17/12/2024.
//
//compile with: g++ -o nouveau_fichier main_4.cpp -std=c++17 -F/Library/Frameworks -framework SDL2 -Wl,-rpath,/Library/Frameworks
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
#include <memory>

//Paramètres de la simulation
const int n_boids = 100;
const int delay = 10;
const float radius_separation = 10;
const float radius_alignment = 20;
const float f_cohesion = 0.01;
const float f_separation = 1;
const float f_alignment = 0.1;
const float f_chase = 0.01;
const float f_flee = 0.01;
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
    float speed;
    boid(float x, float y, float speed){
        position = {x, y};
        velocity = {0, 0};
        this->speed = speed;
    }
};

struct flock {
    std::vector<boid *> boids;
    int flock_size; 
    std::vector<int> color;
    flock(std::vector<boid *> boids, int n, std::vector<int> f_color = {255, 255, 255}) {
        this->boids = boids;
        this->flock_size = n;
        this->color = f_color;
    }
    flock(int n, std::vector<int> f_color = {255, 255, 255}, float speed = 2.0f) {
        for (int i = 0; i < n; ++i){
            boids.push_back(new boid(g.rand(g.eng)*800, g.rand(g.eng)*600, speed));
        }
        this->flock_size = n;
        this->color = f_color;
    }
    std::vector<float> get_mean_position(){
        std::vector<float> mean_position = {0.0f, 0.0f};
        for (const auto& boid : boids) {
            mean_position[0] += boid->position[0];
            mean_position[1] += boid->position[1];
        }
        mean_position[0] /= boids.size();
        mean_position[1] /= boids.size();
        return mean_position;
    }
    std::vector<float> get_mean_speed(){
        std::vector<float> mean_speed = {0.0f, 0.0f};
        for (const auto& boid : boids) {
            mean_speed[0] += boid->velocity[0];
            mean_speed[1] += boid->velocity[1];
        }
        mean_speed[0] /= boids.size();
        mean_speed[1] /= boids.size();
        return mean_speed;
    }
    boid * get_leader_boids(){
        boid * leader = boids[0];
        for (const auto& boid : boids) {
            if (std::sqrt((boid->position[0] - get_mean_position()[0]) * (boid->position[0] - get_mean_position()[0]) + (boid->position[1] - get_mean_position()[1]) * (boid->position[1] - get_mean_position()[1])) > std::sqrt((leader->position[0] - get_mean_position()[0]) * (leader->position[0] - get_mean_position()[0]) + (leader->position[1] - get_mean_position()[1]) * (leader->position[1] - get_mean_position()[1]))){
                leader = boid;
            }
        }
        return leader;

    }
};

struct relation {
    std::vector<flock *> flocks_concerned; 
    flock * chaser;
    flock * prey;
    relation(flock * f1, flock * f2) {
        flocks_concerned = {f1, f2};
        chaser = f1;
        prey = f2;
    }
};

struct world {
    std::vector<flock *> flocks;
    int world_size; //nombre de flock
    std::vector<relation *> relations;
    world(std::vector<flock *> flocks, int n) {
        this->flocks = flocks;
        this->world_size = n;
    }
    std::vector<boid *> boids_in_radius(boid* b, std::vector<flock *> searching_flocks, float radius){ 
        std::vector<boid *> boids_in_radius;
        for (const auto& flock : searching_flocks) {
            for (const auto& boid : flock->boids) {
                if (std::sqrt((b->position[0] - boid->position[0]) * (b->position[0] - boid->position[0]) + (b->position[1] - boid->position[1]) * (b->position[1] - boid->position[1])) < radius) {
                    boids_in_radius.push_back(boid);
                }
            }
        }
        return boids_in_radius;
    }
    void set_chase(flock * chaser, flock * prey) {
        relations.push_back(new relation(chaser, prey));
    }
    void update();
};


struct rule {
    std::vector<float> acceleration = {0.0f, 0.0f};
    float factor; 
};

struct co_rule : public rule {
    co_rule (boid * b, flock * flock) { 
        std::vector<float> mean_position = {0.0f, 0.0f};
        
        for (const auto& boid : flock->boids) {
            mean_position[0] += boid->position[0];
            mean_position[1] += boid->position[1];
        }

        mean_position[0] /= flock->boids.size();
        mean_position[1] /= flock->boids.size();

        acceleration = {mean_position[0] - b->position[0],mean_position[1] - b->position[1]};
        factor = f_cohesion;
    }
};

struct sep_rule : public rule {
    sep_rule (boid * b, flock * flock, world * world, float radius) { 
        std::vector<float> vector_sum = {0.0f, 0.0f};
        std::vector<boid *> closer_boids = world->boids_in_radius(b, {flock}, radius);
        if (!closer_boids.empty()) {
            for (const auto& boid : closer_boids) {
                vector_sum[0] += boid->position[0] - b->position[0];
                vector_sum[1] += boid->position[1] - b->position[1];
            }
            float norm = std::sqrt(vector_sum[0] * vector_sum[0] + vector_sum[1] * vector_sum[1]);
            try {
                if (norm == 0){
                    throw std::runtime_error("seperation rule: norm is 0");
                }
                acceleration[0] = -vector_sum[0] / norm;
                acceleration[1] = -vector_sum[1] / norm;
            }
            catch(std::runtime_error& e) {
                if (e.what() == std::runtime_error("seperation rule: norm is 0").what()) {
                    acceleration = {0, 0};
                }
            }
        }
        factor = f_separation;
    }
};

struct align_rule : public rule {
    align_rule (boid * b, flock * flock, world * world, float radius) {
        std::vector<float> mean_speed = {0.0f, 0.0f};
        std::vector<boid *> closer_boids = world->boids_in_radius(b, {flock}, radius);
        if (!closer_boids.empty()) {
            for (const auto& boid : closer_boids) {
                mean_speed[0] += boid->velocity[0];
                mean_speed[1] += boid->velocity[1];
            }
            acceleration[0] = mean_speed[0] / float(closer_boids.size());
            acceleration[1] = mean_speed[1] / float(closer_boids.size());
        }
        factor = f_alignment;
    }

};

struct chase_rule : public rule {
    chase_rule(boid * b, flock * targeting_flock, world * world, float radius)
    {
        std::vector<float> mean_position = {0.0f, 0.0f};
        std::vector<boid *> closer_boids = world->boids_in_radius(b, {targeting_flock}, radius);
        if (!closer_boids.empty()) {
            for (const auto& boid : closer_boids) {
                mean_position[0] += boid->position[0];
                mean_position[1] += boid->position[1];
            }
            mean_position[0] /= closer_boids.size();
            mean_position[1] /= closer_boids.size();
            acceleration[0] = mean_position[0] - b->position[0];
            acceleration[1] = mean_position[1] - b->position[1];
        }
    }
};

struct flee_rule : public rule {
    flee_rule(boid * b, flock * targeting_flock, world * world, float radius)
    {
        std::vector<float> mean_position = {0.0f, 0.0f};
        std::vector<boid *> closer_boids = world->boids_in_radius(b, {targeting_flock}, radius);
        if (!closer_boids.empty()) {
            for (const auto& boid : closer_boids) {
                mean_position[0] += boid->position[0];
                mean_position[1] += boid->position[1];
            }
            mean_position[0] /= closer_boids.size();
            mean_position[1] /= closer_boids.size();
            acceleration[0] = b->position[0] - mean_position[0];
            acceleration[1] = b->position[1] - mean_position[1];
        }
    }

};



struct V_rule : public rule {

    V_rule(boid * b, flock * flock, world * world, float radius) {
        std::vector<float> mean_position = {0.0f, 0.0f};
        std::vector<boid *> closer_boids = world->boids_in_radius(b, {flock}, radius);
        if (!closer_boids.empty()) {
            for (const auto& boid : closer_boids) {
                mean_position[0] += boid->position[0];
                mean_position[1] += boid->position[1];
            }
            mean_position[0] /= closer_boids.size();
            mean_position[1] /= closer_boids.size();
            acceleration[0] = mean_position[0] - b->position[0];
            acceleration[1] = mean_position[1] - b->position[1];
        }
    }
};

void world::update() {
    for (auto& flock : flocks) {
        for (auto& boid : flock->boids) {
            std::vector<float> acceleration = {0.0f, 0.0f};

            co_rule cohesion_rule(boid, flock);
            sep_rule separation_rule(boid, flock, this, radius_separation);
            align_rule alignment_rule(boid, flock, this, radius_alignment);
            for (const auto& relation : relations) {
                if (relation->flocks_concerned[0] == flock) {
                    chase_rule chase_rule(boid, relation->prey, this, 200);
                    acceleration[0] += chase_rule.acceleration[0] * f_chase;
                    acceleration[1] += chase_rule.acceleration[1] * f_chase;
                }
                if (relation->flocks_concerned[1] == flock) {
                    flee_rule flee_rule(boid, relation->chaser, this, 50);
                    acceleration[0] += flee_rule.acceleration[0] * f_flee;
                    acceleration[1] += flee_rule.acceleration[1] * f_flee;
                }
            }

            
            acceleration[0] += cohesion_rule.acceleration[0] * cohesion_rule.factor; 
            acceleration[1] += cohesion_rule.acceleration[1] * cohesion_rule.factor;
            acceleration[0] += separation_rule.acceleration[0] * separation_rule.factor;
            acceleration[1] += separation_rule.acceleration[1] * separation_rule.factor;
            acceleration[0] += alignment_rule.acceleration[0] * alignment_rule.factor; 
            acceleration[1] += alignment_rule.acceleration[1] * alignment_rule.factor;
            std::cout << "Cohesion acceleration: (" << cohesion_rule.acceleration[0] * cohesion_rule.factor << ", " << cohesion_rule.acceleration[1] * cohesion_rule.factor << ")" << std::endl;
            std::cout << "Separation acceleration: (" << separation_rule.acceleration[0] * separation_rule.factor << ", " << separation_rule.acceleration[1] * separation_rule.factor << ")" << std::endl;
            std::cout << "Alignment acceleration: (" << alignment_rule.acceleration[0] * alignment_rule.factor << ", " << alignment_rule.acceleration[1] * alignment_rule.factor << ")" << std::endl;
            boid->velocity[0] += acceleration[0];
            boid->velocity[1] += acceleration[1];

        }
    }
    for (auto& flock : flocks) {
        for (auto& boid : flock->boids) {
            float speed = std::sqrt(boid->velocity[0] * boid->velocity[0] + boid->velocity[1] * boid->velocity[1]);
            if (speed > boid->speed) {
                boid->velocity[0] = (boid->velocity[0] / speed) * boid->speed;
                boid->velocity[1] = (boid->velocity[1] / speed) * boid->speed;
            }
            boid->position[0] += boid->velocity[0];
            boid->position[1] += boid->velocity[1];
            if (boid->position[0] < 0) {
                boid->position[0] = 0;
                boid->velocity[0] = std::abs(boid->velocity[0]);
            } else if (boid->position[0] > 800) {
                boid->position[0] = 799;
                boid->velocity[0] = -std::abs(boid->velocity[0]);
            }
            if (boid->position[1] < 0) {
                boid->position[1] = 0;
                boid->velocity[1] = std::abs(boid->velocity[1]);
            } else if (boid->position[1] > 600) {
                boid->position[1] = 599;
                boid->velocity[1] = -std::abs(boid->velocity[1]);
            }
        }
    }
};

void paint_boids(flock * my_flock, int scale = 20) {
    for (const auto& b : my_flock->boids) {
        SDL_SetRenderDrawColor(g.renderer, my_flock->color[0], my_flock->color[1], my_flock->color[2], SDL_ALPHA_OPAQUE);
        SDL_Rect r = { int(b->position[0]), int(b->position[1]), 5, 5 };
        SDL_RenderFillRect(g.renderer, &r);
    }
}

void do_render(world * my_world){
    SDL_SetRenderDrawColor(g.renderer, int(g.brightness*255u), int(g.brightness*255u), int(g.brightness*255u), SDL_ALPHA_OPAQUE);
    SDL_RenderClear(g.renderer);
    for (const auto& flock : my_world->flocks) {
        paint_boids(flock, 20);
    }
    SDL_RenderPresent(g.renderer);
}


void do_update(world * my_world) {
    my_world->update();
}

//Le main sert uniquement à l'affichage, toute modification autre se fera par le biais du "do_update"
int main(int argc, const char *argv[]) {
    flock flock1(n_boids, {255, 0, 0},2); // RED FLOCK
    flock flock2(n_boids, {0, 255, 0},3); // GREEN FLOCK
    std::vector<flock *> flocks = {&flock1, &flock2};
    world my_world(flocks, 2);
    my_world.set_chase(&flock1, &flock2);
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0){
        throw std::runtime_error(SDL_GetError());
    }
    g.window = SDL_CreateWindow("Boids Pierre-Louis COPPENS", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
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
        do_render(&my_world);
        do_update(&my_world);
        std::cout << "Update" << std::endl;
        
    }
    SDL_DestroyRenderer(g.renderer);
    SDL_DestroyWindow(g.window);
    SDL_Quit();

    return 0;
}


