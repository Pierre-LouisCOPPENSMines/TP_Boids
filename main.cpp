//
//  main.cpp
//  tp_boids_app
//
//  Created by Pierre-Louis Coppens on 17/12/2024.
//
//compile with: g++ -o nouveau_fichier main.cpp -F/Library/Frameworks -framework SDL2 -Wl,-rpath,/Library/Frameworks
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

 
struct global_t {
    SDL_Window * window = NULL;
    SDL_Renderer * renderer = NULL;
    

    // random
    float brightness;
    std::random_device rd;
    std::default_random_engine eng;
    std::uniform_real_distribution<float> rand;

};

global_t g;

void paint_it_s_work(int ox, int oy, int scale = 20) {
    SDL_SetRenderDrawColor(g.renderer, 0u, 0u, 0u, SDL_ALPHA_OPAQUE);
    for (int j = 0; j < px::height; ++j) {
        for (int i = 0; i < px::width; ++i) {
            if (px::header_data[j*px::width+i] == 0) {
                SDL_Rect r = { i*scale+ox, j*scale+oy, 20, 20 };
                SDL_RenderFillRect(g.renderer, &r);
            }
        }
    }
}

void do_render() {
    SDL_SetRenderDrawColor(g.renderer, int(g.brightness*255u), int(g.brightness*255u), int(g.brightness*255u), SDL_ALPHA_OPAQUE);
    SDL_RenderClear(g.renderer);
    paint_it_s_work(0, 0, 20);

    SDL_RenderPresent(g.renderer);
}

void do_update() {
    if (g.brightness < 1){
        g.brightness = g.brightness + 0.1;
    }
    else {
        g.brightness = 0;
    }
}

//Le main sert uniquement à l'affichage, toute modification autre se fera par le biais du "do_update"
int main(int argc, const char *argv[]) {
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0){
        printf("Error: %s\n",SDL_GetError());
        return 1;
    }
    g.window = SDL_CreateWindow("Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        800,
        600,
        SDL_WINDOW_SHOWN
    );
    if(!g.window){
        printf("Error: %s\n",SDL_GetError());
        return 1;
    }
    g.renderer = SDL_CreateRenderer(g.window, -1, 0);
    if (not g.renderer) {
        return 1;
    }
    bool end = false;
    while (not end) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
            end = true;
            }
        }
        SDL_Delay(500);
        do_update();
        do_render();
    }
    SDL_DestroyRenderer(g.renderer);
    SDL_DestroyWindow(g.window);
    SDL_Quit();
    return 0;
}


