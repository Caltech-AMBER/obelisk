#include <iostream>

#include <mujoco.h>

// MuJoCo data structures
mjModel* m = NULL; // MuJoCo model
mjData* d  = NULL; // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

int main() { std::cout << "Hellow world" << std::endl; }
