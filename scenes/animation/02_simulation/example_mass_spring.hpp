#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_MASS_SPRING_1D

struct particle_element
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void initialize();
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void set_gui(vcl::timer_basic& timer);


    particle_element p_root;
    std::vector<particle_element> points;
    float L0 = 0.2f;                // length of spring at rest
    float K = 5.0f;                 // spring stiffness
    float mu = 0.1f;       // damping coefficient

    size_t N = 10;
    size_t N_prev = 10;

    size_t k = 1;
    size_t k_prev = 1;


    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders
    vcl::segment_drawable_immediate_mode segment_drawer;

    vcl::timer_event timer;
};






#endif
