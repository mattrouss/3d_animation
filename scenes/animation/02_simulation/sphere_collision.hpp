#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_SPHERE_COLLISION

// Structure of a particle
struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces

    vcl::vec3 c; // Color
    float r;     // Radius
};

struct plane_structure
{
    vcl::vec3 a; // Plane center
    vcl::vec3 n; // Plane normal
};

struct gui_scene_structure
{
    bool add_sphere = true;
    float time_interval_new_sphere = 0.5f;
};

struct scene_model : scene_base
{

    vcl::vec3 get_collision_velocity(const vcl::vec3& velocity, const vcl::vec3& plane_normal);

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt);
    void create_new_particle();
    void display_particles(scene_structure& scene);


    std::vector<particle_structure> particles;
    std::vector<plane_structure> planes;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders

    vcl::timer_event timer;
    gui_scene_structure gui_scene;

    float alpha;
    float beta;
};






#endif
