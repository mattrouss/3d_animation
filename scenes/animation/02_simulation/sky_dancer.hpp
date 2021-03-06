#pragma once

#include "main/scene_base/base.hpp"
#include <math.h>
#include <algorithm>
#include <string>

#ifdef SCENE_SKY_DANCER


struct user_parameters_structure
{
    float m;    // Global mass (to be divided by the number of particles)
    float K;    // Global stiffness
    float mu;   // Damping
    float wind; // Wind magnitude;
    float pressure; 
};

struct simulation_parameters_structure
{
    float m;  // mass
    float L0_horizontal; // spring rest length
    float L0_vertical; // spring rest length
};

// Sphere and ground used for collision
struct collision_shapes_structure
{
    float ground_height; // height of the ground (in y-coordinate)
};



struct scene_model : scene_base
{
    vcl::timer_event timer;
    struct skydancer {
     // Simulation parameters
    simulation_parameters_structure simulation_parameters; // parameters that user can control directly
    user_parameters_structure user_parameters;             // parameters adjusted with respect to mesh size (not controled directly by the user)
    

    // Gui parameters
    bool gui_display_wireframe;
    bool gui_display_texture;

    vcl::mesh_drawable ground;
   

    // Particles parameters
    vcl::buffer2D<vcl::vec3> position;
    vcl::buffer2D<vcl::vec3> speed;
    vcl::buffer2D<vcl::vec3> force;

   
    // Cloth mesh elements
    vcl::mesh_drawable cloth;              // Visual model for the cloth
    vcl::buffer<vcl::vec3> normals;        // Normal of the cloth used for rendering and wind force computation
    vcl::buffer<vcl::uint3> connectivity;  // Connectivity of the triangular model

    // Parameters of the shape used for collision
    collision_shapes_structure collision_shapes;

    // Acceleration grid for cloth-on-cloth collisions
    const float cell_size = 0.2f;
    const size_t collision_grid_dim = 50u;
    const vcl::vec3 grid_min = {-5.0f, -5.0f, -5.0f};
    const vcl::vec3 grid_max = {5.0f, 5.0f, 5.0f};
    std::vector<std::vector<size_t>> collision_grid;

    // Store index and position of vertices constrained to have a fixed 3D position
    std::map<int,vcl::vec3> positional_constraints;

    // Textures
    GLuint texture_cloth;
    GLuint texture_wood;

    // Visual elements of the scene
    vcl::mesh_drawable sphere;
    vcl::segment_drawable_immediate_mode segment_drawer;

    // Parameters used to control if the simulation runs when a numerical divergence is detected
    bool simulation_diverged; // Active when divergence is detected
    bool force_simulation;    // Force to run simulation even if divergence is detected
  
    GLuint shader_mesh;  


    void initialize(const size_t id, size_t resolution);
    void collision_constraints();
    void self_collision();
    void compute_forces(const float id, vcl::timer_event timer);
    void compute_spring_forces(const int ku, const int kv);
    void compute_wind_force(const int ku, const int kv, const float id, vcl::timer_event timer);
    void numerical_integration(float h);
    void detect_simulation_divergence(vcl::timer_event timer);
    void hard_constraints();
    void display_elements(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    
    };

    size_t resolution = 40u;
    size_t old_resolution = 40u;
    std::vector<skydancer> skydancers;
    

    void set_gui();
    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void display_ground(scene_structure& scene, gui_structure& gui);
    
};






#endif

