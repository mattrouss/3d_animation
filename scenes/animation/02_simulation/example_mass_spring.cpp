
#include "example_mass_spring.hpp"


#ifdef SCENE_MASS_SPRING_1D

using namespace vcl;


/** Compute spring force applied on particle pi from particle pj */
vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
    vec3 const pji = pj - pi;
    float const L = norm(pji);
    return K * (L - L0) * pji / L;
}


void scene_model::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& )
{

    initialize();
    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = borders_segments;
    borders.uniform.color = {0,0,0};

}

void scene_model::initialize()
{
    p_root.p = {0,0,0};     // Initial position of particle A
    p_root.v = {0,0,0};     // Initial speed of particle A

    points.clear();
    points.resize(N);
    // Initial position and speed of particles
    // ******************************************* //
    for (size_t i = 0; i < N; i++)
    {
        points[i].p = {0.5f * (float)(i + 1), 0, 0};
        points[i].v = {0, 0, 0};
    }

    // Display elements
    // ******************************************* //
    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0,0,1};

    sphere = mesh_primitive_sphere();
    sphere.uniform.transform.scaling = 0.05f;


}





void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui(timer);

    if (N != N_prev)
    {
        initialize();
        N_prev = N;
    }


    // Simulation time step (dt)
    float dt = timer.scale*0.01f;

    // Simulation parameters
    const float m  = 0.05f;        // particle mass
    const vec3 g   = {0,-9.81f,0}; // gravity

    // Forces
    const vec3 f_weight =  m * g;


    // Draw root sphere
    sphere.uniform.transform.translation = p_root.p;
    sphere.uniform.color = {0,0,1};
    draw(sphere, scene.camera, shaders["mesh"]);

    sphere.uniform.color = {1,0,0};

    // Numerical Integration (Verlet)
    for (size_t i = 0; i < points.size(); ++i)
    {
        vec3& p = points[i].p; // position of particle
        vec3& v = points[i].v; // speed of particle

        vec3 F = {};
        if (i == 0)
            F += spring_force(p, p_root.p, L0, K);
        if (i > 0)
            F += spring_force(p, points[i - 1].p, L0, K);
        if (i < points.size() - 1)
            F += spring_force(p, points[i + 1].p, L0, K);

        F += -mu * v;
        F += f_weight;

        v += dt * F / m;
        p += dt * v;
        
        // Display of the result
        sphere.uniform.transform.translation = p;
        draw(sphere, scene.camera, shaders["mesh"]);
    }

    
    // Draw spring between root and 0
    segment_drawer.uniform_parameter.p1 = p_root.p;
    segment_drawer.uniform_parameter.p2 = points[0].p;
    segment_drawer.draw(shaders["segment_im"],scene.camera);

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (i >= points.size() - 1)
            continue;

        // Draw spring between points i and i + 1
        segment_drawer.uniform_parameter.p1 = points[i].p;
        segment_drawer.uniform_parameter.p2 = points[i + 1].p;
        segment_drawer.draw(shaders["segment_im"],scene.camera);
    }

    draw(borders, scene.camera, shaders["curve"]);
}


/** Part specific GUI drawing */
void scene_model::set_gui(timer_basic& timer)
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;

    size_t N_min = 10;
    size_t N_max = 50;

    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.3f s");
    ImGui::SliderScalar("Number of points", ImGuiDataType_U32, &N, &N_min, &N_max);
    ImGui::SliderFloat("Rest Length", &L0, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Spring Stiffness", &K, 0.0f, 50.0f, "%.2f");
    ImGui::SliderFloat("Damping coefficient", &mu, 0.0f, 1.0f, "%.3f");

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();

    bool const restart = ImGui::Button("Restart");
    if(restart) initialize();

}



#endif
