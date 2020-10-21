
#include "sphere_collision.hpp"

#include <random>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;

void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    create_new_particle();
    compute_time_step(dt);

    display_particles(scene);
    draw(borders, scene.camera);

}

void scene_model::compute_time_step(float dt)
{
    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,-9.81f,0);


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v = (1-0.9f*dt) * v + dt * f; // gravity + friction force

        p = p + dt * v;
    }

    // Collisions between spheres
    for (size_t i = 0; i < N; ++i)
    {
        particle_structure& p1 = particles[i];
        for (size_t j = 0; j < N; ++j)
        {
            if (i == j)
                continue;

            particle_structure& p2 = particles[j];
            // Collision detection
            float dist = norm(p1.p - p2.p);
            if (dist <= p1.r + p2.r)
            {
                vec3& v1 = p1.v;
                vec3& v2 = p2.v;
                vec3 const& u = (p1.p - p2.p) / dist;

                if (norm(p1.v - p2.v) > min_relative_speed) {
                    float j = 2 * (p1.m * p2.m) / (p1.m + p2.m) * dot(v2 - v1, u);
                    v1 = alpha * v1 - (beta * j) / p1.m * u;
                    v2 = alpha * v2 + (beta * j) / p2.m * u;

                } else {
                    v1 = mu * v1;
                    v2 = mu * v2;
                }

                float d = p1.r + p2.r - dist;
                p1.p = p1.p + 0.5f * d * u;
                p2.p = p2.p - 0.5f * d * u;
            }

        }
    }
    // Collisions with cube
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        const size_t n = planes.size();
        for (size_t i = 0; i < n; i++) {
            plane_structure const& plane = planes[i];
            float detection = dot(particle.p - plane.a, plane.n);
            if (detection <= particle.r) {
                v = get_collision_velocity(v, plane.n);

                float d = particle.r - detection;
                p = p + d * plane.n;
            }
        }
    }


}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.m = 1.0f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);

    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{

    alpha = 0.5f;
    beta = 0.5f;
    mu = 0.5;

    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    planes = {{{1.0f, 0.0f, 0.0f},{-1.0f, 0.0f, 0.0f}}, {{0.0f, 1.0f, 0.0f},{0.0f, -1.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}}, {{-1.0f, 0.0f, 0.0f},{1.0f, 0.0f, 0.0f}}, {{0.0f, -1.0f, 0.0f},{0.0f, 1.0f, 0.0f}}, {{0.0f, 0.0f, -1.0f},{0.0f, 0.0f, 1.0f}}};

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
        {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
        {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Friction Coefficient Alpha", &alpha, 0.0f, 1.0f);
    ImGui::SliderFloat("Impact Coefficient Beta", &beta, 0.0f, 1.0f);
    ImGui::SliderFloat("Static Coeff Mu", &mu, 0.0f, 1.0f);
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}

vec3 scene_model::get_collision_velocity(const vec3& velocity, const vec3& plane_normal) {
    vec3 v_impact = dot(velocity, plane_normal) * plane_normal;
    vec3 v_friction = velocity - v_impact;

    return alpha * v_friction + (-beta) * v_impact;
}





#endif
