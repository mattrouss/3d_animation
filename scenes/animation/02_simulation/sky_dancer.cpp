
#include "sky_dancer.hpp"


#ifdef SCENE_SKY_DANCER

using namespace vcl;


/** Compute spring force applied on particle pi from particle pj */
vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
    vec3 const pji = pj - pi;
    float const L = norm(pji);
    return K * (L - L0) * pji / L;
}

// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force
void scene_model::skydancer::compute_forces(const float i, vcl::timer_event timer)
{
    const size_t N = force.size();        // Total number of particles of the cloth Nu x Nv
    const int N_dim = int(force.dimension[0]); // Number of particles along one dimension (square dimension)

    simulation_parameters.m = user_parameters.m / float(N); // Constant total mass

    const float m  = simulation_parameters.m;

    // Gravity
    const vec3 g = {0,-9.81f,0};
    for(size_t k=0; k<N; ++k)
        force[k] = m*g;

    // Drag
    const float mu = user_parameters.mu;
    for(size_t k=0; k<N; ++k)
        force[k] += force[k]-mu*speed[k];

    // Springs
    for(int ku=0; ku<N_dim; ++ku) {
        for(int kv=0; kv<N_dim; ++kv) {
            compute_spring_forces(ku, kv);
            compute_wind_force(ku, kv, i, timer);
        } 
    }
}

void scene_model::skydancer::compute_spring_forces(const int ku, const int kv)
{
    const int N_dim = int(force.dimension[0]); // Number of particles along one dimension

    // Get simuation parameters
    const float K = user_parameters.K;
    const float L0_horizontal = simulation_parameters.L0_horizontal;
    const float L0_vertical = simulation_parameters.L0_vertical;

    size_t2 const k = {(size_t)ku, (size_t)kv};

    // Add structural springs
    if (kv == 0)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(N_dim - 1)}], L0_horizontal, K);
    if (kv == N_dim - 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), 0u}], L0_horizontal, K);
    if (ku > 0)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku - 1), size_t(kv)}], L0_vertical, K);
    if (ku < N_dim - 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku + 1), size_t(kv)}], L0_vertical, K);
    if (kv > 0)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(kv - 1)}], L0_horizontal, K);
    if (kv < N_dim - 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(kv + 1)}], L0_horizontal, K);

    // Add shearing springs
    const float L0_diag = std::sqrt(std::pow(L0_horizontal, 2) + std::pow(L0_vertical, 2));
    if (kv == 0)
    {
        if (ku > 0)
            force[k] += spring_force(position[k], position[size_t2{size_t(ku - 1), size_t(N_dim - 1)}], L0_diag, K);
        if (ku < N_dim - 1)
            force[k] += spring_force(position[k], position[size_t2{size_t(ku + 1), size_t(N_dim - 1)}], L0_diag, K);
    }
    if (kv == N_dim - 1)
    {
        if (ku > 0)
            force[k] += spring_force(position[k], position[size_t2{size_t(ku - 1), 0u}], L0_diag, K);
        if (ku < N_dim - 1)
            force[k] += spring_force(position[k], position[size_t2{size_t(ku + 1), 0u}], L0_diag, K);
    }
    if (ku > 0 && kv > 0)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku - 1), size_t(kv - 1)}], L0_diag, K);
    if (ku < N_dim - 1 && kv < N_dim - 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku + 1), size_t(kv + 1)}], L0_diag, K);
    if (ku > 0 && kv < N_dim - 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku - 1), size_t(kv + 1)}], L0_diag, K);
    if (ku < N_dim - 1 && kv > 0)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku + 1), size_t(kv - 1)}], L0_diag, K);

    // Add bending springs
    if (kv == 0)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(N_dim - 2)}], 2 * L0_horizontal, K);
    if (kv == N_dim - 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), 1u}], 2 * L0_horizontal, K);
    if (kv == 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(N_dim - 1)}], 2 * L0_horizontal, K);
    if (kv == N_dim - 2)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), 0u}], 2 * L0_horizontal, K);

    if (ku > 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku - 2), size_t(kv)}], 2 * L0_vertical, K);
    if (ku < N_dim - 2)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku + 2), size_t(kv)}], 2 * L0_vertical, K);
    if (kv > 1)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(kv - 2)}], 2 * L0_horizontal, K);
    if (kv < N_dim - 2)
        force[k] += spring_force(position[k], position[size_t2{size_t(ku), size_t(kv + 2)}], 2 * L0_horizontal, K);

}

void scene_model::skydancer::compute_wind_force(const int ku, const int kv, const float id, vcl::timer_event timer)
{    
    timer.update();
    const float time = timer.t;
    float periodic_y = abs(1.5 * pow(cos(time + id), 4) + sin(time + id) + 3 * pow(sin(time), 5));
    float periodic_x = pow(cos(time + id ), 3) + pow(sin(time + id), 3); 
    
    size_t2 const k = {(size_t)ku, (size_t)kv}; // pixel indexes

    const vec3 normal = normals[ku + int(force.dimension[0]) * kv]; 
    const vec3 wind_force = 2  * vec3{periodic_x, periodic_y, 0};

    force[k] += 0.5f*  std::fabs(dot(normal, wind_force)) * wind_force;
    force[k] += user_parameters.wind/2 * std::fabs(dot(normal, {0, 0, -1.0f})) * vec3{0.0f, 0.0f, -1.0f};

    force[k] += user_parameters.pressure * normal / norm(normal);
}


// Handle detection and response to collision with the shape described in "collision_shapes" variable
void scene_model::skydancer::collision_constraints()
{
    // Handle collisions here (with the ground and the sphere)
    const float ground_height = collision_shapes.ground_height;
    
    const size_t N = position.size();
    for(size_t k=0; k<N; ++k) {
        vec3& p = position[k];
        vec3& v = speed[k];
        // Collision detection with ground
        if (p.y <= ground_height) {
            v.y = 0;
            p.y = ground_height;
        }
    }

    self_collision();

}

void scene_model::skydancer::self_collision()
{
    const float particle_radius = 0.005f;
    const size_t N = position.size();

    for (auto& it : collision_grid)
        it.clear();


    float conversion_factor = 1.0f / cell_size;
    for (size_t i = 0; i < N; ++i)
    {
        vec3 p = position[i] - grid_min;
        size_t hash = p.x + p.z * conversion_factor + p.y * conversion_factor * conversion_factor;
        collision_grid[hash].push_back(i);
    }

    for (auto cell = collision_grid.begin(); cell < collision_grid.end(); ++cell)
    {
        const size_t N_cell = cell->size();
        // Self collision detection
        for (size_t i = 0; i < N_cell; ++i)
        {
            size_t index_1 = (*cell)[i];
            vec3 &p1 = position[index_1];
            for (size_t j = 0; j < N_cell; ++j)
            {
                if (i == j)
                    continue;

                size_t index_2 = (*cell)[j];
                vec3 &p2 = position[index_2];

                const vec3 delta = p1 - p2;
                const float dist = norm(delta);
                const vec3 u = (1 / dist) * delta;
                if (dist < 2 * particle_radius)
                {
                    vec3& v1 = speed[index_2];
                    vec3& v2 = speed[index_2];
    
                    const vec3 v_orth = dot(v2 - v1, u) * u;
                    v1 -= v_orth;
                    v2 += v_orth;
    
                    float d = 2 * particle_radius - dist + 1E-3;
                    p1 += 0.5 * d * u;
                    p2 -= 0.5 * d * u;
                }
            }
        }


    }

}




// Initialize the geometrical model
void scene_model::skydancer::initialize(const size_t idx)
{
    // Number of samples of the model (total number of particles is N_cloth x N_cloth)
    const size_t N_cloth = resolution;
    const float radius = 0.5f;
    const float height = 5.0f;

    // Rest length (length of an edge)
    const float theta = static_cast<float>( 2* 3.14159f / float(N_cloth));
    simulation_parameters.L0_horizontal = radius * std::tan(theta);
    simulation_parameters.L0_vertical = height/float(N_cloth-1);

    // Create cloth mesh in its initial position
    // Horizontal grid of length 1 x 1ve

    auto v1 = vec3({0, 0, 0});
    auto v2 = vec3({2.5, 0, 0.5});
    auto v3 = vec3({-2.5, 0, 0.5});
    std::vector<vec3> positions({v1, v2, v3});
    const mesh base_dancer = mesh_primitive_cylinder(radius, positions[idx], {0, height, 0}, N_cloth, N_cloth, true);

    // Set particle position from cloth geometry
    position = buffer2D_from_vector(base_dancer.position, N_cloth, N_cloth);

    // Initialize collision accelerator grid
    collision_grid.resize(collision_grid_dim * collision_grid_dim * collision_grid_dim);

    // Set hard positional constraints
    positional_constraints.clear();
    for (size_t ku = 0; ku < N_cloth; ++ku)
        positional_constraints[ku * N_cloth] = position[ku * N_cloth];

    // Init particles data (speed, force)
    speed.resize(position.dimension); speed.fill({0,0,0});
    force.resize(position.dimension); force.fill({0,0,0});

    // Store connectivity and normals
    connectivity = base_dancer.connectivity;
    normals      = normal(position.data,connectivity);

    // Send data to GPU
    cloth.clear();
    cloth = mesh_drawable(base_dancer);
    cloth.uniform.shading.specular = 0.0f;
    cloth.shader = shader_mesh;
    cloth.texture_id = texture_cloth;

    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0,0,1};

    sphere = mesh_primitive_sphere();
    sphere.uniform.transform.scaling = 0.05f;

    simulation_diverged = false;
    force_simulation    = false;

    //timer.update();
}

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& gui)
{
    skydancers.resize(3);
    for (size_t i = 0; i < 3;i++)
    {
        gui.show_frame_camera = false;

        std::string colors[3] = {"yellow_dancer.png", "blue_dancer.png", "green_dancer.png"};
        // Load textures
        skydancers[i].texture_cloth = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/" + colors[i]));
        skydancers[i].texture_wood  = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/parking.png"));
        skydancers[i].shader_mesh = shaders["mesh_bf"];

        // Initialize cloth geometry and particles
        skydancers[i].initialize(i);

        // Default value for simulation parameters
        skydancers[i].user_parameters.K    = 600.0f;
        skydancers[i].user_parameters.m    = 5.0f;
        skydancers[i].user_parameters.wind = 1.0f;
        skydancers[i].user_parameters.mu   = 0.02f;
        skydancers[i].user_parameters.pressure = 4.0f;

        // Set collision shapes
        skydancers[i].collision_shapes.ground_height = 0;

        const float half_width_ground = 5.0f;
        skydancers[i].ground = mesh_drawable(mesh_primitive_quad({-half_width_ground,skydancers[i].collision_shapes.ground_height-1e-3f,-half_width_ground}, {half_width_ground, skydancers[i].collision_shapes.ground_height-1e-3f,-half_width_ground}, {half_width_ground,skydancers[i].collision_shapes.ground_height-1e-3f,half_width_ground}, {-half_width_ground,skydancers[i].collision_shapes.ground_height-1e-3f,half_width_ground}));
        skydancers[i].ground.shader = shaders["mesh_bf"];
        skydancers[i].ground.texture_id = skydancers[i].texture_wood;

        skydancers[i].gui_display_texture = true;
        skydancers[i].gui_display_wireframe = false;
    }
}

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    const float dt = timer.update();
    set_gui();
    for(size_t i = 0; i < 3; i++)
    {
        

        // Force constant simulation time step
        float h = dt<=1e-6f? 0.0f : timer.scale*0.001f;

        if( (!skydancers[i].simulation_diverged || skydancers[i].force_simulation) && h>0)
        {
            // Iterate over a fixed number of substeps between each frames
            const size_t number_of_substeps = 4;
            for(size_t k=0; (!skydancers[i].simulation_diverged  || skydancers[i].force_simulation) && k<number_of_substeps; ++k)
            {
                skydancers[i].compute_forces(i, timer);
                skydancers[i].numerical_integration(h);
                skydancers[i].collision_constraints();                 // Detect and solve collision with other shapes

                skydancers[i].hard_constraints();                      // Enforce hard positional constraints

                normal(skydancers[i].position.data, skydancers[i].connectivity, skydancers[i].normals); // Update normals of the cloth
                skydancers[i].detect_simulation_divergence(timer);               // Check if the simulation seems to diverge
            }
        }


        skydancers[i].cloth.update_position(skydancers[i].position.data);
        skydancers[i].cloth.update_normal(skydancers[i].normals.data);

       skydancers[i]. display_elements(shaders, scene, gui);
    }
    display_ground(scene, gui);

}

void scene_model::skydancer::numerical_integration(float h)
{
    const size_t NN = position.size();
    const float m = simulation_parameters.m;

    for(size_t k=0; k<NN; ++k)
    {
        vec3& p = position[k];
        vec3& v = speed[k];
        const vec3& f = force[k];

        v = v + h*f/m;
        p = p + h*v;
    }
}

void scene_model::skydancer::hard_constraints()
{
    // Fixed positions of the cloth
    for(const auto& constraints : positional_constraints)
        position[constraints.first] = constraints.second;
}


void scene_model::skydancer::display_elements(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    glEnable( GL_POLYGON_OFFSET_FILL );

    // Display cloth
    GLuint texture = cloth.texture_id;
    if(!gui_display_texture)
        texture = scene.texture_white;

    glPolygonOffset( 1.0, 1.0 );
    draw(cloth, scene.camera, cloth.shader, texture);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);

    if(gui_display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(cloth, scene.camera, shaders["wireframe_quads"]);
    }


    // Display positional constraint using spheres
    sphere.uniform.transform.scaling = 0.01f;
    sphere.uniform.color = {0,0,1};
    for(const auto& constraints : positional_constraints)  {
        sphere.uniform.transform.translation = constraints.second;
        draw(sphere, scene.camera, shaders["mesh"]);
    }

    sphere.uniform.color = {1,0,0};

}

void scene_model::display_ground(scene_structure& scene, gui_structure&)
{
    // Display ground
    draw(skydancers[0].ground, scene.camera);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
}


// Automatic detection of divergence: stop the simulation if detected
void scene_model::skydancer::detect_simulation_divergence(vcl::timer_event timer)
{
    const size_t NN = position.size();
    for(size_t k=0; simulation_diverged==false && k<NN; ++k)
    {
        const float f = norm(force[k]);
        const vec3& p = position[k];

        if( std::isnan(f) ) // detect NaN in force
        {
            std::cout<<"NaN detected in forces"<<std::endl;
            simulation_diverged = true;
        }

        if( f>1000.0f ) // detect strong force magnitude
        {
            std::cout<<" **** Warning : Strong force magnitude detected "<<f<<" at vertex "<<k<<" ****"<<std::endl;
            simulation_diverged = true;
        }

        if( std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) ) // detect NaN in position
        {
            std::cout<<"NaN detected in positions"<<std::endl;
            simulation_diverged = true;
        }

        if(simulation_diverged==true)
        {
            std::cerr<<" **** Simulation has diverged **** "<<std::endl;
            std::cerr<<" > Stop simulation iterations"<<std::endl;
            timer.stop();
        }
    }

}


void scene_model::set_gui()
{
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    size_t resolution_min = 3u;
    size_t resolution_max = 50u;
    ImGui::SliderScalar("Resolution", ImGuiDataType_U64, &skydancers[0].resolution, &resolution_min, &resolution_max);

    ImGui::SliderFloat("Stiffness", &skydancers[0].user_parameters.K, 1.0f, 3000.0f, "%.2f");
    ImGui::SliderFloat("Damping", &skydancers[0].user_parameters.mu, 0.0f, 0.1f, "%.3f");
    ImGui::SliderFloat("Mass", &skydancers[0].user_parameters.m, 1.0f, 15.0f, "%.2f");
    ImGui::SliderFloat("Wind", &skydancers[0].user_parameters.wind, 0.0f, 30.0f, "%.2f");
    ImGui::SliderFloat("Pressure", &skydancers[0].user_parameters.pressure, 0.0f, 30.0f, "%.2f");

    ImGui::Checkbox("Wireframe",&skydancers[0].gui_display_wireframe);
    ImGui::Checkbox("Texture",&skydancers[0].gui_display_texture);

    bool const stop  = ImGui::Button("Stop anim"); ImGui::SameLine();
    bool const start = ImGui::Button("Start anim");

    skydancers.resize(3);

    for(int i = 0; i < 3; i++)
    {

        if(stop) timer.stop();
        if(start) {
            if(skydancers[i].simulation_diverged )
                skydancers[i].force_simulation=true;
            timer.start();
        }

        if (skydancers[i].resolution != skydancers[i].old_resolution)
        {
            skydancers[i].old_resolution = skydancers[i].resolution;

            skydancers[i].initialize(i);
        }

        bool const restart = ImGui::Button("Restart");
        if(restart) skydancers[i].initialize(i);
    }
}









#endif
