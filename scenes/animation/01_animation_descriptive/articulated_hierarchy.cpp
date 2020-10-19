
#include "articulated_hierarchy.hpp"


#ifdef SCENE_ARTICULATED_HIERARCHY


using namespace vcl;



void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    const float radius_head = 0.20f;
    const float radius_body = 0.5f;
    const float radius_arm = 0.05f;
    const float length_arm = 0.2f;

    vec3 bird_color = {0.0f, 0.3f, 0.6f};

    mesh_drawable body = mesh_drawable(mesh_primitive_sphere(radius_body, {0,0,0}, 40, 40));
    body.uniform.transform.scaling_axis = vec3{0.6f, 0.4f, 1.0f};
    body.uniform.transform.scaling = 0.9f;
    body.uniform.color = bird_color;

    // The geometry of the body is a sphere
    mesh_drawable head = mesh_drawable(mesh_primitive_sphere(radius_head, {0,0,0}, 40, 40));
    head.uniform.color = bird_color;

    // Geometry of the eyes: black spheres
    mesh_drawable eye = mesh_drawable(mesh_primitive_sphere(0.045f, {0,0,0}, 20, 20));
    eye.uniform.color = {0,0,0};

    mesh_drawable beak = mesh_drawable(mesh_primitive_cone(0.07f,{0,0.0f,0},{0,0.3f,0}));
    beak.uniform.color = {1.0f, 0.5f, 0.2f};

    // Shoulder part and arm are displayed as cylinder
    mesh_drawable wing = mesh_primitive_quad({0.0f, 0, -0.2f}, {0.0f, 0, 0.2f}, {0.5f, 0, 0.3f}, {0.5f, 0, -0.3f});
    wing.uniform.transform.scaling = 0.9f;
    mesh_drawable outer_wing = mesh_primitive_quad({0.0f, 0, -0.27f}, {0.0f, 0, 0.27f}, {0.5f, 0, 0.1f}, {0.5f, 0, -0.1f});
    wing.uniform.color = bird_color;
    outer_wing.uniform.color = bird_color;

    // Build the hierarchy:
    // Syntax to add element
    //   hierarchy.add(visual_element, element_name, parent_name, (opt)[translation, rotation])
    hierarchy.add(body, "body");

    hierarchy.add(head, "head", "body", radius_body * vec3(0, 1/2.0f, 1/1.2f));

    // Eyes positions are set with respect to some ratio of the
    hierarchy.add(eye, "eye_left", "head" , radius_head * vec3( 1/3.0f, 1/2.0f, 1/1.5f));
    hierarchy.add(eye, "eye_right", "head", radius_head * vec3(-1/3.0f, 1/2.0f, 1/1.5f));

    const mat3 beak_rotation = rotation_from_axis_angle_mat3({1, 0, 0}, 3.14f / 1.7f);
    // Beak position is set with respect to a ratio of the body
    hierarchy.add(beak, "beak", "head", affine_transform(radius_head * vec3(0.0f, 0.0f, 1/2.0f), beak_rotation));

    // Set the left part of the body arm: shoulder-elbow-arm
    hierarchy.add(wing, "wing_left", "body", {0.2f,0,0});
    hierarchy.add(outer_wing, "outer_wing_left", "wing_left", {0.45f, 0, 0});

    // Set the right part of the body arm: similar to the left part excepted a symmetry is applied along x direction for the shoulder
    hierarchy.add(wing, "wing_right", "body",     {{-0.2f,0,0}, {-1,0,0, 0,1,0, 0,0,1}/*Symmetry*/ } );
    hierarchy.add(outer_wing, "outer_wing_right", "wing_right",{0.45f, 0, 0});


    // Set the same shader for all the elements
    hierarchy.set_shader_for_all_elements(shaders["mesh"]);



    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);

    timer.scale = 0.5f;
}




void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();

    // Current time
    const float t = timer.t;

    /** *************************************************************  **/
    /** Compute the (animated) transformations applied to the elements **/
    /** *************************************************************  **/

    // The body oscillate along the z direction
    //hierarchy["body"].transform.translation = {0,0,0.2f*(1+std::sin(2*3.14f*t))};

    // Rotation of the head around the x axis
    mat3 const R_head = rotation_from_axis_angle_mat3({1,0,0}, 0.4f * std::sin(2 * 3.14f*(t-0.4f)) );
    // Rotation of the wing around the y axis
    mat3 const R_wing = rotation_from_axis_angle_mat3({0,0,1}, 0.7f * std::sin(2*3.14f*(t-0.6f)) );
    mat3 const R_outer_wing = rotation_from_axis_angle_mat3({0,0,1}, 1.0f * std::sin(2*3.14f*(t-0.6f)) );
    // Symmetry in the x-direction between the left/right parts
    mat3 const Symmetry = {-1,0,0, 0,1,0, 0,0,1};

    // Set the rotation to the elements in the hierarchy
    hierarchy["head"].transform.rotation = R_head;

    hierarchy["wing_left"].transform.rotation = R_wing;
    hierarchy["outer_wing_left"].transform.rotation = R_outer_wing;

    hierarchy["wing_right"].transform.rotation = Symmetry * R_wing; //note that the symmetry is already applied by the parent element
    hierarchy["outer_wing_right"].transform.rotation = R_outer_wing; //note that the symmetry is already applied by the parent element

    hierarchy.update_local_to_global_coordinates();


    /** ********************* **/
    /** Display the hierarchy **/
    /** ********************* **/

    if(gui_scene.surface) // The default display
        draw(hierarchy, scene.camera);

    if(gui_scene.wireframe) // Display the hierarchy as wireframe
        draw(hierarchy, scene.camera, shaders["wireframe"]);

    if(gui_scene.skeleton) // Display the skeleton of the hierarchy (debug)
        hierarchy_visual_debug.draw(hierarchy, scene.camera);

}


void scene_model::set_gui()
{
    ImGui::Text("Display: "); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe); ImGui::SameLine();
    ImGui::Checkbox("Surface", &gui_scene.surface);     ImGui::SameLine();
    ImGui::Checkbox("Skeleton", &gui_scene.skeleton);   ImGui::SameLine();

    ImGui::Spacing();
    ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max);
    ImGui::SliderFloat("Time scale", &timer.scale, 0.1f, 3.0f);

}





#endif

