#include "random_forest.hpp"

#include <random>

#ifdef SCENE_RANDOM_FOREST

// Add vcl namespace within the current one - Allows to use function from vcl library without explicitely preceeding their name with vcl::
using namespace vcl;


/** This function is called before the beginning of the animation loop
    It is used to declare and initialize data that will be used later during display */
void scene_model::setup_data(std::map<std::string, GLuint>& shaders, scene_structure&, gui_structure& gui)
{

    const mesh mesh_plane = mesh_primitive_quad({ -2,0,-2 }, { -2,0,2 }, { 2,0,2 }, { 2,0,-2 });
    // Convert the mesh structure into object that can be displayed (mesh_drawable)
    plane = mesh_drawable(mesh_plane); // note that plane is an attribute of the class (declared in .hpp file)
    plane.shader = shaders["mesh"];    // A default shader can be set as an attribute of the object (used when calling the draw function)

    cone = mesh_drawable(mesh_primitive_cone(0.1f,{0,0,0},{0,0.2f,0}));
    cone.uniform.color = {0.0f, 1.0f, 0.0f};
    cone.shader = shaders["mesh"];

    cylinder = mesh_drawable(mesh_primitive_cylinder(0.05f,{0,0,0},{0,0.1f,0}));
    cylinder.uniform.color = {0.6f, 0.4f, 0.2f};
    cylinder.shader = shaders["mesh"];

    const int N_cone = 100;
    positions.resize(N_cone);
    for(int k=0; k<N_cone; ++k)
    {
        bool is_valid_point = false;
        vec3 point;
        while (!is_valid_point)
        {
            float x = vcl::rand_interval(-2,2);
            float z = vcl::rand_interval(-2,2);

            point = {x, 0, z};

            is_valid_point = true;
            for (int i = 0; i < k; i++)
            {
               if (vcl::norm(positions[i] - point) < 0.2f) {
                   is_valid_point = false;
                   break;
               } 
            }
        }
        positions[k] = point;
    }

    // allow by default the display of helper visual frames
    gui.show_frame_worldspace = true;
    gui.show_frame_camera = true;

}



/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_model::frame_draw(std::map<std::string, GLuint>& shaders, scene_structure& scene, gui_structure&)
{

    // ********************************************* //
    // Update timer and GUI
    // ********************************************* //

    // Update the GUI with the timer passed as parameter
    set_gui();

    // ********************************************* //
    // Display objects
    // ********************************************* //

    // ********************************************* //
    // Display plane
    // ********************************************* //

    // the general syntax to display a mesh is:
    //   draw(objectName, camera [,optional: shaderID ]);
    draw(plane, scene.camera);

    const int N = positions.size();
    for(int k=0; k<N; ++k)
    {
        float u = k/(N-1);
        cone.uniform.transform.translation = positions[k] + vec3{0.0f, 0.1f, 0.0f};
        cylinder.uniform.transform.translation = positions[k];
        draw(cone, scene.camera);
        draw(cylinder, scene.camera);
    }



}

/** Update the visual GUI */
void scene_model::set_gui()
{
}

#endif


