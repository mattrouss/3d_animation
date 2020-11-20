# 3D Animation

Simulation of a *skydancer* using a cloth simulation in C++. This project was performed for the 3D Animation class in IMAGE EPITA.
The application uses the **VCL** library written by @drohmer (https://github.com/drohmer/epita_image_animation3d_vcl).

## How to run the project

To run the 3d_animation project, run the following commands in the root dir after having cloned the repo:
```
mkdir build && cd build
cmake .. && make
./pgm
```

There you go, you should have the *skydancer* scene running!

## What is a skydancer ?

![skydancers!](https://i.imgur.com/2pMqoF3.png)

A *skydancer* is a publicitary balloon with a humanoid shape, which is blown up and performs chaotic movements in the sky.
This seemed like a nice twist to the cloth sim problem, so we tried simulating this ourselves!

## General Idea

The general idea behind building the skydancer is to use a cloth simulation, and to put it in a cylinder shape.
We can then attach the bottom ring of the cylinder to the ground, and apply some wind force for it to stay upright. Adding more periodical horizontal winds gives it a more chaotic behaviour that we all love!

## Cloth Simulation

The cloth simulation was implemented using a classic **mass-spring** modelisation, where the cloth is divided in a uniform set of particles. Each particle interacts with its neighbours with different types of springs: *structural, shearing and bending*. These interactions give the structure and behaviour of a regular cloth, once we tweak the parameters. To this we can add collision with the ground and geometrical objects such as spheres, and apply a wind force to the particles.


## Skydancer

After rolling the cloth into a tube, we're almost good to go ! Adding a pressure force in the direction of the cloth normal gives the dancer a more *inflated* look to it.
