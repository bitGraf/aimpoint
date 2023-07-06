# Aimpoint - 6DOF/Orbital propogator sandbox
![Alt text](docs/anim.gif?raw=true)

## Orbital Propogation

Create an orbit from state vectors ($\bar{r}, \bar{v}, t_0$) or Keplerian elements ($a, e, i, \Omega, \omega, M_0, t_0$ ) and propogate assuming constant mean motion. Extract state vectors and render in 3D using OpenGL.

Alternatively, create a 6DOF simulation body with the same initial states as an orbit, and apply gravity as a forcing function, and propagate the body using any of the built-in numerical integrators listed below.

Ground tracks can also be rendered from either case.

Current scenario being modeled plots the orbits of two satellites using the same initial conditions. The Red dot is propagated using constant orbital elements. The grey dot is propagated using numerical integration and a gravity forcing function applying the J2 perturbation term. Over time the two orbits diverge, as can be seen by the orbital plane and angular momentum vector.

![Alt text](docs/scenario.png?raw=true)

##### Controls: 
* Drag right click to rotate view.
* [K] to toggle Keplerian Elements window
* [A] to toggle Anomalies window (with keplerian window visible)
* [G] to toggle Ground Tracks window
* [P] to toggle drawing orbital plane and $\hat{h}$ vector
* [Spacebar] to toggle speed b/w realtime and uncapped
* [Esc] to end the sim

## Other Demos
More demos can be found in the [demos](demos/readme.md) directory.

## Generic 6-DoF simulation
Pretty basic 6 Degrees-of-Freedom rigid body simulation code (i.e. both rotational and translational motion)

Define initial state and the force and moment functions for a given body and the code will perform fixed-step numerial integration to calculate future states.

Currently implements the following fixed-timestep integration schemes:

| Scheme                  | Order | MATLAB Equivalent Solver |
|-------------------------|-------|--------------------------|
| Euler                   |  1st  | ode1                     |
| Heun                    |  2nd  | ode2                     |
| Ralston                 |  2nd  |                          |
| Bogacki-Shampine        |  3rd  | ode3                     |
| Runge-Kutta             |  4th  | ode4                     |
| Runge-Kutta (3/8 Rule)  |  4th  |                          |
| Dormand-Prince          |  5th  | ode5                     |

### Future areas of interest
* Variable timestep integration schemes
* Separate integration scheme for rotation (i.e. Something from [this paper](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/accuraterotation/paper.pdf))
* More visualization tools (plots/readouts of energy, 3D visualizations of state quantities)
* Expand simulation states to include mass and inertia for non-constant mass systems.

### Build Instructions:
Clone the repository:
```
git clone --recursive https://github.com/bitGraf/aimpoint.git
```

Inside the respository:
```
mkdir build
cd build
cmake ..
```

Either `cmake --build .` inside the build directory, or use whatever build environment you have cmake configured for.

---
To enable video outputs from the sim, run cmake with additional flags to include the [DTV](https://github.com/ange-yaghi/direct-to-video) library.
```
cmake .. -DUSE_DTV_LIB="ON" -DUSE_DTV="ON"
```
`USE_DTV_LIB` Brings the library into the project
`USE_DTV` To actually enabl video capture

Note: DTV requires FFmpeg development libraries to be on the system path.