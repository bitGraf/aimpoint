hiyaa =^.^=

## 6-DoF simulation
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