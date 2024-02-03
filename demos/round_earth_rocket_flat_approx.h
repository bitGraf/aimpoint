#include "physics.h"

#include "planet.h"

// assuming flat earth/NED coords
struct rocket_round_earth_flat_ltg : public simulation_body {
    rocket_round_earth_flat_ltg();

    void launch(planet* grav_body, double orbit_height, double Tgo_guess);

    virtual void major_step(double t, double dt) override;
    virtual laml::Vec3_highp force_func(const rigid_body_state* at_state, double t) override;
    //virtual laml::Vec3_highp moment_func(const rigid_body_state& at_state, double t) override;

    // desired final conditions
    double yf, vxf, vyf;
    
    // steering law
    double T, A, B, alpha, Tgo, t0;

    //
    uint64 update_step_num;
    double major_loop_rate;

    // rocket params
    double thrust, mdot;

    // unused for now
    double tof;
    uint32 stage;

    planet* body;
};