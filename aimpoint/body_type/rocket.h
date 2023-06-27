#include "physics.h"

#include "planet.h"

struct rocket : public simulation_body {
    rocket();

    void launch(planet* p);
    virtual void major_step(double t, double dt) override;

    virtual laml::Vec3_highp force_func(const rigid_body_state* at_state, double t) override;
    //virtual laml::Vec3_highp moment_func(const rigid_body_state& at_state, double t) override;

    double tof;
    uint32 stage;
    double pitch;

    double launch_lat, launch_lon, launch_az;
    mat3d ECI2LCI, LCI2ECI;

    planet* earth;
};