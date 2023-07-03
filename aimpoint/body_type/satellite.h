#include "physics.h"

#include "planet.h"

struct satellite_body : public simulation_body {
    void set_orbit_circ(planet* p, double lat, double lon, double alt, double inc);

    virtual laml::Vec3_highp force_func(const rigid_body_state* at_state, double t) override;

    planet* grav_body;
};