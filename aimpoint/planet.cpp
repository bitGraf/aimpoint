#include "planet.h"

planet::planet() : mat_inertial_to_fixed(1.0f) {
    //rotation_rate *= .01*86400;
    //gm = 1.0;
    //eccentricity_sq = 0;
    //eccentricity_sq = 0.5; // doenst work
    //eccentricity_sq = 0.7071; // does work?? confusion...

    mat_inertial_to_fixed = laml::Mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);
    mat_fixed_to_inertial = laml::transpose(mat_inertial_to_fixed);
}

void planet::load_mesh() {
    double polar_radius = equatorial_radius * sqrt(1 - eccentricity_sq);
    //mesh.load_from_mesh_file("../data/unit_sphere.mesh", equatorial_radius);
    mesh.load_from_mesh_file("../data/unit_sphere.mesh", equatorial_radius, equatorial_radius, polar_radius);
    //mesh.load_from_mesh_file("../data/blahaj.mesh", equatorial_radius*0.02f);

    diffuse.load_texture_file("../data/earth.jpg");
}

void planet::update(double t, double dt) {
    yaw += rotation_rate * dt;

    const double d360 = 360.0 * laml::constants::deg2rad<double>;
    while (yaw > d360)
        yaw -= d360;

    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);

    //inertial_to_fixed = laml::Mat3(cy, -sy, 0, 0, 0, 1, sy, cy, 0);
    //mat_inertial_to_fixed = laml::Mat3(cy, sy, 0, -sy, cy, 0, 0, 0, 1);
    mat_inertial_to_fixed = laml::Mat3(cy, -sy, 0, sy, cy, 0, 0, 0, 1);
    mat_fixed_to_inertial = laml::transpose(mat_inertial_to_fixed);
}

vec3d planet::lla_to_fixed(double lat, double lon, double alt) {
    double slat = laml::sind(lat);
    double clat = laml::cosd(lat);

    double slon = laml::sind(lon);
    double clon = laml::cosd(lon);

    double N = equatorial_radius / sqrt(1.0 - eccentricity_sq*slat*slat);

    return vec3d(
        (N + alt)*clat*clon,
        (N + alt)*clat*slon,
        ((1 - eccentricity_sq)*N + alt)*slat);
}
void planet::fixed_to_lla(vec3d pos_fixed, double *lat_out, double *lon_out, double *alt_out) {
    // Bowring's method
    double X = pos_fixed.x;
    double Y = pos_fixed.y;
    double Z = pos_fixed.z;

    *lon_out = laml::atan2d(Y, X);
    double s = sqrt(X*X + Y*Y);
    double f = 1 - sqrt(1 - eccentricity_sq);

    // initial guess
    double red_lat = laml::atand(Z / ((1-f)*2));
    double c_red_lat_3 = laml::cosd(red_lat); c_red_lat_3 = c_red_lat_3*c_red_lat_3*c_red_lat_3;
    double s_red_lat_3 = laml::sind(red_lat); s_red_lat_3 = s_red_lat_3*s_red_lat_3*s_red_lat_3;
    double geo_lat = laml::atand((Z + eccentricity_sq*(1-f)*equatorial_radius*(s_red_lat_3)/(1 - eccentricity_sq)) / (s - eccentricity_sq*equatorial_radius*(c_red_lat_3)));

    double error = 1e5;
    uint32 iterations = 0;
    const double tol = 1e-9;
    const uint32 max_iterations = 5;
    while (abs(error) > tol && iterations < max_iterations) {
        red_lat = laml::atand((1-f)*laml::sind(geo_lat) / laml::cosd(geo_lat));
        c_red_lat_3 = laml::cosd(red_lat); c_red_lat_3 = c_red_lat_3*c_red_lat_3*c_red_lat_3;
        s_red_lat_3 = laml::sind(red_lat); s_red_lat_3 = s_red_lat_3*s_red_lat_3*s_red_lat_3;
        double new_geo_lat = laml::atand((Z + eccentricity_sq*(1-f)*equatorial_radius*(s_red_lat_3)/(1 - eccentricity_sq)) / (s - eccentricity_sq*equatorial_radius*(c_red_lat_3)));
        error = geo_lat - new_geo_lat;
        geo_lat = new_geo_lat;
        iterations++;
    }
    *lat_out = geo_lat;
    
    double s_lat = laml::sind(geo_lat);
    double N = equatorial_radius / sqrt(1 - eccentricity_sq*(s_lat*s_lat));
    *alt_out = s*laml::cosd(geo_lat) + (Z + eccentricity_sq*N*laml::sind(geo_lat))*laml::sind(geo_lat) - N;
}

vec3d planet::fixed_to_inertial(vec3d pos_fixed) {
    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);
    return vec3d(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
}
vec3d planet::fixed_to_inertial(vec3d pos_fixed, double t) {
    double cy = laml::cos(rotation_rate * t);
    double sy = laml::sin(rotation_rate * t);
    return vec3d(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
}
void planet::fixed_to_inertial(vec3d pos_fixed, vec3d vel_fixed, vec3d* pos_inertial, vec3d* vel_inertial) {
    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);

    vec3d pos(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
    vec3d vel(cy*vel_fixed.x - sy*vel_fixed.y, sy*vel_fixed.x + cy*vel_fixed.y, vel_fixed.z);

    vec3d omega(0.0, 0.0, rotation_rate);
    double r = laml::length(pos);

    vel = laml::cross(omega, pos)/r + vel;

    *pos_inertial = pos;
    *vel_inertial = vel;
}
void planet::fixed_to_inertial(vec3d pos_fixed, vec3d vel_fixed, double t, vec3d* pos_inertial, vec3d* vel_inertial) {
    double cy = laml::cos(rotation_rate * t);
    double sy = laml::sin(rotation_rate * t);

    vec3d pos(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
    vec3d vel(cy*vel_fixed.x - sy*vel_fixed.y, sy*vel_fixed.x + cy*vel_fixed.y, vel_fixed.z);

    vec3d omega(0.0, 0.0, rotation_rate);
    double r = laml::length(pos);

    vel = laml::cross(pos, omega)/r + vel;

    *pos_inertial = pos;
    *vel_inertial = vel;
}


vec3d planet::inertial_to_fixed(vec3d pos_inertial){
    double cy =  laml::cos(yaw);
    double sy = -laml::sin(yaw);
    return vec3d(cy*pos_inertial.x - sy*pos_inertial.y, sy*pos_inertial.x + cy*pos_inertial.y, pos_inertial.z);
}
vec3d planet::inertial_to_fixed(vec3d pos_inertial, double t){
    double cy =  laml::cos(rotation_rate * t);
    double sy = -laml::sin(rotation_rate * t);
    return vec3d(cy*pos_inertial.x - sy*pos_inertial.y, sy*pos_inertial.x + cy*pos_inertial.y, pos_inertial.z);
}
void  planet::inertial_to_fixed(vec3d pos_inertial, vec3d vel_inertial, vec3d* pos_fixed, vec3d* vel_fixed){
    double cy =  laml::cos(yaw);
    double sy = -laml::sin(yaw);

    vec3d pos(cy*pos_inertial.x - sy*pos_inertial.y, sy*pos_inertial.x + cy*pos_inertial.y, pos_inertial.z);
    vec3d vel(cy*vel_inertial.x - sy*vel_inertial.y, sy*vel_inertial.x + cy*vel_inertial.y, vel_inertial.z);

    vec3d omega(0.0, 0.0, rotation_rate);
    double r = laml::length(pos);

    vel = vel - laml::cross(omega, pos)/r;

    *pos_fixed = pos;
    *vel_fixed = vel;
}
void planet::inertial_to_fixed(vec3d pos_inertial, vec3d vel_inertial, double t, vec3d* pos_fixed, vec3d* vel_fixed) {
    double cy =  laml::cos(rotation_rate*t);
    double sy = -laml::sin(rotation_rate*t);

    vec3d pos(cy*pos_inertial.x - sy*pos_inertial.y, sy*pos_inertial.x + cy*pos_inertial.y, pos_inertial.z);
    vec3d vel(cy*vel_inertial.x - sy*vel_inertial.y, sy*vel_inertial.x + cy*vel_inertial.y, vel_inertial.z);

    vec3d omega(0.0, 0.0, rotation_rate);
    double r = laml::length(pos);

    vel = vel - laml::cross(omega, pos)/r;

    *pos_fixed = pos;
    *vel_fixed = vel;
}


vec3d planet::gravity(vec3d pos_inertial) {
    double r_mag = laml::length(pos_inertial);
    vec3d r_unit = pos_inertial / r_mag;

    return -gm * r_unit / (r_mag*r_mag);
}

vec3d planet::gravity_J2(vec3d pos_inertial) {
    double r_mag = laml::length(pos_inertial);
    vec3d r_unit = pos_inertial / r_mag;

    const double x = pos_inertial.x;
    const double y = pos_inertial.y;
    const double z = pos_inertial.z;
    const double r_7 = r_mag*r_mag*r_mag*r_mag*r_mag*r_mag*r_mag;

    double F_j2_x = J2*x*(6*z*z - 1.5*(x*x + y*y)) / r_7;
    double F_j2_y = J2*y*(6*z*z - 1.5*(x*x + y*y)) / r_7;
    double F_j2_z = J2*z*(3*z*z - 4.5*(x*x + y*y)) / r_7;

    vec3d f_grav = -gm * r_unit / (r_mag*r_mag);

    return f_grav + vec3d(F_j2_x, F_j2_y, F_j2_z);
}

mat3d planet::create_local_inertial(double lat, double lon, double az) {
    double slat = laml::sind(lat);
    double clat = laml::cosd(lat);

    double slon = laml::sind(lon);
    double clon = laml::cosd(lon);

    double saz = laml::sind(az);
    double caz = laml::cosd(az);

    // 2 step rotation: 3-2 rotation (lon-lat)
    mat3d ECI2NED(
        -clon*slat, -slon, -clat*clon,
        -slat*slon,  clon, -clat*slon,
         clat,       0,    -slat);
    mat3d NED2LCF(caz, -saz, 0,
                  saz, caz, 0,
                  0, 0, 1);

    mat3d LCI2ECI = laml::mul(laml::transpose(ECI2NED), laml::transpose(NED2LCF));

    return LCI2ECI;
}