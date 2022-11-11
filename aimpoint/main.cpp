#include "solver/RigidBodySystem.h"

#include <stdio.h>

int main(int argc, char** argv) {
	using namespace ab_solver;

	RigidBodySystem rbs;

	RigidBody* body1 = new RigidBody();
	body1->pos_y = 10.0;
	body1->vel_y = 5.0;
	body1->mass = 3.0;
	RigidBody* body2 = new RigidBody();
	body2->pos_x = -10.0;
	body2->pos_y = 0;
	body2->vel_x = 5.0;
	body2->vel_y = 5.0;
	RigidBody* body3 = new RigidBody();
	
	rbs.AddRigidBody(body1);
	rbs.AddRigidBody(body2);
	rbs.AddRigidBody(body3);

	OdeSolver* euler_solver = new OdeSolver();

	rbs.Initialize(euler_solver);
	rbs.Process(1/60.0, 1);
	rbs.Reset();

	delete euler_solver;

	delete body1;
	delete body2;
	delete body3;

	printf("Done.\n");
	
	return 0;
}

//#include "Core/Application.h"
//#include "Core/Timing/Clock.h"
//
//int main()
//{
//    aimpoint::Application app;
//    app.Run();
//
//    return 0;
//}