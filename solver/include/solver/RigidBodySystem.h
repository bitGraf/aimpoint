#pragma once

#include <vector>

#include "SystemState.h"
#include "RigidBody.h"
#include "OdeSolver.h"

namespace ab_solver {

	class RigidBodySystem {
	public:
		RigidBodySystem();
		~RigidBodySystem();

		void Reset();
		void Process(double dt, int steps = 1);
		void Initialize(OdeSolver *odeSolver);

		void AddRigidBody(RigidBody *body);
		void RemoveRigidBody(RigidBody *body);
		RigidBody *GetRigidBody(int i);

		int GetRigidBodyCount() const {return m_bodies.size();}

	private:
		void PopulateSystemState();

	private:
		OdeSolver *m_odeSolver;

		std::vector<RigidBody*> m_bodies;

		SystemState m_state;
	};
}