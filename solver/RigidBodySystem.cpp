#include "RigidBodySystem.h"

#include <iostream>
#include <chrono>
#include <assert.h>

namespace ab_solver {

	RigidBodySystem::RigidBodySystem() {
		m_odeSolver = nullptr;
	}

	RigidBodySystem::~RigidBodySystem() {
		m_state.Destroy();
	}

	void RigidBodySystem::Initialize(OdeSolver *odeSolver) {
		m_odeSolver = odeSolver;
	}

	void RigidBodySystem::Reset() {
		m_bodies.clear();
	}

	void RigidBodySystem::Process(double dt, int steps) {
		long long
			odeSolveTime = 0,
			constraintSolveTime = 0,
			forceEvalTime = 0,
			constraintEvalTime = 0;

		PopulateSystemState(); // shouldn't need to do this every frame...

		for (int i = 0; i < steps; ++i) {
			m_odeSolver->Start(&m_state, dt / steps);

			while (true) {
				const bool done = m_odeSolver->Step(&m_state);

				long long evalTime = 0, solveTime = 0;

				auto s0 = std::chrono::steady_clock::now();
				//processForces();
				auto s1 = std::chrono::steady_clock::now();

				//processConstraints(&evalTime, &solveTime);

				auto s2 = std::chrono::steady_clock::now();
				m_odeSolver->Solve(&m_state);
				auto s3 = std::chrono::steady_clock::now();

				constraintSolveTime += solveTime;
				constraintEvalTime += evalTime;
				odeSolveTime +=
					std::chrono::duration_cast<std::chrono::microseconds>(s3 - s2).count();
				forceEvalTime +=
					std::chrono::duration_cast<std::chrono::microseconds>(s1 - s0).count();

				if (done) break;
			}

			m_odeSolver->End();
		}

		const int n_bodies = GetRigidBodyCount();
		for (int i = 0; i < n_bodies; ++i) {
			int offset = i*m_state.NUM_STATE_VARS;
			m_bodies[i]->pos_x = m_state.translational[offset+0];
			m_bodies[i]->pos_y = m_state.translational[offset+1];
			m_bodies[i]->pos_z = m_state.translational[offset+2];

			m_bodies[i]->vel_x = m_state.translational[offset+3];
			m_bodies[i]->vel_y = m_state.translational[offset+4];
			m_bodies[i]->vel_z = m_state.translational[offset+5];
		}

		//m_odeSolveMicroseconds[m_frameIndex] = odeSolveTime;
		//m_constraintSolveMicroseconds[m_frameIndex] = constraintSolveTime;
		//m_forceEvalMicroseconds[m_frameIndex] = forceEvalTime;
		//m_constraintEvalMicroseconds[m_frameIndex] = constraintEvalTime;
		//m_frameIndex = (m_frameIndex + 1) % ProfilingSamples;
	}

	void RigidBodySystem::AddRigidBody(RigidBody *body) {
		m_bodies.push_back(body);
    	body->index = (int)m_bodies.size() - 1;
		printf("Adding body!\n");
	}

	void RigidBodySystem::RemoveRigidBody(RigidBody *body) {
		printf("Removing body!\n");
		m_bodies[body->index] = m_bodies.back();
		m_bodies[body->index]->index = body->index;
		m_bodies.resize(m_bodies.size() - 1);
	}

	RigidBody* RigidBodySystem::GetRigidBody(int i) {
		assert(i < m_bodies.size());
    	return m_bodies[i];
	}

	void RigidBodySystem::PopulateSystemState() {
		const size_t n_bodies = m_bodies.size();

		m_state.Resize(n_bodies);

		for (size_t n = 0; n < n_bodies; n++) {
			size_t body_start = n*m_state.NUM_STATE_VARS;

			m_state.translational[body_start+0] = m_bodies[n]->pos_x;
			m_state.translational[body_start+1] = m_bodies[n]->pos_y;
			m_state.translational[body_start+2] = m_bodies[n]->pos_z;

			m_state.translational[body_start+3] = m_bodies[n]->vel_x;
			m_state.translational[body_start+4] = m_bodies[n]->vel_y;
			m_state.translational[body_start+5] = m_bodies[n]->vel_z;

			m_state.translational[body_start+6] = 0;
			m_state.translational[body_start+7] = 0;
			m_state.translational[body_start+8] = 0;

			m_state.translational[body_start+9] = m_bodies[n]->mass;
		}
	}

}