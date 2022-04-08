#pragma once

#include "common.hpp"

struct BIGroup;

struct BIWorld : public BIBaseObject {
	// Bullet objects
	btDefaultCollisionConfiguration * collision_configuration;
	btCollisionDispatcher * dispatcher;
	btDbvtBroadphase * broadphase;
	btMultiBodyConstraintSolver * solver;
	btMultiBodyDynamicsWorld * dynamics_world;

	// Weakrefs
	PyObject * names_slot;
	PyObject * groups_slot;
	PyObject * motor_controls_slot;
	PyObject * constraints_slot;
	PyObject * updaters_slot;

	PyObject * motor_controls;

	BIGroup * main_group;

	// Data
	btScalar time_step;
	int iterations;
};

extern PyTypeObject * BIWorld_type;
extern PyType_Spec BIWorld_spec;

PyObject * meth_world(PyObject * self, PyObject * args);
