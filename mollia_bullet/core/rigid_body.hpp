#pragma once

#include "common.hpp"

struct BIWorld;

struct BIRigidBody : public BIBaseObject {
	BIWorld * world;
	btRigidBody * body;
	PyObject * groups_slot;
	PyObject * constraints_slot;
};

extern PyTypeObject * BIRigidBody_type;
extern PyType_Spec BIRigidBody_spec;

PyObject * BIWorld_meth_rigid_body(BIWorld * self, PyObject * args);
PyObject * BIRigidBody_meth_remove(BIRigidBody * self);
