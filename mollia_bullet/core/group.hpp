#pragma once

#include "common.hpp"

struct BIWorld;

struct BIGroup : public BIBaseObject {
	BIWorld * world;

	PyObject * bodies_slot;
	PyObject * bodies;
};

extern PyTypeObject * BIGroup_type;
extern PyType_Spec BIGroup_spec;

PyObject * BIWorld_meth_group(BIWorld * self, PyObject * args);
PyObject * BIGroup_meth_remove(BIGroup * self);
