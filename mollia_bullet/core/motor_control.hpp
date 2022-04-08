#pragma once

#include "common.hpp"

struct BIWorld;

struct BIMotorData {
	btScalar max_impulse;
	btScalar target_velocity;
};

struct BIMotorControl : public BIBaseObject {
	BIWorld * world;

	PyObject * motors_slot;
	PyObject * motors;

	PyObject * output_mem[2];
	PyObject * output_array[2];
	BIMotorData * input_data;
	int output_index;
};

extern PyTypeObject * BIMotorControl_type;
extern PyType_Spec BIMotorControl_spec;

PyObject * BIWorld_meth_motor_control(BIWorld * self, PyObject * args);
PyObject * BIMotorControl_meth_remove(BIMotorControl * self);
