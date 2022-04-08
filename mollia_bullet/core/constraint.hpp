#pragma once

#include "common.hpp"

struct BIMotorControl;
struct BIRigidBody;
struct BIWorld;

struct BIConstraint : public BIBaseObject {
	BIWorld * world;
	union {
		btTypedConstraint * constraint;
		btHingeConstraint * hinge;
		btFixedConstraint * fixed;
		btGeneric6DofSpring2Constraint * sixdof;
		btSliderConstraint * slider;
		btPoint2PointConstraint * point_to_point;
		btSliderConstraint * cone_twist;
		btGearConstraint * gear;
	};
	BIMotorControl * motor_control;
	BIRigidBody * body_a;
	BIRigidBody * body_b;
};

extern PyTypeObject * BIConstraint_type;
extern PyType_Spec BIConstraint_spec;

PyObject * BIWorld_meth_constraint(BIWorld * self, PyObject * args);
PyObject * BIConstraint_meth_remove(BIConstraint * self);
