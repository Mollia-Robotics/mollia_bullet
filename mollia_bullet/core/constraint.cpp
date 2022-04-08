#include "constraint.hpp"

#include "motor_control.hpp"
#include "rigid_body.hpp"
#include "world.hpp"

PyObject * BIWorld_meth_constraint(BIWorld * self, PyObject * args) {
	/*
		Return a bullet Constraint.
		The return value is a python Hinge, Fixed, ... object with an _obj propery referencing the internal BIConstraint object.
		The user must not use the _obj directly.
	*/

	// parameters
	PyObject * body_a;
	PyObject * body_b;
	PyObject * constraint_args;
	PyObject * ref;
	int collision;
	int iterations;
	PyObject * name;

	// parse parameters
	if (!PyArg_ParseTuple(args, "OOOOpiO", &body_a, &body_b, &constraint_args, &ref, &collision, &iterations, &name)) {
		return 0;
	}

	btTransform base_transform;

	// calculate base_transform based on ref
	if (ref != Py_None) {
		// take the internal BIRigidBody
		BIRigidBody * ref_rbody = get_slot(ref, BIRigidBody, "_obj");
		assert(ref_rbody->ob_base.ob_type != BIRigidBody_type);
		base_transform = ref_rbody->body->getWorldTransform();
	} else {
		base_transform = default_transform;
	}

	// create a new internal object
	BIConstraint * constraint = PyObject_New(BIConstraint, BIConstraint_type);

	// fill the internal object
	constraint->constraint = 0;
	constraint->motor_control = 0;
	constraint->body_a = get_slot(body_a, BIRigidBody, "_obj");
	constraint->body_b = get_slot(body_b, BIRigidBody, "_obj");

	// constraint specific arguments
	constraint_args = PySequence_Fast(constraint_args, "not iterable");

	// constraint_args can be:
	//   ("old_hinge", pivot, axis)
	//   ("fixed", pivot)
	//   ...

	bi_ensure(constraint->body_a && constraint->body_b && constraint_args);

	int num_cargs = (int)PySequence_Fast_GET_SIZE(constraint_args);
	PyObject ** cargs = PySequence_Fast_ITEMS(constraint_args);
	bi_ensure(num_cargs >= 1 && PyUnicode_CheckExact(cargs[0]));

	// old_hinge from pivot and axis
	if (!PyUnicode_CompareWithASCIIString(cargs[0], "old_hinge")) {
		// this hinge was kept for backward-compatibility

		// number or arguments including the constraint type
		bi_ensure(num_cargs == 3);

		PyObject * pivot = cargs[1];
		PyObject * axis = cargs[2];

		const btTransform & transform_A = constraint->body_a->body->getCenterOfMassTransform();
		const btTransform & transform_B = constraint->body_b->body->getCenterOfMassTransform();

		btVector3 constraint_pivot;
		btVector3 constraint_axis;

		if (pivot == Py_None) {
			constraint_pivot = transform_A.getOrigin();
		} else {
			constraint_pivot = get_vector(pivot);
		}

		if (axis == Py_None) {
			btVector3 top = (transform_A.getOrigin() - constraint_pivot).normalized();
			btVector3 toc = (transform_B.getOrigin() - constraint_pivot).normalized();
			constraint_axis = top.cross(toc);
		} else {
			constraint_axis = get_vector(axis);
		}

		if (PyErr_Occurred()) {
			return 0;
		}

		constraint_axis.normalize();

		constraint_pivot = base_transform * constraint_pivot;
		constraint_axis = base_transform.getBasis() * constraint_axis;

		btVector3 pivot_A = transform_A.getBasis().solve33(constraint_pivot - transform_A.getOrigin());
		btVector3 pivot_B = transform_B.getBasis().solve33(constraint_pivot - transform_B.getOrigin());

		btVector3 axis_A = transform_A.getBasis().solve33(constraint_axis);
		btVector3 axis_B = transform_B.getBasis().solve33(constraint_axis);

		// wrap old_hinge with a python Hinge
		static PyTypeObject * wrapper_type = get_wrapper("Hinge");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet hinge
		constraint->constraint = new btHingeConstraint(*constraint->body_a->body, *constraint->body_b->body, pivot_A, pivot_B, axis_A, axis_B);
	}

	// old_fixed from pivot
	if (!PyUnicode_CompareWithASCIIString(cargs[0], "old_fixed")) {
		// this fixed was kept for backward-compatibility

		// number or arguments including the constraint type
		bi_ensure(num_cargs == 2);

		PyObject * pivot = cargs[1];

		const btTransform & transform_A = constraint->body_a->body->getCenterOfMassTransform();
		const btTransform & transform_B = constraint->body_b->body->getCenterOfMassTransform();

		btVector3 constraint_pivot;

		if (pivot == Py_None) {
			constraint_pivot = transform_A.getOrigin();
		} else {
			constraint_pivot = get_vector(pivot);
		}

		if (PyErr_Occurred()) {
			return 0;
		}

		btVector3 pivot_A = transform_A.getBasis().solve33(constraint_pivot - transform_A.getOrigin());
		btVector3 pivot_B = transform_B.getBasis().solve33(constraint_pivot - transform_B.getOrigin());

		// wrap old_fixed with a python Fixed
		static PyTypeObject * wrapper_type = get_wrapper("Fixed");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet fixed
		constraint->constraint = new btFixedConstraint(*constraint->body_a->body, *constraint->body_b->body, btTransform(transform_A.getBasis().transpose(), pivot_A), btTransform(transform_B.getBasis().transpose(), pivot_B));
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "hinge")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 4);

		// all the init parameters of a btHingeConstraint
		btTransform frame_a = get_transform(cargs[1]);
		btTransform frame_b = get_transform(cargs[2]);
		int ref = PyObject_IsTrue(cargs[3]);

		// wrap with a python Hinge
		static PyTypeObject * wrapper_type = get_wrapper("Hinge");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet hinge
		constraint->constraint = new btHingeConstraint(*constraint->body_a->body, *constraint->body_b->body, frame_a, frame_b, ref);
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "fixed")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 3);

		// all the init parameters of a btFixedConstraint
		btTransform frame_a = get_transform(cargs[1]);
		btTransform frame_b = get_transform(cargs[2]);

		// wrap with a python Fixed
		static PyTypeObject * wrapper_type = get_wrapper("Fixed");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet fixed
		constraint->constraint = new btFixedConstraint(*constraint->body_a->body, *constraint->body_b->body, frame_a, frame_b);
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "sixdof")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 4);

		// all the init parameters of a btGeneric6DofSpring2Constraint
		btTransform frame_a = get_transform(cargs[1]);
		btTransform frame_b = get_transform(cargs[2]);
		int rot_order = PyObject_IsTrue(cargs[3]);

		// wrap with a python SixDof
		static PyTypeObject * wrapper_type = get_wrapper("SixDof");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet sixdof
		constraint->constraint = new btGeneric6DofSpring2Constraint(*constraint->body_a->body, *constraint->body_b->body, frame_a, frame_b, (RotateOrder)rot_order);

		// TODO: parameters for sixdof
		// constraint->sixdof->
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "slider")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 4);

		// all the init parameters of a btSliderConstraint
		btTransform frame_a = get_transform(cargs[1]);
		btTransform frame_b = get_transform(cargs[2]);
		int ref = PyObject_IsTrue(cargs[3]);

		// wrap with a python Slider
		static PyTypeObject * wrapper_type = get_wrapper("Slider");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet slider
		constraint->constraint = new btSliderConstraint(*constraint->body_a->body, *constraint->body_b->body, frame_a, frame_b, ref);
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "point_to_point")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 3);

		// all the init parameters of a btPoint2PointConstraint
		btVector3 pivot_a = get_vector(cargs[1]);
		btVector3 pivot_b = get_vector(cargs[2]);

		// wrap with a python PointToPoint
		static PyTypeObject * wrapper_type = get_wrapper("PointToPoint");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet point_to_point
		constraint->constraint = new btPoint2PointConstraint(*constraint->body_a->body, *constraint->body_b->body, pivot_a, pivot_a);
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "cone_twist")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 3);

		// all the init parameters of a btConeTwistConstraint
		btTransform frame_a = get_transform(cargs[1]);
		btTransform frame_b = get_transform(cargs[2]);

		// wrap with a python Slider
		static PyTypeObject * wrapper_type = get_wrapper("ConeTwist");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet cone twist
		constraint->constraint = new btConeTwistConstraint(*constraint->body_a->body, *constraint->body_b->body, frame_a, frame_b);
	}

	if (!PyUnicode_CompareWithASCIIString(cargs[0], "gear")) {
		// number or arguments including the constraint type
		bi_ensure(num_cargs == 4);

		// all the init parameters of a btGearConstraint
		btVector3 axis_a = get_vector(cargs[1]);
		btVector3 axis_b = get_vector(cargs[2]);
		btScalar ratio = PyFloat_AsDouble(cargs[3]);

		// wrap with a python Gear
		static PyTypeObject * wrapper_type = get_wrapper("Gear");
		constraint->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!constraint->wrapper) {
			return 0;
		}

		// create a bullet gear
		constraint->constraint = new btGearConstraint(*constraint->body_a->body, *constraint->body_b->body, axis_a, axis_b, ratio);
	}

	// ensure constraint type was recognized
	bi_ensure(constraint->constraint);

	// init common constraint slots
	init_slot(constraint->wrapper, "_obj", constraint);
	init_slot(constraint->wrapper, "body_a", new_ref(constraint->body_a->wrapper));
	init_slot(constraint->wrapper, "body_b", new_ref(constraint->body_b->wrapper));
	init_slot(constraint->wrapper, "world", new_ref(self->wrapper));
	init_slot(constraint->wrapper, "name", new_ref(name));

	// store a ref in bodies
	PyList_Append(constraint->body_a->constraints_slot, constraint->wrapper);
	PyList_Append(constraint->body_b->constraints_slot, constraint->wrapper);

	// max_iterations was kept for backward-compatibility
	int max_iterations = self->dynamics_world->getSolverInfo().m_numIterations;
	if (iterations < 0 || iterations > max_iterations) {
		iterations = max_iterations;
	}

	// store world
	constraint->world = self;

	// store a backref to our internal object
	constraint->constraint->setUserConstraintPtr(constraint);

	// constraint->constraint->setOverrideNumSolverIterations(iterations);
	// btJointFeedback * feedback = new btJointFeedback();
	// constraint->constraint->setJointFeedback(feedback);
	// constraint->constraint->enableFeedback(true);

	// add the constraint to the world
	self->dynamics_world->addConstraint(constraint->constraint, !collision);

	// remove contacts between body_a and body_b
	if (!collision) {
		// this is only helpful if body_a and body_b were colliding and they were constrained together with collision disabled
		btBroadphaseProxy * proxy_a = constraint->body_a->body->getBroadphaseProxy();
		btBroadphaseProxy * proxy_b = constraint->body_b->body->getBroadphaseProxy();
		self->broadphase->getOverlappingPairCache()->removeOverlappingPair(proxy_a, proxy_b, self->dispatcher);
	}

	// store named objects (deprecated)
	PyDict_SetItem(self->names_slot, name, constraint->wrapper);

	// store constraints in world
	PyList_Append(self->constraints_slot, constraint->wrapper);

	// release arguments
	Py_DECREF(constraint_args);

	// return the wrapper
	bi_ensure(!PyErr_Occurred());
	return constraint->wrapper;
}

PyObject * BIConstraint_meth_config(BIConstraint * self, PyObject * config) {
	/*
		Not yet used method to configure a constraint.
		This method should allow to serialize and deserialize all the properties of the constraint.
	*/

	if (config == Py_None) {
		Py_RETURN_NONE;
	}

	if (self->constraint->getConstraintType() == HINGE_CONSTRAINT_TYPE) {
		if (PyObject * value = PyDict_GetItemString(config, "frames")) {
			bi_ensure(PyObject_Length(value) == 2);
			self->hinge->setFrames(
				get_transform(PySequence_GetItem(value, 0)),
				get_transform(PySequence_GetItem(value, 1))
			);
			Py_DECREF(value);
		}
	}

	if (self->constraint->getConstraintType() == D6_SPRING_2_CONSTRAINT_TYPE) {
		if (PyObject * value = PyDict_GetItemString(config, "frames")) {
			bi_ensure(PyObject_Length(value) == 2);
			self->fixed->setFrames(
				get_transform(PySequence_GetItem(value, 0)),
				get_transform(PySequence_GetItem(value, 1))
			);
			Py_DECREF(value);
		}
	}

	if (PyObject * value = PyDict_GetItemString(config, "enabled")) {
		self->constraint->setEnabled(PyObject_IsTrue(value));
		Py_DECREF(value);
	}

	Py_RETURN_NONE;
}

PyObject * BIConstraint_meth_remove(BIConstraint * self) {
	/*
		Remove a constraint from the world.
	*/

	// do not loose our object while removing it from other containers
	Py_INCREF(self);

	// release _obj
	init_slot(self->wrapper, "_obj", new_ref(Py_None));

	// release the world
	init_slot(self->wrapper, "world", new_ref(Py_None));

	// remove the motor control if any
	if (self->motor_control) {
		// a motor control cannot exist if a motor was removed
		Py_DECREF(BIMotorControl_meth_remove(self->motor_control));
	}

	// remove constraint from body_a
	PySequence_DelItem(self->body_a->constraints_slot, PySequence_Index(self->body_a->constraints_slot, self->wrapper));

	// remove constraint from body_b
	PySequence_DelItem(self->body_b->constraints_slot, PySequence_Index(self->body_b->constraints_slot, self->wrapper));

	// remove constraint from constraints
	PySequence_DelItem(self->world->constraints_slot, PySequence_Index(self->world->constraints_slot, self->wrapper));

	// remove bullet constraint
	self->world->dynamics_world->removeConstraint(self->constraint);

	// check for errors
	bi_ensure(!PyErr_Occurred());

	// loose our object
	Py_DECREF(self);
	Py_RETURN_NONE;
}

PyObject * BIConstraint_meth_pivot(BIConstraint * self) {
	/*
		Return the pivots in world coordinates
	*/

	btVector3 pivot_a;
	btVector3 pivot_b;

	switch (self->constraint->getConstraintType()) {
		case POINT2POINT_CONSTRAINT_TYPE:
			pivot_a = self->point_to_point->getPivotInA();
			pivot_b = self->point_to_point->getPivotInB();
			break;

		case HINGE_CONSTRAINT_TYPE:
			pivot_a = self->hinge->getFrameOffsetA().getOrigin();
			pivot_b = self->hinge->getFrameOffsetB().getOrigin();
			break;

		case CONETWIST_CONSTRAINT_TYPE:
			pivot_a = self->cone_twist->getFrameOffsetA().getOrigin();
			pivot_b = self->cone_twist->getFrameOffsetB().getOrigin();
			break;

		case SLIDER_CONSTRAINT_TYPE:
			pivot_a = self->slider->getFrameOffsetA().getOrigin();
			pivot_b = self->slider->getFrameOffsetB().getOrigin();
			break;

		case GEAR_CONSTRAINT_TYPE:
			pivot_a = btVector3(0.0, 0.0, 0.0);
			pivot_b = btVector3(0.0, 0.0, 0.0);
			break;

		case D6_SPRING_2_CONSTRAINT_TYPE:
			// a bullet fixed is a bullet sixdof
			pivot_a = self->sixdof->getFrameOffsetA().getOrigin();
			pivot_b = self->sixdof->getFrameOffsetB().getOrigin();
			break;

		default:
			PyErr_BadInternalCall();
			return 0;
	}

	// get world transforms
	btTransform transform_a = self->body_a->body->getWorldTransform();
	btTransform transform_b = self->body_b->body->getWorldTransform();

	// local to world
	pivot_a = transform_a * pivot_a;
	pivot_b = transform_b * pivot_b;

	return Py_BuildValue("(fff)(fff)", pivot_a.x(), pivot_a.y(), pivot_a.z(), pivot_b.x(), pivot_b.y(), pivot_b.z());
}

void BIConstraint_dealloc(BIConstraint * self) {
	Py_TYPE(self)->tp_free(self);
}

PyMethodDef BIConstraint_methods[] = {
	{"config", (PyCFunction)BIConstraint_meth_config, METH_O, 0},
	{"remove", (PyCFunction)BIConstraint_meth_remove, METH_NOARGS, 0},
	{"pivot", (PyCFunction)BIConstraint_meth_pivot, METH_NOARGS, 0},
	{0},
};

PyMemberDef BIConstraint_members[] = {
	{"_body_a", T_OBJECT_EX, offsetof(BIConstraint, body_a), READONLY, 0},
	{"_body_b", T_OBJECT_EX, offsetof(BIConstraint, body_b), READONLY, 0},
	{0},
};

PyType_Slot BIConstraint_slots[] = {
	{Py_tp_methods, BIConstraint_methods},
	{Py_tp_members, BIConstraint_members},
	{Py_tp_dealloc, (void *)BIConstraint_dealloc},
	{0},
};

PyTypeObject * BIConstraint_type;

PyType_Spec BIConstraint_spec = {"mollia_bullet.core.Constraint", sizeof(BIConstraint), 0, Py_TPFLAGS_DEFAULT, BIConstraint_slots};
