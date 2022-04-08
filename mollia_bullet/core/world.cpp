#include "world.hpp"

#include "constraint.hpp"
#include "group.hpp"
#include "motor_control.hpp"
#include "rigid_body.hpp"

PyObject * meth_world(PyObject * self, PyObject * args) {
	/*
		Return a bullet World.
		The return value is a python World object with an _obj propery referencing the internal BIWorld object.
		The user must not use the _obj directly.
	*/

	// parameters
	double time_step;
	PyObject * gravity_arg;
	int iterations;
	int use_old_solver_mode;
	int randomize_solver;

	// parse args
	if (!PyArg_ParseTuple(args, "dOipp", &time_step, &gravity_arg, &iterations, &use_old_solver_mode, &randomize_solver)) {
		return 0;
	}

	// get gravity
	btVector3 gravity = get_vector(gravity_arg, true);

	// create a new internal object
	BIWorld * world = PyObject_New(BIWorld, BIWorld_type);

	// wrap with a python MotorControl
	static PyTypeObject * wrapper_type = get_wrapper("World");
	bi_ensure(wrapper_type);
	world->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
	if (!world->wrapper) {
		return 0;
	}

	// create bullet objects
	world->collision_configuration = new btDefaultCollisionConfiguration();
	world->dispatcher = new btCollisionDispatcher(world->collision_configuration);
	world->broadphase = new btDbvtBroadphase();
	world->solver = new btMultiBodyConstraintSolver();
	world->dynamics_world = new btMultiBodyDynamicsWorld(
		world->dispatcher,
		world->broadphase,
		world->solver,
		world->collision_configuration
	);

	// configure the world
	world->dynamics_world->setGravity(gravity);
	world->dynamics_world->getSolverInfo().m_numIterations = iterations;
	if (use_old_solver_mode) {
		world->dynamics_world->getSolverInfo().m_solverMode &= ~SOLVER_USE_2_FRICTION_DIRECTIONS;
	}
	if (randomize_solver) {
		world->dynamics_world->getSolverInfo().m_solverMode |= SOLVER_RANDMIZE_ORDER;
	}

	// fill the internal object
	world->time_step = time_step;
	world->iterations = iterations;

	// create wrapper slots
	world->names_slot = PyDict_New();
	world->groups_slot = PyList_New(0);
	world->motor_controls_slot = PyList_New(0);
	world->constraints_slot = PyList_New(0);
	world->updaters_slot = PyList_New(0);

	// internal motor control list
	world->motor_controls = PyList_New(0);

	// init world slots
	init_slot(world->wrapper, "_obj", world);

	init_slot(world->wrapper, "_gravity", Py_BuildValue("ddd", gravity.x(), gravity.y(), gravity.z()));
	init_slot(world->wrapper, "_time_step", PyFloat_FromDouble(time_step));
	init_slot(world->wrapper, "_iterations", PyLong_FromLong(iterations));

	init_slot(world->wrapper, "names", world->names_slot);
	init_slot(world->wrapper, "groups", world->groups_slot);
	init_slot(world->wrapper, "motor_controls", world->motor_controls_slot);
	init_slot(world->wrapper, "constraints", world->constraints_slot);
	init_slot(world->wrapper, "updaters", world->updaters_slot);

	// create the main group
	static PyObject * main_group_args = Py_BuildValue("([])");
	PyObject * main_group = BIWorld_meth_group(world, main_group_args);
	init_slot(world->wrapper, "main_group", main_group);
	world->main_group = get_slot(main_group, BIGroup, "_obj");

	// check for errors
	bi_ensure(!PyErr_Occurred());

	// return the wrapper
	return world->wrapper;
}

PyObject * BIWorld_meth_destroy(BIWorld * self) {
	/*
		Destroy a world object will help the GC find the python objects due to there will be no circular refs.
	*/

	// do not loose our object while removing it from other containers
	Py_INCREF(self);

	// release _obj
	init_slot(self->wrapper, "_obj", new_ref(Py_None));

	// release the world
	init_slot(self->wrapper, "main_group", new_ref(Py_None));

	// remove motor controls
	int num_motor_controls = (int)PyList_GET_SIZE(self->motor_controls_slot);
	while (num_motor_controls--) {
		// take the internal motor control object
		BIMotorControl * motor_control = get_slot(PyList_GET_ITEM(self->motor_controls_slot, num_motor_controls), BIMotorControl, "_obj");
		// call remove on the internal motor control object
		Py_DECREF(BIMotorControl_meth_remove(motor_control));
	}

	// remove constraints
	int num_constraints = (int)PyList_GET_SIZE(self->constraints_slot);
	while (num_constraints--) {
		// take the internal constraint object
		BIConstraint * constraint = get_slot(PyList_GET_ITEM(self->constraints_slot, num_constraints), BIConstraint, "_obj");
		// call remove on the internal constraint object
		Py_DECREF(BIConstraint_meth_remove(constraint));
	}

	// remove rigid bodies
	int num_rigid_bodies = (int)PyList_GET_SIZE(self->main_group->bodies_slot);
	while (num_rigid_bodies--) {
		// take the internal rigid body object
		BIRigidBody * rigid_body = get_slot(PyList_GET_ITEM(self->main_group->bodies_slot, num_rigid_bodies), BIRigidBody, "_obj");
		// call remove on the internal rigid body object
		Py_DECREF(BIRigidBody_meth_remove(rigid_body));
	}

	// remove groups
	int num_groups = (int)PyList_GET_SIZE(self->groups_slot);
	while (num_groups--) {
		// take the internal group object
		BIGroup * group = get_slot(PyList_GET_ITEM(self->groups_slot, num_groups), BIGroup, "_obj");
		// call remove on the internal group object
		Py_DECREF(BIGroup_meth_remove(group));
	}

	// loose our object
	Py_DECREF(self);
	Py_RETURN_NONE;
}

PyObject * BIWorld_meth_simulate(BIWorld * self) {
	/*
		Step simulation
	*/

	// motor control before simulate
	int num_motor_controls = (int)PyList_GET_SIZE(self->motor_controls);
	for (int k = 0; k < num_motor_controls; ++k) {
		// take the internal motor control object
		BIMotorControl * motor_control = (BIMotorControl *)PyList_GET_ITEM(self->motor_controls, k);
		bi_ensure(Py_TYPE(motor_control) == BIMotorControl_type);

		// iterate the internal constraint objects
		int num_motors = (int)PyList_GET_SIZE(motor_control->motors);
		for (int i = 0; i < num_motors; ++i) {
			// take the internal constraint object
			BIConstraint * motor = (BIConstraint *)PyList_GET_ITEM(motor_control->motors, i);

			// add custom error checking for nan
			if (isnan(motor_control->input_data[i].max_impulse) || isnan(motor_control->input_data[i].target_velocity)) {
				PyErr_Format(PyExc_ValueError, "nan in motor control");
				return 0;
			}

			// control the bullet motors
			motor->hinge->setMaxMotorImpulse(motor_control->input_data[i].max_impulse);
			motor->hinge->setMotorTargetVelocity(motor_control->input_data[i].target_velocity);
		}
	}

	// step simulation
	self->dynamics_world->stepSimulation(self->time_step, 0, self->time_step);

	// provide feedback for the motor controls
	for (int k = 0; k < num_motor_controls; ++k) {
		// take the internal motor control object
		BIMotorControl * motor_control = (BIMotorControl *)PyList_GET_ITEM(self->motor_controls, k);

		// organize front and back buffer for the output
		btScalar * output_data[2] = {
			(btScalar *)PyMemoryView_GET_BUFFER(motor_control->output_mem[1 - motor_control->output_index])->buf,
			(btScalar *)PyMemoryView_GET_BUFFER(motor_control->output_mem[motor_control->output_index])->buf,
		};

		// iterate the internal constraint objects
		int num_motors = (int)PyList_GET_SIZE(motor_control->motors);
		for (int i = 0; i < num_motors; ++i) {
			// take the internal constraint object
			BIConstraint * motor = (BIConstraint *)PyList_GET_ITEM(motor_control->motors, i);

			// fill position and velocity
			output_data[1][i] = motor->hinge->getHingeAngle();
			output_data[0][i] = output_data[1][i] - output_data[0][i];
		}

		// swap the front and back buffer for the output
		motor_control->output_index = 1 - motor_control->output_index;
	}

	// check for errors
	if (PyErr_Occurred()) {
		return 0;
	}

	// call updaters
	int num_updaters = (int)PyList_GET_SIZE(self->updaters_slot);
	for (int i = 0; i < num_updaters; ++i) {
		Py_XDECREF(PyObject_CallFunction(PyList_GET_ITEM(self->updaters_slot, i), 0));
	}

	// check for errors
	if (PyErr_Occurred()) {
		return 0;
	}

	Py_RETURN_NONE;
}

namespace {

/*
	Deprecated
*/

struct HelperVertex {
	float vertex[3];
	float color[3];
	float alpha;
};

void write_helper_vertex(char *& ptr, const btVector3 & vert, const btVector3 & color, const btScalar & alpha = 1.0);

inline void write_vector(char *& ptr, const btVector3 & value) {
	((float *)ptr)[0] = (float)value.x();
	((float *)ptr)[1] = (float)value.y();
	((float *)ptr)[2] = (float)value.z();
	ptr += 12;
}

inline void write_color(char *& ptr, const btVector3 & value, const btScalar & alpha) {
	((float *)ptr)[0] = (float)value.x();
	((float *)ptr)[1] = (float)value.y();
	((float *)ptr)[2] = (float)value.z();
	((float *)ptr)[3] = (float)alpha;
	ptr += 16;
}

void write_helper_vertex(char *& ptr, const btVector3 & vert, const btVector3 & color, const btScalar & alpha) {
	write_vector(ptr, vert);
	write_color(ptr, color, alpha);
}

}

PyObject * BIWorld_meth_contact_helper(BIWorld * self) {
	/*
		Deprecated
	*/
	int numManifolds = self->dynamics_world->getDispatcher()->getNumManifolds();
	int expected_size = 0;

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold * contactManifold = self->dynamics_world->getDispatcher()->getManifoldByIndexInternal(i);
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint & pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 1e-3) {
				expected_size += 2 * sizeof(HelperVertex);
				expected_size += 2 * sizeof(HelperVertex);
			}
		}
	}

	PyObject * res = PyBytes_FromStringAndSize(0, expected_size);
	char * ptr = PyBytes_AsString(res);

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold * contactManifold = self->dynamics_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject * obA = contactManifold->getBody0();
		const btCollisionObject * obB = contactManifold->getBody1();
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint & pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 1e-3) {
				const btVector3 & ptA = pt.getPositionWorldOnA();
				const btVector3 & ptB = pt.getPositionWorldOnB();
				const btVector3 & normalOnB = pt.m_normalWorldOnB;

				#define HEXRGB(val) btVector3(val >> 16 & 0xFF, val >> 8 & 0xFF, val >> 0 & 0xFF) / 0xFF
				write_helper_vertex(ptr, ptA, HEXRGB(0xFF0000), 0.6);
				write_helper_vertex(ptr, ptA - normalOnB * (0.05 + pt.getDistance() * 3.0), HEXRGB(0xFF0000), 0.6);
				write_helper_vertex(ptr, ptB, HEXRGB(0x00FF00), 0.6);
				write_helper_vertex(ptr, ptB + normalOnB * (0.05 + pt.getDistance() * 3.0), HEXRGB(0x00FF00), 0.6);
				#undef HEXRGB
			}
		}
	}

	int size = (int)(ptr - PyBytes_AsString(res));
	if (size != expected_size) {
		bi_fatal();
	}

	return res;
}

PyObject * BIWorld_meth_contacts_between2(BIWorld * self, PyObject * args) {
	/*
		Deprecated
	*/
	PyObject * obj1;
	PyObject * obj2;
	btScalar eps;
	int local;

	int args_ok = PyArg_ParseTuple(
		args,
		"OOdp",
		&obj1,
		&obj2,
		&eps,
		&local
	);

	if (!args_ok) {
		return 0;
	}

	BIRigidBody * rbody1 = get_slot(obj1, BIRigidBody, "_obj");
	BIRigidBody * rbody2 = get_slot(obj2, BIRigidBody, "_obj");

	btCollisionObject * collision_obj1 = rbody1->body;
	btCollisionObject * collision_obj2 = rbody2->body;

	btVector3 obj_origin = collision_obj1->getWorldTransform().getOrigin();
	btMatrix3x3 obj_inv_basis = collision_obj1->getWorldTransform().getBasis().transpose();

	int numManifolds = self->dynamics_world->getDispatcher()->getNumManifolds();
	int length = 0;

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold * contactManifold = self->dynamics_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject * obA = contactManifold->getBody0();
		const btCollisionObject * obB = contactManifold->getBody1();
		bool pass = false;
		pass |= obA == collision_obj1 && obB == collision_obj2;
		pass |= obA == collision_obj2 && obB == collision_obj1;
		if (!pass) {
			continue;
		}
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint & pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < eps) {
				length += 1;
			}
		}
	}

	PyObject * lst = PyList_New(length);
	int idx = 0;

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold * contactManifold = self->dynamics_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject * obA = contactManifold->getBody0();
		const btCollisionObject * obB = contactManifold->getBody1();
		bool pass = false;
		pass |= obA == collision_obj1 && obB == collision_obj2;
		pass |= obA == collision_obj2 && obB == collision_obj1;
		if (!pass) {
			continue;
		}
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint & pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < eps) {
				btVector3 ptA;
				btVector3 ptB;
				btVector3 normalOnB;
				if (obA == collision_obj1) {
					ptA = pt.getPositionWorldOnA();
					ptB = pt.getPositionWorldOnB();
					normalOnB = pt.m_normalWorldOnB;
				} else {
					ptA = pt.getPositionWorldOnB();
					ptB = pt.getPositionWorldOnA();
					normalOnB = -pt.m_normalWorldOnB;
				}
				if (local) {
					ptA = obj_inv_basis * (ptA - obj_origin);
					ptB = obj_inv_basis * (ptB - obj_origin);
					normalOnB = obj_inv_basis * normalOnB;
				}
				btScalar distance = pt.getDistance();
				PyObject * tup = PyTuple_New(4);
				PyTuple_SET_ITEM(tup, 0, Py_BuildValue("fff", ptA.x(), ptA.y(), ptA.z()));
				PyTuple_SET_ITEM(tup, 1, Py_BuildValue("fff", -normalOnB.x(), -normalOnB.y(), -normalOnB.z()));
				PyTuple_SET_ITEM(tup, 2, PyFloat_FromDouble(distance));
				PyTuple_SET_ITEM(tup, 3, PyFloat_FromDouble(pt.m_appliedImpulse));
				PyList_SET_ITEM(lst, idx++, tup);
			}
		}
	}

	return lst;
}

int BIWorld_set_iterations(BIWorld * self, PyObject * value) {
	/*
		Set the global num iterations for the constraints
	*/
	int iterations = PyLong_AsLong(value);
	if (PyErr_Occurred()) {
		return -1;
	}

	// change the bullet object
	self->dynamics_world->getSolverInfo().m_numIterations = iterations;

	// set the value in the wrapper
	PyObject_SetAttrString(self->wrapper, "_iterations", PyLong_FromLong(iterations));
	return 0;
}

int BIWorld_set_gravity(BIWorld * self, PyObject * value) {
	/*
		Set the gravity
	*/
	btVector3 gravity = get_vector(value, true);
	if (PyErr_Occurred()) {
		return -1;
	}

	// change the bullet object
	self->dynamics_world->setGravity(gravity);

	// set the value in the wrapper
	PyObject_SetAttrString(self->wrapper, "_gravity", Py_BuildValue("ddd", gravity.x(), gravity.y(), gravity.z()));
	return 0;
}

void BIWorld_dealloc(BIWorld * self) {
	Py_DECREF(self->motor_controls);

	if (self->dynamics_world) {
		for (int i = self->dynamics_world->getNumConstraints() - 1; i >= 0; --i) {
			btTypedConstraint * constraint = self->dynamics_world->getConstraint(i);
			self->dynamics_world->removeConstraint(constraint);
			delete constraint;
		}

		for (int i = self->dynamics_world->getNumCollisionObjects() - 1; i >= 0; --i) {
			btCollisionObject * body = self->dynamics_world->getCollisionObjectArray()[i];
			btCollisionShape * shape = body->getCollisionShape();
			self->dynamics_world->removeCollisionObject(body);
			delete body;
			delete shape;
		}

		delete self->dynamics_world;
	}

	if (self->solver) {
		delete self->solver;
	}

	if (self->broadphase) {
		delete self->broadphase;
	}

	if (self->dispatcher) {
		delete self->dispatcher;
	}

	if (self->collision_configuration) {
		delete self->collision_configuration;
	}

	Py_TYPE(self)->tp_free(self);
}

PyMethodDef BIWorld_methods[] = {
	{"destroy", (PyCFunction)BIWorld_meth_destroy, METH_NOARGS, 0},
	{"rigid_body", (PyCFunction)BIWorld_meth_rigid_body, METH_VARARGS, 0},
	{"constraint", (PyCFunction)BIWorld_meth_constraint, METH_VARARGS, 0},
	{"motor_control", (PyCFunction)BIWorld_meth_motor_control, METH_VARARGS, 0},
	{"group", (PyCFunction)BIWorld_meth_group, METH_VARARGS, 0},
	{"simulate", (PyCFunction)BIWorld_meth_simulate, METH_NOARGS, 0},
	{"contact_helper", (PyCFunction)BIWorld_meth_contact_helper, METH_NOARGS, 0},
	{"contacts_between2", (PyCFunction)BIWorld_meth_contacts_between2, METH_VARARGS, 0},
	{0},
};

PyGetSetDef BIWorld_getset[] = {
	{"iterations", 0, (setter)BIWorld_set_iterations, 0, 0},
	{"gravity", 0, (setter)BIWorld_set_gravity, 0, 0},
	{0},
};

PyMemberDef BIWorld_members[] = {
	{"_names_slot", T_OBJECT_EX, offsetof(BIWorld, names_slot), READONLY, 0},
	{"_groups_slot", T_OBJECT_EX, offsetof(BIWorld, groups_slot), READONLY, 0},
	{"_motor_controls_slot", T_OBJECT_EX, offsetof(BIWorld, motor_controls_slot), READONLY, 0},
	{"_constraints_slot", T_OBJECT_EX, offsetof(BIWorld, constraints_slot), READONLY, 0},
	{"_updaters_slot", T_OBJECT_EX, offsetof(BIWorld, updaters_slot), READONLY, 0},
	{"_main_group", T_OBJECT_EX, offsetof(BIWorld, main_group), READONLY, 0},
	{0},
};

PyType_Slot BIWorld_slots[] = {
	{Py_tp_methods, BIWorld_methods},
	{Py_tp_getset, BIWorld_getset},
	{Py_tp_members, BIWorld_members},
	{Py_tp_dealloc, (void *)BIWorld_dealloc},
	{0},
};

PyTypeObject * BIWorld_type;

PyType_Spec BIWorld_spec = {"mollia_bullet.core.World", sizeof(BIWorld), 0, Py_TPFLAGS_DEFAULT, BIWorld_slots};
