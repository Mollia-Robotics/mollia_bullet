#include "rigid_body.hpp"

#include "constraint.hpp"
#include "group.hpp"
#include "world.hpp"

#define va_vec(vec) vec.x(), vec.y(), vec.z()
#define va_quat(quat) quat.x(), quat.y(), quat.z(), quat.w()

PyObject * BIWorld_meth_rigid_body(BIWorld * self, PyObject * args) {
	/*
		Return a bullet RigidBody.
		The return value is a python Box, Sphere, ... object with an _obj propery referencing the internal BIRigidBody object.
		The user must not use the _obj directly.
	*/

	// parameters
	double mass;
	PyObject * shape_arg;
	PyObject * origin_arg;
	PyObject * orientation_arg;
	PyObject * ref;
	int group;
	int mask;
	PyObject * name;

	// parse parameters
	if (!PyArg_ParseTuple(args, "dOOOOiiO", &mass, &shape_arg, &origin_arg, &orientation_arg, &ref, &group, &mask, &name)) {
		return 0;
	}

	// create a new internal object
	BIRigidBody * rbody = PyObject_New(BIRigidBody, BIRigidBody_type);

	// fill the internal object
	rbody->groups_slot = PyList_New(0);
	rbody->constraints_slot = PyList_New(0);

	// start with an unknown shape
	btCollisionShape * shape = 0;

	// parse origin and rotation
	btTransform transform;
	transform.setOrigin(get_vector(origin_arg, true));
	transform.setRotation(get_quaternion(orientation_arg, true));

	if (ref != Py_None) {
		// calculate transform based on ref
		BIRigidBody * ref_rbody = get_slot(ref, BIRigidBody, "_obj");
		bi_ensure(Py_TYPE(ref_rbody) == BIRigidBody_type);
		transform = ref_rbody->body->getWorldTransform() * transform;
	}

	// shape specific arguments
	shape_arg = PySequence_Fast(shape_arg, "not iterable");
	bi_ensure(shape_arg);

	// shape_arg can be:
	//   ("box", width, length, height)
	//   ("sphere", radius)
	//   ...

	int num_sargs = (int)PySequence_Fast_GET_SIZE(shape_arg);
	PyObject ** sargs = PySequence_Fast_ITEMS(shape_arg);

	bi_ensure(num_sargs >= 1 && PyUnicode_CheckExact(sargs[0]));

	if (!PyUnicode_CompareWithASCIIString(sargs[0], "box")) {
		// number or arguments including the shape type
		bi_ensure(num_sargs == 4);

		btVector3 half_extens = btVector3(PyFloat_AsDouble(sargs[1]), PyFloat_AsDouble(sargs[2]), PyFloat_AsDouble(sargs[3]));
		bi_ensure(!PyErr_Occurred());

		// wrap with a python Box
		static PyTypeObject * wrapper_type = get_wrapper("Box");
		rbody->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!rbody->wrapper) {
			return 0;
		}

		// init box specific slots
		init_slot(rbody->wrapper, "size", Py_BuildValue("ddd", half_extens.x(), half_extens.y(), half_extens.z()));

		// create a bullet box
		shape = new btBoxShape(half_extens);
	}

	if (!PyUnicode_CompareWithASCIIString(sargs[0], "sphere")) {
		// number or arguments including the shape type
		bi_ensure(num_sargs == 2);

		btScalar radius = PyFloat_AsDouble(sargs[1]);
		bi_ensure(!PyErr_Occurred());

		// wrap with a python Sphere
		static PyTypeObject * wrapper_type = get_wrapper("Sphere");
		rbody->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
		if (!rbody->wrapper) {
			return 0;
		}

		// init sphere specific slots
		init_slot(rbody->wrapper, "radius", Py_BuildValue("d", radius));

		// create a bullet sphere
		shape = new btSphereShape(radius);
	}

	// ensure shape type was recognized
	bi_ensure(shape);

	// init common rigid body slots
	init_slot(rbody->wrapper, "_obj", rbody);
	init_slot(rbody->wrapper, "_stiffness", Py_BuildValue("ddO", BT_LARGE_FLOAT, 0.1, Py_False));
	init_slot(rbody->wrapper, "_friction", Py_BuildValue("ddd", 0.0, 0.0, 0.0));
	init_slot(rbody->wrapper, "world", new_ref(self->wrapper));
	init_slot(rbody->wrapper, "mass", PyFloat_FromDouble(mass));
	init_slot(rbody->wrapper, "group", PyLong_FromLong(group));
	init_slot(rbody->wrapper, "mask", PyLong_FromLong(mask));
	init_slot(rbody->wrapper, "color", Py_BuildValue("ddd", 1.0, 1.0, 1.0));
	init_slot(rbody->wrapper, "visible", new_ref(Py_True));
	init_slot(rbody->wrapper, "name", new_ref(name));

	init_slot(rbody->wrapper, "groups", rbody->groups_slot);
	init_slot(rbody->wrapper, "constraints", rbody->constraints_slot);

	btVector3 local_inertia = btVector3(0.0, 0.0, 0.0);

	if (mass) {
		// calculate local_inertia based on the shape
		shape->calculateLocalInertia(mass, local_inertia);
	}

	// store world
	rbody->world = self;

	// create a bullet rigid body
	rbody->body = new btRigidBody(mass, 0, shape, local_inertia);

	// store a backref to our internal object
	rbody->body->setUserPointer(rbody);

	// setup rigid body
	rbody->body->setCenterOfMassTransform(transform);
	rbody->body->setRollingFriction(1e-3);
	rbody->body->setSpinningFriction(1e-3);
	rbody->body->setActivationState(DISABLE_DEACTIVATION);

	// add the rigid body to the world
	self->dynamics_world->addRigidBody(rbody->body, group, mask);

	// release arguments
	Py_DECREF(shape_arg);

	// store named objects (deprecated)
	PyDict_SetItem(self->names_slot, name, rbody->wrapper);

	// store rigid body in the main group
	PyList_Append(self->main_group->bodies_slot, rbody->wrapper);
	PyList_Append(self->main_group->bodies, (PyObject *)rbody);

	// add main group to the list of groups
	PyList_Append(rbody->groups_slot, self->main_group->wrapper);

	// return the wrapper
	bi_ensure(!PyErr_Occurred());
	return rbody->wrapper;
}

PyObject * BIRigidBody_meth_contacts(BIRigidBody * self, PyObject * args) {
	/*
		Return the list of contacts
	*/

	// parameters
	PyObject * other;
	int mask;
	btScalar eps;
	int local;

	// parse args
	if (!PyArg_ParseTuple(args, "Oidp", &other, &mask, &eps, &local)) {
		return 0;
	}

	// take the internal rigid body object
	BIRigidBody * other_body = (other == Py_None) ? NULL : get_slot(other, BIRigidBody, "_obj");

	const btCollisionObject * ob1 = self->body;
	btVector3 origin = ob1->getWorldTransform().getOrigin();
	btMatrix3x3 basis = ob1->getWorldTransform().getBasis();

	// the number of contact manifolds
	int num_manifolds = self->world->dynamics_world->getDispatcher()->getNumManifolds();

	// prepare result
	PyObject * res = PyList_New(0);

	for (int i = 0; i < num_manifolds; i++) {
		// check if the rigid body is part of the contact manifold
		btPersistentManifold * contactManifold = self->world->dynamics_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject * obA = contactManifold->getBody0();
		const btCollisionObject * obB = contactManifold->getBody1();
		if (obA != ob1 && obB != ob1) {
			continue;
		}

		// the number of contacts (max 4)
		int num_contacts = contactManifold->getNumContacts();
		for (int j = 0; j < num_contacts; j++) {
			// get the bullet contact info
			btManifoldPoint & pt = contactManifold->getContactPoint(j);

			// filter for the other object
			const btCollisionObject * ob2 = obA == ob1 ? obB : obA;
			if (other_body != NULL && other_body->body != ob2) {
				continue;
			}

			// get the group of the other object
			int group = ob2->getBroadphaseHandle()->m_collisionFilterGroup;

			// filter for group
			if ((group & mask) && pt.getDistance() < eps) {
				// prepare data for the result
				btVector3 ptA;
				btVector3 ptB;
				btVector3 normB;

				// map A to ob1 and B to ob2
				if (obA == ob1) {
					ptA = pt.getPositionWorldOnA();
					ptB = pt.getPositionWorldOnB();
					normB = pt.m_normalWorldOnB;
				} else {
					ptA = pt.getPositionWorldOnB();
					ptB = pt.getPositionWorldOnA();
					normB = -pt.m_normalWorldOnB;
				}

				if (local) {
					// convert to local
					ptA = basis.solve33(ptA - origin);
					ptB = basis.solve33(ptB - origin);
					normB = basis.solve33(normB);
				}

				// build and store contact info in result
				btScalar distance = pt.getDistance();
				btScalar impulse = pt.m_appliedImpulse;
				PyObject * other = ((BIRigidBody *)ob2->getUserPointer())->wrapper;
				PyObject * contact = Py_BuildValue(
					"O(ddd)(ddd)(ddd)dd",
					other,
					ptA.x(), ptA.y(), ptA.z(),
					ptB.x(), ptB.y(), ptB.z(),
					normB.x(), normB.y(), normB.z(),
					distance,
					impulse
				);
				PyList_Append(res, contact);
				Py_DECREF(contact);
			}
		}
	}

	// the return value is a list
	return res;
}

struct ContactCallback : public btManifoldResult {
	ContactCallback(btCollisionObjectWrapper * obA, btCollisionObjectWrapper * obB) : btManifoldResult(obA, obB), distance(BT_INFINITY) {
	}

	void addContactPoint(const btVector3 & normalOnBInWorld, const btVector3 & pointInWorld, btScalar depth) {
		distance = depth;
	}

public:
	btScalar distance;
};

PyObject * BIRigidBody_meth_penetration(BIRigidBody * self, PyObject * args) {
	/*
		How deep is the penetration between two rigid bodies
	*/

	// parameters
	PyObject * wrapper;

	// parse args
	if (!PyArg_ParseTuple(args, "O", &wrapper)) {
		return 0;
	}

	// take the internal rigid body object
	BIRigidBody * other = get_slot(wrapper, BIRigidBody, "_obj");

	// create bullet collision object wrappers
	btCollisionObjectWrapper obA(0, self->body->getCollisionShape(), self->body, self->body->getWorldTransform(), -1, -1);
	btCollisionObjectWrapper obB(0, other->body->getCollisionShape(), other->body, other->body->getWorldTransform(), -1, -1);

	// select a collision algorithm
	btCollisionAlgorithm * algorithm = self->world->dispatcher->findAlgorithm(&obA, &obB, 0, BT_CLOSEST_POINT_ALGORITHMS);

	// prepare result
	ContactCallback contact_point_result = ContactCallback(&obA, &obB);

	// run the collision algorithm
	algorithm->processCollision(&obA, &obB, self->world->dynamics_world->getDispatchInfo(), &contact_point_result);

	// return penetration if there is any
	if (contact_point_result.distance < 0.0) {
		return PyFloat_FromDouble(-contact_point_result.distance);
	}

	// return zero otherwise
	return PyFloat_FromDouble(0.0);
}

PyObject * BIRigidBody_meth_apply_force(BIRigidBody * self, PyObject * args) {
	/*
		Apply force on a rigid body at a local point
	*/

	// parameters
	PyObject * force;
	PyObject * origin;

	// parse args
	if (!PyArg_ParseTuple(args, "OO", &force, &origin)) {
		return 0;
	}

	// apply force on bullet rigid body
	self->body->applyForce(get_vector(force), get_vector(origin, true));
	Py_RETURN_NONE;
}

PyObject * BIRigidBody_meth_apply_torque(BIRigidBody * self, PyObject * torque) {
	// apply torque on bullet rigid body
	self->body->applyTorque(get_vector(torque));
	Py_RETURN_NONE;
}

PyObject * BIRigidBody_meth_config(BIRigidBody * self, PyObject * config) {
	/*
		Not yet used method to configure a rigid body.
		This method should allow to serialize and deserialize all the properties of the rigid body.
	*/

	if (config == Py_None) {
		btVector3 origin = self->body->getWorldTransform().getOrigin();
		btQuaternion rotation = self->body->getWorldTransform().getRotation();
		btVector3 linear_velocity = self->body->getLinearVelocity();
		btVector3 angular_velocity = self->body->getAngularVelocity();
		btScalar mass = 1.0 / self->body->getInvMass();
		btVector3 inertia = self->body->getLocalInertia();
		btVector3 linear_factor = self->body->getLinearFactor();
		btVector3 angular_factor = self->body->getAngularFactor();
		btScalar linear_friction = self->body->getFriction();
		btScalar spinning_friction = self->body->getSpinningFriction();
		btScalar rolling_friction = self->body->getRollingFriction();
		btVector3 anisotropic_friction = self->body->getAnisotropicFriction();
		int anisotropic_friction_mode = 0;
		if (self->body->hasAnisotropicFriction(btCollisionObject::CF_ANISOTROPIC_FRICTION)) {
			anisotropic_friction_mode |= btCollisionObject::CF_ANISOTROPIC_FRICTION;
		}
		if (self->body->hasAnisotropicFriction(btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION)) {
			anisotropic_friction_mode |= btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION;
		}
		btScalar contact_stiffness = self->body->getContactStiffness();
		btScalar contact_damping = self->body->getContactDamping();
		btScalar linear_damping = self->body->getLinearDamping();
		btScalar angular_damping = self->body->getAngularDamping();
		btScalar restitution = self->body->getRestitution();
		int activation_state = self->body->getActivationState();
		int collision_flags = self->body->getCollisionFlags();
		int flags = self->body->getFlags();
		int group = self->body->getBroadphaseHandle()->m_collisionFilterGroup;
		int mask = self->body->getBroadphaseHandle()->m_collisionFilterMask;
		return Py_BuildValue(
			"{s((fff)(ffff))s(fff)s(fff)s(f(fff))s(fff)s(fff)sfsfsfs((fff)i)s(ff)s(ff)sfsisisisisi}",
			"world_transform", va_vec(origin), va_quat(rotation),
			"linear_velocity", va_vec(linear_velocity),
			"angular_velocity", va_vec(angular_velocity),
			"mass_props", mass, va_vec(inertia),
			"linear_factor", va_vec(linear_factor),
			"angular_factor", va_vec(angular_factor),
			"linear_friction", linear_friction,
			"spinning_friction", spinning_friction,
			"rolling_friction", rolling_friction,
			"anisotropic_friction", va_vec(anisotropic_friction), anisotropic_friction_mode,
			"contact_stiffness_and_damping", contact_stiffness, contact_damping,
			"damping", linear_damping, angular_damping,
			"restitution", restitution,
			"activation_state", activation_state,
			"collision_flags", collision_flags,
			"flags", flags,
			"group", group,
			"mask", mask
		);
	}

	if (PyObject * value = PyDict_GetItemString(config, "world_transform")) {
		self->body->setWorldTransform(get_transform(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "linear_velocity")) {
		self->body->setLinearVelocity(get_vector(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "angular_velocity")) {
		self->body->setAngularVelocity(get_vector(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "mass_props")) {
		bi_ensure(PyObject_Length(value) == 2);
		self->body->setMassProps(
			PyFloat_AsDouble(PySequence_GetItem(value, 0)),
			get_vector(PySequence_GetItem(value, 1))
		);
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "linear_factor")) {
		self->body->setLinearFactor(get_vector(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "angular_factor")) {
		self->body->setAngularFactor(get_vector(value));
		Py_DECREF(value);
	}

	if (PyObject * value = PyDict_GetItemString(config, "linear_friction")) {
		self->body->setFriction(PyFloat_AsDouble(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "spinning_friction")) {
		self->body->setSpinningFriction(PyFloat_AsDouble(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "rolling_friction")) {
		self->body->setRollingFriction(PyFloat_AsDouble(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "anisotropic_friction")) {
		bi_ensure(PyObject_Length(value) == 2);
		self->body->setAnisotropicFriction(
			get_vector(PySequence_GetItem(value, 0)),
			PyLong_AsLong(PySequence_GetItem(value, 1))
		);
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "contact_stiffness_and_damping")) {
		bi_ensure(PyObject_Length(value) == 2);
		self->body->setContactStiffnessAndDamping(
			PyFloat_AsDouble(PySequence_GetItem(value, 0)),
			PyFloat_AsDouble(PySequence_GetItem(value, 1))
		);
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "damping")) {
		bi_ensure(PyObject_Length(value) == 2);
		self->body->setDamping(
			PyFloat_AsDouble(PySequence_GetItem(value, 0)),
			PyFloat_AsDouble(PySequence_GetItem(value, 1))
		);
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "restitution")) {
		self->body->setRestitution(PyFloat_AsDouble(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "activation_state")) {
		self->body->setActivationState(PyLong_AsLong(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "collision_flags")) {
		self->body->setCollisionFlags(PyLong_AsLong(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "flags")) {
		self->body->setFlags(PyLong_AsLong(value));
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "group")) {
		self->body->getBroadphaseHandle()->m_collisionFilterGroup = PyLong_AsLong(value);
		Py_DECREF(value);
	}
	if (PyObject * value = PyDict_GetItemString(config, "mask")) {
		self->body->getBroadphaseHandle()->m_collisionFilterMask = PyLong_AsLong(value);
		Py_DECREF(value);
	}
	bi_ensure(!PyErr_Occurred());
	Py_RETURN_NONE;
}

PyObject * BIRigidBody_meth_remove(BIRigidBody * self) {
	/*
		Remove a constraint from the world.
	*/

	// do not loose our object while removing it from other containers
	Py_INCREF(self);

	// release _obj
	init_slot(self->wrapper, "_obj", new_ref(Py_None));

	// release the world
	init_slot(self->wrapper, "world", new_ref(Py_None));

	int num_groups = (int)PyList_GET_SIZE(self->groups_slot);
	while (num_groups--) {
		// get the internal object
		BIGroup * group = get_slot(PyList_GET_ITEM(self->groups_slot, num_groups), BIGroup, "_obj");
		// find the body in the group
		Py_ssize_t index = PySequence_Index(group->bodies_slot, self->wrapper);
		// remove body from bodies
		PySequence_DelItem(group->bodies_slot, index);
		// remove body from internal bodies
		PySequence_DelItem(group->bodies, index);
	}

	int num_constraints = (int)PyList_GET_SIZE(self->constraints_slot);
	while (num_constraints--) {
		// remove constraints that contain this body
		Py_XDECREF(PyObject_CallMethod(PyList_GET_ITEM(self->constraints_slot, num_constraints), "remove", 0));
		bi_ensure(!PyErr_Occurred());
	}

	// remove bullet rigid body
	self->world->dynamics_world->removeRigidBody(self->body);

	// check for errors
	bi_ensure(!PyErr_Occurred());

	// loose our object
	Py_DECREF(self);
	Py_RETURN_NONE;
}

int BIRigidBody_set_stiffness(BIRigidBody * self, PyObject * value) {
	/*
		Set the stiffness and damping
	*/

	// parameters
	btScalar contact_stiffness;
	btScalar contact_damping;
	int contact_stiffness_flag;

	// parse args
	if (!PyArg_ParseTuple(value, "ddp", &contact_stiffness, &contact_damping, &contact_stiffness_flag)) {
		return -1;
	}

	// change the stiffness and damping
	// this call will set the CF_HAS_CONTACT_STIFFNESS_DAMPING flag
	self->body->setContactStiffnessAndDamping(contact_stiffness, contact_damping);

	// handle the collision flags
	if (contact_stiffness_flag) {
		self->body->setCollisionFlags(self->body->getCollisionFlags() | btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING);
	} else {
		self->body->setCollisionFlags(self->body->getCollisionFlags() & ~btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING);
	}

	// store the stiffness values in the wrapper
	init_slot(self->wrapper, "_stiffness", Py_BuildValue("ddO", contact_stiffness, contact_damping, contact_stiffness_flag ? Py_True : Py_False));
	return 0;
}

int BIRigidBody_set_friction(BIRigidBody * self, PyObject * value) {
	/*
		Set the linear_friction, rolling_friction and spinning_friction
	*/

	// parameters
	btScalar linear_friction;
	btScalar rolling_friction;
	btScalar spinning_friction;

	// parse args
	if (!PyArg_ParseTuple(value, "ddd", &linear_friction, &rolling_friction, &spinning_friction)) {
		return -1;
	}

	// modify the bullet rigid body
	self->body->setFriction(linear_friction);
	self->body->setRollingFriction(rolling_friction);
	self->body->setSpinningFriction(spinning_friction);

	// store the frition values in the wrapper
	init_slot(self->wrapper, "_friction", Py_BuildValue("ddd", linear_friction, rolling_friction, spinning_friction));
	return 0;
}

PyObject * BIRigidBody_get_transform(BIRigidBody * self) {
	/*
		Return the world transform of the rigid body
	*/
	btVector3 origin = self->body->getWorldTransform().getOrigin();
	btQuaternion rotation = self->body->getWorldTransform().getRotation();
	return Py_BuildValue("(ddd)(dddd)", origin.x(), origin.y(), origin.z(), rotation.x(), rotation.y(), rotation.z(), rotation.w());
}

PyObject * BIRigidBody_get_origin(BIRigidBody * self) {
	/*
		Return the origin of the rigid body in world
	*/
	btVector3 origin = self->body->getWorldTransform().getOrigin();
	return Py_BuildValue("ddd", origin.x(), origin.y(), origin.z());
}

int BIRigidBody_set_origin(BIRigidBody * self, PyObject * value) {
	/*
		Deprecated origin setter
	*/
	btTransform transform = self->body->getWorldTransform();
	transform.setOrigin(get_vector(value));
	self->body->setWorldTransform(transform);
	return 0;
}

PyObject * BIRigidBody_get_basis(BIRigidBody * self) {
	/*
		Return the basis of the rigid body in world
	*/
	btMatrix3x3 basis = self->body->getWorldTransform().getBasis();
	return Py_BuildValue(
		"ddddddddd",
		basis.getRow(0).x(),
		basis.getRow(0).y(),
		basis.getRow(0).z(),
		basis.getRow(1).x(),
		basis.getRow(1).y(),
		basis.getRow(1).z(),
		basis.getRow(2).x(),
		basis.getRow(2).y(),
		basis.getRow(2).z()
	);
}

int BIRigidBody_set_basis(BIRigidBody * self, PyObject * value) {
	/*
		Deprecated basis setter
	*/
	btTransform transform = self->body->getWorldTransform();
	transform.setRotation(get_quaternion(value));
	self->body->setWorldTransform(transform);
	return 0;
}

void BIRigidBody_dealloc(BIRigidBody * self) {
	Py_TYPE(self)->tp_free(self);
}

PyMethodDef BIRigidBody_methods[] = {
	{"contacts", (PyCFunction)BIRigidBody_meth_contacts, METH_VARARGS, 0},
	{"penetration", (PyCFunction)BIRigidBody_meth_penetration, METH_VARARGS, 0},
	{"apply_force", (PyCFunction)BIRigidBody_meth_apply_force, METH_VARARGS, 0},
	{"apply_torque", (PyCFunction)BIRigidBody_meth_apply_torque, METH_O, 0},
	{"config", (PyCFunction)BIRigidBody_meth_config, METH_O, 0},
	{"remove", (PyCFunction)BIRigidBody_meth_remove, METH_NOARGS, 0},
	{0},
};

PyGetSetDef BIRigidBody_getset[] = {
	{"stiffness", 0, (setter)BIRigidBody_set_stiffness, 0, 0},
	{"friction", 0, (setter)BIRigidBody_set_friction, 0, 0},
	{"transform", (getter)BIRigidBody_get_transform, 0, 0},
	{"origin", (getter)BIRigidBody_get_origin, (setter)BIRigidBody_set_origin, 0, 0},
	{"basis", (getter)BIRigidBody_get_basis, (setter)BIRigidBody_set_basis, 0, 0},
	{0},
};

PyMemberDef BIRigidBody_members[] = {
	{"_groups_slot", T_OBJECT_EX, offsetof(BIRigidBody, groups_slot), READONLY, 0},
	{"_constraints_slot", T_OBJECT_EX, offsetof(BIRigidBody, constraints_slot), READONLY, 0},
	{0},
};

PyType_Slot BIRigidBody_slots[] = {
	{Py_tp_methods, BIRigidBody_methods},
	{Py_tp_getset, BIRigidBody_getset},
	{Py_tp_members, BIRigidBody_members},
	{Py_tp_dealloc, (void *)BIRigidBody_dealloc},
	{0},
};

PyTypeObject * BIRigidBody_type;

PyType_Spec BIRigidBody_spec = {"mollia_bullet.core.RigidBody", sizeof(BIRigidBody), 0, Py_TPFLAGS_DEFAULT, BIRigidBody_slots};
