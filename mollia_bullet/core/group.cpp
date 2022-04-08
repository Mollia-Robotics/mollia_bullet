#include "group.hpp"

#include "world.hpp"
#include "color_mesh.hpp"
#include "rigid_body.hpp"

PyObject * BIWorld_meth_group(BIWorld * self, PyObject * args) {
	/*
		Return a group of bodies.
		The return value is a python Group object with an _obj propery referencing the internal BIGroup object.
		The user must not use the _obj directly.
	*/

	// parameters
	PyObject * bodies;

	// parse parameters
	if (!PyArg_ParseTuple(args, "O", &bodies)) {
		return 0;
	}

	// create a new internal object
	BIGroup * group = PyObject_New(BIGroup, BIGroup_type);

	// wrap with a python Group
	static PyTypeObject * wrapper_type = get_wrapper("Group");
	bi_ensure(wrapper_type);
	group->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
	if (!group->wrapper) {
		return 0;
	}

	// fill the internal object
	group->bodies_slot = PySequence_List(bodies);
	group->world = self;
	bi_ensure(group->bodies_slot);

	int num_bodies = (int)PyList_GET_SIZE(group->bodies_slot);
	group->bodies = PyList_New(num_bodies);

	for (int i = 0; i < num_bodies; ++i) {
		// take the body wrapper
		BIRigidBody * rbody = get_slot(PyList_GET_ITEM(bodies, i), BIRigidBody, "_obj");
		// ensure its type
		bi_ensure(rbody && Py_TYPE(rbody) == BIRigidBody_type);
		// store the internal object in a separate list
		PyList_SET_ITEM(group->bodies, i, (PyObject *)new_ref(rbody));
		// store group
		PyList_Append(rbody->groups_slot, group->wrapper);
	}

	// init group slots
	init_slot(group->wrapper, "_obj", group);
	init_slot(group->wrapper, "bodies", group->bodies_slot);
	init_slot(group->wrapper, "world", new_ref(self->wrapper));

	// store a ref in groups
	PyList_Append(self->groups_slot, group->wrapper);

	// return the wrapper
	bi_ensure(!PyErr_Occurred());
	return group->wrapper;
}

PyObject * BIGroup_meth_remove(BIGroup * self) {
	/*
		Remove the group and all the objects in the group
	*/

	// do not loose our object while removing it from other containers
	Py_INCREF(self);

	// release _obj
	init_slot(self->wrapper, "_obj", new_ref(Py_None));

	// release the world
	init_slot(self->wrapper, "world", new_ref(Py_None));

	int num_bodies = (int)PyList_GET_SIZE(self->bodies);
	while (num_bodies--) {
		Py_DECREF(BIRigidBody_meth_remove((BIRigidBody *)PyList_GET_ITEM(self->bodies, num_bodies)));
	}

	PyObject_SetAttrString(self->wrapper, "_obj", new_ref(Py_None));
	PySequence_DelItem(self->world->groups_slot, PySequence_Index(self->world->groups_slot, self->wrapper));

	bi_ensure(!PyErr_Occurred());

	// loose our object
	Py_DECREF(self);
	Py_RETURN_NONE;
}

PyObject * BIGroup_meth_center_of_mass(BIGroup * self) {
	/*
		Return the center of mass in world
	*/

	btScalar total_mass = 0.0;
	btVector3 center_of_mass = btVector3(0.0, 0.0, 0.0);

	int num_bodies = (int)PyList_GET_SIZE(self->bodies);
	for (int i = 0; i < num_bodies; ++i) {
		// take an internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		btScalar mass = 1.0 / rbody->body->getInvMass();
		center_of_mass += rbody->body->getCenterOfMassPosition() * mass;
		total_mass += mass;
	}
	center_of_mass /= total_mass;
	return Py_BuildValue("ddd", center_of_mass.x(), center_of_mass.y(), center_of_mass.z());
}

PyObject * BIGroup_meth_aabb(BIGroup * self) {
	/*
		Return the axis aligned bounding box
	*/

	int num_bodies = (int)PyList_GET_SIZE(self->bodies);
	if (!num_bodies) {
		// return an empty box
		return Py_BuildValue("(ddd)(ddd)", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	}

	// measure the first object
	btVector3 minAabb;
	btVector3 maxAabb;
	BIRigidBody * first = (BIRigidBody *)PyList_GET_ITEM(self->bodies, 0);
	first->body->getAabb(minAabb, maxAabb);

	// min max with the other object
	for (int i = 1; i < num_bodies; ++i) {
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		btVector3 minAabbTemp;
		btVector3 maxAabbTemp;
		rbody->body->getAabb(minAabbTemp, maxAabbTemp);
		minAabb.setMin(minAabbTemp);
		maxAabb.setMax(maxAabbTemp);
	}

	return Py_BuildValue("(ddd)(ddd)", minAabb.x(), minAabb.y(), minAabb.z(), maxAabb.x(), maxAabb.y(), maxAabb.z());
}

PyObject * BIGroup_meth_save_state(BIGroup * self) {
	/*
		Serialize the state
	*/

	// the number of bodies
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);

	// the size of the result in bytes
	int size = num_bodies * 13 * sizeof(btScalar);

	// allocate a bytes object
	PyObject * res = PyBytes_FromStringAndSize(0, size);
	btScalar * ptr = (btScalar *)PyBytes_AS_STRING(res);

	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		const btVector3 & origin = rbody->body->getWorldTransform().getOrigin();
		const btQuaternion & rotation = rbody->body->getWorldTransform().getRotation();
		const btVector3 & linear_velocity = rbody->body->getLinearVelocity();
		const btVector3 & angular_velocity = rbody->body->getAngularVelocity();
		// store the origin
		*ptr++ = origin.x();
		*ptr++ = origin.y();
		*ptr++ = origin.z();
		// store the rotation
		*ptr++ = rotation.x();
		*ptr++ = rotation.y();
		*ptr++ = rotation.z();
		*ptr++ = rotation.w();
		// store the linear_velocity
		*ptr++ = linear_velocity.x();
		*ptr++ = linear_velocity.y();
		*ptr++ = linear_velocity.z();
		// store the angular_velocity
		*ptr++ = angular_velocity.x();
		*ptr++ = angular_velocity.y();
		*ptr++ = angular_velocity.z();
	}

	// the return value is a bytes object
	return res;
}

PyObject * BIGroup_meth_load_state(BIGroup * self, PyObject * savedata) {
	/*
		Load the
	*/

	// the number of bodies
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);

	// the size of the state in bytes
	int size = num_bodies * 13 * sizeof(btScalar);
	if (PyBytes_Size(savedata) != size) {
		PyErr_BadInternalCall();
		return 0;
	}

	btScalar * ptr = (btScalar *)PyBytes_AS_STRING(savedata);

	for (int i = 0; i < num_bodies; ++i) {
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		// load the origin
		btVector3 origin = btVector3(ptr[0], ptr[1], ptr[2]);
		ptr += 3;
		// load the rotation
		btQuaternion rotation = btQuaternion(ptr[0], ptr[1], ptr[2], ptr[3]);
		ptr += 4;
		// load the linear_velocity
		btVector3 linear_velocity = btVector3(ptr[0], ptr[1], ptr[2]);
		ptr += 3;
		// load the angular_velocity
		btVector3 angular_velocity = btVector3(ptr[0], ptr[1], ptr[2]);
		ptr += 3;

		// change the internal state of the bullet rigid body
		rbody->body->setWorldTransform(btTransform(rotation, origin));
		rbody->body->setLinearVelocity(linear_velocity);
		rbody->body->setAngularVelocity(angular_velocity);

		// clear cache for the changed objects
		rbody->world->dynamics_world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
			rbody->body->getBroadphaseHandle(),
			rbody->world->dynamics_world->getDispatcher()
		);
	}

	Py_RETURN_NONE;
}

PyObject * BIGroup_meth_apply_transform(BIGroup * self, PyObject * arg) {
	/*
		Transform all the objects in a group
	*/

	btTransform transform = get_transform(arg);
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);
	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		// apply transform from the left
		rbody->body->setWorldTransform(transform * rbody->body->getWorldTransform());
	}
	Py_RETURN_NONE;
}

PyObject * BIGroup_meth_apply_force(BIGroup * self, PyObject * arg) {
	/*
		Batch apply force
	*/

	// iterate on the input sequence
	PyObject * seq = PySequence_Fast(arg, "not iterable");
	bi_ensure(seq);

	// the number of items
	int num_forces = (int)PySequence_Fast_GET_SIZE(seq);
	PyObject ** forces = PySequence_Fast_ITEMS(seq);
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);
	bi_ensure(num_forces == num_bodies);

	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		rbody->body->applyCentralForce(get_vector(forces[i]));
	}

	Py_RETURN_NONE;
}

PyObject * BIGroup_meth_apply_torque(BIGroup * self, PyObject * arg) {
	/*
		Batch apply force
	*/

	// iterate on the input sequence
	PyObject * seq = PySequence_Fast(arg, "not iterable");
	bi_ensure(seq);

	// the number of items
	int num_torques = (int)PySequence_Fast_GET_SIZE(seq);
	PyObject ** torques = PySequence_Fast_ITEMS(seq);
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);
	bi_ensure(num_torques == num_bodies);

	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		rbody->body->applyTorque(get_vector(torques[i]));
	}

	Py_RETURN_NONE;
}

PyObject * BIGroup_meth_transforms(BIGroup * self) {
	/*
		Return the glsl friendly representation of the world transform.
		Can be useful for instanced rendering.
	*/

	// the number of bodies
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);

	// the size of the result in bytes
	int size = num_bodies * 12 * sizeof(float);

	// allocate result
	PyObject * res = PyBytes_FromStringAndSize(0, size);
	float * ptr = (float *)PyBytes_AS_STRING(res);

	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		const btVector3 & origin = rbody->body->getWorldTransform().getOrigin();
		const btMatrix3x3 & basis = rbody->body->getWorldTransform().getBasis();
		// glsl vec3 origin
		*ptr++ = (float)origin.x();
		*ptr++ = (float)origin.y();
		*ptr++ = (float)origin.z();
		// glsl mat3 basis
		*ptr++ = (float)basis.getRow(0).x();
		*ptr++ = (float)basis.getRow(1).x();
		*ptr++ = (float)basis.getRow(2).x();
		*ptr++ = (float)basis.getRow(0).y();
		*ptr++ = (float)basis.getRow(1).y();
		*ptr++ = (float)basis.getRow(2).y();
		*ptr++ = (float)basis.getRow(0).z();
		*ptr++ = (float)basis.getRow(1).z();
		*ptr++ = (float)basis.getRow(2).z();
	}

	// the return value is a bytes object
	return res;
}

inline void write_vertex(float *& ptr, float *& src, const btTransform & transform) {
	/*
		Helper function to write color_mesh
	*/
	btVector3 vertex = transform * btVector3(src[0], src[1], src[2]);
	*ptr++ = (float)vertex.x();
	*ptr++ = (float)vertex.y();
	*ptr++ = (float)vertex.z();
	src += 3;
}

inline void write_normal(float *& ptr, float *& src, const btMatrix3x3 & basis) {
	/*
		Helper function to write color_mesh
	*/
	btVector3 normal = basis * btVector3(src[0], src[1], src[2]);
	*ptr++ = (float)normal.x();
	*ptr++ = (float)normal.y();
	*ptr++ = (float)normal.z();
	src += 3;
}

PyObject * BIGroup_meth_color_mesh(BIGroup * self) {
	/*
		Calculate the color mesh.
	*/

	// the number of bodies
	int num_bodies = (int)PyList_GET_SIZE(self->bodies);

	// the number of vertices in the output
	int vertices = 0;
	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		// skip objects with visible=False
		if (!PyObject_IsTrue(get_slot(rbody->wrapper, PyObject, "visible"))) {
			continue;
		}
		vertices += color_mesh::mesh_vertices(rbody->body->getCollisionShape());
	}

	// allocate the result
	PyObject * res = PyBytes_FromStringAndSize(NULL, vertices * sizeof(color_mesh::Vertex));
	color_mesh::Vertex * ptr = (color_mesh::Vertex *)PyBytes_AS_STRING(res);

	// write mesh data
	for (int i = 0; i < num_bodies; ++i) {
		// take the internal rigid body object
		BIRigidBody * rbody = (BIRigidBody *)PyList_GET_ITEM(self->bodies, i);
		// skip objects with visible=False
		if (!PyObject_IsTrue(get_slot(rbody->wrapper, PyObject, "visible"))) {
			continue;
		}
		color_mesh::vec3 color = color_mesh::vec3_from_bt(get_vector(get_slot(rbody->wrapper, PyObject, "color")));
		color_mesh::write_mesh(ptr, rbody->body->getWorldTransform(), rbody->body->getCollisionShape(), color);
	}

	// the return value is a bytes object
	return res;
}

void BIGroup_dealloc(BIGroup * self) {
	Py_DECREF(self->bodies);
	Py_TYPE(self)->tp_free(self);
}

PyMethodDef BIGroup_methods[] = {
	{"remove", (PyCFunction)BIGroup_meth_remove, METH_NOARGS, 0},
	{"save_state", (PyCFunction)BIGroup_meth_save_state, METH_NOARGS, 0},
	{"load_state", (PyCFunction)BIGroup_meth_load_state, METH_O, 0},
	{"apply_transform", (PyCFunction)BIGroup_meth_apply_transform, METH_O, 0},
	{"apply_force", (PyCFunction)BIGroup_meth_apply_force, METH_O, 0},
	{"apply_torque", (PyCFunction)BIGroup_meth_apply_torque, METH_O, 0},
	{"aabb", (PyCFunction)BIGroup_meth_aabb, METH_NOARGS, 0},
	{"transforms", (PyCFunction)BIGroup_meth_transforms, METH_NOARGS, 0},
	{"center_of_mass", (PyCFunction)BIGroup_meth_center_of_mass, METH_NOARGS, 0},
	{"color_mesh", (PyCFunction)BIGroup_meth_color_mesh, METH_NOARGS, 0},
	{0},
};

PyMemberDef BIGroup_members[] = {
	{"_bodies_slot", T_OBJECT_EX, offsetof(BIGroup, bodies_slot), READONLY, 0},
	{"_bodies", T_OBJECT_EX, offsetof(BIGroup, bodies), READONLY, 0},
	{0},
};

PyType_Slot BIGroup_slots[] = {
	{Py_tp_methods, BIGroup_methods},
	{Py_tp_members, BIGroup_members},
	{Py_tp_dealloc, (void *)BIGroup_dealloc},
	{0},
};

PyTypeObject * BIGroup_type;

PyType_Spec BIGroup_spec = {"mollia_bullet.core.Group", sizeof(BIGroup), 0, Py_TPFLAGS_DEFAULT, BIGroup_slots};
