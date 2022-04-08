#include "common.hpp"

#include "constraint.hpp"
#include "group.hpp"
#include "motor_control.hpp"
#include "rigid_body.hpp"
#include "world.hpp"

PyMethodDef methods[] = {
	{"world", (PyCFunction)meth_world, METH_VARARGS, 0},
	{0},
};

PyModuleDef moduledef = {PyModuleDef_HEAD_INIT, "mollia_bullet.core", 0, -1, methods, 0, 0, 0, 0};

#define xstr(s) str(s)
#define str(s) #s

extern "C" PyObject * PyInit_core() {
	PyObject * module = PyModule_Create(&moduledef);

	// create internal types
	BIWorld_type = (PyTypeObject *)PyType_FromSpec(&BIWorld_spec);
	BIRigidBody_type = (PyTypeObject *)PyType_FromSpec(&BIRigidBody_spec);
	BIConstraint_type = (PyTypeObject *)PyType_FromSpec(&BIConstraint_spec);
	BIMotorControl_type = (PyTypeObject *)PyType_FromSpec(&BIMotorControl_spec);
	BIGroup_type = (PyTypeObject *)PyType_FromSpec(&BIGroup_spec);

	// register internal types
	PyModule_AddObject(module, "World", (PyObject *)new_ref(BIWorld_type));
	PyModule_AddObject(module, "RigidBody", (PyObject *)new_ref(BIRigidBody_type));
	PyModule_AddObject(module, "Constraint", (PyObject *)new_ref(BIConstraint_type));
	PyModule_AddObject(module, "MotorControl", (PyObject *)new_ref(BIMotorControl_type));
	PyModule_AddObject(module, "Group", (PyObject *)new_ref(BIGroup_type));

	return module;
}
