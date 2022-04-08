#pragma once
#include <Python.h>
#include <structmember.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btMinkowskiSumShape.h>

#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>

#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#endif

struct BIBaseObject {
	PyObject_HEAD
	PyObject * wrapper;
};

struct Trace {
	const char * function;
	const char * filename;
	int line;
};

inline void _bi_fatal_error(const Trace & trace) {
	#if defined(_WIN32) || defined(_WIN64)
	DWORD mode = 0;
	GetConsoleMode(GetStdHandle(STD_OUTPUT_HANDLE), &mode);
	SetConsoleMode(GetStdHandle(STD_OUTPUT_HANDLE), mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
	#endif
	printf("%s: \x1b[33m%s:%d\x1b[m\n", trace.function, trace.filename, trace.line);
	if (PyObject * err = PyErr_Occurred()) {
		PyObject * exc_type;
		PyObject * exc_value;
		PyObject * exc_traceback;
		PyErr_Fetch(&exc_type, &exc_value, &exc_traceback);
		printf("%s: \x1b[33m%s\x1b[m\n", ((PyTypeObject *)exc_type)->tp_name, PyUnicode_AsUTF8(PyObject_Str(exc_value)));
	}
	printf("\n\x1b[31mStack Trace:\n");
	PyObject * traceback = PyImport_ImportModule("traceback");
	PyObject * print_stack = PyObject_GetAttrString(traceback, "print_stack");
	PyObject_CallFunction(print_stack, NULL);
	printf("  File \"%s\", line %d, in <mollia_bullet.core>\n", trace.filename, trace.line);
	printf("    return %s(...)\x1b[m\n\n", trace.function);
	PyObject * code = PyImport_ImportModule("code");
	PyObject * interact = PyObject_GetAttrString(code, "interact");
	PyObject_CallFunction(interact, "sOOs", "", Py_None, PyEval_GetLocals(), "");
	exit(0);
}

#define bi_fatal() _bi_fatal_error({__FUNCTION__, __FILE__, __LINE__})
#define bi_fatal_deep() _bi_fatal_error(trace)
#define bi_ensure(cond) if (!(cond)) bi_fatal();
#define bi_ensure_deep(cond) if (!(cond)) bi_fatal_deep();

static btVector3 default_vector = btVector3(0.0, 0.0, 0.0);
static btQuaternion default_quaternion = btQuaternion(0.0, 0.0, 0.0, 1.0);
static btTransform default_transform = btTransform(default_quaternion, default_vector);

#define new_ref(obj) (Py_INCREF(obj), obj)

template <typename T>
T clamp(const T & x, const T & a, const T & b) {
	if (x < a) {
		return a;
	}
	if (x > b) {
		return b;
	}
	return x;
}

inline btVector3 _get_vector(const Trace & trace, PyObject * obj, bool none=false, const btVector3 & fallback=default_vector) {
	/*
		Return a bullet btVector3 from obj or return fallback if obj is None and none is true
	*/
	if (obj == Py_None && none) {
		return fallback;
	}
	PyObject * seq = PySequence_Fast(obj, "not iterable");
	bi_ensure_deep(seq);
	if (PySequence_Fast_GET_SIZE(seq) == 3) {
		// accept vector
		btVector3 res = btVector3(
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 0)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 1)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 2))
		);
		Py_DECREF(seq);
		bi_ensure_deep(!PyErr_Occurred());
		return res;
	}
	bi_fatal_deep();
	return fallback;
}

inline btQuaternion _get_quaternion(const Trace & trace, PyObject * obj, bool none=false, const btQuaternion & fallback=default_quaternion) {
	/*
		Return a bullet btQuaternion from obj or return fallback if obj is None and none is true
	*/
	if (obj == Py_None && none) {
		return fallback;
	}
	PyObject * seq = PySequence_Fast(obj, "not iterable");
	bi_ensure_deep(seq);
	if (PySequence_Fast_GET_SIZE(seq) == 4) {
		// accept quaternion
		btQuaternion res = btQuaternion(
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 0)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 1)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 2)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 3))
		);
		Py_DECREF(seq);
		bi_ensure_deep(!PyErr_Occurred());
		return res.normalize();
	}
	if (PySequence_Fast_GET_SIZE(seq) == 9) {
		// accept matrix
		btQuaternion res;
		btMatrix3x3(
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 0)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 1)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 2)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 3)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 4)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 5)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 6)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 7)),
			PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 8))
		).getRotation(res);
		Py_DECREF(seq);
		bi_ensure_deep(!PyErr_Occurred());
		return res.normalize();
	}
	bi_fatal();
	return fallback;
}

inline btTransform _get_transform(const Trace & trace, PyObject * obj, bool none=false, const btTransform & fallback=default_transform) {
	/*
		Return a bullet btTransform from obj or return fallback if obj is None and none is true
	*/
	if (obj == Py_None && none) {
		return fallback;
	}
	PyObject * seq = PySequence_Fast(obj, "not iterable");
	bi_ensure_deep(seq);
	if (PySequence_Fast_GET_SIZE(seq) == 2) {
		// accept (vector, quaternion) or (vector, matrix)
		btTransform res = btTransform(
			_get_quaternion(trace, PySequence_Fast_GET_ITEM(seq, 1), true),
			_get_vector(trace, PySequence_Fast_GET_ITEM(seq, 0), true)
		);
		Py_DECREF(seq);
		bi_ensure_deep(!PyErr_Occurred());
		return res;
	}
	bi_fatal();
	return fallback;
}

#define get_vector(...) _get_vector({__FUNCTION__, __FILE__, __LINE__}, __VA_ARGS__)
#define get_quaternion(...) _get_quaternion({__FUNCTION__, __FILE__, __LINE__}, __VA_ARGS__)
#define get_transform(...) _get_transform({__FUNCTION__, __FILE__, __LINE__}, __VA_ARGS__)

inline PyTypeObject * get_wrapper(const char * name) {
	/*
		Return a class declared in python
	*/

	// lookup mollia_bullet
	PyObject * module_name = PyUnicode_FromString("mollia_bullet");
	bi_ensure(module_name);
	PyObject * mod = PyImport_GetModule(module_name);
	Py_DECREF(module_name);
	bi_ensure(mod);

	// get wrapper from mollia_bullet
	PyTypeObject * wrapper = (PyTypeObject *)PyObject_GetAttrString(mod, name);
	bi_ensure(wrapper);

	// ensure wrapper is a class
	bi_ensure(wrapper->tp_flags & Py_TPFLAGS_BASETYPE);

	// return a new ref
	Py_INCREF(wrapper);
	return wrapper;
}

inline int init_slot(PyObject * obj, const char * key, void * value) {
	/*
		PyObject_SetAttrString without incref
	*/
	int res = PyObject_SetAttrString(obj, key, (PyObject *)value);
	bi_ensure(!PyErr_Occurred());
	Py_DECREF(value);
	return res;
}

inline PyObject * _get_slot(PyObject * obj, const char * key) {
	/*
		PyObject_GetAttrString without incref
	*/
	PyObject * res = PyObject_GetAttrString(obj, key);
	bi_ensure(res);
	Py_DECREF(res);
	return res;
}

#define get_slot(obj, type, key) (type *)_get_slot(obj, key)
