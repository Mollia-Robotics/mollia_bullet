#include "motor_control.hpp"

#include "constraint.hpp"
#include "world.hpp"

PyObject * BIMotorControl_meth_reset(BIMotorControl * self);

PyObject * BIWorld_meth_motor_control(BIWorld * self, PyObject * args) {
	/*
		Return a velocity motor control.
		The return value is a python MotorControl object with an _obj propery referencing the internal BIMotorControl object.
		The user must not use the _obj directly.
	*/

	// parameters
	PyObject * motors;

	// parse parameters
	if (!PyArg_ParseTuple(args, "O", &motors)) {
		return 0;
	}

	// create a new internal object
	BIMotorControl * motor_control = PyObject_New(BIMotorControl, BIMotorControl_type);

	// wrap with a python MotorControl
	static PyTypeObject * wrapper_type = get_wrapper("MotorControl");
	bi_ensure(wrapper_type);
	motor_control->wrapper = PyObject_CallObject((PyObject *)wrapper_type, 0);
	if (!motor_control->wrapper) {
		return 0;
	}

	// fill the internal object
	motor_control->motors_slot = PySequence_List(motors);
	motor_control->world = self;
	bi_ensure(motor_control->motors_slot);

	// create a list with the internal objects to avoid dict lookup in simulate
	int num_motors = (int)PyList_GET_SIZE(motor_control->motors_slot);
	motor_control->motors = PyList_New(num_motors);

	// input and output size in bytes
	int input_data_size = sizeof(BIMotorData) * num_motors;
	int output_data_size = sizeof(btScalar) * num_motors;

	// allocate a single array for both input and output
	char * data = (char *)malloc(input_data_size + output_data_size * 2);
	memset(data, 0, input_data_size + output_data_size * 2);
	motor_control->input_data = (BIMotorData *)data;

	// create some memory views
	PyObject * input_mem = PyMemoryView_FromMemory(data, input_data_size, PyBUF_WRITE);
	motor_control->output_mem[0] = PyMemoryView_FromMemory(data + input_data_size, output_data_size, PyBUF_READ);
	motor_control->output_mem[1] = PyMemoryView_FromMemory(data + input_data_size + output_data_size, output_data_size, PyBUF_READ);
	motor_control->output_index = 0;

	// import numpy
	PyObject * numpy = PyImport_ImportModule("numpy");
	if (!numpy) {
		return 0;
	}

	// take ndarray from numpy
	PyObject * ndarray = PyObject_GetAttrString(numpy, "ndarray");
	if (!ndarray) {
		return 0;
	}

	// create an ndarray for the input
	PyObject * input_array = PyObject_CallFunction(ndarray, "(ii)sO", num_motors, 2, "f8", input_mem);
	if (!input_array) {
		return 0;
	}

	// create the front buffer for the output
	// this array is used for double buffering the output and store the velocity with no extra free/alloc cost
	motor_control->output_array[0] = PyObject_CallFunction(ndarray, "isO", num_motors, "f8", motor_control->output_mem[0]);
	if (!motor_control->output_array[0]) {
		return 0;
	}

	// create the back buffer for the output
	// this array is used for double buffering the output and store the velocity with no extra free/alloc cost
	motor_control->output_array[1] = PyObject_CallFunction(ndarray, "isO", num_motors, "f8", motor_control->output_mem[1]);
	if (!motor_control->output_array[1]) {
		return 0;
	}

	for (int i = 0; i < num_motors; ++i) {
		// take the motor wrapper
		PyObject * motor_wrapper = PyList_GET_ITEM(motor_control->motors_slot, i);
		// take the internal constraint object
		BIConstraint * motor = get_slot(motor_wrapper, BIConstraint, "_obj");
		// ensure its type
		bi_ensure(motor && Py_TYPE(motor) == BIConstraint_type);
		// store the internal object in a separate list
		PyList_SET_ITEM(motor_control->motors, i, (PyObject *)new_ref(motor));
		// a constraint can be controller by a single motor_control
		bi_ensure(!motor->motor_control);
		// store motor_control
		init_slot(motor->wrapper, "motor_control", new_ref(motor_control->wrapper));
		motor->motor_control = motor_control;
		// enable motor
		motor->hinge->enableMotor(true);
	}

	// init motor_control slots
	init_slot(motor_control->wrapper, "_obj", motor_control);
	init_slot(motor_control->wrapper, "motors", motor_control->motors_slot);
	init_slot(motor_control->wrapper, "world", new_ref(self->wrapper));
	init_slot(motor_control->wrapper, "input_mem", input_mem);
	init_slot(motor_control->wrapper, "input_array", input_array);

	// store a ref in motor_controls
	PyList_Append(self->motor_controls_slot, motor_control->wrapper);
	PyList_Append(self->motor_controls, (PyObject *)motor_control);

	// read the motor angles
	Py_XDECREF(BIMotorControl_meth_reset(motor_control));

	// return the wrapper
	bi_ensure(!PyErr_Occurred());
	return motor_control->wrapper;
}

PyObject * BIMotorControl_meth_remove(BIMotorControl * self) {
	// do not loose our object while removing it from other containers
	Py_INCREF(self);

	// release _obj
	init_slot(self->wrapper, "_obj", new_ref(Py_None));

	// release the world
	init_slot(self->wrapper, "world", new_ref(Py_None));

	int num_motors = (int)PyList_GET_SIZE(self->motors);
	for (int i = 0; i < num_motors; ++i) {
		// take the internal object
		BIConstraint * motor = (BIConstraint *)PyList_GET_ITEM(self->motors, i);
		// unset the motor_control
		init_slot(motor->wrapper, "motor_control", new_ref(Py_None));
		motor->motor_control = 0;
		// disable motor
		motor->hinge->enableMotor(false);
	}

	// find the motor_control in the motor_controls
	Py_ssize_t index = PySequence_Index(self->world->motor_controls_slot, self->wrapper);

	// remove it from the motor_controls
	PySequence_DelItem(self->world->motor_controls_slot, index);

	// remove it from the internal motor_controls
	PySequence_DelItem(self->world->motor_controls, index);

	// loose our object
	Py_DECREF(self);
	Py_RETURN_NONE;
}

PyObject * BIMotorControl_meth_position(BIMotorControl * self) {
	/*
		Return the motor angles
	*/
	return new_ref(self->output_array[1 - self->output_index]);
}

PyObject * BIMotorControl_meth_velocity(BIMotorControl * self) {
	/*
		Return the motor velocities
	*/
	return new_ref(self->output_array[self->output_index]);
}

PyObject * BIMotorControl_meth_reset(BIMotorControl * self) {
	/*
		Read the current motor angles and get rid of the stored velocities
	*/

	// get the allocated memory
	int num_motors = (int)PyList_GET_SIZE(self->motors);
	btScalar * output_data[2] = {
		(btScalar *)PyMemoryView_GET_BUFFER(self->output_mem[0])->buf,
		(btScalar *)PyMemoryView_GET_BUFFER(self->output_mem[1])->buf,
	};

	for (int i = 0; i < num_motors; ++i) {
		// take the internal constraint object
		BIConstraint * motor = (BIConstraint *)PyList_GET_ITEM(self->motors, i);
		// velocity cannot be determined, we have no information about the past when a reset is made
		output_data[0][i] = 0.0;
		// the position is the current motor angle
		output_data[1][i] = motor->hinge->getHingeAngle();
	}

	// reset the index
	self->output_index = 0;

	Py_RETURN_NONE;
}

void BIMotorControl_dealloc(BIMotorControl * self) {
	Py_DECREF(self->motors);
	Py_TYPE(self)->tp_free(self);
}

PyMethodDef BIMotorControl_methods[] = {
	{"position", (PyCFunction)BIMotorControl_meth_position, METH_NOARGS, 0},
	{"velocity", (PyCFunction)BIMotorControl_meth_velocity, METH_NOARGS, 0},
	{"reset", (PyCFunction)BIMotorControl_meth_reset, METH_NOARGS, 0},
	{0},
};

PyType_Slot BIMotorControl_slots[] = {
	{Py_tp_methods, BIMotorControl_methods},
	{Py_tp_dealloc, (void *)BIMotorControl_dealloc},
	{0},
};

PyTypeObject * BIMotorControl_type;

PyType_Spec BIMotorControl_spec = {"mollia_bullet.core.MotorControl", sizeof(BIMotorControl), 0, Py_TPFLAGS_DEFAULT, BIMotorControl_slots};
