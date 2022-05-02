#include <Python.h>
#include <structmember.h>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletCollision/CollisionShapes/btMinkowskiSumShape.h>

struct World {
    PyObject_HEAD
    btDefaultCollisionConfiguration * configuration;
    btCollisionDispatcher * dispatcher;
    btDbvtBroadphase * broadphase;
    btMultiBodyConstraintSolver * solver;
    btMultiBodyDynamicsWorld * world;
};

struct RigidBody {
    PyObject_HEAD
    btRigidBody * rigid_body;
    double mass;
    int group;
    int mask;
};

struct Constraint {
    PyObject_HEAD
    RigidBody * parent;
    RigidBody * child;
    btFixedConstraint * constraint;
};

struct Transform {
    PyObject_HEAD
    btTransform transform;
};

PyTypeObject * World_type;
PyTypeObject * RigidBody_type;
PyTypeObject * Constraint_type;
PyTypeObject * Transform_type;

PyObject * helper;
PyObject * rotation_order_map;

Transform * identity_transform;

PyObject * build_transform(btTransform transform) {
    btVector3 p = transform.getOrigin();
    btQuaternion r = transform.getRotation();
    return Py_BuildValue("{s[ddd]s[dddd]}", "position", p.x(), p.y(), p.z(), "rotation", r.x(), r.y(), r.z(), r.w());
}

PyObject * build_shape(btCollisionShape * shape) {
    switch (shape->getShapeType()) {
        case EMPTY_SHAPE_PROXYTYPE: {
            return Py_BuildValue("{ss}", "type", "empty");
        }
        case BOX_SHAPE_PROXYTYPE: {
            btBoxShape * box = (btBoxShape *)shape;
            btVector3 size = box->getHalfExtentsWithMargin();
            return Py_BuildValue("{sss[ddd]}", "type", "box", "size", size.x() * 2.0, size.y() * 2.0, size.z() * 2.0);
        }
        case SPHERE_SHAPE_PROXYTYPE: {
            btSphereShape * sphere = (btSphereShape *)shape;
            return Py_BuildValue("{sssd}", "type", "sphere", "radius", sphere->getRadius());
        }
        case CAPSULE_SHAPE_PROXYTYPE: {
            btCapsuleShapeZ * capsule = (btCapsuleShapeZ *)shape;
            return Py_BuildValue("{sssdsd}", "type", "capsule", "radius", capsule->getRadius(), "height", capsule->getHalfHeight() * 2.0);
        }
        case CYLINDER_SHAPE_PROXYTYPE: {
            btCylinderShapeZ * cylinder = (btCylinderShapeZ *)shape;
            return Py_BuildValue("{sssdsd}", "type", "cylinder", "radius", cylinder->getRadius(), "height", cylinder->getHalfExtentsWithMargin().z() * 2.0);
        }
        case CONE_SHAPE_PROXYTYPE: {
            btConeShapeZ * cone = (btConeShapeZ *)shape;
            return Py_BuildValue("{sssdsd}", "type", "cone", "radius", cone->getRadius(), "height", cone->getHeight());
        }
        case CONVEX_HULL_SHAPE_PROXYTYPE: {
            btConvexHullShape * chull = (btConvexHullShape *)shape;
            int count = chull->getNumPoints();
            PyObject * vertices = PyList_New(count);
            for (int i = 0; i < count; ++i) {
                btVector3 vertex = chull->getPoints()[i];
                PyList_SetItem(vertices, i, Py_BuildValue("[ddd]", vertex.x(), vertex.y(), vertex.z()));
            }
            return Py_BuildValue("{sssN}", "type", "convexhull", "vertices", vertices);
        }
        case MULTI_SPHERE_SHAPE_PROXYTYPE: {
            btMultiSphereShape * msphere = (btMultiSphereShape *)shape;
            int count = msphere->getSphereCount();
            PyObject * spheres = PyList_New(count);
            for (int i = 0; i < count; ++i) {
                btVector3 pos = msphere->getSpherePosition(i);
                PyList_SetItem(spheres, i, Py_BuildValue("[dddd]", pos.x(), pos.y(), pos.z(), msphere->getSphereRadius(i)));
            }
            return Py_BuildValue("{sssN}", "type", "multisphere", "spheres", spheres);
        }
        case MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE: {
            btMinkowskiSumShape * minkowski = (btMinkowskiSumShape *)shape;
            btCollisionShape * shape_a = (btCollisionShape *)minkowski->getShapeA();
            btCollisionShape * shape_b = (btCollisionShape *)minkowski->getShapeB();
            PyObject * transform_a = build_transform(minkowski->getTransformA());
            PyObject * transform_b = build_transform(minkowski->GetTransformB());
            PyObject * a = Py_BuildValue("{sNsN}", "transform", transform_a, "shape", build_shape(shape_a));
            PyObject * b = Py_BuildValue("{sNsN}", "transform", transform_b, "shape", build_shape(shape_b));
            return Py_BuildValue("{sssNsN}", "type", "minkowski", "a", a, "b", b);
        }
        case COMPOUND_SHAPE_PROXYTYPE: {
            btCompoundShape * compound = (btCompoundShape *)shape;
            PyObject * children = PyList_New(compound->getNumChildShapes());
            for (int i = 0; i < compound->getNumChildShapes(); ++i) {
                btCollisionShape * child_shape = compound->getChildShape(i);
                PyObject * transform = build_transform(compound->getChildTransform(i));
                PyList_SetItem(children, i, Py_BuildValue("{sNsN}", "transform", transform, "shape", build_shape(child_shape)));
            }
            return Py_BuildValue("{sssN}", "type", "compound", "children", children);
        }
    }
    Py_RETURN_NONE;
}

PyObject * build_shape(btRigidBody * body) {
    return build_shape(body->getCollisionShape());
}

double td(PyObject * tuple, int index) {
    return PyFloat_AsDouble(PyTuple_GetItem(tuple, index));
}

int optional_vector(PyObject * arg, btVector3 * result) {
    if (arg == Py_None) {
        *result = {0.0, 0.0, 0.0};
        return 1;
    }
    PyObject * args = PySequence_Tuple(arg);
    if (!args) {
        return 0;
    }
    double x, y, z;
    if (!PyArg_ParseTuple(args, "ddd", &x, &y, &z)) {
        Py_DECREF(args);
        return 0;
    }
    *result = {x, y, z};
    Py_DECREF(args);
    return 1;
}

int optional_quaternion(PyObject * arg, btQuaternion * result) {
    if (arg == Py_None) {
        *result = {0.0, 0.0, 0.0, 1.0};
        return 1;
    }
    PyObject * args = PySequence_Tuple(arg);
    if (!args) {
        return 0;
    }
    double x, y, z, w;
    if (!PyArg_ParseTuple(args, "dddd", &x, &y, &z, &w)) {
        Py_DECREF(args);
        return 0;
    }
    *result = {x, y, z, w};
    Py_DECREF(args);
    return 1;
}

btTransform get_transform(PyObject * arg) {
    return btTransform(btQuaternion(td(arg, 3), td(arg, 4), td(arg, 5), td(arg, 6)), btVector3(td(arg, 0), td(arg, 1), td(arg, 2)));
}

btCollisionShape * get_shape(PyObject * arg) {
    if (arg == Py_None) {
        return new btEmptyShape();
    }
    switch (PyLong_AsLong(PyTuple_GetItem(arg, 0))) {
        case 0: return new btEmptyShape();
        case 1: return new btBoxShape({td(arg, 1) * 0.5, td(arg, 2) * 0.5, td(arg, 3) * 0.5});
        case 2: return new btSphereShape(td(arg, 1));
        case 3: return new btCapsuleShapeZ(td(arg, 1), td(arg, 2));
        case 4: return new btCylinderShapeZ({td(arg, 1), td(arg, 1), td(arg, 2)});
        case 5: return new btConeShapeZ(td(arg, 1), td(arg, 2));
        case 6: {
            btConvexHullShape * shape = new btConvexHullShape();
            PyObject * lst = PyTuple_GetItem(arg, 1);
            for (int i = 0; i < PyList_Size(lst); ++i) {
                PyObject * item = PyList_GetItem(lst, i);
                shape->addPoint({td(item, 0), td(item, 1), td(item, 2)}, false);
            }
            shape->recalcLocalAabb();
            return shape;
        }
        case 7: {
            PyObject * lst = PyTuple_GetItem(arg, 1);
            int count = (int)PyList_Size(lst);
            btVector3 * points = new btVector3[count];
            double * radi = new double[count];
            for (int i = 0; i < count; ++i) {
                PyObject * item = PyList_GetItem(lst, i);
                points[i] = {td(item, 0), td(item, 1), td(item, 2)};
                radi[i] = td(item, 3);
            }
            btMultiSphereShape * shape = new btMultiSphereShape(points, radi, count);
            delete[] points;
            delete[] radi;
            return shape;
        }
        case 8: {
            btConvexShape * shape_a = (btConvexShape *)get_shape(PyTuple_GetItem(arg, 2));
            btConvexShape * shape_b = (btConvexShape *)get_shape(PyTuple_GetItem(arg, 4));
            btMinkowskiSumShape * shape = new btMinkowskiSumShape(shape_a, shape_b);
            shape->setTransformA(get_transform(PyTuple_GetItem(arg, 1)));
            shape->setTransformB(get_transform(PyTuple_GetItem(arg, 3)));
            return shape;
        }
        case 9: {
            PyObject * lst = PyTuple_GetItem(arg, 1);
            int count = (int)PyList_Size(lst);
            btCompoundShape * shape = new btCompoundShape(false, count);
            for (int i = 0; i < count; ++i) {
                PyObject * item = PyList_GetItem(lst, i);
                btTransform transform = get_transform(PyTuple_GetItem(item, 0));
                shape->addChildShape(transform, get_shape(PyTuple_GetItem(item, 1)));
            }
            return shape;
        }
    }
    return NULL;
}

int optional_rigid_body(PyObject * arg, RigidBody ** result) {
    if (arg == Py_None) {
        *result = NULL;
        return 1;
    }
    if (Py_TYPE(arg) != RigidBody_type) {
        PyErr_Format(PyExc_ValueError, "not a RigidBody");
        return 0;
    }
    *result = (RigidBody *)arg;
    return 1;
}

int optional_transform(PyObject * arg, Transform ** result) {
    if (arg == Py_None) {
        *result = identity_transform;
        return 1;
    }
    if (Py_TYPE(arg) != Transform_type) {
        PyErr_Format(PyExc_ValueError, "not a Transform");
        return 0;
    }
    *result = (Transform *)arg;
    return 1;
}

Transform * meth_transform(PyObject * self, PyObject * args, PyObject * kwargs) {
    static char * keywords[] = {"position", "rotation", NULL};

    btVector3 position = {0.0, 0.0, 0.0};
    btQuaternion rotation = {0.0, 0.0, 0.0, 1.0};
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|O&O&", keywords, optional_vector, &position, optional_quaternion, &rotation)) {
        return NULL;
    }

    Transform * res = PyObject_New(Transform, Transform_type);
    res->transform = btTransform(rotation, position);
    return res;
}

World * meth_world(PyObject * self, PyObject * args, PyObject * kwargs) {
    static char * keywords[] = {"gravity", "max_solver_iterations", NULL};

    int max_solver_iterations = 10;
    btVector3 gravity = {0.0, 0.0, 0.0};

    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|O&i", keywords, optional_vector, &gravity, &max_solver_iterations)) {
        return NULL;
    }

    World * res = PyObject_New(World, World_type);
    res->configuration = new btDefaultCollisionConfiguration();
    res->dispatcher = new btCollisionDispatcher(res->configuration);
    res->broadphase = new btDbvtBroadphase();
    res->solver = new btMultiBodyConstraintSolver();
    res->world = new btMultiBodyDynamicsWorld(res->dispatcher, res->broadphase, res->solver, res->configuration);

    res->world->getSolverInfo().m_numIterations = max_solver_iterations;
    res->world->getSolverInfo().m_solverMode |= SOLVER_RANDMIZE_ORDER;
    res->world->setGravity(gravity);

    return res;
}

RigidBody * World_meth_rigid_body(World * self, PyObject * args, PyObject * kwargs) {
    static char * keywords[] = {"mass", "shape", "inertia", "transform", "linear_factor", "angular_factor", "friction", "spinning_friction", "rolling_friction", "group", "mask", NULL};
    double mass = 0.0;
    PyObject * shape_info = Py_None;
    PyObject * inertia = Py_None;
    Transform * transform = identity_transform;
    btVector3 linear_factor = {1.0, 1.0, 1.0};
    btVector3 angular_factor = {1.0, 1.0, 1.0};
    double friction = 0.0;
    double spinning_friction = 0.0;
    double rolling_friction = 0.0;
    int group = 1;
    int mask = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|dOOO&O&O&dddii", keywords, &mass,
        &shape_info, &inertia, optional_transform, &transform,
        optional_vector, &linear_factor, optional_vector, &angular_factor,
        &friction, &spinning_friction, &rolling_friction,
        &group, &mask)) {
        return NULL;
    }

    PyObject * parsed_shape = PyObject_CallMethod(helper, "parse_shape", "O", shape_info);
    if (!parsed_shape) {
        return NULL;
    }

    btCollisionShape * shape = get_shape(parsed_shape);
    Py_DECREF(parsed_shape);

    btVector3 inertia_tensor = {0.0, 0.0, 0.0};
    if (inertia == Py_None) {
        shape->calculateLocalInertia(mass, inertia_tensor);
    } else {
        if (!optional_vector(inertia, &inertia_tensor)) {
            return NULL;
        }
    }

    RigidBody * res = PyObject_New(RigidBody, RigidBody_type);

    res->mass = mass;
    res->group = group;
    res->mask = mask;
    res->rigid_body = new btRigidBody(mass, 0, shape, inertia_tensor);
    res->rigid_body->setWorldTransform(transform->transform);
    res->rigid_body->setActivationState(DISABLE_DEACTIVATION);
    res->rigid_body->setFriction(friction);
    res->rigid_body->setRollingFriction(rolling_friction);
    res->rigid_body->setSpinningFriction(spinning_friction);
    res->rigid_body->setLinearFactor(linear_factor);
    res->rigid_body->setLinearFactor(angular_factor);
    self->world->addRigidBody(res->rigid_body, group, mask);

    return res;
}

Constraint * World_meth_constraint(World * self, PyObject * args, PyObject * kwargs) {
    static char * keywords[] = {"parent", "child", "parent_pivot", "child_pivot", "rotation_order", NULL};
    RigidBody * parent = NULL;
    RigidBody * child = NULL;
    Transform * parent_transform = identity_transform;
    Transform * child_transform = identity_transform;
    PyObject * rotation_order = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|O&O&O&O&O", keywords, optional_rigid_body, &parent, optional_rigid_body, &child, optional_transform, &parent_transform, optional_transform, &child_transform, &rotation_order)) {
        return NULL;
    }
    if (!child) {
        return NULL;
    }
    RotateOrder rot = RO_XYZ;
    if (rotation_order != Py_None) {
        PyObject * value = PyDict_GetItem(rotation_order_map, rotation_order);
        if (!value) {
            return NULL;
        }
        rot = (RotateOrder)PyLong_AsLong(value);
    }
    btFixedConstraint * constraint = new btFixedConstraint(*parent->rigid_body, *child->rigid_body, parent_transform->transform, child_transform->transform);
    constraint->setRotationOrder(rot);
    self->world->addConstraint(constraint);
    Constraint * res = PyObject_New(Constraint, Constraint_type);
    Py_INCREF(parent);
    res->parent = parent;
    Py_INCREF(child);
    res->child = child;
    res->constraint = constraint;
    return res;
}

PyObject * Constraint_meth_configure(Constraint * self, PyObject * args, PyObject * kwargs) {
    static char * keywords[] = {
        "dof", "motor", "spring", "servo", "servo_target", "target_velocity", "max_motor_force", "stiffness",
        "damping", "bounce", "lower_limit", "upper_limit", "equilibrium_point", NULL,
    };

    int dof;
    int motor = false;
    int spring = false;
    int servo = false;
    double servo_target = 0.0;
    double target_velocity = 0.0;
    double max_motor_force = 0.0;
    double stiffness = 0.0;
    double damping = 0.0;
    double bounce = 0.0;
    double lower_limit = 0.0;
    double upper_limit = 0.0;
    double equilibrium_point = 0.0;

    self->constraint->setLimit(dof, lower_limit, upper_limit);
    self->constraint->enableMotor(dof, !!motor);
    self->constraint->enableSpring(dof, !!spring);
    self->constraint->setServo(dof, !!servo);
    self->constraint->setServoTarget(dof, servo_target);
    self->constraint->setTargetVelocity(dof, servo_target);
    self->constraint->setMaxMotorForce(dof, max_motor_force);
    self->constraint->setStiffness(dof, stiffness);
    self->constraint->setDamping(dof, damping);
    self->constraint->setBounce(dof, bounce);
    self->constraint->setEquilibriumPoint(dof, equilibrium_point);

    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "I|pppddddddddd", keywords, &dof, &servo_target, &target_velocity, &max_motor_force, &stiffness, &damping, &bounce, &lower_limit, &upper_limit, &equilibrium_point)) {
        return NULL;
    }
    Py_RETURN_NONE;
}

PyObject * World_meth_shapes(World * self) {
    int count = (int)self->world->getNumCollisionObjects();
    PyObject * res = PyList_New(count);
    for (int i = 0; i < count; ++i) {
        btRigidBody * body = (btRigidBody *)self->world->getCollisionObjectArray()[i];
        PyList_SetItem(res, i, build_shape(body));
    }
    return res;
}

PyObject * World_meth_frame(World * self) {
    int count = (int)self->world->getNumCollisionObjects();
    PyObject * res = PyBytes_FromStringAndSize(NULL, count * 32);
    float * ptr = (float *)PyBytes_AsString(res);
    for (int i = 0; i < count; ++i) {
        btRigidBody * body = (btRigidBody *)self->world->getCollisionObjectArray()[i];
        btTransform transform = body->getWorldTransform();
        btVector3 position = transform.getOrigin();
        btQuaternion rotation = transform.getRotation();
        *ptr++ = (float)position.x();
        *ptr++ = (float)position.y();
        *ptr++ = (float)position.z();
        *ptr++ = (float)0.0;
        *ptr++ = (float)rotation.x();
        *ptr++ = (float)rotation.y();
        *ptr++ = (float)rotation.z();
        *ptr++ = (float)rotation.w();
    }
    return res;
}

PyObject * World_meth_update(World * self) {
    self->world->stepSimulation(1.0 / 60.0, 0, 1.0 / 60.0);
    Py_RETURN_NONE;
}

PyObject * Transform_meth_position(Transform * self) {
    btVector3 position = self->transform.getOrigin();
    return Py_BuildValue("(ddd)", position.x(), position.y(), position.z());
}

PyObject * Transform_meth_rotation(Transform * self) {
    btQuaternion rotation = self->transform.getRotation();
    return Py_BuildValue("(dddd)", rotation.x(), rotation.y(), rotation.z(), rotation.w());

}

Transform * Transform_meth_inverse(Transform * self) {
    Transform * res = PyObject_New(Transform, Transform_type);
    res->transform = self->transform.inverse();
    return res;
}

Transform * Transform_multiply(Transform * self, Transform * other) {
    if (Py_TYPE(self) != Transform_type || Py_TYPE(other) != Transform_type) {
        return NULL;
    }
    Transform * res = PyObject_New(Transform, Transform_type);
    res->transform = self->transform * other->transform;
    return res;
}

Transform * RigidBody_get_transform(RigidBody * self) {
    Transform * res = PyObject_New(Transform, Transform_type);
    res->transform = self->rigid_body->getWorldTransform();
    return res;
}

void default_dealloc(PyObject * self) {
    Py_TYPE(self)->tp_free(self);
}

PyMethodDef World_methods[] = {
    {"rigid_body", (PyCFunction)World_meth_rigid_body, METH_VARARGS | METH_KEYWORDS, NULL},
    {"constraint", (PyCFunction)World_meth_constraint, METH_VARARGS | METH_KEYWORDS, NULL},
    {"shapes", (PyCFunction)World_meth_shapes, METH_NOARGS, NULL},
    {"frame", (PyCFunction)World_meth_frame, METH_NOARGS, NULL},
    {"update", (PyCFunction)World_meth_update, METH_NOARGS, NULL},
    {},
};

PyMethodDef RigidBody_methods[] = {
    {},
};

PyMethodDef Constraint_methods[] = {
    {"configure", (PyCFunction)Constraint_meth_configure, METH_NOARGS, NULL},
    {},
};

PyMethodDef Transform_methods[] = {
    {"position", (PyCFunction)Transform_meth_position, METH_NOARGS, NULL},
    {"rotation", (PyCFunction)Transform_meth_rotation, METH_NOARGS, NULL},
    {"inverse", (PyCFunction)Transform_meth_inverse, METH_NOARGS, NULL},
    {},
};

PyGetSetDef RigidBody_getset[] = {
    {"transform", (getter)RigidBody_get_transform, NULL, NULL},
    {},
};

PyMemberDef RigidBody_members[] = {
    {"mass", T_DOUBLE, offsetof(RigidBody, mass), READONLY, NULL},
    {},
};

PyMemberDef Constraint_members[] = {
    {"parent", T_OBJECT_EX, offsetof(Constraint, parent), READONLY, NULL},
    {"child", T_OBJECT_EX, offsetof(Constraint, child), READONLY, NULL},
    {},
};

PyType_Slot World_slots[] = {
    {Py_tp_methods, World_methods},
    {Py_tp_dealloc, default_dealloc},
    {},
};

PyType_Slot RigidBody_slots[] = {
    {Py_tp_methods, RigidBody_methods},
    {Py_tp_getset, RigidBody_getset},
    {Py_tp_members, RigidBody_members},
    {Py_tp_dealloc, default_dealloc},
    {},
};

PyType_Slot Constraint_slots[] = {
    {Py_tp_methods, Constraint_methods},
    {Py_tp_members, Constraint_members},
    {Py_tp_dealloc, default_dealloc},
    {},
};

PyType_Slot Transform_slots[] = {
    {Py_tp_methods, Transform_methods},
    {Py_nb_multiply, Transform_multiply},
    {Py_tp_dealloc, default_dealloc},
    {},
};

PyType_Spec World_spec = {"mymodule.World", sizeof(World), 0, Py_TPFLAGS_DEFAULT, World_slots};
PyType_Spec RigidBody_spec = {"mymodule.RigidBody", sizeof(RigidBody), 0, Py_TPFLAGS_DEFAULT, RigidBody_slots};
PyType_Spec Constraint_spec = {"mymodule.Constraint", sizeof(Constraint), 0, Py_TPFLAGS_DEFAULT, Constraint_slots};
PyType_Spec Transform_spec = {"mymodule.Transform", sizeof(Transform), 0, Py_TPFLAGS_DEFAULT, Transform_slots};

PyMethodDef module_methods[] = {
    {"transform", (PyCFunction)meth_transform, METH_VARARGS | METH_KEYWORDS, NULL},
    {"world", (PyCFunction)meth_world, METH_VARARGS | METH_KEYWORDS, NULL},
    {},
};

PyModuleDef module_def = {PyModuleDef_HEAD_INIT, "mollia_bullet", NULL, -1, module_methods};

extern "C" PyObject * PyInit_mollia_bullet() {
    PyObject * module = PyModule_Create(&module_def);
    helper = PyImport_ImportModule("_mollia_bullet");
    rotation_order_map = PyDict_New();
    PyDict_SetItemString(rotation_order_map, "xyz", PyLong_FromLong(RO_XYZ));
    PyDict_SetItemString(rotation_order_map, "xzy", PyLong_FromLong(RO_XZY));
    PyDict_SetItemString(rotation_order_map, "yxz", PyLong_FromLong(RO_YXZ));
    PyDict_SetItemString(rotation_order_map, "yzx", PyLong_FromLong(RO_YZX));
    PyDict_SetItemString(rotation_order_map, "zxy", PyLong_FromLong(RO_ZXY));
    PyDict_SetItemString(rotation_order_map, "zyx", PyLong_FromLong(RO_ZYX));
    World_type = (PyTypeObject *)PyType_FromSpec(&World_spec);
    RigidBody_type = (PyTypeObject *)PyType_FromSpec(&RigidBody_spec);
    Constraint_type = (PyTypeObject *)PyType_FromSpec(&Constraint_spec);
    Transform_type = (PyTypeObject *)PyType_FromSpec(&Transform_spec);
    identity_transform = PyObject_New(Transform, Transform_type);
    identity_transform->transform = btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), {0.0, 0.0, 0.0});
    return module;
}
