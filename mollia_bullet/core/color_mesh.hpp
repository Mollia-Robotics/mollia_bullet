#pragma once

#include "common.hpp"

namespace color_mesh {

struct mat3 {
	float m[9];
};

struct vec3 {
	float x, y, z;
};

// color_mesh vertex layout
struct Vertex {
	vec3 vertex;
	vec3 normal;
	vec3 color;
};

vec3 operator + (const vec3 & a, const vec3 & b);
vec3 operator * (const vec3 & a, const vec3 & b);
vec3 operator * (const vec3 & a, const float & b);
vec3 operator * (const mat3 & a, const vec3 & b);

mat3 mat3_from_bt(const btMatrix3x3 & m);
vec3 vec3_from_bt(const btVector3 & v);

extern const int num_sphere_vertices;
extern const int num_box_vertices;

void sphere_mesh(Vertex *& data, const btTransform & transform, btSphereShape * shape, const vec3 & color);
void box_mesh(Vertex *& data, const btTransform & transform, btBoxShape * shape, const vec3 & color);

static int mesh_vertices(btCollisionShape * shape) {
	/*
		The number of vertices required to render shape
		Not yet implemented shapes have 0 vertices
	*/
	switch (shape->getShapeType()) {
		case SPHERE_SHAPE_PROXYTYPE:
			return num_sphere_vertices;
		case BOX_SHAPE_PROXYTYPE:
			return num_box_vertices;
	}
	return 0;
}

static void write_mesh(Vertex *& data, const btTransform & transform, btCollisionShape * shape, const vec3 & color) {
	/*
		Renders a color_mesh for shape
		Not yet implemented shapes are skipped
	*/
	switch (shape->getShapeType()) {
		case SPHERE_SHAPE_PROXYTYPE:
			sphere_mesh(data, transform, (btSphereShape *)shape, color);
			break;
		case BOX_SHAPE_PROXYTYPE:
			box_mesh(data, transform, (btBoxShape *)shape, color);
			break;
	}
}

}
