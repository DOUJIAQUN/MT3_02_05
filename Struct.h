#pragma once
#include "Vector3.h"
#include "Matrix4x4.h"
struct Line {
	Vector3 origir;//!<始点
	Vector3 diff;//!<終点への差分ベクトル
};

struct Ray {
	Vector3 origin;//!<始点
	Vector3 diff;//!<終点への差分ベクトル 
};

struct Segment {
	Vector3 origin;//!<始点
	Vector3 diff;//!<終点への差分ベクトル
};

struct Sphere {
    Vector3 center;
    float radius;
};

struct Plane
{
	Vector3 normal;//!<法線
	float distance;//!<距離
};
//三角形
struct Triangle {
    Vector3 vertices[3]; //!< 顶点
};
struct AABB {
	Vector3 min;
	Vector3 max;

};
