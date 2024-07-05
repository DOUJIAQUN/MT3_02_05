#pragma once
#include "Vector3.h"
#include "Matrix4x4.h"
#include"Struct.h"
#include <cstdint>
#include <assert.h>
#include <cmath>
#define M_PI 3.14159265358979323846
#include <Novice.h>
#include <algorithm>
/// <summary>
/// Vector3関数
/// </summary>
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix);

Vector3 Project(const Vector3& v1, const Vector3& v2);

Vector3 ClosestPoint(const Vector3& point, const Segment& segment);

Vector3 Subtract(const Vector3& v1, const Vector3& v2);

Vector3 Add(const Vector3& v1, const Vector3& v2);

Vector3 Multiply(float scalar, const Vector3& v);

Vector3 Normalize(const Vector3& v);

Vector3 Cross(const Vector3& v1, const Vector3& v2);

Vector3 Perpendicular(const Vector3& vector);
/// <summary>
/// Matrix4x4関数
/// </summary>
Matrix4x4 Multiply(const Matrix4x4& matrix1, const Matrix4x4& matrix2);

Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate);

Matrix4x4 Inverse(const Matrix4x4& m);

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip);

Matrix4x4 MakeOrthographicMatrix(float left, float right, float top, float bottom, float nearClip, float farClip);

Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth);

/// <summary>
/// 他の関数
/// </summary>
float Dot(const Vector3& v1, const Vector3& v2);

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix);

void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);

void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);

void DrawTriangle(
    const Triangle& triangle,
    const Matrix4x4& viewProjectionMatrix,
    const Matrix4x4& viewportMatrix,
    uint32_t color);

// AABBを描画する関数
void DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);

//当たり判定
//球と球
bool IsCollisionBall(const Sphere& s1, const Sphere& s2);
//球和平面
bool IsCollisionPlane(const Sphere& sphere, const Plane& plane);
//線と平面
bool IsCollisionSegment(const Segment& segment, const Plane& plane);
//三角形と線
bool IsCollisionTriangle(const Triangle& triangle, const Segment& segment);
//AABBとAABB
bool IsCollisionBox(const AABB& aabb1, const AABB& aabb2);
