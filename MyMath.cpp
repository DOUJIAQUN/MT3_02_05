#include "MyMath.h"

/// <summary>
/// Vector3関数
/// </summary>
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
    Vector3 result;
    result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0];
    result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1];
    result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2];
    float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];

    assert(w != 0.0f);

    result.x /= w;
    result.y /= w;
    result.z /= w;

    return result;
}

Vector3 Project(const Vector3& v1, const Vector3& v2) {
    float dotProduct = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    float lengthSquared = v2.x * v2.x + v2.y * v2.y + v2.z * v2.z;
    float scalar = dotProduct / lengthSquared;
    Vector3 result = { v2.x * scalar, v2.y * scalar, v2.z * scalar };
    return result;
}

Vector3 ClosestPoint(const Vector3& point, const Segment& segment) {
     Vector3 segmentVector = Subtract(segment.diff, segment.origin);
    Vector3 pointToSegmentStart = Subtract(point, segment.origin);

    Vector3 projection = Project(pointToSegmentStart, segmentVector);

    float t = (projection.x * segmentVector.x + projection.y * segmentVector.y + projection.z * segmentVector.z) /
              (segmentVector.x * segmentVector.x + segmentVector.y * segmentVector.y + segmentVector.z * segmentVector.z);

    t = fmax(0.0f, fmin(1.0f, t));

    return {
        segment.origin.x + segmentVector.x * t,
        segment.origin.y + segmentVector.y * t,
        segment.origin.z + segmentVector.z * t
    };
}

Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
    return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

Vector3 Add(const Vector3& v1, const Vector3& v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

Vector3 Multiply(float scalar, const Vector3& v) {
    return {scalar * v.x, scalar * v.y, scalar * v.z};
}

Vector3 Normalize(const Vector3& v) {
    float length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length != 0) {
        return { v.x / length, v.y / length, v.z / length };
    }
    return { 0.0f, 0.0f, 0.0f };
}

Vector3 Cross(const Vector3& v1, const Vector3& v2) {
    Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

Vector3 Perpendicular(const Vector3& vector) {
    if (vector.x != 0.0f || vector.y != 0.0f) {
        return {-vector.y, vector.x, 0.0f};
    }
    return {0.0f, -vector.z, vector.y};
}
/// <summary>
/// Matrix4x4関数
/// </summary>
Matrix4x4 Multiply(const Matrix4x4& matrix1, const Matrix4x4& matrix2) {
    Matrix4x4 result = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                result.m[i][j] += matrix1.m[i][k] * matrix2.m[k][j];
            }
        }
    }
    return result;
}

Matrix4x4 MakeAffineMatrix(const Vector3& scale,const Vector3& rotation,const Vector3& translation) {
	// Scale
	Matrix4x4 Scale = {0};
	Scale.m[0][0] = scale.x;
	Scale.m[1][1] = scale.y;
	Scale.m[2][2] = scale.z;
	Scale.m[3][3] = 1;
	// Rotation
	Matrix4x4 RotationZ = {0};
	RotationZ.m[0][0] = cosf(rotation.z);
	RotationZ.m[0][1] = sinf(rotation.z);
	RotationZ.m[1][0] = -sinf(rotation.z);
	RotationZ.m[1][1] = cosf(rotation.z);
	RotationZ.m[2][2] = RotationZ.m[3][3] = 1;
	Matrix4x4 RotationX = {0};
	RotationX.m[1][1] = cosf(rotation.x);
	RotationX.m[1][2] = sinf(rotation.x);
	RotationX.m[2][1] = -sinf(rotation.x);
	RotationX.m[2][2] = cosf(rotation.x);
	RotationX.m[0][0] = RotationX.m[3][3] = 1;
	Matrix4x4 RotationY = {0};
	RotationY.m[0][0] = cosf(rotation.y);
	RotationY.m[2][0] = sinf(rotation.y);
	RotationY.m[0][2] = -sinf(rotation.y);
	RotationY.m[2][2] = cosf(rotation.y);
	RotationY.m[1][1] = RotationY.m[3][3] = 1;
	Matrix4x4 Rotation = Multiply(RotationX, Multiply(RotationY, RotationZ));
	// Translation
	Matrix4x4 Translation = {0};
	Translation.m[0][0] = Translation.m[1][1] = Translation.m[2][2] = Translation.m[3][3] = 1;
	Translation.m[3][0] = translation.x;
	Translation.m[3][1] = translation.y;
	Translation.m[3][2] = translation.z;

	return Multiply(Scale, Multiply(Rotation, Translation));
}

Matrix4x4 Inverse(const Matrix4x4& m) {
    float determinant =
        + m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3]
        + m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1]
        + m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2]
        - m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1]
        - m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3]
        - m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2]
        - m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3]
        - m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1]
        - m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2]
        + m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1]
        + m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3]
        + m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2]
        + m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3]
        + m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1]
        + m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2]
        - m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1]
        - m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3]
        - m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2]
        - m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0]
        - m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0]
        - m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0]
        + m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0]
        + m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0]
        + m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0];

    Matrix4x4 result = {};
    float recpDeterminant = 1.0f / determinant;

    result.m[0][0] = (m.m[1][1] * m.m[2][2] * m.m[3][3] + m.m[1][2] * m.m[2][3] * m.m[3][1] +
                      m.m[1][3] * m.m[2][1] * m.m[3][2] - m.m[1][3] * m.m[2][2] * m.m[3][1] -
                      m.m[1][2] * m.m[2][1] * m.m[3][3] - m.m[1][1] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
    result.m[0][1] = (-m.m[0][1] * m.m[2][2] * m.m[3][3] - m.m[0][2] * m.m[2][3] * m.m[3][1] -
                      m.m[0][3] * m.m[2][1] * m.m[3][2] + m.m[0][3] * m.m[2][2] * m.m[3][1] +
                      m.m[0][2] * m.m[2][1] * m.m[3][3] + m.m[0][1] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
    result.m[0][2] = (m.m[0][1] * m.m[1][2] * m.m[3][3] + m.m[0][2] * m.m[1][3] * m.m[3][1] +
                      m.m[0][3] * m.m[1][1] * m.m[3][2] - m.m[0][3] * m.m[1][2] * m.m[3][1] -
                      m.m[0][2] * m.m[1][1] * m.m[3][3] - m.m[0][1] * m.m[1][3] * m.m[3][2]) * recpDeterminant;
    result.m[0][3] = (-m.m[0][1] * m.m[1][2] * m.m[2][3] - m.m[0][2] * m.m[1][3] * m.m[2][1] -
                      m.m[0][3] * m.m[1][1] * m.m[2][2] + m.m[0][3] * m.m[1][2] * m.m[2][1] +
                      m.m[0][2] * m.m[1][1] * m.m[2][3] + m.m[0][1] * m.m[1][3] * m.m[2][2]) * recpDeterminant;

    result.m[1][0] = (-m.m[1][0] * m.m[2][2] * m.m[3][3] - m.m[1][2] * m.m[2][3] * m.m[3][0] -
                      m.m[1][3] * m.m[2][0] * m.m[3][2] + m.m[1][3] * m.m[2][2] * m.m[3][0] +
                      m.m[1][2] * m.m[2][0] * m.m[3][3] + m.m[1][0] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
    result.m[1][1] = (m.m[0][0] * m.m[2][2] * m.m[3][3] + m.m[0][2] * m.m[2][3] * m.m[3][0] +
                      m.m[0][3] * m.m[2][0] * m.m[3][2] - m.m[0][3] * m.m[2][2] * m.m[3][0] -
                      m.m[0][2] * m.m[2][0] * m.m[3][3] - m.m[0][0] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
    result.m[1][2] = (-m.m[0][0] * m.m[1][2] * m.m[3][3] - m.m[0][2] * m.m[1][3] * m.m[3][0] -
                      m.m[0][3] * m.m[1][0] * m.m[3][2] + m.m[0][3] * m.m[1][2] * m.m[3][0] +
                      m.m[0][2] * m.m[1][0] * m.m[3][3] + m.m[0][0] * m.m[1][3] * m.m[3][2]) * recpDeterminant;
    result.m[1][3] = (m.m[0][0] * m.m[1][2] * m.m[2][3] + m.m[0][2] * m.m[1][3] * m.m[2][0] +
                      m.m[0][3] * m.m[1][0] * m.m[2][2] - m.m[0][3] * m.m[1][2] * m.m[2][0] -
                      m.m[0][2] * m.m[1][0] * m.m[2][3] - m.m[0][0] * m.m[1][3] * m.m[2][2]) * recpDeterminant;

    result.m[2][0] = (m.m[1][0] * m.m[2][1] * m.m[3][3] + m.m[1][1] * m.m[2][3] * m.m[3][0] +
                      m.m[1][3] * m.m[2][0] * m.m[3][1] - m.m[1][3] * m.m[2][1] * m.m[3][0] -
                      m.m[1][1] * m.m[2][0] * m.m[3][3] - m.m[1][0] * m.m[2][3] * m.m[3][1]) * recpDeterminant;
    result.m[2][1] = (-m.m[0][0] * m.m[2][1] * m.m[3][3] - m.m[0][1] * m.m[2][3] * m.m[3][0] -
                      m.m[0][3] * m.m[2][0] * m.m[3][1] + m.m[0][3] * m.m[2][1] * m.m[3][0] +
                      m.m[0][1] * m.m[2][0] * m.m[3][3] + m.m[0][0] * m.m[2][3] * m.m[3][1]) * recpDeterminant;
    result.m[2][2] = (m.m[0][0] * m.m[1][1] * m.m[3][3] + m.m[0][1] * m.m[1][3] * m.m[3][0] +
                      m.m[0][3] * m.m[1][0] * m.m[3][1] - m.m[0][3] * m.m[1][1] * m.m[3][0] -
                      m.m[0][1] * m.m[1][0] * m.m[3][3] - m.m[0][0] * m.m[1][3] * m.m[3][1]) * recpDeterminant;
    result.m[2][3] = (-m.m[0][0] * m.m[1][1] * m.m[2][3] - m.m[0][1] * m.m[1][3] * m.m[2][0] -
                      m.m[0][3] * m.m[1][0] * m.m[2][1] + m.m[0][3] * m.m[1][1] * m.m[2][0] +
                      m.m[0][1] * m.m[1][0] * m.m[2][3] + m.m[0][0] * m.m[1][3] * m.m[2][1]) * recpDeterminant;

    result.m[3][0] = (-m.m[1][0] * m.m[2][1] * m.m[3][2] - m.m[1][1] * m.m[2][2] * m.m[3][0] -
                      m.m[1][2] * m.m[2][0] * m.m[3][1] + m.m[1][2] * m.m[2][1] * m.m[3][0] +
                      m.m[1][1] * m.m[2][0] * m.m[3][2] + m.m[1][0] * m.m[2][2] * m.m[3][1]) * recpDeterminant;
    result.m[3][1] = (m.m[0][0] * m.m[2][1] * m.m[3][2] + m.m[0][1] * m.m[2][2] * m.m[3][0] +
                      m.m[0][2] * m.m[2][0] * m.m[3][1] - m.m[0][2] * m.m[2][1] * m.m[3][0] -
                      m.m[0][1] * m.m[2][0] * m.m[3][2] - m.m[0][0] * m.m[2][2] * m.m[3][1]) * recpDeterminant;
    result.m[3][2] = (-m.m[0][0] * m.m[1][1] * m.m[3][2] - m.m[0][1] * m.m[1][2] * m.m[3][0] -
                      m.m[0][2] * m.m[1][0] * m.m[3][1] + m.m[0][2] * m.m[1][1] * m.m[3][0] +
                      m.m[0][1] * m.m[1][0] * m.m[3][2] + m.m[0][0] * m.m[1][2] * m.m[3][1]) * recpDeterminant;
    result.m[3][3] = (m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[0][1] * m.m[1][2] * m.m[2][0] +
                      m.m[0][2] * m.m[1][0] * m.m[2][1] - m.m[0][2] * m.m[1][1] * m.m[2][0] -
                      m.m[0][1] * m.m[1][0] * m.m[2][2] - m.m[0][0] * m.m[1][2] * m.m[2][1]) * recpDeterminant;

    return result;
}

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
    Matrix4x4 result = {};

    float tanHalfFovY = tanf(fovY * 0.5f);
    float cot = 1.0f / tanHalfFovY;

    result.m[0][0] = cot / aspectRatio;
    result.m[0][1] = 0.0f;
    result.m[0][2] = 0.0f;
    result.m[0][3] = 0.0f;

    result.m[1][0] = 0.0f;
    result.m[1][1] = cot;
    result.m[1][2] = 0.0f;
    result.m[1][3] = 0.0f;

    result.m[2][0] = 0.0f;
    result.m[2][1] = 0.0f;
    result.m[2][2] = farClip / (farClip - nearClip);
    result.m[2][3] = 1.0f;

    result.m[3][0] = 0.0f;
    result.m[3][1] = 0.0f;
    result.m[3][2] = -(nearClip * farClip) / (farClip - nearClip);
    result.m[3][3] = 0.0f;

    return result;
}

Matrix4x4 MakeOrthographicMatrix(float left, float right, float top, float bottom, float nearClip, float farClip) {
    Matrix4x4 result;

    result.m[0][0] = 2.0f / (right - left);
    result.m[0][1] = 0.0f;
    result.m[0][2] = 0.0f;
    result.m[0][3] = 0.0f;

    result.m[1][0] = 0.0f;
    result.m[1][1] = 2.0f / (top - bottom);
    result.m[1][2] = 0.0f;
    result.m[1][3] = 0.0f;

    result.m[2][0] = 0.0f;
    result.m[2][1] = 0.0f;
    result.m[2][2] = 1.0f / (farClip - nearClip);
    result.m[2][3] = 0.0f;

    result.m[3][0] = (right + left) / (left - right);
    result.m[3][1] = (top + bottom) / (bottom - top);
    result.m[3][2] = nearClip / (nearClip - farClip);
    result.m[3][3] = 1.0f;

    return result;
}

Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
    Matrix4x4 result;

    result.m[0][0] = width / 2.0f;
    result.m[0][1] = 0.0f;
    result.m[0][2] = 0.0f;
    result.m[0][3] = 0.0f;

    result.m[1][0] = 0.0f;
    result.m[1][1] = -height / 2.0f;
    result.m[1][2] = 0.0f;
    result.m[1][3] = 0.0f;

    result.m[2][0] = 0.0f;
    result.m[2][1] = 0.0f;
    result.m[2][2] = maxDepth - minDepth;
    result.m[2][3] = 0.0f;

    result.m[3][0] = left + width / 2.0f;
    result.m[3][1] = top + height / 2.0f;
    result.m[3][2] = minDepth;
    result.m[3][3] = 1.0f;

    return result;
}

///他の関数
float Dot(const Vector3& v1, const Vector3& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
    const float kGridHalfwidth = 2.0f;
    const uint32_t kSubdivision = 10;
    const float kGridEvery = (kGridHalfwidth * 2.0f) / float(kSubdivision);

    for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
        float x = -kGridHalfwidth + (xIndex * kGridEvery);
        unsigned int color = 0xAAAAAAFF;

        Vector3 start{ x, 0.0f, -kGridHalfwidth };
        Vector3 end{ x, 0.0f, kGridHalfwidth };

        Vector3 startScreen = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
        Vector3 endScreen = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

        if (x == 0.0f) {
            color = 0x000000FF; // 黑色
        }
        Novice::DrawLine(int(startScreen.x), int(startScreen.y), int(endScreen.x), int(endScreen.y), color);
    }

    for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
        float z = -kGridHalfwidth + (zIndex * kGridEvery);
        unsigned int color = 0xAAAAAAFF;

        Vector3 start{ kGridHalfwidth, 0.0f, z };
        Vector3 end{ -kGridHalfwidth, 0.0f, z };

        Vector3 startScreen = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
        Vector3 endScreen = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

        if (z == 0.0f) {
            color = 0x000000FF; // 黑色
        }
        Novice::DrawLine(int(startScreen.x), int(startScreen.y), int(endScreen.x), int(endScreen.y), color);
    }
}

void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
    const uint32_t kSubdivision = 16; // 分割数
    const float kLonEvery = 2 *(float)M_PI / kSubdivision; // 经度分割1つの角度
    const float kLatEvery =(float) M_PI / kSubdivision; // 纬度分割1つの角度

    // 緯度の方向に分割 -π/2 ~ π/2
    for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
        float lat = -(float)M_PI / 2.0f + kLatEvery  * latIndex; // 現在の緯度

        // 経度の方向に分割 0 ~ 2π
        for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
            float lon = lonIndex * kLonEvery; // 現在の経度

            // world座標でのa, b, cを求める
            Vector3 a, b, c;

            a.x = sphere.center.x + sphere.radius * cosf(lat) * cosf(lon);
            a.y = sphere.center.y + sphere.radius * sinf(lat);
            a.z = sphere.center.z + sphere.radius * cosf(lat) * sinf(lon);

            b.x = sphere.center.x + sphere.radius * cosf(lat + kLatEvery) * cosf(lon);
            b.y = sphere.center.y + sphere.radius * sinf(lat + kLatEvery);
            b.z = sphere.center.z + sphere.radius * cosf(lat + kLatEvery) * sinf(lon);

            c.x = sphere.center.x + sphere.radius * cosf(lat) * cosf(lon + kLonEvery);
            c.y = sphere.center.y + sphere.radius * sinf(lat);
            c.z = sphere.center.z + sphere.radius * cosf(lat) * sinf(lon + kLonEvery);

            // a, b, cをScreen座標系で変換
            a = Transform(a, viewProjectionMatrix);
            a = Transform(a, viewportMatrix);

            b = Transform(b, viewProjectionMatrix);
            b = Transform(b, viewportMatrix);

            c = Transform(c, viewProjectionMatrix);
            c = Transform(c, viewportMatrix);

            // a, b, cで線を引く
            Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, color);
            Novice::DrawLine((int)a.x, (int)a.y, (int)c.x, (int)c.y, color);
        }
    }
}

void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
    Vector3 center = Multiply(plane.distance, plane.normal);
    
    Vector3 perpendicular1 = Normalize(Perpendicular(plane.normal));
    Vector3 perpendicular2 = Cross(plane.normal, perpendicular1);

    Vector3 points[4];
    points[0] = Add(center, Add(Multiply(2.0f, perpendicular1), Multiply(2.0f, perpendicular2)));
    points[1] = Add(center, Add(Multiply(2.0f, perpendicular1), Multiply(-2.0f, perpendicular2)));
    points[2] = Add(center, Add(Multiply(-2.0f, perpendicular1), Multiply(-2.0f, perpendicular2)));
    points[3] = Add(center, Add(Multiply(-2.0f, perpendicular1), Multiply(2.0f, perpendicular2)));

    for (int32_t i = 0; i < 4; ++i) {
        points[i] = Transform(Transform(points[i], viewProjectionMatrix), viewportMatrix);
    }

    for (int32_t i = 0; i < 4; ++i) {
        int32_t nextIndex = (i + 1) % 4;
        Novice::DrawLine((int)points[i].x, (int)points[i].y, (int)points[nextIndex].x, (int)points[nextIndex].y, color);
    }
}

void DrawTriangle(
    const Triangle& triangle, 
    const Matrix4x4& viewProjectionMatrix, 
    const Matrix4x4& viewportMatrix, 
    uint32_t color) {
    
    Vector3 screenVertices[3];
    for (int i = 0; i < 3; ++i) {
        screenVertices[i] = Transform(Transform(triangle.vertices[i], viewProjectionMatrix), viewportMatrix);
    }
    for (int i = 0; i < 3; ++i) {
        int next = (i + 1) % 3;
        Novice::DrawLine((int)screenVertices[i].x, (int)screenVertices[i].y, 
                         (int)screenVertices[next].x, (int)screenVertices[next].y, color);
    }
}

// AABBを描画する関数
void DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
    Vector3 vertices[8];
    
    vertices[0] = Transform(Transform({ aabb.min.x, aabb.min.y, aabb.min.z }, viewProjectionMatrix), viewportMatrix);
    vertices[1] = Transform(Transform({ aabb.max.x, aabb.min.y, aabb.min.z }, viewProjectionMatrix), viewportMatrix);
    vertices[2] = Transform(Transform({ aabb.max.x, aabb.max.y, aabb.min.z }, viewProjectionMatrix), viewportMatrix);
    vertices[3] = Transform(Transform({ aabb.min.x, aabb.max.y, aabb.min.z }, viewProjectionMatrix), viewportMatrix);
    vertices[4] = Transform(Transform({ aabb.min.x, aabb.min.y, aabb.max.z }, viewProjectionMatrix), viewportMatrix);
    vertices[5] = Transform(Transform({ aabb.max.x, aabb.min.y, aabb.max.z }, viewProjectionMatrix), viewportMatrix);
    vertices[6] = Transform(Transform({ aabb.max.x, aabb.max.y, aabb.max.z }, viewProjectionMatrix), viewportMatrix);
    vertices[7] = Transform(Transform({ aabb.min.x, aabb.max.y, aabb.max.z }, viewProjectionMatrix), viewportMatrix);
    
    // Bottom face
    Novice::DrawLine((int)vertices[0].x, (int)vertices[0].y, (int)vertices[1].x, (int)vertices[1].y, color);
    Novice::DrawLine((int)vertices[1].x, (int)vertices[1].y, (int)vertices[2].x, (int)vertices[2].y, color);
    Novice::DrawLine((int)vertices[2].x, (int)vertices[2].y, (int)vertices[3].x, (int)vertices[3].y, color);
    Novice::DrawLine((int)vertices[3].x, (int)vertices[3].y, (int)vertices[0].x, (int)vertices[0].y, color);
    
    // Top face
    Novice::DrawLine((int)vertices[4].x, (int)vertices[4].y, (int)vertices[5].x, (int)vertices[5].y, color);
    Novice::DrawLine((int)vertices[5].x, (int)vertices[5].y, (int)vertices[6].x, (int)vertices[6].y, color);
    Novice::DrawLine((int)vertices[6].x, (int)vertices[6].y, (int)vertices[7].x, (int)vertices[7].y, color);
    Novice::DrawLine((int)vertices[7].x, (int)vertices[7].y, (int)vertices[4].x, (int)vertices[4].y, color);
    
    // Vertical lines
    Novice::DrawLine((int)vertices[0].x, (int)vertices[0].y, (int)vertices[4].x, (int)vertices[4].y, color);
    Novice::DrawLine((int)vertices[1].x, (int)vertices[1].y, (int)vertices[5].x, (int)vertices[5].y, color);
    Novice::DrawLine((int)vertices[2].x, (int)vertices[2].y, (int)vertices[6].x, (int)vertices[6].y, color);
    Novice::DrawLine((int)vertices[3].x, (int)vertices[3].y, (int)vertices[7].x, (int)vertices[7].y, color);
}


//当たり判定
//球と球
bool IsCollisionBall(const Sphere& s1, const Sphere& s2) {
    float distanceSquared = (s1.center.x - s2.center.x) * (s1.center.x - s2.center.x) +
                            (s1.center.y - s2.center.y) * (s1.center.y - s2.center.y) +
                            (s1.center.z - s2.center.z) * (s1.center.z - s2.center.z);
    float radiusSum = s1.radius + s2.radius;
    return distanceSquared <= (radiusSum * radiusSum);
}
//球和平面
bool IsCollisionPlane(const Sphere& sphere, const Plane& plane) {
    
    float distance = Dot(plane.normal, sphere.center) - plane.distance;
   
    return fabs(distance) <= sphere.radius;
}
//線と平面
bool IsCollisionSegment(const Segment& segment, const Plane& plane) {
    // 计算 t = (d - (n · p0)) / (n · (p1 - p0))
    float dot = Dot(plane.normal, segment.diff);
    if (dot == 0.0f) {
        return false; // 線と平面平行
    }

    float t = (plane.distance - Dot(plane.normal, segment.origin)) / dot;
    return (t >= 0.0f && t <= 1.0f);
}
//三角形と線
bool IsCollisionTriangle(const Triangle& triangle, const Segment& segment) {
     Vector3 edge1 = Subtract(triangle.vertices[1], triangle.vertices[0]);
    Vector3 edge2 = Subtract(triangle.vertices[2], triangle.vertices[0]);
    Vector3 h = Cross(segment.diff, edge2);
    float a = Dot(edge1, h);
    if (a > -0.00001f && a < 0.00001f) {
        return false; // 線と三角形平行
    }
    float f = 1.0f / a;
    Vector3 s = Subtract(segment.origin, triangle.vertices[0]);
    float u = f * Dot(s, h);
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    Vector3 q = Cross(s, edge1);
    float v = f * Dot(segment.diff, q);
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    // 计算線と三角形平面の交点 t
    float t = f * Dot(edge2, q);
    if (t > 0.0f && t < 1.0f) {
        return true;
    }
    return false;
}
//AABBとAABB
bool IsCollisionBox(const AABB& aabb1, const AABB& aabb2) {
   if ((aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x) &&
        (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y) &&
        (aabb1.min.z <= aabb2.max.z && aabb1.max.z >= aabb2.min.z)) {
        return true;
    }
    return false;
}