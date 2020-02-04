#ifndef _H_MYMATHS_MATRICES
#define _H_MYMATHS_MATRICES

#include "vectors.h"

// DEFINE STRUCTURES
typedef struct mat2 {
  inline mat2() {
    _11 = _22 = 1.0f;
    _12 = _21 = 0.0f;
  }
  inline mat2(float f11, float f12, float f21, float f22) {
    _11 = f11;
    _12 = f12;
    _21 = f21;
    _22 = f22;
  }
  union {
    struct {
      float _11, _12, _21, _22;
    };
    float asArray[4];
  };
  inline float* operator[](int i) { return &(asArray[i * 2]); }
} mat2;

typedef struct mat3 {
  inline mat3() {
    _11 = _22 = _33 = 1.0f;
    _12 = _13 = _21 = 0.0f;
    _23 = _31 = _32 = 0.0f;
  }

  inline mat3(float f11, float f12, float f13, float f21, float f22, float f23,
              float f31, float f32, float f33) {
    _11 = f11;
    _12 = f12;
    _13 = f13;
    _21 = f21;
    _22 = f22;
    _23 = f23;
    _31 = f31;
    _32 = f32;
    _33 = f33;
  }

  union {
    struct {
      float _11, _12, _13, _21, _22, _23, _31, _32, _33;
    };
    float asArray[9];
  };
  inline float* operator[](int i) { return &(asArray[i * 3]); }
} mat3;

typedef struct mat4 {
  inline mat4() {
    _11 = _22 = _33 = _44 = 1.0f;
    _12 = _13 = _14 = _21 = 0.0f;
    _23 = _24 = _31 = _32 = 0.0f;
    _34 = _41 = _42 = _43 = 0.0f;
  }
  inline mat4(float f11, float f12, float f13, float f14, float f21, float f22,
              float f23, float f24, float f31, float f32, float f33, float f34,
              float f41, float f42, float f43, float f44) {
    _11 = f11;
    _12 = f12;
    _13 = f13;
    _14 = f14;
    _21 = f21;
    _22 = f22;
    _23 = f23;
    _24 = f24;
    _31 = f31;
    _32 = f32;
    _33 = f33;
    _34 = f34;
    _41 = f41;
    _42 = f42;
    _43 = f43;
    _44 = f44;
  }
  union {
    struct {
      float _11, _12, _13, _14, _21, _22, _23, _24, _31, _32, _33, _34, _41,
          _42, _43, _44;
    };
    float asArray[16];
  };
  inline float* operator[](int i) { return &(asArray[i * 4]); }
} mat4;

void Transpose(const float* srcMat, float* dstMat, int srcRows, int srcCols);
mat2 Transpose(const mat2& matrix);
mat3 Transpose(const mat3& matrix);
mat4 Transpose(const mat4& matrix);

mat2 operator*(const mat2& matrix, float scalar);
mat3 operator*(const mat3& matrix, float scalar);
mat4 operator*(const mat4& matrix, float scalar);

bool Multiply(float* out, const float* matA, int aRows, int aCols,
              const float* matB, int bRows, int bCols);
mat2 operator*(const mat2& matA, const mat2& matB);
mat3 operator*(const mat3& matA, const mat3& matB);
mat4 operator*(const mat4& matA, const mat4& matB);

mat2 Cut(const mat3& mat, int row, int col);
mat3 Cut(const mat4& mat, int row, int col);

mat2 Minor(const mat2& mat);
mat3 Minor(const mat3& mat);
mat4 Minor(const mat4& mat);

void Cofactor(float* out, const float* minor, int rows, int cols);
mat3 Cofactor(const mat3& mat);
mat2 Cofactor(const mat2& mat);
mat4 Cofactor(const mat4& mat);

float Determinant(const mat2& matrix);
float Determinant(const mat3& mat);
float DeterminantFastMat3(const mat3& mat);
float Determinant(const mat4& mat);

mat2 Adjugate(const mat2& mat);
mat3 Adjugate(const mat3& mat);
mat4 Adjugate(const mat4& mat);

mat2 Inverse(const mat2& mat);
mat3 Inverse(const mat3& mat);
mat4 Inverse(const mat4& mat);

// TRANSLATION
mat4 Translation(float x, float y, float z);
mat4 Translation(const vec3& pos);
vec3 GetTranslation(const mat4& mat);

// SCALLING
mat4 Scale(float x, float y, float z);
mat4 Scale(const vec3& vec);
vec3 GetScale(const mat4& mat);

// ROTATION EULER ANGLES (CAN GIMBAL LOCK)
mat4 Rotation(float pitch, float yaw, float roll);
mat3 Rotation3x3(float pitch, float yaw, float roll);

mat4 ZRotation(float angle);
mat3 ZRotation3x3(float angle);

mat4 XRotation(float angle);
mat3 XRotation3x3(float angle);

mat4 YRotation(float angle);
mat3 YRotation3x3(float angle);

mat4 AxisAngle(const vec3& axis, float angle);
mat3 AxisAngle3x3(const vec3& axis, float angle);

// MULTIPLICATION POINT/VECTOR by MATRIX (PRE-MULTIPLICATION)
vec3 MultiplyPoint(const vec3& vec, const mat4& mat);
vec3 MultiplyVector(const vec3& vec, const mat4& mat);
vec3 MultiplyVector(const vec3& vec, const mat3& mat);

// SCALE -> ROTATE -> TRANSLATE (WORLD TRANSFORM)
mat4 Transform(const vec3& scale, const vec3& eulerRotation,
               const vec3& translate);
mat4 Transform(const vec3& scale, const vec3& rotationAxis, float rotationAngle,
               const vec3& translate);

// VIEW TRANSFORM
mat4 LookAt(const vec3& position, const vec3& target, const vec3& up);

#endif  // !_H_MATH_MATRICES
