#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cstdio>
#include <cmath>

#define sign(x) (( x > 0 ) - ( x < 0 ))

extern "C"
{
    // LU decomoposition of a general matrix
    void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);

    // generate inverse of a matrix given its LU decomposition
    void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
}
//***** General Functions *****
//Angle Wrap (-pi,pi)
double
anglewrap(const double x);

//Saturation
double
saturation (double val, double minval, double maxval);

// ***** Quaternion - Euler - Rotation matrix *****
//Quaternion norm
double
quat_norm (const geometry_msgs::Quaternion q);

double
quat_norm (const geometry_msgs::Quaternion *q);

//Quaternion Normalize
void
quat_normalize (geometry_msgs::Quaternion *q);

//Quaternion multiplication
void
quat_multiply (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c);

//Quaternion inversion (conj)
void
quat_inverse (const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *q_inv);

//Quaternion division
void
quat_divide (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c);

//Rotate a quaternion by axis angles - used also in quaternion differential equation
void
quat_rotation (const geometry_msgs::Vector3 axis, geometry_msgs::Quaternion *qr);

//Rotate a quaternion by a vector and qet the rotated vector
void
quat_vector3_rotate (geometry_msgs::Quaternion q, geometry_msgs::Vector3 v, geometry_msgs::Vector3 *res);

//Quaternion normalize and auto-complete w
void
quat_normal_wcomplete (geometry_msgs::Quaternion *q);

//Quaternion duality - get equivalent with positive w
void
quat_equiv_wpos_get (geometry_msgs::Quaternion *q);

//Quaternion duality - get equivalent with negative w
void
quat_equiv_wneg_get (geometry_msgs::Quaternion *q);

//Quaternion to Rotation Matrix - (Reb)
void
quat2rotmtx (const geometry_msgs::Quaternion q, double *rotmtx);

//Rotation Matrix to Quaternion - (Reb)
void
rotmtx2quat (const double *rotmtx, geometry_msgs::Quaternion *q);

//Quaternion to Euler-ZYX
void
quat2eulerZYX (const geometry_msgs::Quaternion q, geometry_msgs::Vector3 *euler);

//Euler-ZYX to Quaternion
void
eulerZYX2quat (const geometry_msgs::Vector3 euler, geometry_msgs::Quaternion *q);

//Euler-ZYX to Rotation Matrix - (Reb)
void
eulerZYX2rotmtx (const geometry_msgs::Vector3 euler, double *rotmtx);

//Rotation Matrix to Euler-ZYX - (Reb)
void
rotmtx2eulerZYX (const double *rotmtx, geometry_msgs::Vector3 *euler);

// ***** Manipulation of Matrices *****
// Inverse Matrix - GaussJordan
void inverseGJ(const double *mtx_a, double *mtx_inv, const unsigned int dim);

// Inverse Matrix Fast
int inverseFast (double* A, double* Ainv, int N);

//Cholesky
void
cholesky (const unsigned int dim, const double *mtx_a, double *mtx_l);

//Singular Value Decomposition NxN (fast ~ unstable)
void
svdF (double *A, double *U, double *S, double *V, const unsigned int dim);

// res = A 3x3 * B 3x3
void
multi_mtx_mtx_3X3 (const double *a, const double *b, double *res);

// res = A 3x3 * b 3xn
void 
multi_mtx_mtx_3Xn (const double *a, const double *b, double *res, const unsigned int n);

// res = A' 3x3 * B 3x3
void
multi_mtxT_mtx_3X3 (const double *a, const double *b, double *res);

// res = A' 3x3 * B 3xn
void 
multi_mtxT_mtx_3Xn (const double *a, const double *b, double *res, const unsigned int n);

// res = mtx 3x3 * b 3x1
void
multi_mtx_vec_3X3(const double *mtx, const double *vec, double *res);

// res = mtx' 3x3 * b 3x1
void
multi_mtxT_vec_3X3(const double *mtx, const double *vec, double *res);

//Create diagonal NxN matrix from a vector Nx1
void
diag_arrayN (double *vec, double *A, unsigned int n);

//Create diag block matrix by two nxn matrices
void
block_diag (double *A, double *B, unsigned int dima, unsigned int dimb, double *C);

//***** Quaternion and Vector 3 msgs operators *****
//Cross product of Vector3
void
vector3_cross (geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, geometry_msgs::Vector3 *c);

//Norm of Vector3
double vector3_norm (geometry_msgs::Vector3 vec);

//Euclidean Distance of two vectors (Vector3)
double
vector3_distance (geometry_msgs::Vector3 veca, geometry_msgs::Vector3 vecb);

///----------------------------------------
// Matrix 3x3 * Vector3
//geometry_msgs::Vector3 operator* (const double *mtx, geometry_msgs::Vector3& vec) ;

// Matrix Transpose * Vector3
//geometry_msgs::Vector3 operator/ (const double *mtx, geometry_msgs::Vector3& vec) ;

// - Vector3
//geometry_msgs::Vector3 operator- (geometry_msgs::Vector3& vec) ;

// Vector3+ Vector3
//geometry_msgs::Vector3 operator+ (const geometry_msgs::Vector3& a, geometry_msgs::Vector3& b);

// Vector3 -  Vector3
//geometry_msgs::Vector3 operator- (geometry_msgs::Vector3& a,geometry_msgs::Vector3& b) ;

// scale a*Vector3
//geometry_msgs::Vector3 operator* (const double a, geometry_msgs::Vector3& vec);

// Vector3 * scale a
//geometry_msgs::Vector3 operator* (geometry_msgs::Vector3& vec, const double a);

//Without &
// Matrix 3x3 * Vector3
geometry_msgs::Vector3 operator* (const double *mtx, geometry_msgs::Vector3 vec) ;

// Matrix Transpose * Vector3
geometry_msgs::Vector3 operator/ (const double *mtx, geometry_msgs::Vector3 vec) ;

// - Vector3
geometry_msgs::Vector3 operator- (geometry_msgs::Vector3 vec) ;

// Vector3+ Vector3
geometry_msgs::Vector3 operator+ (const geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);

// Vector3 -  Vector3
geometry_msgs::Vector3 operator- (geometry_msgs::Vector3 a,geometry_msgs::Vector3 b) ;

// scale a*Vector3
geometry_msgs::Vector3 operator* (const double a, geometry_msgs::Vector3 vec);

// Vector3 * scale a
geometry_msgs::Vector3 operator* (geometry_msgs::Vector3 vec, const double a);
