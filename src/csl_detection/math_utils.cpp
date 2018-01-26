#include "math_utils/math_utls.hpp"

//***** General Functions *****
//Angle Wrap (-pi,pi)
double
anglewrap(const double x)
{
	double y=x;

	if (x > M_PI)
		y=x-2.0*M_PI;

	else if (x<-M_PI)
		y=x+2.0*M_PI;

	return y;
}

//Saturation 
double
saturation(double val, double minval, double maxval)
{
	return std::max(minval,std::min(val,maxval));
}

// ***** Quaternion - Euler - Rotation matrix *****
//Quaternion norm
double
quat_norm (const geometry_msgs::Quaternion q)
{
	return sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

double
quat_norm (const geometry_msgs::Quaternion *q)
{
	return sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

//Quaternion Normalize
void
quat_normalize (geometry_msgs::Quaternion *q)
{
	double norm =quat_norm (q);
	q -> w /= norm;
	q -> x /= norm;
	q -> y /= norm;
	q -> z /= norm;
}

//Quaternion multiplication
void
quat_multiply (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c)
{ 
	//Matlab Equivalent ///////divide::: quat_product(quat_inverse( b ), a);
	c -> w = a.w*b.w - (a.x*b.x + a.y*b.y + a.z*b.z);
	c -> x = (a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y);
	c -> y = (a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x);
	c -> z = (a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w);
	quat_normalize(c);
}

//Quaternion inversion (conj)
void
quat_inverse (const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *q_inv)
{
	double w = q.w, x = q.x, y = q.y, z = q.z;
	double norm =quat_norm (q);

	q_inv -> w = +w / norm;
	q_inv -> x = -x / norm;
	q_inv -> y = -y / norm;
	q_inv -> z = -z / norm;
}

//Quaternion division
void
quat_divide (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c)
{
	geometry_msgs::Quaternion b_inv;
	quat_inverse (b, &b_inv);
	quat_multiply (b_inv,a,c);
}

//Rotate a quaternion by axis angles - used also in quaternion differential equation
void
quat_rotation (const geometry_msgs::Vector3 axis, geometry_msgs::Quaternion *qr)
{
	qr->w=1.0;

	double norm = vector3_norm(axis);
	double hnorm, sinhnorm;

	if (norm>=1e-7) {
		hnorm = 0.5*norm;
		sinhnorm = sin(hnorm);

		qr->w = cos(hnorm);
		qr->x = (axis.x/norm)*sinhnorm;
		qr->y = (axis.y/norm)*sinhnorm;
		qr->z = (axis.z/norm)*sinhnorm;
	}
}

//Rotate a quaternion by a vector and qet the rotated vector
void
quat_vector3_rotate (geometry_msgs::Quaternion q, geometry_msgs::Vector3 v, geometry_msgs::Vector3 *res)
{
	geometry_msgs::Quaternion vq, q_inv, qa, qb;
	vq.w = 0.0; vq.x = v.x; vq.y = v.y; vq.z = v.z;
	quat_inverse (q, & q_inv);
	quat_multiply (q, vq, & qa);
	quat_multiply (qa, q_inv, & qb);
	res -> x = qb.x;
	res -> y = qb.y;
	res -> z = qb.z;
}

//Quaternion normalize and auto-complete w
void
quat_normal_wcomplete (geometry_msgs::Quaternion *q)
{
	double x = q->x, y = q->y, z = q->z;
	q -> w = sqrt (1.0 - (x*x + y*y + z*z));
}

//Quaternion duality - get equivalent with positive w
void
quat_equiv_wpos_get (geometry_msgs::Quaternion *q)
{
	if (q -> w < 0.0)
	{
		q->w = -(q->w);
		q->x = -(q->x); q->y = -(q->y); q->z = -(q->z);
	}
}

//Quaternion duality - get equivalent with negative w
void
quat_equiv_wneg_get (geometry_msgs::Quaternion *q)
{
	if (q -> w > 0.0)
	{
		q->w = -(q->w);
		q->x = -(q->x); q->y = -(q->y); q->z = -(q->z);
	}
}

//Quaternion to Rotation Matrix - (Reb)
void
quat2rotmtx (const geometry_msgs::Quaternion q, double *rotmtx)
{
	const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
	const double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
	const double q01 = q0*q1, q02 = q0*q2, q03 = q0*q3;
	const double q12 = q1*q2, q13 = q1*q3;
	const double q23 = q2*q3;
	rotmtx [0] = q00 + q11 - q22 - q33;
	rotmtx [1] = 2.0*(q12 - q03);
	rotmtx [2] = 2.0*(q02 + q13);
	rotmtx [3] = 2.0*(q12 + q03);
	rotmtx [4] = q00 - q11 + q22 - q33;
	rotmtx [5] = 2.0*(q23 - q01);
	rotmtx [6] = 2.0*(q13 - q02);
	rotmtx [7] = 2.0*(q01 + q23);
	rotmtx [8] = q00 - q11 - q22 + q33;
}

//Rotation Matrix to Quaternion - (Reb)
void
rotmtx2quat (const double *rotmtx, geometry_msgs::Quaternion *q)
{
	q->z = 0.0;
	double sqtrp1;
	double sqtrp1x2;
	double d[3];
	double sqdip1;


	const double tr = rotmtx[0] + rotmtx[4] + rotmtx[8];
	if (tr>0)
	{
		sqtrp1 = sqrt(tr + 1.0);
		sqtrp1x2 = 2.0*sqtrp1;

		q->w = 0.5*sqtrp1;
		q->x = (rotmtx[7] - rotmtx[5])/sqtrp1x2;
		q->y = (rotmtx[2] - rotmtx[6])/sqtrp1x2;
		q->z = (rotmtx[3] - rotmtx[1])/sqtrp1x2;
	}
	else
	{
		d[0]=rotmtx[0];
		d[1]=rotmtx[4];
		d[2]=rotmtx[8];

		if ((d[1] > d[0]) && (d[1] > d[2]))
		{
			sqdip1 = sqrt(d[1] - d[0] - d[2] + 1.0 );
			q->y = 0.5*sqdip1;

			if ( abs(sqdip1) > 1e-7 )
				sqdip1 = 0.5/sqdip1;

			q->w = (rotmtx[2] - rotmtx[6])*sqdip1;
			q->x = (rotmtx[3] + rotmtx[1])*sqdip1;
			q->z = (rotmtx[7] + rotmtx[5])*sqdip1;
		}
		else if (d[2] > d[0])
		{
			//max value at R(3,3)
			sqdip1 = sqrt(d[2] - d[0] - d[1] + 1.0 );

			q->z = 0.5*sqdip1;

			if ( abs(sqdip1) > 1e-7 )
				sqdip1 = 0.5/sqdip1;

			q->w = (rotmtx[3] - rotmtx[1])*sqdip1;
			q->x = (rotmtx[2] + rotmtx[6])*sqdip1;
			q->y = (rotmtx[7] + rotmtx[5])*sqdip1;
		}
		else
		{
			// max value at R(1,1)
			sqdip1 = sqrt(d[0] - d[1] - d[2] + 1.0 );

			q->x = 0.5*sqdip1;

			if ( abs(sqdip1) > 1e-7 )
				sqdip1 = 0.5/sqdip1;

			q->w = (rotmtx[7] - rotmtx[5])*sqdip1;
			q->y = (rotmtx[3] + rotmtx[1])*sqdip1;
			q->z = (rotmtx[2] + rotmtx[6])*sqdip1;
		}
	}

}

//Quaternion to Euler-ZYX
void
quat2eulerZYX (const geometry_msgs::Quaternion q, geometry_msgs::Vector3 *euler)
{
	const double tol = 0.99999f;
	const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
	const double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
	const double q01 = q0*q1, q23 = q2*q3;
	const double q12 = q1*q2, q03 = q0*q3;
	double test = q1*q3 - q0*q2;
	if (test > +tol)
	{
		euler->x = atan2 (-2.0*(q23-q01), q00-q11+q22-q33);
		euler->y = +0.5 * M_PI;
		euler->z = 0.0;
		return;
	}
	else if (test < -tol)
	{
		euler->x = atan2 (-2.0*(q23-q01), q00-q11+q22-q33);
		euler->y = -0.5 * M_PI;
		euler->z = tol;
		return;
	}
	else
	{
		euler->x = atan2 (2.0*(q23+q01), q00-q11-q22+q33);
		euler->y = asin (2.0*test);
		euler->z = atan2 (2.0*(q12 + q03), q00 + q11 - q22 - q33);
		return;
	}
}

//Euler-ZYX to Quaternion
void
eulerZYX2quat (const geometry_msgs::Vector3 euler, geometry_msgs::Quaternion *q)
{
	double cpsi, spsi, ctheta, stheta, cphi, sphi;
	cpsi = cos (0.5 * euler.z); spsi = sin (0.5 * euler.z);
	ctheta = cos (0.5 * euler.y); stheta = sin (0.5 * euler.y);
	cphi = cos (0.5 * euler.x); sphi = sin (0.5 * euler.x);
	q -> w = cphi*ctheta*cpsi + sphi*stheta*spsi;
	q -> x = sphi*ctheta*cpsi - cphi*stheta*spsi;
	q -> y = cphi*stheta*cpsi + sphi*ctheta*spsi;
	q -> z = cphi*ctheta*spsi - sphi*stheta*cpsi;
}

//Euler-ZYX to Rotation Matrix - (Reb)
void
eulerZYX2rotmtx (const geometry_msgs::Vector3 euler, double *rotmtx)
{
	double cpsi, spsi, ctheta, stheta, cphi, sphi;
	/**** Calculate trigonometric values. ****/
	cpsi = cos (euler.z); spsi = sin (euler.z);
	ctheta = cos (euler.y); stheta = sin (euler.y);
	cphi = cos (euler.x); sphi = sin (euler.x);
	/**** Calculate rotation matrix. ****/
	rotmtx [0] = cpsi * ctheta;
	rotmtx [1] = sphi * cpsi * stheta - cphi * spsi;
	rotmtx [2] = cphi * cpsi * stheta + sphi * spsi;
	rotmtx [3] = spsi * ctheta;
	rotmtx [4] = sphi * spsi * stheta + cphi * cpsi;
	rotmtx [5] = cphi * spsi * stheta - sphi * cpsi;
	rotmtx [6] = -stheta;
	rotmtx [7] = sphi * ctheta;
	rotmtx [8] = cphi * ctheta;
}

//Rotation Matrix to Euler-ZYX - (Reb)
void
rotmtx2eulerZYX (const double *rotmtx, geometry_msgs::Vector3 *euler)
{
	const double tol = 0.99999f;
	const double test = -rotmtx[6];
	if (test > +tol)
	{
		euler->x = atan2(-rotmtx[5], rotmtx[4]);
		euler->y = +0.5 * M_PI;
		euler->z = 0.0;
	}
	else if (test < -tol)
	{
		euler->x = atan2(-rotmtx[5], rotmtx[4]);
		euler->y = -0.5 * M_PI;
		euler->z = 0.0;
	}
	else
	{
		euler->x = atan2 (rotmtx[7], rotmtx[8]);
		euler->y = asin (-rotmtx[6]);
		euler->z = atan2 (rotmtx[3], rotmtx[0]);
	}
}

// ***** Manipulation of Matrices *****
// Inverse Matrix - GaussJordan
void inverseGJ (const double *mtx_a, double *mtx_inv, const unsigned int dim)
{
	unsigned int iii, jjj, kkk;
	double pivot_element, row_multiplier;
	double *mtx_z;
	/**** Construct mtx_z. ****/
	mtx_z = (double*) malloc (dim * dim * sizeof (double));
	for (iii = 0; iii < dim; ++ iii)
		for (jjj = 0; jjj < dim; ++ jjj)
			mtx_z [iii * dim + jjj] = mtx_a [iii * dim + jjj];
	/**** Initialize mtx_inv. ****/
	for (iii = 0; iii < dim; ++ iii)
		for (jjj = 0; jjj < dim; ++ jjj)
			mtx_inv [iii * dim + jjj] = 0.0;
	for (kkk = 0; kkk < dim; ++ kkk)
		mtx_inv [kkk * dim + kkk] = 1.0;
	/**** Solve for the inverse mtx_inv of mtx_a (forwards). ****/
	for (iii = 0; iii < dim; ++ iii) {
		pivot_element = mtx_z [iii * dim + iii];
		for (jjj = iii; jjj < dim; ++ jjj) {
			mtx_z [iii * dim + jjj] /= pivot_element;
		}
		for (jjj = 0; jjj < dim; ++ jjj) {
			mtx_inv [iii * dim + jjj] /= pivot_element;
		}
		for (jjj = iii + 1; jjj < dim; ++ jjj) {
			row_multiplier = mtx_z [jjj * dim + iii];
			for (kkk = iii; kkk < dim; ++ kkk) {
				mtx_z [jjj * dim + kkk] -=
						row_multiplier * mtx_z [iii * dim + kkk];
			}
			for (kkk = 0; kkk < dim; ++ kkk) {
				mtx_inv [jjj * dim + kkk] -=
						row_multiplier * mtx_inv [iii * dim + kkk];
			}
		}
	}
	/**** Solve for the inverse mtx_inv of mtx_a (backwards). ****/
	for (int iii = dim - 1; iii >= 0; -- iii) {
		for (int jjj = iii - 1; jjj >= 0; -- jjj) {
			row_multiplier = mtx_z [jjj * dim + iii];
			mtx_z [jjj * dim + iii] -= row_multiplier * mtx_z [iii * dim + iii];
			for (kkk = 0; kkk < dim; ++ kkk) {
				mtx_inv [jjj * dim + kkk] -=
						row_multiplier * mtx_inv [iii * dim + kkk];
			}
		}
	}
	/**** Free resources. ****/
	free (mtx_z);
}

// Inverse Matrix Fast
int inverseFast (double* A, double* Ainv, int N)
{
	int *IPIV = new int[N+1];
	int LWORK = N*N;
	double *WORK = new double[LWORK];
	int INFO;

	memcpy(Ainv,A,LWORK*sizeof(double));


	dgetrf_(&N,&N,Ainv,&N,IPIV,&INFO);
	dgetri_(&N,Ainv,&N,IPIV,WORK,&LWORK,&INFO);

	delete IPIV;
	delete WORK;

	return INFO;
}

//Cholesky
void
cholesky (const unsigned int dim, const double *mtx_a, double *mtx_l)
{
	unsigned int iii, jjj, kkk;
	double sum;
	/**** Initialize 'mtx_l'.
	 ****/
	for (iii = 0; iii < dim; ++ iii)
		for (jjj = 0; jjj < dim; ++ jjj)
			mtx_l [iii * dim + jjj] = 0.0;
	/**** Calculate 'mtx_l'.
	 ****/
	for (iii = 0; iii < dim; ++ iii)
	{
		/**** Calculate the i-th column's, main-diagonal element of 'mtx_l'.
		 ****/
		sum = 0.0;
		for (kkk = 0; kkk < iii; ++ kkk)
			sum += mtx_l [iii * dim + kkk] * mtx_l [iii * dim + kkk];
		mtx_l [iii * dim + iii] =
				sqrt (mtx_a [iii * dim + iii] - sum);
		/**** Calculate the i-th column's, off-diagonal elements of 'mtx_l'.
		 ****/
		for (jjj = iii + 1; jjj < dim; ++ jjj)
		{
			sum = 0.0;
			for (kkk = 0; kkk < iii; ++ kkk)
				sum += mtx_l [jjj * dim + kkk] * mtx_l [iii * dim + kkk];
			mtx_l [jjj * dim + iii] =
					(mtx_a [jjj * dim + iii] - sum) / mtx_l [iii * dim + iii];
		}
	}
}

//Singular Value Decomposition NxN (fast ~ unstable)
void
svdF (double *A, double *U, double *S, double *V, const unsigned int dim)
{
	unsigned int loopmax = 100*dim;
	unsigned int loopcount = 0;
	double tol = 1e-7;
	unsigned int iii,jjj;
	double Err = 1e7;
	double *R, *Ri;
	double *SST;

	double asum;

	R = (double*) malloc (dim * dim * sizeof (double));
	Ri = (double*) malloc (dim * dim * sizeof (double));
	SST = (double*) malloc (dim * dim * sizeof (double));

	//Initialize U,S and V;
	for (iii = 0; iii < dim; ++ iii)
		for (jjj = 0; jjj < dim; ++ jjj)
		{
			S [iii * dim + jjj] = A [jjj * dim + iii];
			if (iii==jjj)
			{
				U [iii * dim + jjj] = 1.0;
				V [iii * dim + jjj] = 1.0;
			}
			else
			{
				U [iii * dim + jjj] = 0.0;
				V [iii * dim + jjj] = 0.0;
			}
		}

	while (Err>tol and loopcount++<loopmax)
	{
		//Compute S,U
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = 0; jjj < dim; ++ jjj)
				SST[iii * dim + jjj]=S[iii * dim + jjj]*S[jjj*dim+iii];

		cholesky (dim, SST, R);
		inverseFast(R,Ri,dim);
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = 0; jjj < dim; ++ jjj)
				U[iii * dim + jjj]*=U[iii * dim + jjj]*S[jjj*dim+iii]*Ri[iii * dim + jjj];
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = 0; jjj < dim; ++ jjj)
				S[iii * dim + jjj]=R[iii * dim + jjj];

		//Compute S,V
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = 0; jjj < dim; ++ jjj)
				SST[iii * dim + jjj]=S[iii * dim + jjj]*S[jjj*dim+iii];

		cholesky (dim, SST, R);
		inverseFast(R,Ri,dim);
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = 0; jjj < dim; ++ jjj)
				V[iii * dim + jjj]*=V[iii * dim + jjj]*S[jjj*dim+iii]*Ri[iii * dim + jjj];
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = 0; jjj < dim; ++ jjj)
				S[iii * dim + jjj]=R[iii * dim + jjj];

		//Exit flag
		asum = 0;
		for (iii = 0; iii < dim; ++ iii)
			for (jjj = iii+1; jjj < dim; ++ jjj)
				Err = asum + abs(S[iii * dim + jjj]);

	}

	free(R);
	free(Ri);
	free(SST);
}

// res = A 3x3 * B 3x3
void 
multi_mtx_mtx_3X3 (const double *a, const double *b, double *res)
{
	unsigned int iii, jjj, kkk;
	for (iii = 0; iii < 3; ++ iii) {
		for (jjj = 0; jjj < 3; ++ jjj) {
			res [iii * 3 + jjj] = 0.0;
			for (kkk = 0; kkk < 3; ++ kkk)
				res [iii * 3 + jjj] += a [iii * 3 + kkk] * b [kkk * 3 + jjj];
		}
	}
}

// res = A 3x3 * b 3xn
void 
multi_mtx_mtx_3Xn (const double *a, const double *b, double *res, const unsigned int n)
{
	unsigned int iii, jjj, kkk;
	for (iii = 0; iii < 3; ++ iii) {
		for (jjj = 0; jjj < n; ++ jjj) {
			res [iii * n + jjj] = 0.0;
			for (kkk = 0; kkk < 3; ++ kkk)
				res [iii * n + jjj] += a [iii * 3 + kkk] * b [kkk*n + jjj];
		}
	}
}

// res = A' 3x3 * B 3x3
void
multi_mtxT_mtx_3X3 (const double *a, const double *b, double *res)
{
	int iii, jjj, kkk;
	for (iii = 0; iii < 3; ++ iii) {
		for (jjj = 0; jjj < 3; ++ jjj) {
			res [iii * 3 + jjj] = 0.0;
			for (kkk = 0; kkk < 3; ++ kkk)
				res [iii * 3 + jjj] += a [kkk * 3 + iii] * b [kkk* 3 + jjj];
		}
	}
}

// res = A' 3x3 * B 3xn
void 
multi_mtxT_mtx_3Xn (const double *a, const double *b, double *res, const unsigned int n)
{
	unsigned int iii, jjj, kkk;
	for (iii = 0; iii < 3; ++ iii) {
		for (jjj = 0; jjj < n; ++ jjj) {
			res [iii * n + jjj] = 0.0;
			for (kkk = 0; kkk < 3; ++ kkk)
				res [iii * n + jjj] += a [kkk * 3 + iii] * b [kkk*n + jjj];
		}
	}
}

void
multi_mtx_vec_3X3(const double *mtx, const double *vec, double *res)
{
	res[0] = mtx[0] * vec[0] + mtx[1] * vec[1] + mtx[2] * vec[2];
	res[1] = mtx[3] * vec[0] + mtx[4] * vec[1] + mtx[5] * vec[2];
	res[2] = mtx[6] * vec[0] + mtx[7] * vec[1] + mtx[8] * vec[2];
}

void
multi_mtxT_vec_3X3(const double *mtx, const double *vec, double *res)
{
	res[0] = mtx[0] * vec[0] + mtx[3] * vec[1] + mtx[6] * vec[2];
	res[1] = mtx[1] * vec[0] + mtx[4] * vec[1] + mtx[7] * vec[2];
	res[2] = mtx[2] * vec[0] + mtx[5] * vec[1] + mtx[8] * vec[2];
}


//Create diagonal NxN matrix from a vector Nx1
void
diag_arrayN (double *vec, double *A, unsigned int n)
{
	unsigned int iii, jjj;
	for (iii = 0; iii < n; ++ iii)
		for (jjj = 0; jjj < n; ++ jjj)
		{
			if (iii==jjj)
			{
				A[iii * n + jjj]= vec[iii];
			}
			else
			{
				A[iii * n + jjj] = 0.0;
			}
		}
}

//Create diag block matrix by two nxn matrices
void
block_diag (double *A, double *B, unsigned int dima, unsigned int dimb, double *C)
{
	unsigned int iii, jjj, sum;

	sum = dima + dimb;

	for (iii = 0; iii < sum; ++ iii)
		for (jjj = 0; jjj < sum; ++ jjj)
			C[iii * sum + jjj] = 0.0;

	for (iii = 0; iii < dima; ++ iii)
		for (jjj = 0; jjj < dima; ++ jjj)
			C[iii * dima + jjj] = A[iii * dima + jjj];

	for (iii = 0; iii < dimb; ++ iii)
		for (jjj = 0; jjj < dimb; ++ jjj)
			C[(iii + dima)* dimb + jjj+dima] = B[iii * dimb + jjj];

}

//***** Quaternion and Vector 3 msgs operators *****
//Cross product of Vector3
void
vector3_cross (geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, geometry_msgs::Vector3 *c)
{
	c->x = a.y*b.z - a.z*b.y;
	c->y = a.z*b.x - a.x*b.z;
	c->z = a.x*b.y - a.y*b.x;
}

//Norm of Vector3
double
vector3_norm (geometry_msgs::Vector3 vec)
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

//Euclidean Distance of two vectors (Vector3)
double
vector3_distance (geometry_msgs::Vector3 veca, geometry_msgs::Vector3 vecb)
{
	geometry_msgs::Vector3 vec;
	vec.x = veca.x - vecb.x;
	vec.y = veca.y - vecb.y;
	vec.z = veca.z - vecb.z;
	return vector3_norm(vec);
}

////------------------------------------

// Matrix 3x3 * Vector3
/*geometry_msgs::Vector3
operator* (const double *mtx, geometry_msgs::Vector3& vec)
{
	geometry_msgs::Vector3 c;
	c.x = mtx[0]*vec.x+mtx[1]*vec.y+mtx[2]*vec.z;
	c.y = mtx[3]*vec.x+mtx[4]*vec.y+mtx[5]*vec.z;
	c.z = mtx[6]*vec.x+mtx[7]*vec.y+mtx[8]*vec.z;

	return c;
}*/

// Matrix Transpose * Vector3
/*geometry_msgs::Vector3
operator/ (const double *mtx, geometry_msgs::Vector3& vec)
{
	geometry_msgs::Vector3 c;
	c.x = mtx[0]*vec.x+mtx[3]*vec.y+mtx[6]*vec.z;
	c.y = mtx[1]*vec.x+mtx[4]*vec.y+mtx[7]*vec.z;
	c.z = mtx[2]*vec.x+mtx[5]*vec.y+mtx[8]*vec.z;

	return c;
}

// - Vector3
geometry_msgs::Vector3 operator- (geometry_msgs::Vector3& vec)
{
	geometry_msgs::Vector3 c;
	c.x = -vec.x;
	c.y = -vec.y;
	c.z = -vec.z;

	return c;
}

// Vector3+ Vector3
geometry_msgs::Vector3
operator+ (const geometry_msgs::Vector3& a, geometry_msgs::Vector3& b)
{
	geometry_msgs::Vector3 c;
	c.x = a.x+b.x;
	c.y = a.y+b.y;
	c.z = a.z+b.z;

	return c;
}

// Vector3 -  Vector3
geometry_msgs::Vector3
operator- (geometry_msgs::Vector3& a,geometry_msgs::Vector3& b)
{
	geometry_msgs::Vector3 c;
	c.x = a.x-b.x;
	c.y = a.y-b.y;
	c.z = a.z-b.z;

	return c;
}

// scale a*Vector3
geometry_msgs::Vector3
operator* (const double a, geometry_msgs::Vector3& vec)
{
	geometry_msgs::Vector3 c;
	c.x = a*vec.x;
	c.y = a*vec.y;
	c.z = a*vec.z;

	return c;
}

// Vector3 * scale a
geometry_msgs::Vector3
operator* (geometry_msgs::Vector3& vec, const double a)
{
	geometry_msgs::Vector3 c;
	c.x = a*vec.x;
	c.y = a*vec.y;
	c.z = a*vec.z;

	return c;
}*/

//Without &
// Matrix 3x3 * Vector3
geometry_msgs::Vector3
operator* (const double *mtx, geometry_msgs::Vector3 vec)
{
	geometry_msgs::Vector3 c;
	c.x = mtx[0]*vec.x+mtx[1]*vec.y+mtx[2]*vec.z;
	c.y = mtx[3]*vec.x+mtx[4]*vec.y+mtx[5]*vec.z;
	c.z = mtx[6]*vec.x+mtx[7]*vec.y+mtx[8]*vec.z;

	return c;
}

// Matrix Transpose * Vector3
geometry_msgs::Vector3
operator/ (const double *mtx, geometry_msgs::Vector3 vec)
{
	geometry_msgs::Vector3 c;
	c.x = mtx[0]*vec.x+mtx[3]*vec.y+mtx[6]*vec.z;
	c.y = mtx[1]*vec.x+mtx[4]*vec.y+mtx[7]*vec.z;
	c.z = mtx[2]*vec.x+mtx[5]*vec.y+mtx[8]*vec.z;

	return c;
}

// - Vector3
geometry_msgs::Vector3 operator- (geometry_msgs::Vector3 vec)
{
	geometry_msgs::Vector3 c;
	c.x = -vec.x;
	c.y = -vec.y;
	c.z = -vec.z;

	return c;
}

// Vector3+ Vector3
geometry_msgs::Vector3
operator+ (const geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
	geometry_msgs::Vector3 c;
	c.x = a.x+b.x;
	c.y = a.y+b.y;
	c.z = a.z+b.z;

	return c;
}

// Vector3 -  Vector3
geometry_msgs::Vector3
operator- (geometry_msgs::Vector3 a,geometry_msgs::Vector3 b)
{
	geometry_msgs::Vector3 c;
	c.x = a.x-b.x;
	c.y = a.y-b.y;
	c.z = a.z-b.z;

	return c;
}

// scale a*Vector3
geometry_msgs::Vector3
operator* (const double a, geometry_msgs::Vector3 vec)
{
	geometry_msgs::Vector3 c;
	c.x = a*vec.x;
	c.y = a*vec.y;
	c.z = a*vec.z;

	return c;
}

// Vector3 * scale a
geometry_msgs::Vector3
operator* (geometry_msgs::Vector3 vec, const double a)
{
	geometry_msgs::Vector3 c;
	c.x = a*vec.x;
	c.y = a*vec.y;
	c.z = a*vec.z;

	return c;
}
