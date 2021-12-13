#include "Collision.h"

bool
Collision::
getIntersection(double f_dst1, double f_dst2, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double& t, Eigen::Vector3d& hit)
{
	if ( (f_dst1 * f_dst2) >= 0.0) return false;
	if ( f_dst1 == f_dst2) return false; 
	t = -f_dst1/(f_dst2-f_dst1) ;
	hit = p0 + t*(p1-p0);

	return true;
}
bool
Collision::
inBox(const Eigen::Vector3d& hit, const Eigen::Vector3d& b_min, const Eigen::Vector3d& b_max, int axis)
{
	if ( axis==0 && hit[2] > b_min[2] && hit[2] < b_max[2] && hit[1] > b_min[1] && hit[1] < b_max[1]) return true;
	if ( axis==1 && hit[2] > b_min[2] && hit[2] < b_max[2] && hit[0] > b_min[0] && hit[0] < b_max[0]) return true;
	if ( axis==2 && hit[0] > b_min[0] && hit[0] < b_max[0] && hit[1] > b_min[1] && hit[1] < b_max[1]) return true;
	return false;
}
double
Collision::
checkLineBox(const Eigen::Isometry3d& T_b,
					const Eigen::Vector3d& b_min,
					const Eigen::Vector3d& b_max,
					const Eigen::Vector3d& _l0,
					const Eigen::Vector3d& _l1,
					Eigen::Vector3d& hit)
{
	Eigen::Isometry3d T_b_inv = T_b.inverse();

	Eigen::Vector3d l0 = T_b_inv*_l0;
	Eigen::Vector3d l1 = T_b_inv*_l1;
	if (l1[0] < b_min[0] && l0[0] < b_min[0]) return -1;
	if (l1[0] > b_max[0] && l0[0] > b_max[0]) return -1;
	if (l1[1] < b_min[1] && l0[1] < b_min[1]) return -1;
	if (l1[1] > b_max[1] && l0[1] > b_max[1]) return -1;
	if (l1[2] < b_min[2] && l0[2] < b_min[2]) return -1;
	if (l1[2] > b_max[2] && l0[2] > b_max[2]) return -1;
	if (l0[0] > b_min[0] && l0[0] < b_max[0] &&
		l0[1] > b_min[1] && l0[1] < b_max[1] &&
		l0[2] > b_min[2] && l0[2] < b_max[2]) 
	{
		hit = T_b*l0; 
		return 1.0;
	}
	double t;
	if ( (getIntersection( l0[0]-b_min[0], l1[0]-b_min[0], l0, l1, t, hit) && inBox( hit, b_min, b_max, 0 ))
		|| (getIntersection( l0[1]-b_min[1], l1[1]-b_min[1], l0, l1, t, hit) && inBox( hit, b_min, b_max, 1 )) 
		|| (getIntersection( l0[2]-b_min[2], l1[2]-b_min[2], l0, l1, t, hit) && inBox( hit, b_min, b_max, 2 )) 
		|| (getIntersection( l0[0]-b_max[0], l1[0]-b_max[0], l0, l1, t, hit) && inBox( hit, b_min, b_max, 0 )) 
		|| (getIntersection( l0[1]-b_max[1], l1[1]-b_max[1], l0, l1, t, hit) && inBox( hit, b_min, b_max, 1 )) 
		|| (getIntersection( l0[2]-b_max[2], l1[2]-b_max[2], l0, l1, t, hit) && inBox( hit, b_min, b_max, 2 )))
	{
		hit = T_b*hit;
		return t;
	}
	return -1;
}

double
Collision::
checkLineSphere(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& center, double r, Eigen::Vector3d& hit)
{
	Eigen::Vector3d d = p1 - p0;
	double norm = d.norm();
	d *= 1.0/norm;
	Eigen::Vector3d m = p0 - center;
	double b = m.dot(d);
	double c = m.dot(m) - r*r;

	if(c > 0.0 && b > 0.0) return -1;

	double discr = b*b - c;

	if(discr < 0.0) return -1;

	double t = -b - std::sqrt(discr);

	if(t < 0.0)
		t = 0.0;
	hit = p0 + t*d;
	t /= norm;

	return t;

}