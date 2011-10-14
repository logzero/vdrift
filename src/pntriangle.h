#ifndef _PNTRIANGLE_H
#define _PNTRIANGLE_H

#include "LinearMath/btVector3.h"

// p = p0 * (1 - f) + p1 * f, n normal axis
inline btScalar getFraction(
	const btVector3 & p0, const btVector3 & p1,
	const btVector3 & p, const btVector3 & n)
{
	btScalar np = n.dot(p);
	btScalar np0 = n.dot(p0);
	btScalar np1 = n.dot(p1);
	return (np0 - np) / (np0 - np1);
}

// p = p0 * (1 - u - v) + p1 * u + p2 * v
inline void getBarycentricCoords(
	const btVector3 & p0, const btVector3 & p1, const btVector3 & p2,
	const btVector3 & p, btScalar & u, btScalar & v)
{
	btVector3 e0 = p1 - p0;
	btVector3 e1 = p2 - p0;
	btVector3 e2 = p - p0;
	btScalar dot00 = e0.dot(e0);
	btScalar dot01 = e0.dot(e1);
	btScalar dot02 = e0.dot(e2);
	btScalar dot11 = e1.dot(e1);
	btScalar dot12 = e1.dot(e2);
	btScalar invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	v = (dot00 * dot12 - dot01 * dot02) * invDenom;
}

// ray hit for t > 0, return u, v, t
inline btVector3 castRay(
	const btVector3 & p0, const btVector3 & p1, const btVector3 & p2,
	const btVector3 & org, const btVector3 & dir, const btScalar t)
{
	btVector3 p10 = p1 - p0;
	btVector3 p20 = p2 - p0;

	btVector3 p = dir.cross(p20);
	btScalar a = p10.dot(p);
	if (a > -1E-5 && a < 1E-5) return btVector3(0, 0, 0);

	btScalar f = 1 / a;
	btVector3 s = org - p0;
	btScalar u = f * s.dot(p);
	if (u < 0.0 || u > 1.0) return btVector3(0, 0, 0);

	btVector3 q = s.cross(p10);
	btScalar v = f * dir.dot(q);
	if (v < 0.0 || u + v > 1.0) return btVector3(0, 0, 0);

	return btVector3(u, v, f * p20.dot(q));
}

// Triangle normal quadratic interpolation
class TriangleNormal
{
public:
	void set(
		const btVector3 & p0, const btVector3 & p1, const btVector3 & p2,
		const btVector3 & n0, const btVector3 & n1, const btVector3 & n2)
	{
		// edges
		btVector3 p10 = p1 - p0;
		btVector3 p12 = p1 - p2;
		btVector3 p20 = p2 - p0;

		// vij = 2 * (pj - pi) * (ni + nj) / ((pj - pi) * (pj - pi))
		btScalar v01 = 2 * p10.dot(n0 + n1) / p10.dot(p10);
		btScalar v12 = 2 * p12.dot(n1 + n2) / p12.dot(p12);
		btScalar v20 = 2 * p20.dot(n2 + n0) / p20.dot(p20);

		btVector3 h110 = n0 + n1 - p10 * v01;
		btVector3 h011 = n1 + n2 + p12 * v12;
		btVector3 h101 = n2 + n0 + p20 * v20;

		// normal coefficients
		n200 = n0;
		n020 = n1;
		n002 = n2;
		n110 = h110.normalize();
		n011 = h011.normalize();
		n101 = h101.normalize();
	}

	btVector3 getNormal(btScalar u, btScalar v) const
	{
		// quadratic bezier
		btScalar w = 1 - u - v;
		btVector3 n =
			n200 * w * w + n020 * u * u + n002 * v * v +
			n110 * w * u + n011 * u * v + n101 * w * v;
		return n.normalize();
	}

private:
	btVector3 n200, n020, n002, n110, n011, n101;
};

// Triangle point cubic interpolation
class TrianglePoint
{
public:
	void set(
		const btVector3 & p0, const btVector3 & p1, const btVector3 & p2,
		const btVector3 & n0, const btVector3 & n1, const btVector3 & n2)
	{
		// edges
		btVector3 p10 = p1 - p0;
		btVector3 p12 = p1 - p2;
		btVector3 p20 = p2 - p0;

		// normal plane projection coefficients
		// wij = (pj - pi) * ni
		btScalar w01 = p10.dot(n0);
		btScalar w10 = -p10.dot(n1);
		btScalar w12 = -p12.dot(n1);
		btScalar w21 = p12.dot(n2);
		btScalar w20 = -p20.dot(n0);
		btScalar w02 = p20.dot(n2);

		// vertex coefficients
		b300 = p0;
		b030 = p1;
		b003 = p2;

		// tangent coefficients
		b210 = (p0 * 2 + p1 - n0 * w01);// * 1/3;
		b120 = (p1 * 2 + p0 - n1 * w10);// * 1/3;
		b021 = (p1 * 2 + p2 - n1 * w12);// * 1/3;
		b012 = (p2 * 2 + p1 - n2 * w21);// * 1/3;
		b102 = (p2 * 2 + p0 - n2 * w20);// * 1/3;
		b201 = (p0 * 2 + p2 - n0 * w02);// * 1/3;

		// center coefficient
		btVector3 e = (b210 + b120 + b021 + b012 + b102 + b201) * 1/3;// * 1/2;
		btVector3 v = (p0 + p1 + p2);// * 1/3;
		b111 = e * 1.5 - v;//b111 = e + (e - v) * 1/2;
	}

	// Barycentric coordinates of p
	void getBarycentric(const btVector3 & p, btScalar & u,  btScalar & v) const
	{
		getBarycentricCoords(b300, b030, b003, p, u, v);
	}

	btVector3 getPoint(btScalar u, btScalar v) const
	{
		// cubic bezier
		btScalar w = 1 - u - v;
		btVector3 p =
			b300 * w * w * w + b030 * u * u * u + b003 * v * v * v +
			b210 * w * w * u + b120 * w * u * u + b201 * w * w * v +//b210 * 3 * w * w * u + b120 * 3 * w * u * u + b201 * 3 * w * w * v +
			b021 * u * u * v + b102 * w * v * v + b012 * u * v * v +//b021 * 3 * u * u * v + b102 * 3 * w * v * v + b012 * 3 * u * v * v +
			b111 * w * u * v;//b111 * 6 * w * u * v;
		return p;
	}

private:
	btVector3 b300, b030, b003;
	btVector3 b210, b120, b021, b012, b102, b201;
	btVector3 b111;
};

// Curved pn-triangle
class Triangle : public TrianglePoint, public TriangleNormal
{
public:
	void set(
		const btVector3 & p0, const btVector3 & p1, const btVector3 & p2,
		const btVector3 & n0, const btVector3 & n1, const btVector3 & n2)
	{
		TrianglePoint::set(p0, p1, p2, n0, n1, n2);
		TriangleNormal::set(p0, p1, p2, n0, n1, n2);
	}
};

#endif
