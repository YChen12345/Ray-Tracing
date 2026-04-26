#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		float dot_1 = Dot(n, r.dir);
		if (dot_1==0)
		{
			return false;
		}
		float dot_2 = Dot(n, r.o);
		t = -(dot_2 + d) / dot_1;
		if (t < 0)
		{
			return false;
		}
		return true;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool IntersectTest(const Ray& r, float& t, float& u, float& v) const
	{
		float dot_1 = Dot(n, r.dir);
		if (dot_1 == 0)
		{
			return false;
		}
		float dot_2 = Dot(n, r.o);
		t = (d - dot_2) / dot_1;
		if (t < 0)
		{
			return false;
		}
		Vec3 p = r.at(t);
		float invArea = 0.5f / area;
		u = Dot(Cross(e1, (p - vertices[1].p)), n) * invArea;
		if (u < 0 || u > 1)
		{
			return false;
		}
		v = Dot(Cross(e2, (p - vertices[2].p)), n) * invArea;
		if (v < 0 || u + v > 1)
		{
			return false;
		}
		return true;
	}
	bool MollerTrumbore(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 v0 = vertices[0].p;
		Vec3 v1 = vertices[1].p;
		Vec3 v2 = vertices[2].p;
		Vec3 edge1 = v1 - v0;
		Vec3 edge2 = v2 - v0;
		Vec3 T = r.o - v0;
		float det = Dot(edge1, Cross(edge2, -r.dir));
		if (det == 0) {
			return false;
		}
		float invdet = 1 / det;
		u = Dot(T, Cross(edge2, -r.dir)) * invdet;
		if (u < 0 || u > 1) {
			return false;
		}
		v = Dot(edge1, Cross(T, -r.dir)) * invdet;
		if (v < 0 || u + v > 1) {
			return false;
		}
		t = Dot(edge1, Cross(edge2, T)) * invdet;
		if (t < 0) {
			return false;
		}
		return true;
	}
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		//return IntersectTest(r, t, u, v);
		return MollerTrumbore(r, t, u, v);
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		float sqrt_r1 = sqrtf(r1);
		float alpha = 1.0f - sqrt_r1;
		float beta = r2 * sqrt_r1;
		float gamma = 1.0f - alpha - beta;
		if (area > 0) {
			pdf = 1.0f / area;
		}
		else
		{
			pdf = 0;
		}
		Vec3 pos = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
		return pos;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		t = std::min(tentry, texit);
		return (tentry <= texit && texit > 0);
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		return (tentry <= texit && texit > 0);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 l = r.o - centre;
		float b = Dot(l, r.dir);
		float c = Dot(l, l) - radius * radius;
		float dis = b * b - 4 * c;
		if (dis < 0) {
			return false;
		}
		float sq_dis = sqrtf(dis);
		float t0 = -(b + sq_dis) / 2;
		if (t0 > 0) {
			t = t0;
			return true;
		}
		float t1 = -(b - sq_dis) / 2;
		if (t1 > 0) {
			t = t1;
			return true;
		}
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset;
	unsigned int num;

	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// Add BVH building code here
		outputTriangles = inputTriangles;
		//buildBVH(outputTriangles, 0, outputTriangles.size());
		SAHBuildBVH(outputTriangles, 0, outputTriangles.size());
	}
	void buildBVH(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end)
	{
		if (begin >= end) {
			return;
		}
		offset = begin;
		num = end - begin;
		computeBounds(triangles, begin, end);
		if (num <= MAXNODE_TRIANGLES)
		{
			return;
		}
		Vec3 extent = bounds.max - bounds.min;
		int axis = getAxis(extent);
		float split = getSplit(bounds, extent);
		unsigned int mid = begin;
		std::vector<Triangle> tris;
		tris.reserve(end - begin);
		for (unsigned int i = begin; i < end; i++)
		{
			float value = 0;
			if (axis == 0) 
			{
				value = triangles[i].centre().x;
			}
			else if (axis == 1) 
			{
				value = triangles[i].centre().y;
			}
			else 
			{
				value = triangles[i].centre().z;
			} 
			if (value < split)
			{
				tris.push_back(triangles[i]);
				mid++;
			}
		}
		for (unsigned int i = begin; i < end; i++)
		{
			float value = 0;
			if (axis == 0)
			{
				value = triangles[i].centre().x;
			}
			else if (axis == 1)
			{
				value = triangles[i].centre().y;
			}
			else
			{
				value = triangles[i].centre().z;
			}
			if (value >= split)
			{
				tris.push_back(triangles[i]);
			}
		}
		if (mid == begin || mid == end)
		{
			mid = (begin + end) / 2;
			MedianSplit(triangles, begin, end, axis, mid);
		}
		for (unsigned int i = 0; i < tris.size(); i++)
		{
			triangles[begin + i] = tris[i];
		}
		l = new BVHNode();
		l->buildBVH(triangles, begin, mid);
		r = new BVHNode();
		r->buildBVH(triangles, mid, end);
		num = 0;
	}
	void computeBounds(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end)
	{
		bounds.reset();
		for (unsigned int i = begin; i < end; i++)
		{
			bounds.extend(triangles[i].vertices[0].p);
			bounds.extend(triangles[i].vertices[1].p);
			bounds.extend(triangles[i].vertices[2].p);
		}
	}
	void computeBounds(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end, AABB& outputbounds)
	{
		outputbounds.reset();
		for (unsigned int i = begin; i < end; i++)
		{
			outputbounds.extend(triangles[i].vertices[0].p);
			outputbounds.extend(triangles[i].vertices[1].p);
			outputbounds.extend(triangles[i].vertices[2].p);
		}
	}
	float getSplit(const AABB& bounds, const Vec3& v) {
		if (v.x > v.y && v.x > v.z)
		{
			return bounds.min.x + v.x / 2;
		}
		else if (v.y > v.x && v.y > v.z)
		{
			return bounds.min.y + v.y / 2;
		}
		else
		{
			return bounds.min.z + v.z / 2;
		}
	}
	int getAxis(const Vec3& v) {
		if (v.x > v.y && v.x > v.z)
		{
			return 0;
		}
		else if (v.y > v.x && v.y > v.z)
		{
			return 1;
		}
		else
		{
			return 2;
		}
	}
	float getAxisValue(const Vec3& v, int axis)
	{
		if (axis == 0) return v.x;
		if (axis == 1) return v.y;
		return v.z;
	}

	void trianglesCenterBounds(const std::vector<Triangle>& triangles, unsigned int begin, unsigned int end, AABB& outputbounds)
	{
		outputbounds.reset();
		for (unsigned int i = begin; i < end; i++)
		{
			outputbounds.extend(triangles[i].centre());
		}
	}
	bool SAH(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end,int& axis, float& split) 
	{
		num = end - begin;
		if (num <= MAXNODE_TRIANGLES) 
		{
			return false;
		}
		computeBounds(triangles, begin, end);
		float pa = bounds.area();
		AABB tcBounds;
		trianglesCenterBounds(triangles, begin, end, tcBounds);
		float minCost = FLT_MAX;
		int bestAxis = -1;
		int bestBin = -1;
		float leafCost = TRIANGLE_COST * num;
		for (int axis_ = 0; axis_ < 3; axis_++)
		{
			float aMin = getAxisValue(tcBounds.min, axis_);
			float aMax = getAxisValue(tcBounds.max, axis_);
			float aExtent = aMax - aMin;
			if (aExtent <= 0) {
				continue;
			}			
			AABB binBounds[BUILD_BINS];
			unsigned int binCount[BUILD_BINS];
			for (int i = 0; i < BUILD_BINS; i++)
			{
				binBounds[i].reset();
				binCount[i] = 0;
			}
			for (unsigned int i = begin; i < end; i++)
			{
				float c = getAxisValue(triangles[i].centre(), axis_);

				int binIndex = (int)(((c - aMin) / aExtent) * BUILD_BINS);
				if (binIndex < 0) {
					binIndex = 0;
				}
				if (binIndex >= BUILD_BINS) {
					binIndex = BUILD_BINS - 1;
				}
				binCount[binIndex]++;
				binBounds[binIndex].extend(triangles[i].vertices[0].p);
				binBounds[binIndex].extend(triangles[i].vertices[1].p);
				binBounds[binIndex].extend(triangles[i].vertices[2].p);
			}

			for (int i = 0; i < BUILD_BINS - 1; i++)
			{
				AABB lb;
				AABB rb;
				unsigned int lnum = 0;
				unsigned int rnum = 0;
				lb.reset();
				rb.reset();	
				for (int j = 0; j <= i; j++)
				{
					if (binCount[j] > 0)
					{
						lb.extend(binBounds[j].min);
						lb.extend(binBounds[j].max);
						lnum += binCount[j];
					}
				}
				for (int j = i + 1; j < BUILD_BINS; j++)
				{
					if (binCount[j] > 0)
					{
						rb.extend(binBounds[j].min);
						rb.extend(binBounds[j].max);
						rnum += binCount[j];
					}
				}
				if (lnum == 0 || rnum == 0) {
					continue;
				}			
				float splitcost = 
					TRAVERSE_COST + (lb.area() / pa) * lnum * TRIANGLE_COST +(rb.area() / pa) * rnum * TRIANGLE_COST;
				if (splitcost < minCost)
				{
					minCost = splitcost;
					bestAxis = axis_;
					bestBin = i;
				}
			}
		}
		if (bestAxis == -1)
		{
			return false;
		}
		if (minCost >= leafCost) 
		{
			return false;
		}
		float aMin = getAxisValue(tcBounds.min, bestAxis);
		float aMax = getAxisValue(tcBounds.max, bestAxis);
		float aExtent = aMax - aMin;
		axis = bestAxis;
		split = aMin + aExtent * ((float)(bestBin + 1) / (float)BUILD_BINS);
		return true;
	}
	int SplitTriangles(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end, int axis, float split)
	{
		int mid = begin;
		for (int i = begin; i < end; i++)
		{
			float value = getAxisValue(triangles[i].centre(), axis);
			if (value < split)
			{
				std::swap(triangles[i], triangles[mid]);
				mid++;
			}
		}
		return mid;
	}
	void SAHBuildBVH(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end)
	{
		if (begin >= end) {
			return;
		}
		offset = begin;
		num = end - begin;
		computeBounds(triangles, begin, end);
		if (num <= MAXNODE_TRIANGLES) {
			return;
		}	
		int axis = 0;
		float split = 0;
		if (!SAH(triangles, begin, end, axis, split)) {
			return;
		}
		unsigned int mid = SplitTriangles(triangles, begin, end, axis, split);
		if (mid == begin || mid == end) {
			mid = (begin + end) / 2;
			MedianSplit(triangles, begin, end, axis, mid);
		}
		l = new BVHNode();
		l->SAHBuildBVH(triangles, begin, mid);
		r = new BVHNode();
		r->SAHBuildBVH(triangles, mid, end);
		num = 0;
	}
	void MedianSplit(std::vector<Triangle>& triangles, unsigned int begin, unsigned int end,int axis, unsigned int mid)
	{
		for (int i = begin; i <= mid; i++)
		{
			int minIndex = i;
			for (int j = i + 1; j < end; j++)
			{
				float a = getAxisValue(triangles[j].centre(), axis);
				float b = getAxisValue(triangles[minIndex].centre(), axis);
				if (a < b)
				{
					minIndex = j;
				}
			}
			std::swap(triangles[i], triangles[minIndex]);
		}
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		float t;
		if (!bounds.rayAABB(ray, t))
		{
			return;
		}
		if (intersection.t < t)
		{
			return;
		}
		if (l == NULL && r == NULL)
		{		
			for (int i = 0; i < num; i++)
			{
				float t;
				float u;
				float v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t && t>0)
					{
						intersection.t = t;
						intersection.ID = offset + i;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}
		if (l != NULL)
		{
			l->traverse(ray, triangles, intersection);
		}
		if (r != NULL)
		{
			r->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		float t;
		if (!bounds.rayAABB(ray, t))
		{
			return true;
		}
		if (maxT < t)
		{
			return true;
		}
		if (l == NULL && r == NULL)
		{		
			for (int i = 0; i < num; i++)
			{
				float t;
				float u;
				float v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					if (t < maxT && t>0)
					{
						return false;
					}
				}
			}
			return true;
		}
		if (l != NULL)
		{
			if (!l->traverseVisible(ray, triangles, maxT)) {
				return false;
			}
		}
		if (r != NULL)
		{
			if (!r->traverseVisible(ray, triangles, maxT)) {
				return false;
			}
		}
		return true;
	}
};
