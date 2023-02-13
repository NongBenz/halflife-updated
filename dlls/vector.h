/***
*
*	Copyright (c) 1996-2002, Valve LLC. All rights reserved.
*	
*	This product contains software technology licensed from Id 
*	Software, Inc. ("Id Technology").  Id Technology (c) 1996 Id Software, Inc. 
*	All Rights Reserved.
*
*   Use, distribution, and modification of this source code and/or resulting
*   object code is restricted to non-commercial enhancements to products from
*   Valve LLC.  All other use, distribution, or modification is prohibited
*   without written permission from Valve LLC.
*
****/

#pragma once

#include <cmath>

constexpr float kFloatEpsilon = 0.01f;
constexpr float kFloatEqualEpsilon = 0.001f;
constexpr float kFloatCmpEpsilon = 1.192092896e-07f;
constexpr float kMathPi = 3.141592653589793f;
constexpr float kDegreeToRadians = kMathPi / 180.0f;
constexpr float kRadiansToDegree = 180.0f / kMathPi;

template <typename T>
constexpr T clamp(const T& x, const T& a, const T& b)
{
	return (std::min)((std::max)(x, a), b);
}

static inline bool fzero(const float e)
{
	return abs(e) < kFloatEpsilon;
}

static inline bool fequal(const float a, const float b)
{
	return abs(a - b) < kFloatEqualEpsilon;
}

constexpr float rad2deg(const float r)
{
	return r * kRadiansToDegree;
}

constexpr float deg2rad(const float d)
{
	return d * kDegreeToRadians;
}

constexpr float modAngles(const float a)
{
	return 360.0f / 65536.0f * (static_cast<int>(a * (65536.0f / 360.0f)) & 65535);
}

constexpr float normalizeAngles(const float a)
{
	return 360.0f / 65536.0f * (static_cast<int>((a + 180.0f) * (65536.0f / 360.0f)) & 65535) - 180.0f;
}

constexpr float anglesDifference(const float a, const float b)
{
	return normalizeAngles(a - b);
}


//=========================================================
// 2DVector - used for many pathfinding and many other
// operations that are treated as planar rather than 3d.
//=========================================================
class Vector2D
{
public:
	constexpr Vector2D() = default;
	constexpr Vector2D(const Vector2D&) = default;
	constexpr Vector2D& operator=(const Vector2D&) = default;

	constexpr Vector2D(float X, float Y)
		: x(X), y(Y)
	{
	}

	[[nodiscard]] constexpr Vector2D operator+(const Vector2D& v) const { return Vector2D(x + v.x, y + v.y); }
	[[nodiscard]] constexpr Vector2D operator-(const Vector2D& v) const { return Vector2D(x - v.x, y - v.y); }
	[[nodiscard]] constexpr Vector2D operator*(float fl) const { return Vector2D(x * fl, y * fl); }
	[[nodiscard]] constexpr Vector2D operator/(float fl) const { return Vector2D(x / fl, y / fl); }

	[[nodiscard]] float Length() const { return static_cast<float>(sqrt(x * x + y * y)); }

	[[nodiscard]] Vector2D Normalize() const
	{
		float flLen = Length();
		if (flLen == 0)
		{
			return Vector2D(0, 0);
		}
		else
		{
			flLen = 1 / flLen;
			return Vector2D(x * flLen, y * flLen);
		}
	}

	vec_t x = 0, y = 0;
};

[[nodiscard]] constexpr float DotProduct(const Vector2D& a, const Vector2D& b)
{
	return (a.x * b.x + a.y * b.y);
}

[[nodiscard]] constexpr Vector2D operator*(float fl, const Vector2D& v)
{
	return v * fl;
}

//=========================================================
// 3D Vector
//=========================================================
class Vector // same data-layout as engine's vec3_t,
{			 //		which is a vec_t[3]
public:
	// Construction/destruction
	constexpr Vector() = default;
	constexpr Vector(const Vector&) = default;
	constexpr Vector& operator=(const Vector&) = default;

	constexpr Vector(float X, float Y, float Z)
		: x(X), y(Y), z(Z)
	{
	}

	constexpr Vector(float rgfl[3])
		: x(rgfl[0]), y(rgfl[1]), z(rgfl[2])
	{
	}

	// Operators
	[[nodiscard]] constexpr Vector operator-() const { return Vector(-x, -y, -z); }
	[[nodiscard]] constexpr bool operator==(const Vector& v) const { return x == v.x && y == v.y && z == v.z; }
	[[nodiscard]] constexpr bool operator!=(const Vector& v) const { return !(*this == v); }
	[[nodiscard]] constexpr Vector operator+(const Vector& v) const { return Vector(x + v.x, y + v.y, z + v.z); }
	[[nodiscard]] constexpr Vector operator-(const Vector& v) const { return Vector(x - v.x, y - v.y, z - v.z); }
	[[nodiscard]] constexpr Vector operator+(const float fl) const { return Vector(x + fl, y + fl, z + fl); }
	[[nodiscard]] constexpr Vector operator-(const float fl) const { return Vector(x - fl, y - fl, z - fl); }
	[[nodiscard]] constexpr Vector operator*(const float fl) const { return Vector(x * fl, y * fl, z * fl); }
	[[nodiscard]] constexpr Vector operator/(const float fl) const { return Vector(x / fl, y / fl, z / fl); }

	// Methods
	constexpr void CopyToArray(float* rgfl) const { rgfl[0] = x, rgfl[1] = y, rgfl[2] = z; }

	[[nodiscard]] constexpr float LengthSquared() const { return x * x + y * y + z * z; }
	[[nodiscard]] float Length() const { return static_cast<float>(sqrt(LengthSquared())); }
	[[nodiscard]] constexpr operator float*() { return &x; }			 // Vectors will now automatically convert to float * when needed
	[[nodiscard]] constexpr operator const float*() const { return &x; } // Vectors will now automatically convert to float * when needed

	[[nodiscard]] Vector Normalize() const
	{
		float flLen = Length();
		if (flLen == 0)
			return Vector(0, 0, 1); // ????
		flLen = 1 / flLen;
		return Vector(x * flLen, y * flLen, z * flLen);
	}

	[[nodiscard]] constexpr Vector2D Make2D() const
	{
		return {x, y};
	}

	[[nodiscard]] float Length2D() const { return static_cast<float>(sqrt(x * x + y * y)); }








	const Vector& operator+=(const Vector& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;

		return *this;
	}

	const Vector& operator-=(const Vector& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;

		return *this;
	}

	const Vector& operator*=(const float rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;

		return *this;
	}

	const Vector& operator/=(const float rhs)
	{
		const auto inv = 1.f / (rhs + kFloatEqualEpsilon);

		x *= inv;
		y *= inv;
		z *= inv;

		return *this;
	}


	float distance(const Vector& rhs) const
	{
		return (*this - rhs).Length();
	}

	float distance2d(const Vector& rhs) const
	{
		return (*this - rhs).Length2D();
	}

	float distanceSq(const Vector& rhs) const
	{
		return (*this - rhs).LengthSquared();
	}

	Vector get2d() const
	{
		return {x, y, 0.0f};
	}

	Vector normalize() const
	{
#if defined(CR_HAS_SSE)
		return SimdVec3Wrap{x, y, z}.normalize();
#else
		auto len = Length() + kFloatCmpEpsilon;

		if (fzero(len))
		{
			return {0.0f, 0.0f, 1.0f};
		}
		len = 1.0f / len;
		return {x * len, y * len, z * len};
#endif
	}

	Vector normalize2d() const
	{
#if defined(CR_HAS_SSE)
		return SimdVec3Wrap{x, y}.normalize();
#else
		auto len = Length2D() + kFloatCmpEpsilon;

		if (fzero(len))
		{
			return {0.0f, 1.0f, 0.0f};
		}
		len = 1.0f / len;
		return {x * len, y * len, 0.0f};
#endif
	}

	bool empty() const
	{
		return fzero(x) && fzero(y) && fzero(z);
	}

	void clear()
	{
		x = y = z = 0.0f;
	}

	Vector clampAngles()
	{
		x = normalizeAngles(x);
		y = normalizeAngles(y);
		z = 0.0f;

		return *this;
	}

	float pitch() const
	{
		if (fzero(z))
		{
			return 0.0f;
		}
		return deg2rad(atan2f(z, Length2D()));
	}

	float yaw() const
	{
		if (fzero(x) && fzero(y))
		{
			return 0.0f;
		}
		return rad2deg(atan2f(y, x));
	}

	Vector angles() const
	{
		if (fzero(x) && fzero(y))
		{
			return {z > 0.0f ? 90.0f : 270.0f, 0.0, 0.0f};
		}
		return {rad2deg(atan2f(z, Length2D())), rad2deg(atan2f(y, x)), 0.0f};
	}

	void angleVectors(Vector* forward, Vector* right, Vector* upward) const
	{
#if defined(CR_HAS_SSE)
		static SimdVec3Wrap s, c;
		SimdVec3Wrap{x, y, z}.angleVectors(s, c);
#else
		static Vector s, c, r;

		r = {deg2rad(x), deg2rad(y), deg2rad(z)};

		s = {sinf(r.x), sinf(r.y), sinf(r.z)};
		c = {cosf(r.x), cosf(r.y), cosf(r.z)};
#endif

		if (forward)
		{
			*forward = {c.x * c.y, c.x * s.y, -s.x};
		}

		if (right)
		{
			*right = {-s.z * s.x * c.y + c.z * s.y, -s.z * s.x * s.y - c.z * c.y, -s.z * c.x};
		}

		if (upward)
		{
			*upward = {c.z * s.x * c.y + s.z * s.y, c.z * s.x * s.y - s.z * c.y, c.z * c.x};
		}
	}

	const Vector& forward()
	{
		static Vector s_fwd{};
		angleVectors(&s_fwd, nullptr, nullptr);

		return s_fwd;
	}

	const Vector& upward()
	{
		static Vector s_up{};
		angleVectors(nullptr, nullptr, &s_up);

		return s_up;
	}

	const Vector& right()
	{
		static Vector s_right{};
		angleVectors(nullptr, &s_right, nullptr);

		return s_right;
	}










	// Members
	vec_t x = 0, y = 0, z = 0;
};

[[nodiscard]] constexpr Vector operator*(float fl, const Vector& v)
{
	return v * fl;
}

[[nodiscard]] constexpr float DotProduct(const Vector& a, const Vector& b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

[[nodiscard]] constexpr Vector CrossProduct(const Vector& a, const Vector& b)
{
	return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
