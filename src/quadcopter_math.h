#ifndef QUADCOPTER_MATH_H
#define QUADCOPTER_MATH_H

#include <cerrno>
#include <cmath>

const float pi = 3.14159265359f;

struct Quatf;
struct Vec3f;

struct Vec3f
{
    float x;
    float y;
    float z;

    constexpr Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
	constexpr Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}

	const Vec3f& operator=(const Vec3f& v2)
	{
		x = v2.x;
		y = v2.y;
		z = v2.z;

		return *this;
	}

	constexpr Vec3f operator+(const Vec3f& v2)
	{
		return Vec3f(x + v2.x, y + v2.y, z + v2.z);
	}

	const Vec3f& operator+=(const Vec3f& v2)
	{
		x += v2.x;
		y += v2.y;
		z += v2.z;

		return *this; 
	}

	constexpr Vec3f operator-(const Vec3f& v2)
	{
		return Vec3f(x - v2.x, y - v2.y, z - v2.z);
	}

	const Vec3f& operator-=(const Vec3f& v2)
	{
		x -= v2.x;
		y -= v2.y;
		z -= v2.z;

		return *this; 
	}

	constexpr Vec3f operator*(float scale)
	{
		return Vec3f(x * scale, y * scale, z * scale);
	}

	const Vec3f& operator*=(float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;

		return *this; 
	}

	Vec3f operator*(const Quatf& q) const;
	const Vec3f& operator*=(const Quatf& q2);

	constexpr float len2() const
	{
		return x*x + y*y + z*z;
	}

	float len() const
	{
		return sqrtf(len2());
	}

	const Vec3f& normalize()
	{
		float length = len();

		if(length != 0.0f)
		{
			float inv_len = 1.0f / length;

			x *= inv_len;
			y *= inv_len;
			z *= inv_len;
		}

		return *this; 
	}

	Vec3f normalized() const
	{
		float length = len();

		if(length != 0.0f)
		{
			float inv_len = 1.0f / length;
			return Vec3f(x * inv_len, y * inv_len, z * inv_len);
		}
		else
		{
			return *this;
		}
	}

	constexpr Vec3f cross(const Vec3f& v2) const
	{
		return Vec3f(
			y * v2.z - z * v2.y,
			z * v2.x - x * v2.z,
			x * v2.y - y * v2.x
		);
	}

	constexpr float dot(const Vec3f& v2) const
	{
		return x * v2.x + y * v2.y + z * v2.z;
	}
};

struct Quatf
{
	float x;
	float y;
	float z;
	float w;

	constexpr Quatf() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}

	constexpr Quatf(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

	const Quatf& operator=(const Quatf& q2)
	{
		x = q2.x;
		y = q2.y;
		z = q2.z;
		w = q2.w;

		return *this;
	}

	constexpr Quatf conjugate() const
	{
		return Quatf(-x, -y, -z, w);
	}

	Quatf(const Vec3f& axis, float angle)
	{
		angle *= 0.5f;
		float s = sin(angle);
		
		x = axis.x * s;
		y = axis.y * s;
		z = axis.z * s;
		w = cos(angle);
	}

	Quatf operator*(const Quatf& q2) const
	{
		Quatf res;

		res.x = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
		res.y = w * q2.y + y * q2.w + z * q2.x - x * q2.z;
		res.z = w * q2.z + z * q2.w + x * q2.y - y * q2.x;
		res.w = w * q2.w - x * q2.x - y * q2.y - z * q2.z;

		return res;
	}
};

inline Vec3f Vec3f::operator*(const Quatf& q2) const
{
	Quatf vec(x, y, z, 0.0);
	Quatf result;

    result = vec * q2.conjugate();
	result = q2 * result;

	return Vec3f(result.x, result.y, result.z);
}

inline const Vec3f& Vec3f::operator*=(const Quatf& q2)
{
	Quatf vec(x, y, z, 0.0);
	Quatf result;

    result = vec * q2.conjugate();
	result = q2 * result;

	x = result.x;
	y = result.y;
	z = result.z;

	return *this;
}

#endif

