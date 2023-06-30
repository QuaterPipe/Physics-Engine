#include "physics/Collision/Transform.hpp"
#include <iostream>

namespace physics
{
	Transform::Transform() noexcept
		: position(0, 0), centerOfRotation(0, 0)
	{
	}

	Transform::Transform(const Transform& transform) noexcept
		: position(transform.position), centerOfRotation(transform.centerOfRotation),
		scale(transform.scale), rotation(transform.rotation)
	{
	}


	Transform::~Transform() noexcept
	{
	}

	Transform& Transform::operator=(const Transform& transform) noexcept
	{
		position = transform.position;
		centerOfRotation = transform.centerOfRotation;
		scale = transform.scale;
		rotation = transform.rotation;
		return *this;
	}

	Transform& Transform::operator=(Transform&& transform) noexcept
	{
		position = transform.position;
		centerOfRotation = transform.centerOfRotation;
		scale = transform.scale;
		rotation = transform.rotation;
		return *this;
	}


	bool Transform::operator==(const Transform& other) const noexcept
	{
		return position == other.position && scale == other.scale && rotation == other.rotation;
	}

	bool Transform::operator!=(const Transform& other) const noexcept
	{
		return !(position == other.position && scale == other.scale && rotation == other.rotation);
	}

	Transform Transform::operator*(const Transform& other) const noexcept
	{
		Transform t;
		t.rotation = rotation * other.rotation;
		t.scale = scale * other.scale;
		t.position = position + other.position;
		return t;
	}

	
	f64 Transform::GetAngle() const noexcept
	{
		return geo::Vector2(1, 1).Angle(rotation * geo::Vector2(1, 1));
	}

	geo::Vector2 Transform::GetScaleAsVector() const noexcept
	{
		return scale * geo::Vector2(1, 1);
	}

	void Transform::Rotate(f64 theta) noexcept
	{
		rotation = rotation * geo::Matrix2(theta);
	}

	void Transform::SetAngle(f64 theta) noexcept
	{
		rotation = geo::Matrix2(theta);
	}

	void Transform::Scale(f64 xScale, f64 yScale) noexcept
	{
		scale[0][0] = xScale;
		scale[1][1] = yScale;
	}

	void Transform::Translate(geo::Vector2 offset) noexcept
	{
		position += offset;
	}

	geo::Matrix3 Transform::GetTransformationMatrix() const noexcept
	{
		geo::Matrix3 result(rotation * scale);
		result[0][2] = centerOfRotation.x - rotation[0][0] * centerOfRotation.x - rotation[0][1] * centerOfRotation.y;
		result[1][2] = centerOfRotation.y - rotation[1][0] * centerOfRotation.x - rotation[1][1] * centerOfRotation.y;
		result[0][2] += position.x;
		result[1][2] += position.y;
		return result;
	}

	void Transform::Integrate(f64 dt, const geo::Vector2& velocity, f64 angularVelocity, const geo::Vector2& force,
		f64 torque, u32 steps) noexcept
	{
		/*auto rhs = [&](f64 t, const geo::Vector& x) {
			geo::Vector output(4);
			output[0] = x[2];
			output[1] = x[3];
			output[2] = forcex;
			output[3] = force.y;
		};
		geo::Vector X(4);
		X[0] = position.x;
		X[1] = position.y;
		X[2] = velocity.x;
		X[3] = velocity.y;
		f64 t = 0;
		for (int stepNumber = 0; stepNumber <= steps; stepNumber++)
		{

			geo::Vector k1 = rhs( t, X );
			geo::Vector k2 = rhs( t + (dt / (f64)steps) / 2.0,  X + (dt / (f64)steps) / 2.0 * k1 );
			geo::Vector k3 = rhs( t + (dt / (f64)steps) / 2.0, X + (dt / (f64)steps) / 2.0 * k2 );
			geo::Vector k4 = rhs( t + (dt / (f64)steps), X + (dt / (f64)steps) * k3 );

			X += (dt / (f64)steps) / 6.0 * ( k1 + 2.0 * k2 + 2.0 * k3 + k4 );
			t += (dt / (f64)steps);
		}*/
	}

	geo::Vector2 Transform::TransformVector(const geo::Vector2& v) const noexcept
	{
		geo::Vector3 tmp(v.x, v.y, 1);
		tmp = GetTransformationMatrix() * tmp;
		return geo::Vector2(tmp.x, tmp.y);
	}
}