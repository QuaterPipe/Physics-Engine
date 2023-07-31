#include "physics/Collision/Transform.hpp"
#include <iostream>

namespace physics
{
	Transform::Transform() noexcept
		: position(0, 0), centerOfMass(0, 0)
	{
	}

	Transform::Transform(const Transform& transform) noexcept
		: position(transform.position), centerOfMass(transform.centerOfMass),
		scale(transform.scale), rotation(transform.rotation)
	{
	}


	Transform::~Transform() noexcept
	{
	}

	Transform& Transform::operator=(const Transform& transform) noexcept
	{
		position = transform.position;
		centerOfMass = transform.centerOfMass;
		scale = transform.scale;
		rotation = transform.rotation;
		return *this;
	}

	Transform& Transform::operator=(Transform&& transform) noexcept
	{
		position = transform.position;
		centerOfMass = transform.centerOfMass;
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
		result[0][2] = centerOfMass.x - rotation[0][0] * centerOfMass.x - rotation[0][1] * centerOfMass.y;
		result[1][2] = centerOfMass.y - rotation[1][0] * centerOfMass.x - rotation[1][1] * centerOfMass.y;
		result[0][2] += position.x;
		result[1][2] += position.y;
		return result;
	}

	geo::Vector2 Transform::TransformVector(const geo::Vector2& v) const noexcept
	{
		geo::Vector3 tmp(v.x, v.y, 1);
		tmp = GetTransformationMatrix() * tmp;
		return geo::Vector2(tmp.x, tmp.y);
	}

	void Transform::SymplecticEulerIntegrate(f64* position, f64* velocity, f64* acceleration, f64 dt)
	{
		*velocity += *acceleration * dt;
		*position += *velocity * dt;
	}

	struct State
	{
		f64 x, v;
	};

	struct Derivative
	{
		f64 dx, dv;
	};
	
	Derivative Evaluate(State initial, f64 dt, Derivative d, f64 acceleration)
	{

		State state;
		state.x = initial.x + d.dx * dt;
		state.v = initial.v + d.dv * dt;

		Derivative output;
		output.dx = state.v;
		output.dv = acceleration;
		return output;
	}

	void Integrate(State& state,
		f64 dt, f64 acceleration)
	{
		Derivative a, b, c, d;

		a = Evaluate(state, 0.0, Derivative(), acceleration);
		b = Evaluate(state, dt * 0.5, a, acceleration);
		c = Evaluate(state, dt * 0.5, b, acceleration);
		d = Evaluate(state, dt, c, acceleration);

		float dxdt = 1.0 / 6.0 *
			(a.dx + 2.0 * (b.dx + c.dx) + d.dx);

		float dvdt = 1.0 / 6.0 *
			(a.dv + 2.0 * (b.dv + c.dv) + d.dv);

		state.x = state.x + dxdt * dt;
		state.v = state.v + dvdt * dt;
	}

	void Transform::RK4Integrate(f64* position, f64* velocity, f64* acceleration, f64 dt)
	{
		State s;
		s.x = *position;
		s.v = *velocity;
		Integrate(s, dt, *acceleration);
		*position = s.x;
		*velocity = s.v;
	}
}