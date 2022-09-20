#include "../../include/physics/Collision/Transform.hpp"

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

	
	f64 Transform::GetAngle() const noexcept
	{
		return geo::Vector2(1, 1).Angle(rotation * geo::Vector2(1, 1));
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
		scale.a = xScale;
		scale.d = yScale;
	}

	void Transform::Translate(geo::Vector2 offset) noexcept
	{
		position += offset;
	}

	geo::Matrix3 Transform::GetTransformationMatrix() const noexcept
	{
		geo::Matrix3 result(rotation * scale);
		result.c = position.x;
		result.f = position.y;
		return result;
	}

	void Transform::Integrate(f64 dt, const geo::Vector2& velocity, f64 angularVelocity, const geo::Vector2& force,
		f64 torque, u32 steps) noexcept
	{
		/*auto rhs = [&](f64 t, const geo::Vector& x) {
			geo::Vector output(4);
			output[0] = x[2];
			output[1] = x[3];
			output[2] = force.x;
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

	// rk4 integration from: https://scicomp.stackexchange.com/questions/19020/applying-the-runge-kutta-method-to-second-order-odes
	namespace rk4
	{
		geo::Vector2 Acceleration(const State& state, const f64& t)
		{
			const f64 k = 15, b = 0.1;
			return -k * state.pos - b * state.vel;
		}

		Derivative Evaluate(const State& initial, const f64& t, const f64& dt, const Derivative& d)
		{
			State state;
			state.pos = initial.pos * d.dpos * dt;
			state.orient = initial.orient * d.dorient * dt;
 			state.vel = initial.vel * d.dvel * dt;
 			state.angVel = initial.angVel * d.dangVel * dt;
			Derivative output;
			output.dpos = state.vel;
			output.dorient = state.orient;
			output.dvel = Acceleration(state, t + dt);
			// acceleration for scalars
			output.dangVel = (-15. * acos(state.orient.a) - 0.1 * state.angVel);
			return output;
		}

		State Integrate(const State& state, f64 t, f64 dt)
		{
			Derivative a, b, c, d;
			a = Evaluate( state, t, 0.0f, Derivative() );
	        b = Evaluate( state, t, dt*0.5f, a );
	        c = Evaluate( state, t, dt*0.5f, b );
	        d = Evaluate( state, t, dt, c );
	        geo::Vector2 dxdt = 1.0f / 6.0f * 
	            ( a.dpos + 2.0f * ( b.dpos + c.dpos ) + d.dpos );
	        
	        geo::Vector2 dvdt = 1.0f / 6.0f * 
	            ( a.dvel + 2.0f * ( b.dvel + c.dvel ) + d.dvel );
			State result;
	        result.pos = state.pos + dxdt * dt;
	        result.vel = state.vel + dvdt * dt;
			return result;
		}
	}
}