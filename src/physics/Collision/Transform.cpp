#include "../../include/physics/Collision/Transform.hpp"

namespace physics
{
	using namespace serialization;
	Transform::Transform() noexcept
	: Serializable(), Hashable()
	{
		position = geo::Vector(0, 0);
		centerOfRotation = geo::Vector(0, 0);
		scale = geo::Matrix2();
		rotation = geo::Matrix2();
	}

	Transform::~Transform() noexcept
	{
		
	}

	bool Transform::operator==(const Transform& other) const noexcept
	{
		return position == other.position && scale == other.scale && rotation == other.rotation;
	}

	bool Transform::operator!=(const Transform& other) const noexcept
	{
		return !(position == other.position && scale == other.scale && rotation == other.rotation);
	}

	geo::Matrix3 Transform::GetTransformationMatrix() const noexcept
	{
		geo::Matrix3 result(rotation * scale);
		result.c = position.x;
		result.f = position.y;
		return result;
	}

	geo::Vector Transform::TransformVector(const geo::Vector& v) const noexcept
	{
		geo::Vector3 tmp(v.x, v.y, 1);
		tmp = GetTransformationMatrix() * tmp;
		return geo::Vector(tmp.x, tmp.y);
	}

	Serializable* Transform::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	std::vector<unsigned char> Transform::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	unsigned char Transform::GetByte(const size_t& index) const
	{
		return 0x01;
	}

	unsigned long Transform::TotalByteSize() const noexcept
	{
		return 0UL;
	}

	std::vector<unsigned char> Transform::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
	// rk4 integration from: https://scicomp.stackexchange.com/questions/19020/applying-the-runge-kutta-method-to-second-order-odes
	namespace rk4
	{
		geo::Vector Acceleration(const State& state, const f64& t)
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
	        geo::Vector dxdt = 1.0f / 6.0f * 
	            ( a.dpos + 2.0f * ( b.dpos + c.dpos ) + d.dpos );
	        
	        geo::Vector dvdt = 1.0f / 6.0f * 
	            ( a.dvel + 2.0f * ( b.dvel + c.dvel ) + d.dvel );
			State result;
	        result.pos = state.pos + dxdt * dt;
	        result.vel = state.vel + dvdt * dt;
			return result;
		}
	}
}