#include "physics/Collision/Transform.hpp"
#include <iostream>

namespace physics
{
	Transform::Transform() noexcept
		: _position(0, 0), _centerOfMass(0, 0)
	{
	}

	Transform::Transform(const Transform& transform) noexcept
		: _position(transform._position), _centerOfMass(transform._centerOfMass),
		_scale(transform._scale), _rotation(transform._rotation), _transformationMatrix(transform.GetTransformationMatrix()), _rotxScale(_rotation * _scale)
	{
	}

	Transform::Transform(const geo::Vector2& position, const geo::Vector2& com, const geo::Vector2& scale, const geo::Matrix2& rotation) noexcept
		: _position(position), _centerOfMass(com), _scale(scale.x, 0, 0, scale.y), _rotation(rotation), _transformationMatrix(_rotation * _scale), _rotxScale(_rotation * _scale)
	{
		_transformationMatrix(0, 2) = (_centerOfMass.x - _rotation(0, 0) * _centerOfMass.x - _rotation(0, 1) * _centerOfMass.y) + _position.x;
		_transformationMatrix(1, 2) = (_centerOfMass.y - _rotation(1, 0) * _centerOfMass.x - _rotation(1, 1) * _centerOfMass.y) + _position.y;
	}


	Transform::Transform(const geo::Vector2& position, const geo::Vector2& com, const geo::Vector2& scale, f64 rotation) noexcept
		: _position(position), _centerOfMass(com), _scale(scale.x, 0, 0, scale.y), _rotation(rotation), _transformationMatrix(_rotation * _scale), _rotxScale(_rotation * _scale)
	{
		_transformationMatrix(0, 2) = (_centerOfMass.x - _rotation(0, 0) * _centerOfMass.x - _rotation(0, 1) * _centerOfMass.y) + _position.x;
		_transformationMatrix(1, 2) = (_centerOfMass.y - _rotation(1, 0) * _centerOfMass.x - _rotation(1, 1) * _centerOfMass.y) + _position.y;
	}



	Transform::~Transform() noexcept
	{
	}

	Transform& Transform::operator=(const Transform& transform) noexcept
	{
		_position = transform._position;
		_centerOfMass = transform._centerOfMass;
		_scale = transform._scale;
		_rotation = transform._rotation;
		_transformationMatrix = transform.GetTransformationMatrix();
		_rotxScale = transform._rotxScale;
		return *this;
	}

	Transform& Transform::operator=(Transform&& transform) noexcept
	{
		_position = transform._position;
		_centerOfMass = transform._centerOfMass;
		_scale = transform._scale;
		_rotation = transform._rotation;
		_transformationMatrix = transform.GetTransformationMatrix();
		_rotxScale = transform._rotxScale;
		return *this;
	}


	bool Transform::operator==(const Transform& other) const noexcept
	{
		return _position == other._position && _centerOfMass == other._centerOfMass && _scale == other._scale && _rotation == other._rotation ;
	}

	bool Transform::operator!=(const Transform& other) const noexcept
	{
		return !(_position == other._position && _scale == other._scale && _rotation == other._rotation);
	}

	// currently broken
	Transform Transform::operator*(const Transform& other) const noexcept
	{
		return Transform(_position + other.GetPosition(), _centerOfMass + other.GetCOM(), GetScale() * other.GetScale(), _rotation * other.GetRotation());;
	}

	
	f64 Transform::GetAngle() const noexcept
	{
		return  _rotation.Angle();
	}

	geo::Vector2 Transform::GetCOM() const noexcept
	{
		return _centerOfMass;
	}

	Transform Transform::GetInverseTransform() const noexcept
	{
		return Transform(-_position, -_centerOfMass, geo::Vector2(1.0 / _scale(0, 0), 1.0 / _scale(1, 1)), -_rotation.Angle());
	}

	geo::Vector2 Transform::GetPosition() const noexcept
	{
		return _position;
	}

	geo::Matrix2 Transform::GetRotation() const noexcept
	{
		return _rotation;
	}

	geo::Vector2 Transform::GetScale() const noexcept
	{
		return geo::Vector2(_scale(0, 0), _scale(1, 1));
	}

	void Transform::Rotate(f64 theta) noexcept
	{
		_rotation = _rotation * geo::Matrix2(theta);
		_rotxScale = _rotation * _scale;
		_transformationMatrix(0, 0) = _rotxScale(0, 0);
		_transformationMatrix(0, 1) = _rotxScale(0, 1);
		_transformationMatrix(1, 0) = _rotxScale(1, 0);
		_transformationMatrix(1, 1) = _rotxScale(1, 1);
	}

	void Transform::SetAngle(f64 theta) noexcept
	{
		_rotation = geo::Matrix2(theta);
		_rotxScale = _rotation * _scale;
		_transformationMatrix(0, 0) = _rotxScale(0, 0);
		_transformationMatrix(0, 1) = _rotxScale(0, 1);
		_transformationMatrix(1, 0) = _rotxScale(1, 0);
		_transformationMatrix(1, 1) = _rotxScale(1, 1);
	}

	void Transform::SetCOM(const geo::Vector2& com) noexcept
	{
		_centerOfMass = com;
		_transformationMatrix(0, 2) = (_centerOfMass.x - _rotation(0, 0) * _centerOfMass.x - _rotation(0, 1) * _centerOfMass.y) + _position.x;
		_transformationMatrix(1, 2) = (_centerOfMass.y - _rotation(1, 0) * _centerOfMass.x - _rotation(1, 1) * _centerOfMass.y) + _position.y;
	}

	void Transform::SetCOM(f64 x, f64 y) noexcept
	{
		_centerOfMass.Set(x, y);
		_transformationMatrix(0, 2) = (_centerOfMass.x - _rotation(0, 0) * _centerOfMass.x - _rotation(0, 1) * _centerOfMass.y) + _position.x;
		_transformationMatrix(1, 2) = (_centerOfMass.y - _rotation(1, 0) * _centerOfMass.x - _rotation(1, 1) * _centerOfMass.y) + _position.y;
	}

	void Transform::SetPosition(const geo::Vector2& position) noexcept
	{
		_position = position;
		_transformationMatrix(0, 2) = (_centerOfMass.x - _rotation(0, 0) * _centerOfMass.x - _rotation(0, 1) * _centerOfMass.y) + _position.x;
		_transformationMatrix(1, 2) = (_centerOfMass.y - _rotation(1, 0) * _centerOfMass.x - _rotation(1, 1) * _centerOfMass.y) + _position.y;
	}

	void Transform::SetPosition(f64 x, f64 y) noexcept
	{
		_position.Set(x, y);
		_transformationMatrix(0, 2) = _centerOfMass.x - _rotation(0, 0) * _centerOfMass.x - _rotation(0, 1) * _centerOfMass.y + _position.x;
		_transformationMatrix(1, 2) = _centerOfMass.y - _rotation(1, 0) * _centerOfMass.x - _rotation(1, 1) * _centerOfMass.y + _position.y;
	}

	void Transform::SetRotation(const geo::Matrix2& rotation) noexcept
	{
		_rotation = rotation;
		_rotxScale = _rotation * _scale;
		_transformationMatrix(0, 0) = _rotxScale(0, 0);
		_transformationMatrix(0, 1) = _rotxScale(0, 1);
		_transformationMatrix(1, 0) = _rotxScale(1, 0);
		_transformationMatrix(1, 1) = _rotxScale(1, 1);
	}

	void Transform::SetScale(const geo::Vector2& scale) noexcept
	{
		_scale(0, 0) = scale.x;
		_scale(1, 1) = scale.y;
		_rotxScale = _rotation * _scale;
		_transformationMatrix(0, 0) = _rotxScale(0, 0);
		_transformationMatrix(0, 1) = _rotxScale(0, 1);
		_transformationMatrix(1, 0) = _rotxScale(1, 0);
		_transformationMatrix(1, 1) = _rotxScale(1, 1);
	}

	void Transform::SetScale(f64 xScale, f64 yScale) noexcept
	{
		_scale(0, 0) = xScale;
		_scale(1, 1) = yScale;
		_rotxScale = _rotation * _scale;
		_transformationMatrix(0, 0) = _rotxScale(0, 0);
		_transformationMatrix(0, 1) = _rotxScale(0, 1);
		_transformationMatrix(1, 0) = _rotxScale(1, 0);
		_transformationMatrix(1, 1) = _rotxScale(1, 1);
	}

	void Transform::Scale(f64 x_scale, f64 y_scale) noexcept
	{
		_scale(0, 0) = x_scale;
		_scale(1, 1) = y_scale;
		_rotxScale =_rotation * _scale;
		_transformationMatrix(0, 0) = _rotxScale(0, 0);
		_transformationMatrix(0, 1) = _rotxScale(0, 1);
		_transformationMatrix(1, 0) = _rotxScale(1, 0);
		_transformationMatrix(1, 1) = _rotxScale(1, 1);
	}

	void Transform::Translate(geo::Vector2 offset) noexcept
	{
		SetPosition(_position + offset);
	}

	geo::Matrix3 Transform::GetTransformationMatrix() const noexcept
	{
		return _transformationMatrix;
	}

	geo::Vector2 Transform::TransformVector(const geo::Vector2& v) const noexcept
	{
		geo::Vector2 result = _rotxScale * v;
		result.x += _transformationMatrix(0, 2);
		result.y += _transformationMatrix(1, 2);
		return result;
	}

	void Transform::SymplecticEulerIntegrate(f64* _position, f64* velocity, f64* acceleration, f64 dt) noexcept
	{
		*velocity += *acceleration * dt;
		*_position += *velocity * dt;
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
		f64 dt, f64 acceleration) noexcept
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

	void Transform::RK4Integrate(f64* _position, f64* velocity, f64* acceleration, f64 dt) noexcept
	{
		State s;
		s.x = *_position;
		s.v = *velocity;
		Integrate(s, dt, *acceleration);
		*_position = s.x;
		*velocity = s.v;
	}

	bool Transform::IsUnitTransform() const noexcept
	{
		return _position == geo::Vector2::Origin && _centerOfMass == geo::Vector2::Origin && !_rotation.Angle() &&
			_scale(0, 0) == 1 && _scale(1, 1) == 1 && !_scale(0, 1) && _scale(1, 0);
	}
}