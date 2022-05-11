#include "../include/geometry/Math.hpp"
#include "../include/geometry/Line.hpp"
#include <iostream>

namespace geo
{
	Line::Line() noexcept
	{
	}

	Line::Line(const Vector& a, const Vector& b) noexcept
	{
		this->a = a;
		this->b = b;
		this->_length = Distance(this->a, this->b);
		this->_angle = GetAngle(this->a, this->b);
	}

	Line::Line(const Line& l) noexcept
	{
		this->a = l.a();
		this->b = l.b();
		this->_length = Distance(this->a, this->b);
		this->_angle = GetAngle(this->a, this->b);
	}

	Line::~Line() noexcept
	{
	}

	bool Line::operator==(const Line& l) const noexcept
	{
		return (this->a == l.a() && this->b == l.b());
	}

	bool Line::operator!=(const Line& l) const noexcept
	{
		return !(this->a == l.a() && this->b == l.b());
	}

	f64 Line::angle() const noexcept
	{
		return _angle;
	}

	Vector Line::GetVectorAlongLine(const f64& distance, const bool& startFromA) const noexcept
	{
		Vector begin = startFromA ? this->b - this->a : this->a - this->b;
		begin.Normalize();
		return begin * distance + (!startFromA ? this->b : this->a);
	}

	Line Line::GetPerpendicular() const noexcept
	{
		double xdif = b.x - a.x;
		double ydif = b.y - a.y;
		return Line(Vector(b.x - ydif / 2, b.y + xdif / 2), Vector(b.x + ydif / 2, b.y - xdif / 2));
	}

	f64 Line::length() const noexcept
	{
		return this->_length;
	}

	void Line::Move(const f64& offsetX, const f64& offsetY) noexcept
	{
		this->a.Move(offsetX, offsetY);
		this->b.Move(offsetX, offsetY);
	}

	std::string Line::ToString() const noexcept
	{
		return "(" + this->a.ToString() + ", " + this->b.ToString() + ")";
	}

	std::tuple<std::tuple<f64, f64>, std::tuple<f64, f64>> Line::ToTuple() const noexcept
	{
		return std::tuple<std::tuple<f64, f64>, std::tuple<f64, f64>>(this->a.ToTuple(), this->b.ToTuple());
	}

	void Line::Rotate(const Vector& pivot, const f64& angle) noexcept
	{
		this->a.Rotate(pivot, angle);
		this->b.Rotate(pivot, angle);
		Update();
	}

	void Line::Update() noexcept
	{
		this->_length = Distance(this->a, this->b);
		this->_angle = GetAngle(this->a, this->b);
	}

	bool Line::VectorIsOnLine(const Vector& v) const noexcept
	{
		return Distance(a, v) + Distance(v, b) == Distance(a, b);
	}
}