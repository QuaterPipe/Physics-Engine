#include "physics/Collision/Collision.hpp"

namespace physics
{
	BoxCollider::BoxCollider() noexcept
	: pos(0, 0), dimensions(1, 1)
	{
	}

	BoxCollider::BoxCollider(const f64& width, const f64& height) noexcept
	: pos(0, 0), dimensions(width, height)
	{
	}

	BoxCollider::BoxCollider(const geo::Vector2& pos, const geo::Vector2& dimensions) noexcept
	: pos(pos), dimensions(dimensions.Abs())
	{
	}

	BoxCollider::BoxCollider(const BoxCollider& b) noexcept
	: pos(b.pos), dimensions(b.dimensions.Abs())
	{
	}

	BoxCollider::~BoxCollider() noexcept
	{
	}

	bool BoxCollider::operator==(const Collider& b) const noexcept
	{
		if (typeid(b).name() != typeid(*this).name())
			return false;
		return pos == dynamic_cast<const BoxCollider&>(b).pos && dimensions == dynamic_cast<const BoxCollider&>(b).dimensions;
	}

	bool BoxCollider::operator!=(const Collider& b) const noexcept
	{
		if (typeid(b).name() != typeid(*this).name())
			return true;
		return pos != dynamic_cast<const BoxCollider&>(b).pos || dimensions != dynamic_cast<const BoxCollider&>(b).dimensions;
	}

	BoxCollider BoxCollider::BoundingBox(const Transform& t) const noexcept
	{
		PolygonCollider p(geo::Vector2(0, 0), pos, geo::Vector2(x + width, y), pos + dimensions, {geo::Vector2(x, y + height)});
		return p.BoundingBox(t);
	}

	Collider* BoxCollider::Clone() const noexcept
	{
		return new BoxCollider(*this);
	}

	BoxCollider& BoxCollider::operator=(const BoxCollider& b)
	{
		if (*this != b)
		{
			pos = geo::Vector2(b.pos);
			dimensions = geo::Vector2(b.dimensions);
			x = pos.x;
			y = pos.y;
			width = dimensions.x;
			height = dimensions.y;
		}
		return *this;
	}
	
	geo::Vector2 BoxCollider::GetCenter() const noexcept
	{
		return pos + dimensions / 2;
	}

	geo::Vector2 BoxCollider::Max() const noexcept
	{
		return pos + dimensions;
	}

	geo::Vector2 BoxCollider::Min() const noexcept
	{
		return pos;
	}

	bool BoxCollider::Overlaps(const BoxCollider& b) const noexcept
	{
		auto numInRange = [&] (double value, double minVal, double maxVal){
			return (value >= minVal) && (value <= maxVal);
		};
		bool xOverlaps = numInRange(x, b.x, b.x + b.width) ||
			numInRange(b.x, x, x + width);
		bool yOverlaps = numInRange(y, b.y, b.y + b.height) ||
			numInRange(b.y, y, y + height);
		return xOverlaps && yOverlaps;
	}

	std::vector<geo::Vector2> BoxCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector2> v;
		v.push_back(t.TransformVector(pos));
		v.push_back(t.TransformVector(geo::Vector2(x + width, y)));
		v.push_back(t.TransformVector(pos + dimensions));
		v.push_back(t.TransformVector(geo::Vector2(x, y + height)));
		return v;
	}

}