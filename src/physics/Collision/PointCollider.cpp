#include "../../include/physics/Collision/PointCollider.hpp"

namespace physics
{
	PointCollider::PointCollider()
	: Collider()
	{
	}

	PointCollider::PointCollider(const f64& x, const f64& y)
	: Collider(), position(x, y)
	{
		
	}

	PointCollider::PointCollider(const geo::Vector2& pos)
	: Collider(), position(pos)
	{
	}

	PointCollider::PointCollider(const PointCollider& p)
	: Collider(), position(p.position)
	{
	}


	bool PointCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return false;
		return position == dynamic_cast<const PointCollider&>(c).position;
	}

	bool PointCollider::operator!=(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return true;
		return position != dynamic_cast<const PointCollider&>(c).position;
	}

	BoxCollider PointCollider::BoundingBox(const Transform& t) const noexcept
	{
		BoxCollider b;
		b.pos = t.TransformVector(position);
		b.dimensions.Set(1, 1);
		return b;
	}

	Collider* PointCollider::Clone() const noexcept
	{
		return new PointCollider(*this);
	}

	geo::Vector2 PointCollider::GetCenter() const noexcept
	{
		return position;
	}

	std::vector<geo::Vector2> PointCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector2> v;
		v.push_back(t.TransformVector(position));
		return v;
	}

	geo::Vector2 PointCollider::Max() const noexcept
	{
		return position;
	}

	geo::Vector2 PointCollider::Min() const noexcept
	{
		return position;
	}

}