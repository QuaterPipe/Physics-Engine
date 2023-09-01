#include "physics/Collision/Collision.hpp"

namespace physics
{
	CircleCollider::CircleCollider(geo::Vector2 center, f64 radius) noexcept
	: center(center), radius(fabs(radius))
	{
	}

	CircleCollider::CircleCollider(f64 radius) noexcept
	: center(), radius(fabs(radius))
	{
	}

	CircleCollider::CircleCollider() noexcept
	{
	}

	CircleCollider::CircleCollider(const CircleCollider& c) noexcept
	: center(c.center), radius(fabs(c.radius))
	{
	}

	CircleCollider::~CircleCollider() noexcept {}

	
	bool CircleCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() == typeid(*this).name())
			return false;
		return dynamic_cast<const CircleCollider&>(c).radius == radius && dynamic_cast<const CircleCollider&>(c).center == center;
	}

	bool CircleCollider::operator!=(const Collider& c) const noexcept
	{
		if (typeid(c).name() == typeid(*this).name())
			return true;
		return dynamic_cast<const CircleCollider&>(c).radius != radius || dynamic_cast<const CircleCollider&>(c).center != center;
	}

	BoxCollider CircleCollider::BoundingBox(const Transform& t) const noexcept
	{
		geo::Vector2 c = t.TransformVector(center);
		return BoxCollider(c - radius, geo::Vector2(radius * 2, radius * 2));
	}

	bool CircleCollider::Contains(const geo::Vector2& point, const Transform& t) const noexcept
	{
		return geo::DistanceSquared(point, t.TransformVector(center)) <= SQRD(radius * geo::Max(t.GetScale().x, t.GetScale().y));
	}

	Collider* CircleCollider::Clone() const noexcept
	{
		return new CircleCollider(*this);
	}

	geo::Vector2 CircleCollider::GetCenter() const noexcept
	{
		return center;
	}
	
	geo::Vector2 CircleCollider::Max() const noexcept
	{
		return center + radius;
	}

	geo::Vector2 CircleCollider::Min() const noexcept
	{
		return center - radius;
	}


	std::vector<geo::Vector2> CircleCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector2> v;
		v.push_back(t.TransformVector(center));
		return v;
	}
	
}