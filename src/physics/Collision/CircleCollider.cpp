#include "physics/Collision/Collision.hpp"

namespace physics
{
	CircleCollider::CircleCollider(Vector2 center, f64 radius) noexcept
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
		Vector2 c = t.TransformVector(center);
		return BoxCollider(c, Vector2(radius * 2, radius * 2));
	}

	bool CircleCollider::Contains(const Vector2& point, const Transform& t) const noexcept
	{
		return DistanceSquared(point, t.TransformVector(center)) <= SQRD(radius * physics::Max(t.GetScale().x, t.GetScale().y));
	}

	Collider* CircleCollider::Clone() const noexcept
	{
		return new CircleCollider(*this);
	}

	Vector2 CircleCollider::GetCenter() const noexcept
	{
		return center;
	}
	
	Vector2 CircleCollider::Max() const noexcept
	{
		return center + radius;
	}

	Vector2 CircleCollider::Min() const noexcept
	{
		return center - radius;
	}


	std::vector<Vector2> CircleCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<Vector2> v(1);
		v[0] = t.TransformVector(center);
		return v;
	}
	
}