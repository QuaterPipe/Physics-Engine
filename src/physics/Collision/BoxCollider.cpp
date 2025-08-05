#include "physics/Collision/Collision.hpp"
#include <iostream>

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

	BoxCollider::BoxCollider(const Vector2& pos, const Vector2& dimensions) noexcept
	: pos(pos), dimensions(dimensions)
	{
	}

	BoxCollider::BoxCollider(const BoxCollider& b) noexcept
	: pos(b.pos), dimensions(b.dimensions)
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
		PolygonCollider p(*this);
		return p.BoundingBox(t);
	}

	Collider* BoxCollider::Clone() const noexcept
	{
		return new BoxCollider(*this);
	}

	bool BoxCollider::Contains(const Vector2& point, const Transform& t) const noexcept
	{
		Vector2 pt = t.GetInverseTransform().TransformVector(point);
		return x <= pt.x && pt.x <= x + width && y <= pt.y && pt.y <= y + height;
	}
	
	f64 BoxCollider::CrossSectionalArea(const Vector2& direction) const noexcept
	{
		Vector2 topl(x - width / 2, y + height / 2), bottoml(x - width / 2, y - height / 2);
		Vector2 topr(x + width / 2, y + height / 2), bottomr(x + width / 2, y - height / 2);
		Vector2 arr[4] = { topl, bottoml, topr, bottomr };
		Vector2 d(direction.Normalized());
		f64 minP = std::numeric_limits<f64>::infinity(), maxP = -std::numeric_limits<f64>::infinity();
		for (Vector2& v : arr)
		{
			f64 proj = v.Dot(d);
			if (proj < minP)
				minP = proj;
			if (proj > maxP)
				maxP = proj;
		}
		return maxP - minP;
	}


	BoxCollider& BoxCollider::operator=(const BoxCollider& b)
	{
		if (*this != b)
		{
			pos = Vector2(b.pos);
			dimensions = Vector2(b.dimensions);
			x = pos.x;
			y = pos.y;
			width = dimensions.x;
			height = dimensions.y;
		}
		return *this;
	}
	
	Vector2 BoxCollider::GetCenter() const noexcept
	{
		return pos;
	}

	Vector2 BoxCollider::Max() const noexcept
	{
		return pos + (dimensions * 0.5);
	}

	Vector2 BoxCollider::Min() const noexcept
	{
		return pos - (dimensions * 0.5);
	}

	inline bool numInRange(f64 value, f64 minVal, f64 maxVal)
	{
		return (value >= minVal) && (value <= maxVal);
	}

	bool BoxCollider::Overlaps(const BoxCollider& b) const noexcept
	{
		f64 bw = b.width * 0.5;
		f64 bh = b.width * 0.5;
		f64 w = width * 0.5;
		f64 h = height * 0.5;
		bool xOverlaps = numInRange(x - w, b.x - bw, b.x + bw) ||
			numInRange(b.x - bw, x - w, x + w);
		bool yOverlaps = numInRange(y - h, b.y - bh, b.y + bh) ||
			numInRange(b.y - bh, y - h, y + h);
		return xOverlaps && yOverlaps;
	}

	std::vector<Vector2> BoxCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<Vector2> v(4);
		v[0] = t.TransformVector(pos - (dimensions * 0.5));
		v[1] = t.TransformVector(Vector2(x + (width * 0.5), y - (height * 0.5)));
		v[2] = t.TransformVector(pos + (dimensions * 0.5));
		v[3] = t.TransformVector(Vector2(x - (width * 0.5), y + (height * 0.5)));
		return v;
	}

}