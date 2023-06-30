#include "physics/Collision/Collision.hpp"

namespace physics
{
	MeshCollider::MeshCollider() noexcept
	{
	}

	MeshCollider::MeshCollider(const std::vector<Collider*>& colliders) noexcept
	{
		this->colliders = colliders;
	}

	MeshCollider::MeshCollider(const MeshCollider& c) noexcept
	{
		for (Collider* cldr : c.colliders)
		{
			if (cldr)
			{
				this->colliders.push_back((*cldr).Clone());
			}
		}
	}

	MeshCollider::~MeshCollider() noexcept
	{
	}

	bool MeshCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return false;
		if (colliders.size() != dynamic_cast<const MeshCollider&>(c).colliders.size())
			return false;
		return colliders == dynamic_cast<const MeshCollider&>(c).colliders;
	}

	bool MeshCollider::operator!=(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return true;
		if (colliders.size() != dynamic_cast<const MeshCollider&>(c).colliders.size())
			return true;
		return colliders != dynamic_cast<const MeshCollider&>(c).colliders;
	}

	BoxCollider MeshCollider::BoundingBox(const Transform& t) const noexcept
	{
		BoxCollider smallest;
		BoxCollider largest;
		bool reachedSmallest = false;
		bool reachedLargest = false;
		for (Collider* c: colliders)
		{
			if (!reachedSmallest && !reachedLargest)
			{
				smallest = c->BoundingBox(t);
				reachedSmallest = true;
				largest = c->BoundingBox(t);
				reachedLargest = true;
			}
			if (smallest.pos > c->BoundingBox(t).pos)
				smallest = c->BoundingBox(t);
			if (largest.pos < c->BoundingBox(t).pos)
				largest = c->BoundingBox(t);
		}
		BoxCollider result;
		result.pos = smallest.pos;
		result.x = largest.x - smallest.x + largest.width;
		result.y = largest.y - smallest.y + largest.height;
		return result;
	}

	geo::Vector2 getCentroid(std::vector<geo::Vector2> points)
	{
		if (points.size())
		{
			geo::Vector2 first = points.at(0);
			geo::Vector2 last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geo::Vector2 p1, p2;
			// absolutely no clue what this does, it just works lol
			for (size_t i = 0, j = points.size() - 1; i < points.size(); j=i++)
			{
				p1 = points[i]; p2 = points[j];
				f = (p1.y - first.y) * (p2.x - first.x) - (p2.y - first.y) * (p1.x - first.x);
				twiceArea += f;
				x += (p1.x + p2.x - 2 * first.x) * f;
				y += (p1.y + p2.y - 2 * first.y) * f;
			}
			f = twiceArea * 3;
			return geo::Vector2(x / f + first.x, y / f + first.y);
		}
		else
			return geo::Vector2::Origin;
	}

	geo::Vector2 MeshCollider::GetCenter() const noexcept
	{
		std::vector<geo::Vector2> coms;
		for (Collider* c : colliders)
		{
			coms.push_back(c->GetCenter());
		}
		return getCentroid(coms);
	}

	std::vector<geo::Vector2> MeshCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector2> v;
		for (Collider* c : colliders)
		{
			auto p = c->GetPoints(t);
			v.insert(v.end(), p.begin(), p.end());
		}
		return v;
	}

	Collider* MeshCollider::Clone() const noexcept
	{
		return (Collider*)new MeshCollider(*this);
	}

	geo::Vector2 MeshCollider::Max() const noexcept
	{
		std::vector<geo::Vector2> maxes;
		for (auto& c : this->colliders)
		{
			maxes.push_back(c->Max());
		}
		return *std::max(maxes.begin(), maxes.end());
	}

	geo::Vector2 MeshCollider::Min() const noexcept
	{
		std::vector<geo::Vector2> mins;
		for (auto& c : this->colliders)
		{
			mins.push_back(c->Min());
		}
		return *std::min(mins.begin(), mins.end());
	}
}