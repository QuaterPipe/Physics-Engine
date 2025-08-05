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

	Collider* MeshCollider::Clone() const noexcept
	{
		return (Collider*)new MeshCollider(*this);
	}

	bool MeshCollider::Contains(const Vector2& point, const Transform& t) const noexcept
	{
		for (Collider* c : colliders)
		{
			if (c->Contains(point, t))
				return true;
		}
		return false;
	}

	f64 MeshCollider::CrossSectionalArea(const Vector2& direction) const noexcept
	{
		return BoundingBox().CrossSectionalArea(direction); // cannot compute exact CSA unfortunately
	}

	Vector2 MeshCollider::GetCenter() const noexcept
	{
		std::vector<Vector2> coms;
		for (Collider* c : colliders)
		{
			coms.push_back(c->GetCenter());
		}
		return Centroid(coms);
	}

	std::vector<Vector2> MeshCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<Vector2> v;
		for (Collider* c : colliders)
		{
			auto p = c->GetPoints(t);
			v.insert(v.end(), p.begin(), p.end());
		}
		return v;
	}

	Vector2 MeshCollider::Max() const noexcept
	{
		std::vector<Vector2> maxes;
		for (auto& c : this->colliders)
		{
			maxes.push_back(c->Max());
		}
		return *std::max(maxes.begin(), maxes.end());
	}

	Vector2 MeshCollider::Min() const noexcept
	{
		std::vector<Vector2> mins;
		for (auto& c : this->colliders)
		{
			mins.push_back(c->Min());
		}
		return *std::min(mins.begin(), mins.end());
	}
}