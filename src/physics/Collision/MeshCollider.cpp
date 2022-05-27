#include "../../include/physics/Collision/Collision.hpp"

namespace physics
{
	using namespace serialization;
	MeshCollider::MeshCollider() noexcept
	{
		classCode = 0x05;
	}

	MeshCollider::MeshCollider(const std::vector<Collider*>& colliders) noexcept
	{
		classCode = 0x05;
		this->colliders = colliders;
	}

	MeshCollider::MeshCollider(const MeshCollider& c) noexcept
	{
		classCode = 0x05;
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

	BoxCollider MeshCollider::BoundingBox(const Transform& t) const noexcept
	{
		BoxCollider smallest;
		BoxCollider largest;
		bool reachedSmallest;
		bool reachedLargest;
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

	geo::Vector getCentroid(std::vector<geo::Vector> points)
	{
		if (points.size())
		{
			geo::Vector first = points.at(0);
			geo::Vector last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geo::Vector p1, p2;
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
			return geo::Vector(x / f + first.x, y / f + first.y);
		}
		else
			return geo::Vector::Origin;
	}

	geo::Vector MeshCollider::GetCenter() const noexcept
	{
		std::vector<geo::Vector> coms;
		for (Collider* c : colliders)
		{
			coms.push_back(c->GetCenter());
		}
		return getCentroid(coms);
	}

	std::vector<geo::Vector> MeshCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector> v;
		for (Collider* c : colliders)
		{
			auto p = c->GetPoints(t);
			v.insert(v.end(), p.begin(), p.end());
		}
		return v;
	}

	Collider* MeshCollider::Clone() const
	{
		return (Collider*)new MeshCollider(*this);
	}

	bool MeshCollider::Equals(const MeshCollider& other) const noexcept
	{
		if (other.colliders.size() != colliders.size())
			return false;
		bool equals = true;
		for (size_t i = 0; i < colliders.size(); i++)
			equals = equals && colliders[i]->Equals(*other.colliders[i]);
		return equals;
	}

	bool MeshCollider::NotEquals(const MeshCollider& other) const noexcept
	{
		return !operator==(other);
	}

	std::vector<unsigned char> MeshCollider::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	geo::Vector MeshCollider::Max() const noexcept
	{
		std::vector<geo::Vector> maxes;
		for (auto& c : this->colliders)
		{
			maxes.push_back(c->Max());
		}
		return *std::max(maxes.begin(), maxes.end());
	}

	geo::Vector MeshCollider::Min() const noexcept
	{
		std::vector<geo::Vector> mins;
		for (auto& c : this->colliders)
		{
			mins.push_back(c->Min());
		}
		return *std::min(mins.begin(), mins.end());
	}

	Serializable* MeshCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char MeshCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
	}

	unsigned long MeshCollider::TotalByteSize() const noexcept
	{
		return sizeof(*this);
	}

	std::vector<unsigned char> MeshCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}