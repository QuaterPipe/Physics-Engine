#include "../include/physics/Collision/Collision.hpp"

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
			this->colliders.push_back(cldr->Clone());
		}
	}

	MeshCollider::~MeshCollider() noexcept {}

	geometry::Vector getCentroid(std::vector<geometry::Vector> points)
	{
		if (points.size())
		{
			geometry::Vector first = points.at(0);
			geometry::Vector last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geometry::Vector p1, p2;
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
			return geometry::Vector(x / f + first.x, y / f + first.y);
		}
		else
			return geometry::Vector::Origin;
	}

	geometry::Vector MeshCollider::GetCenter() const noexcept
	{
		std::vector<geometry::Vector> coms;
		for (Collider* c : colliders)
		{
			coms.push_back(c->GetCenter());
		}
		return getCentroid(coms);
	}

	Collider* MeshCollider::Clone() const
	{
		return (Collider*)new MeshCollider(*this);
	}

	geometry::Vector MeshCollider::Max() const noexcept
	{
		std::vector<geometry::Vector> maxes;
		for (auto& c : this->colliders)
		{
			maxes.push_back(c->Max());
		}
		return *std::max(maxes.begin(), maxes.end());
	}

	geometry::Vector MeshCollider::Min() const noexcept
	{
		std::vector<geometry::Vector> mins;
		for (auto& c : this->colliders)
		{
			mins.push_back(c->Min());
		}
		return *std::min(mins.begin(), mins.end());
	}

	/*Serializable* MeshCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		MeshCollider
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
		std::vector<unsigned char> bytes = Archive::ReadBytes((reader)&pos.x, sizeof(pos.x));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&pos.y, sizeof(pos.y));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&dimensions.x, sizeof(dimensions.x));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&dimensions.y, sizeof(dimensions.y));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		return vec;
	}*/
}