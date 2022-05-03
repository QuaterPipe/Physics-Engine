#pragma once
#include "../../geometry/main.hpp"
#include "../Collision/Collision.hpp"
#include <ostream>
#include <vector>
#include <string>

namespace physics
{
	std::ostream& operator<<(std::ostream& os, const geo::Vector& v);
	std::ostream& operator<<(std::ostream& os, const geo::Line& l);
	std::ostream& operator<<(std::ostream& os, const Transform& t);
	std::ostream& operator<<(std::ostream& os, const PolygonCollider& p);
	std::ostream& operator<<(std::ostream& os, const geo::Matrix2& m);
	std::ostream& operator<<(std::ostream& os, const CollisionPoints& c);
	template <typename T>
	inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
	{
		os << "{";
		size_t index = 0;
		for (const T& var: v)
		{
			if (index == v.size() - 1)
			{
				os << var;
			}
			else
			{
				os << var << ", ";
			}
			index++;
		}
		return os << "}";
	}
}