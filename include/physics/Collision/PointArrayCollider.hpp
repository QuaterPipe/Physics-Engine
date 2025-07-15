#pragma once
#include "Collider.hpp"
namespace physics
{
	struct PointArrayCollider : public Collider
	{
		private:
		public:
			std::vector<geo::Vector2> points;
	};
}