#pragma once
#include "Collider.hpp"
namespace physics
{
	struct PointArrayCollider : public Collider
	{
		private:
		public:
			std::vector<Vector2> points;
	};
}