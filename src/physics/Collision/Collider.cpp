#include "../../include/physics/Collision/Collision.hpp"

namespace physics
{
	Collider::Collider() noexcept
	{
	}

	Collider* Collider::Clone() const noexcept
	{
		return NULL;
	}
	Collider::~Collider() noexcept
	{
	}
}