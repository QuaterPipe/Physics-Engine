#pragma once
namespace physics
{
	struct Component
	{
		virtual Component* Clone() const noexcept;
	};
}