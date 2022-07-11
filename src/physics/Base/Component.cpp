#include "../../include/physics/Base/Component.hpp"

namespace physics
{

	Component::Component()
	{
	}

	Component::Component(int64_t id)
	: id(id), Tag("Base_Component")
	{
	}

	Component::Component(const Component& c)
	: id(c.id), Tag(c.Tag)
	{
	}

	Component* Component::Clone() const noexcept
	{
		return new Component(*this);
	}

	template <const char* name>
	bool Component::InheritsFrom()
	{
		return std::count(inheritedTags.begin(), inheritedTags.end(), name);
	}
}
