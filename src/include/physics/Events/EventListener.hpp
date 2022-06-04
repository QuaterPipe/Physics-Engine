#pragma once
#include "Events.hpp"
#include <functional>
namespace physics::event
{
	class EventListener
	{
		public:
			std::vector<std::function<void(const Event&)>> callbacks;
			size_t AddFunc(const std::function<void(const Event&)>& callback) noexcept
			{
				callbacks.push_back(callback);
				return callbacks.size();
			}
	
			void RemoveFunc(const size_t& index) noexcept
			{
				if (index < callbacks.size())
					callbacks.erase(callbacks.begin() + index);
			}
	
			void OnEvent(const Event& event) const noexcept
			{
				for (auto& func: callbacks)
					func(event);
			}
	};
}