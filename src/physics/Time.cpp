#include "../include/physics/Time.hpp"
namespace physics
{
	void Time::Tick()
	{
		std::chrono::time_point<clock> t = clock::now();
		microDeltaTime = std::chrono::duration_cast<std::chrono::microseconds>(t - time).count();
		deltaTime = microDeltaTime * 0.001;
	}
}