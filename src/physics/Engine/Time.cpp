#include "../../include/physics/Engine/Time.hpp"
namespace physics
{
	void Time::Tick()
	{
		std::chrono::time_point<clock> t = clock::now();
		microDeltaTime = (double)std::chrono::duration_cast<std::chrono::microseconds>(t - time).count();
		deltaTime = microDeltaTime * 0.001;
		time = clock::now();
	}

	void Timer::Start()
	{
		start = clock::now();
		deltaTime = -1;
		microDeltaTime = -1;
	}

	void Timer::Stop()
	{
		stop = clock::now();
		microDeltaTime = (double)std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
		deltaTime = microDeltaTime * 0.001;
	}

	void Timer::Reset()
	{
		microDeltaTime = -1;
		deltaTime = 1;
	}
}