#pragma once
#include <chrono>

namespace physics
{
	class Time final
	{
		private:
			typedef std::chrono::high_resolution_clock clock;
		public:
			static inline std::chrono::time_point<clock> time = clock::now();
			/// Time since last Tick in Milliseconds
			static inline double deltaTime = -1;
			///  Time since last Tick in Microseconds
			static inline double microDeltaTime = -1;
			static void Tick();
	};

	class Timer
	{
		private:
			typedef std::chrono::high_resolution_clock clock;
		public:
			std::chrono::time_point<clock> start;
			std::chrono::time_point<clock> stop;
			double deltaTime = -1;
			double microDeltaTime = -1;
			void Start();
			void Stop();
			void Reset();
	};
}