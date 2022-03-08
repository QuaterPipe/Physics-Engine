#include <chrono>

namespace physics
{
	class Time final
	{
		private:
			typedef std::chrono::high_resolution_clock clock;
		public:
			static inline std::chrono::time_point<clock> time = clock::now();
			static inline double deltaTime = -1;
			static inline double microDeltaTime = -1;
			static void Tick();
	};
}