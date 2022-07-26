#include "../../geometry/main.hpp"

namespace physics
{
	struct Transform;
	struct Transform
	{
		public:
			geo::Vector2 position = geo::Vector2(0, 0);
			geo::Vector2 centerOfRotation = geo::Vector2(0, 0);
			geo::Matrix2 scale = geo::Matrix2();
			geo::Matrix2 rotation = geo::Matrix2();
			Transform() noexcept;
			Transform(const Transform& transform) noexcept;
			Transform& operator=(const Transform& transform) noexcept;
			Transform& operator=(Transform&& transform) noexcept;
			virtual ~Transform() noexcept;
			bool operator==(const Transform& other) const noexcept;
			bool operator!=(const Transform& other) const noexcept;
			geo::Matrix3 GetTransformationMatrix() const noexcept;
			geo::Vector2 TransformVector(const geo::Vector2& v) const noexcept;
	};

	namespace rk4
	{
		struct State
		{
			geo::Vector2 pos;
			geo::Vector2 vel;
			geo::Matrix2 orient;
			f64 angVel;
		};

		struct Derivative
		{
			geo::Vector2 dpos;
			geo::Vector2 dvel;
			geo::Matrix2 dorient;
			f64 dangVel;
		};

		geo::Vector2 Acceleration(const State& state, const f64& t);
		Derivative Evaluate(const State& initial, const f64& t, const f64& dt, const Derivative& d);
		State Integrate(const State& state, f64 t, f64 dt);
	}
}