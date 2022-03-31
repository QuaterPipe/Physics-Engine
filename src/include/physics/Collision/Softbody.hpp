#include "Collision.hpp"
#include "CollisionObject.hpp"
namespace physics
{
	struct MassPoint
	{
		geometry::Vector position;
		geometry::Vector velocity;
		geometry::Vector force;
		f64 mass = 0;
		MassPoint();
		MassPoint(geometry::Vector position, geometry::Vector velocity, geometry::Vector force, f64 mass) noexcept;
	};

	struct Spring
	{
		MassPoint* a = NULL;
		MassPoint* b = NULL;
		f64 stiffness = 1e6;
		f64 restingLength = 0;
		f64 dampingFactor = 1e3;
		f64 ForceExerting() const noexcept;
	};

	struct Softbody : public CollisionObject
	{
		private:
			std::vector<PolygonCollider> _colliders;
		public:
			std::vector<std::vector<MassPoint>> points;
			std::vector<Spring> springs;
			unsigned width;
			unsigned height;
			bool usesGravity;
			Softbody() noexcept;
			Softbody(const Transform& t, unsigned width, unsigned height) noexcept;
			Softbody(const Softbody& s) noexcept;
			Softbody(Softbody && s) noexcept;
			Softbody& operator=(const Softbody& s) noexcept;
			Softbody& operator=(Softbody && s) noexcept;
			void ApplyForce(const geometry::Vector& force, const geometry::Vector& point=geometry::Vector::Infinity) noexcept;
			void ApplySpringForces() noexcept;
			void FixCollapsing() noexcept;
			void UpdateCollider() noexcept;
	};
}