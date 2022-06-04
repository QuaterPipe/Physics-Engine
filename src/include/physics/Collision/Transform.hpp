#include "../../geometry/main.hpp"
#include "../Base/Hashable.hpp"
#include "../Base/Serializable.hpp"
#include "../Base/Component.hpp"

namespace physics
{
	struct Transform;
	struct Transform : public serialization::Serializable, public Hashable
	{
		protected:
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			geo::Vector position = geo::Vector(0, 0);
			geo::Vector centerOfRotation = geo::Vector(0, 0);
			geo::Matrix2 scale = geo::Matrix2();
			geo::Matrix2 rotation = geo::Matrix2();
			Transform() noexcept;
			virtual ~Transform() noexcept;
			bool operator==(const Transform& other) const noexcept;
			bool operator!=(const Transform& other) const noexcept;
			geo::Matrix3 GetTransformationMatrix() const noexcept;
			geo::Vector TransformVector(const geo::Vector& v) const noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
				const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};

	namespace rk4
	{
		struct State
		{
			geo::Vector pos;
			geo::Vector vel;
			geo::Matrix2 orient;
			f64 angVel;
		};

		struct Derivative
		{
			geo::Vector dpos;
			geo::Vector dvel;
			geo::Matrix2 dorient;
			f64 dangVel;
		};

		geo::Vector Acceleration(const State& state, const f64& t);
		Derivative Evaluate(const State& initial, const f64& t, const f64& dt, const Derivative& d);
		State Integrate(const State& state, f64 t, f64 dt);
	}
}