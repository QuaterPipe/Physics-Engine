#include "../../geometry/main.hpp"
#include "../Engine/Hashable.hpp"
#include "Serializable.hpp"

namespace physics
{
	struct Transform;
	struct Transform : public serialization::Serializable, public Hashable
	{
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
}