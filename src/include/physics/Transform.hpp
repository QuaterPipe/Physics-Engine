#include "../geometry/main.hpp"
#include "Hashable.hpp"
#include "Serializable.hpp"

namespace physics
{
	struct Transform;
	struct Transform : public serialization::Serializable, public Hashable
	{
		geometry::Vector position = geometry::Vector(0, 0);
		geometry::Vector centerOfRotation = geometry::Vector(0, 0);
		geometry::Matrix2 scale = geometry::Matrix2();
		geometry::Matrix2 rotation = geometry::Matrix2();
		Transform() noexcept;
		virtual ~Transform() noexcept;
		bool operator==(const Transform& other) const noexcept;
		bool operator!=(const Transform& other) const noexcept;
		geometry::Matrix3 GetTransformationMatrix() const noexcept;
		geometry::Vector TransformVector(const geometry::Vector& v) const noexcept;
		virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
		virtual byte GetByte(const size_t& index) const override;
		virtual unsigned long TotalByteSize() const noexcept override;
		virtual std::vector<byte> Serialize() const noexcept override;
	};
}