#include "../include/physics/Collision/Transform.hpp"

namespace physics
{
	using namespace serialization;
	Transform::Transform() noexcept
	: Hashable(), Serializable()
	{
		position = geometry::Vector(0, 0);
		centerOfRotation = geometry::Vector(0, 0);
		scale = geometry::Matrix2();
		rotation = geometry::Matrix2();
	}

	Transform::~Transform() noexcept
	{
		
	}

	bool Transform::operator==(const Transform& other) const noexcept
	{
		return position == other.position && scale == other.scale && rotation == other.rotation;
	}

	bool Transform::operator!=(const Transform& other) const noexcept
	{
		return !(position == other.position && scale == other.scale && rotation == other.rotation);
	}

	geometry::Matrix3 Transform::GetTransformationMatrix() const noexcept
	{
		geometry::Matrix3 result(rotation * scale);
		result.c = position.x;
		result.f = position.y;
		return result;
	}

	geometry::Vector Transform::TransformVector(const geometry::Vector& v) const noexcept
	{
		geometry::Vector3 tmp(v.x, v.y, 1);
		tmp = GetTransformationMatrix() * tmp;
		return geometry::Vector(tmp.x, tmp.y);
	}

	Serializable* Transform::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char Transform::GetByte(const size_t& index) const
	{
		return 0x01;
	}

	unsigned long Transform::TotalByteSize() const noexcept
	{
		return 0UL;
	}

	std::vector<unsigned char> Transform::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}