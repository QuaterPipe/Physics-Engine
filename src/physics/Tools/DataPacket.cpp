#include "../../include/physics/Collision/Serializable.hpp"

namespace physics::serialization
{
	DataPacket::DataPacket(const size_t& size) noexcept
	{
		bytes.reserve(size);
	}

	DataPacket::DataPacket(const std::vector<unsigned char>& bytes) noexcept
	{
		this->bytes = bytes;
	}

	DataPacket DataPacket::operator+(const DataPacket& other) const noexcept
	{
		std::vector<unsigned char> newBytes = bytes;
		newBytes.insert(newBytes.end(), other.bytes.begin(), other.bytes.end());
		return DataPacket(newBytes);
	}

	void DataPacket::Reverse() noexcept
	{
		std::vector<unsigned char> newBytes(bytes.rbegin(), bytes.rend());
		bytes = newBytes;
		
	}
}