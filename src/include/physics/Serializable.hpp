#pragma once
#include "Serializable.hpp"
#include <fstream>
#include <map>
#include <stdint.h>
#include <typeinfo>
#include <vector>
namespace physics::serialization
{
	class Archive;
	class Serializable;

	class Archive
	{
		private:
			std::map<int16_t, Serializable*> classMap;
			std::vector<unsigned char> bytes;
		public:
			Archive();
			Archive& operator<<(const Serializable& object);
			std::ostream& operator<<(std::ostream& stream);
			Archive& operator>>(std::vector<Serializable*>& vec);
			std::istream& operator>>(std::istream& stream);
			bool AddClass(Serializable* classDeserializer, uint16_t classCode) noexcept;
			const std::vector<unsigned char>& GetBytes() const noexcept;
			bool RemoveClass(uint16_t classCode) noexcept;
			static std::vector<unsigned char> ReadBytes(const unsigned char* object, const size_t& size);
			static void WriteBytes(unsigned char* object, const std::vector<unsigned char>::const_iterator& bytes, const size_t& length);
	};

	class Serializable
	{
		protected:
			typedef unsigned char byte;
			typedef unsigned char* writer;
			typedef const unsigned char* reader;
			unsigned long byteSize = sizeof(uint16_t);
		public:
			uint16_t classCode;
			virtual Serializable* Deserialize(const std::vector<byte>& v, const size_t& index, const size_t& length) const = 0;
			virtual byte GetByte(const size_t& index) const = 0;
			virtual unsigned long TotalByteSize() const noexcept = 0;
			virtual std::vector<byte> Serialize() const noexcept = 0;
	};
}