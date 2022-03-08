#include "../include/physics/Serializable.hpp"
#define in :

namespace physics::serialization
{
	Archive::Archive()
	{
	}

	Archive& Archive::operator<<(const Serializable& object)
	{
		std::vector<unsigned char> objectBytes = object.Serialize();
		// size of the object.	
		int32_t len = objectBytes.size() + 1;
		const unsigned char* tmp = (const unsigned char*)&len;
		// pushing back the length of the object in bytes.
		for (int i = 0; i < 4; i++)
			bytes.push_back(tmp[i]);
		tmp = (const unsigned char*)&object.classCode;
		// pushing back the class code of the object.
		for (int i = 0; i < 2; i++)
			bytes.push_back(tmp[i]);
		for (unsigned char b in objectBytes)
			bytes.push_back(b);
		return *this;
	}

	std::ostream& Archive::operator<<(std::ostream& stream)
	{
		for (unsigned char byte in bytes)
		{
			stream<<byte;
		}
		return stream;
	}

	Archive& Archive::operator>>(std::vector<Serializable*>& vec)
	{
		uint16_t objClassCode = 0;
		size_t offset = 0;
		int32_t objLen = 0;
		unsigned char* valPtr = nullptr;
		for (auto iter = bytes.begin(); iter < bytes.end() + 6; iter++)
		{
			// to get the length of the object.
			offset += 4;
			valPtr = (unsigned char*)&objLen; //not sketchy at all...
			for (int i = 0; i < 4; i++)
				valPtr[i] = iter[i];
			iter += 4;
			// getting object class code.
			valPtr = (unsigned char*)&objClassCode;
			offset += 2;
			for (int i = 0; i < 2; i++)
				valPtr[i] = iter[i];
			iter += 2;
			if (!classMap.count(objClassCode))
				throw std::runtime_error("class code not in map");
			try
				{vec.push_back(classMap[objClassCode]->Deserialize(bytes, offset, objLen));}
			catch (const std::runtime_error& e)
				{throw e;}
			offset += objLen;
			iter += objLen;
		}
		return *this;
	}

	std::istream& Archive::operator>>(std::istream& stream)
	{
		unsigned char byte;
		while (!stream.eof())
		{
			stream >> byte;
			bytes.push_back(byte);
		}
		return stream;
	}

	bool Archive::AddClass(Serializable* classDeserializer, uint16_t classCode) noexcept
	{
		if (classMap.count(classCode))
			return false;
		classMap[classCode] = classDeserializer;
		return true;
	}

	const std::vector<unsigned char>& Archive::GetBytes() const noexcept
	{
		return bytes;
	}

	bool Archive::RemoveClass(uint16_t classCode) noexcept
	{
		if (!classMap.count(classCode))
			return false;
		classMap.erase(classCode);
		return true;
	}

	std::vector<unsigned char> Archive::ReadBytes(const unsigned char* object, const size_t& size)
	{
		std::vector<unsigned char> target;
#if BIG_ENDIAN
		for (size_t i = 0; i < size; i++)
			target.push_back(object[i]);
#elif SMALL_ENDIAN
		for (size_t i = 0; i < size; i++)
			target.push_back(object[size - 1 - i]);
#endif
		return target;
	}

	std::vector<unsigned char> Archive::ReadBytesOfObject(const unsigned char* object,
		const size_t& size, std::vector<size_t> memberIndexes)
	{
		std::vector<unsigned char> bytes;
		if (!memberIndexes.size())
			return ReadBytes(object, size);
// wont matter.
#if BIG_ENDIAN
		return ReadBytes(object, size);
// here's where its painful.
#elif SMALL_ENDIAN
		// make sure all indexes are in order
		geometry::QuickSort(memberIndexes.begin(), 0, memberIndexes.size());
		if (memberIndexes.at(0) == 0)
			memberIndexes.erase(memberIndexes.begin());
		if ()
			;
			
#endif
		return bytes;
	}
			

	void Archive::WriteBytes(unsigned char* object, const std::vector<unsigned char>::const_iterator& bytes, const size_t& length)
	{
#if BIG_ENDIAN
		for (size_t i = 0; i < length; i++)
			object[i] = bytes[i];
#elif SMALL_ENDIAN
		for (size_t i = 0; i < length; i++)
			object[i] = bytes[length - 1 - i];
#endif
	}
}