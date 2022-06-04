#include "../../include/physics/Base/Hashable.hpp"
namespace physics
{
	Hashable::~Hashable() noexcept {}
// const int p = 31, m = 1e9 + 7;
// int hash = 0;
// long p_pow = 1;
// for(int i = 0; i < n; i++) {
//		hash = (hash + (s[i] - 'a' + 1) * p_pow) % m;
//		p_pow = (p_pow * p) % m;
// }
// return hash;
	long long Hashable::Hash() const noexcept
	{
		const long long prime = 31, m = 1e18 + 7;
		long long h = 0;
		long long p_pow = 1;
		auto vec = GetBytes();
		for (size_t i = 0; i < vec.size(); i++)
		{
			h = (h + (vec[i] - 'a' + 1) * p_pow) % m;
			p_pow = (p_pow * prime) % m;
		}
		return h;
	}

	bool Hashable::operator==(const Hashable& other) const noexcept
	{
		return Hash() == other.Hash();
	}

	bool Hashable::operator!=(const Hashable& other) const noexcept
	{
		return Hash() != other.Hash();
	}

	bool Hashable::Equals(const Hashable& other) const noexcept
	{
		return *this == other;
	}

	bool Hashable::NotEquals(const Hashable& other) const noexcept
	{
		return *this != other;
	}

	std::vector<unsigned char> ToBytes(const void* data, size_t size)
	{
		const unsigned char* reader = (const unsigned char*)data;
		std::vector<unsigned char> v;
		for (size_t i = 0; i < size; i++)
			v.push_back(reader[i]);
		return v;
	}
}