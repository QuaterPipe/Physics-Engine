#include "geometry/Span.hpp"
#include "geometry/Math.hpp"

namespace geo
{
	Span::Span() noexcept
		: min(-std::numeric_limits<f64>::infinity()), max(std::numeric_limits<f64>::infinity())
	{
	}

	Span::Span(f64 min, f64 max) noexcept
		: min(min), max(max)
	{
	}

	Span::Span(const Span& s) noexcept
		: min(s.min), max(s.max)
	{

	}

	bool Span::Contains(f64 x) const noexcept
	{
		return min <= x && x <= max;
	}

	void Span::ExtendLower(f64 delta) noexcept
	{
		min -= delta;
	}

	void Span::ExtendUpper(f64 delta) noexcept
	{
		max += delta;
	}

	f64 Span::Length() const noexcept
	{
		return max - min;
	}

	bool Span::Overlaps(const Span& other) const noexcept
	{
		return (min <= other.min && other.min <= max) || (other.min <= min && min <= other.max);
	}

	const Span Span::Empty = Span(std::numeric_limits<f64>::infinity(), -std::numeric_limits<f64>::infinity());
	const Span Span::Universe = Span(-std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity());
}