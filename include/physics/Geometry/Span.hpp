#pragma once
typedef double f64;
namespace physics
{
	struct Span
	{
		f64 min;
		f64 max;
		Span() noexcept;
		Span(f64 min, f64 max) noexcept;
		Span(const Span& s) noexcept;
		bool Contains(f64 x) const noexcept;
		void ExtendLower(f64 delta) noexcept;
		void ExtendUpper(f64 delta) noexcept;
		f64 Length() const noexcept;
		bool Overlaps(const Span& other) const noexcept;
		static const Span Empty, Universe;
	};
}