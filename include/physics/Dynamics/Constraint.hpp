#pragma once
#include "geometry/main.hpp"
#include "Dynamicbody.hpp"
namespace physics
{
	struct Constraint
	{
		Dynamicbody* a = nullptr;
		Dynamicbody* b = nullptr;
		f64 biasFactor = 0.01;
		f64 accumulatedLambda = 0;
		f64 accumulatedAngularLambda = 0;
		f64 dampingFactor = 0.01;
		geo::Span lambdaBounds;
		virtual void CorrectPosition() noexcept = 0;
		virtual void UpdateConstraint(f64 dt) noexcept = 0;
		virtual void WarmStart() noexcept = 0;
		void Reset() noexcept;
	};

	struct AnchorConstraint : public Constraint
	{
		geo::Vector2 position;
		geo::Vector2 anchor;
		AnchorConstraint(Dynamicbody* A, const geo::Vector2& position, const geo::Vector2& anchor) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct AngleConstraint : public Constraint
	{
		f64 angle;
		AngleConstraint(Dynamicbody* A, Dynamicbody* B, f64 angle) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct DistanceConstraint : public Constraint
	{
		f64 distance;
		geo::Vector2 AAnchor;
		geo::Vector2 BAnchor;
		DistanceConstraint(Dynamicbody* A, Dynamicbody* B, f64 distance, const geo::Vector2& AAnchor, const geo::Vector2& BAnchor) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct HingeConstraint : public Constraint
	{
		geo::Vector2 aHingePoint;
		geo::Vector2 bHingePoint;
		HingeConstraint(Dynamicbody* A, Dynamicbody* B, const geo::Vector2& AHingePoint, const geo::Vector2& BHinglePoint) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct RopeConstraint : public Constraint
	{
		geo::Span bounds;
		geo::Vector2 AAnchor;
		geo::Vector2 BAnchor;
		RopeConstraint(Dynamicbody* A, Dynamicbody* B, const geo::Span& bounds, const geo::Vector2& AAnchor, const geo::Vector2& BAnchor) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct SliderConstraint : public Constraint
	{
		geo::Vector2 anchor;
		geo::Vector2 direction;
		geo::Vector2 origin;
		geo::Span bounds;
		bool rotationLock = true;
		f64 angleOffset = 0;
		SliderConstraint(Dynamicbody* A, const geo::Vector2& anchor, const geo::Vector2& direction, const geo::Vector2& Origin, const geo::Span& bounds = geo::Span::Universe) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct SliderContactConstraint : public Constraint
	{
		geo::Vector2 anchor;
		geo::Vector2 direction;
		geo::Vector2 origin;
		geo::Span bounds;
		bool rotationLock = true;
		f64 angleOffset = 0;
		SliderContactConstraint(Dynamicbody* A, const geo::Vector2& anchor, const geo::Vector2& direction, const geo::Vector2& Origin, const geo::Span& bounds = geo::Span::Universe) noexcept;
		void CorrectPosition() noexcept override;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};
}