#pragma once
#include "physics/Geometry/main.hpp"
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
		Span lambdaBounds;
		bool selfcollision = true;
		virtual void UpdateConstraint(f64 dt) noexcept = 0;
		virtual void WarmStart() noexcept = 0;
		void Reset() noexcept;
	};

	struct AnchorConstraint : public Constraint
	{
		Vector2 position;
		Vector2 anchor;
		AnchorConstraint(Dynamicbody* A, const Vector2& position, const Vector2& anchor) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct AngleConstraint : public Constraint
	{
		f64 angle;
		AngleConstraint(Dynamicbody* A, Dynamicbody* B, f64 angle) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct DistanceConstraint : public Constraint
	{
		f64 distance;
		Vector2 AAnchor;
		Vector2 BAnchor;
		DistanceConstraint(Dynamicbody* A, Dynamicbody* B, f64 distance, const Vector2& AAnchor, const Vector2& BAnchor) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct HingeConstraint : public Constraint
	{
		Vector2 aHingePoint;
		Vector2 bHingePoint;
		bool rotationLock = false;
		f64 angleOffset = 0;
		HingeConstraint(Dynamicbody* A, Dynamicbody* B, const Vector2& AHingePoint, const Vector2& BHinglePoint) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct RopeConstraint : public Constraint
	{
		Span bounds;
		Vector2 AAnchor;
		Vector2 BAnchor;
		RopeConstraint(Dynamicbody* A, Dynamicbody* B, const Span& bounds, const Vector2& AAnchor, const Vector2& BAnchor) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct SliderConstraint : public Constraint
	{
		Vector2 anchor;
		Vector2 direction;
		Vector2 origin;
		Span bounds;
		bool rotationLock = true;
		f64 angleOffset = 0;
		SliderConstraint(Dynamicbody* A, const Vector2& anchor, const Vector2& direction, const Vector2& Origin, const Span& bounds = Span::Universe) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};

	struct SliderContactConstraint : public Constraint
	{
		Vector2 AAnchor;
		Vector2 BAnchor;
		Vector2 direction;
		Span bounds;
		bool rotationLock = true;
		f64 angleOffset = 0;
		SliderContactConstraint(Dynamicbody* A, Dynamicbody* B, const Vector2& AAnchor, const Vector2& BAnchor, const Vector2& direction, const Span& bounds = Span::Universe) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
		void WarmStart() noexcept override;
	};
}