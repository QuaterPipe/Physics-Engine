#pragma once
#include "geometry/main.hpp"
#include "Dynamicbody.hpp"
namespace physics
{
	struct Constraint
	{
		Dynamicbody* a = nullptr;
		Dynamicbody* b = nullptr;
		virtual void UpdateConstraint(f64 dt) noexcept = 0;
	};

	struct AnchorConstraint : public Constraint
	{
		geo::Vector2 anchorPoint;
		geo::Vector2 pointOnBody;
		AnchorConstraint(Dynamicbody* a, geo::Vector2 anchorPoint, geo::Vector2 pointOnBody) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
	};

	struct AngleConstraint : public Constraint
	{
		f64 angle;
		AngleConstraint(Dynamicbody* a, Dynamicbody* b, f64 angle) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
	};

	struct DistanceConstraint : public Constraint
	{
		f64 distance;
		DistanceConstraint(Dynamicbody* a, Dynamicbody* b, f64 distance) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
	};

	struct HingeConstraint : public Constraint
	{
		geo::Vector2 aHingePoint;
		geo::Vector2 bHingePoint;
		HingeConstraint(Dynamicbody* a, Dynamicbody* b, geo::Vector2 AHingePoint, geo::Vector2 BHinglePoint) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
	};

	struct RopeConstraint : public Constraint
	{
		f64 maxDistance;
		RopeConstraint(Dynamicbody* a, Dynamicbody* b, f64 maxDistance) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
	};

	struct SliderConstraint : public Constraint
	{
		geo::Vector2 direction;
		geo::Vector2 origin;
		SliderConstraint(Dynamicbody* a) noexcept;
		void UpdateConstraint(f64 dt) noexcept override;
	};
}