#include "Dynamicbody.hpp"
#pragma once

namespace physics
{
	struct Dynamicbody;
	struct Joint
	{
		public:
			Dynamicbody* a = NULL;
			Dynamicbody* b = NULL;
			geo::Vector aContact;
			geo::Vector bContact;
			geo::Vector maxForce = geo::Vector::Infinity;
			virtual void Update(f64 dt) noexcept = 0;
	};

	struct DistanceJoint : public Joint
	{
		public:
			f64 length;
			DistanceJoint(Dynamicbody* a, Dynamicbody* b, f64 length) noexcept;
			virtual void Update(f64 dt) noexcept override;
	};

	struct SpringJoint : public DistanceJoint
	{
		public:
			f64 stiffness = 2;
			f64 dampingFactor = 0.04;
			f64 ForceExerting() const noexcept;
			virtual void Update(f64 dt) noexcept override;		
	};

	struct HingeJoint : public Joint
	{
		public:
			bool locked;
			f64 angularFriction;
			HingeJoint(Dynamicbody* a, Dynamicbody* b, f64 angularFriction);
			virtual void Update(f64 dt) noexcept override;
	};
}