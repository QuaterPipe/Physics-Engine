#include "physics/Dynamics/Constraint.hpp"
#include "physics/Dynamics/Dynamicbody.hpp"
#include <iostream>

namespace physics
{
	void Constraint::Reset() noexcept
	{
		accumulatedLambda = 0;
		accumulatedAngularLambda = 0;
	}

	AnchorConstraint::AnchorConstraint(Dynamicbody* A, const Vector2& position, const Vector2& anchor) noexcept
		: position(position), anchor(anchor)
	{
		a = A;
	}

	void AnchorConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a)
			return;
		Vector2 anchA = a->transform.TransformVector(anchor);
		Vector2 relativePos = anchA - position;
		f64 curDis = relativePos.GetMagnitude();
		f64 offset = -curDis;
		if (fabs(offset) >= 0)
		{
			Vector2 offsetDir = relativePos / curDis;
			Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
			Vector2 relativeVel = a->velocity + Vector2::Cross(a->angularVelocity, ra);
			f64 constraintMass = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
			if (constraintMass > EPSILON)
			{
				f64 vDot = relativeVel.Dot(offsetDir);
				f64 bias = -(biasFactor / dt) * offset;
				f64 lambda = -(vDot + bias) / constraintMass;
				f64 oldLambda = accumulatedLambda;
				accumulatedLambda += lambda;
				lambda = accumulatedLambda - oldLambda;
				a->ApplyImpulse(offsetDir * lambda, ra);
			}
		}
	}

	void AnchorConstraint::WarmStart() noexcept
	{
		if (!a)
			return;
		Vector2 anchA = a->transform.TransformVector(anchor);
		Vector2 relativePos = anchA - position;
		f64 curDis = relativePos.GetMagnitude();
		f64 offset = -curDis;
		if (fabs(offset) < EPSILON)
			return;
		Vector2 offsetDir = relativePos / curDis;
		Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
		f64 constraintMass = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
		if (constraintMass < EPSILON)
			return;
		a->ApplyImpulse(offsetDir * accumulatedLambda, ra);
	}

	AngleConstraint::AngleConstraint(Dynamicbody* A, Dynamicbody* B, f64 angle) noexcept
		: angle(angle)
	{
		a = A;
		b = B;
	}

	void AngleConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a)
			return;
		f64 relAngVel = a->angularVelocity - b->angularVelocity;
		f64 rotOffset = a->transform.GetAngle() - b->transform.GetAngle();
		f64 bias = -(biasFactor / dt) * rotOffset;

		f64 angLambda = -(relAngVel + bias) / (a->GetInvInertia() + b->GetInvInertia());
		f64 oldAngLambda = accumulatedAngularLambda;
		accumulatedAngularLambda += angLambda;
		angLambda = accumulatedAngularLambda - oldAngLambda;
		a->angularVelocity += angLambda * a->GetInvInertia();
		b->angularVelocity -= angLambda * b->GetInvInertia();
	}

	void AngleConstraint::WarmStart() noexcept
	{
		a->angularVelocity += accumulatedAngularLambda * a->GetInvInertia();
		b->angularVelocity -= accumulatedAngularLambda * b->GetInvInertia();
	}

	DistanceConstraint::DistanceConstraint(Dynamicbody* A, Dynamicbody* B, f64 distance, const Vector2& AAnchor, const Vector2& BAnchor) noexcept
		: distance(fabs(distance)), AAnchor(AAnchor), BAnchor(BAnchor)
	{
		a = A;
		b = B;
	}

	void DistanceConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a || !b)
			return;
		Vector2 anchA = a->transform.TransformVector(AAnchor);
		Vector2 anchB = b->transform.TransformVector(BAnchor);
		Vector2 relativePos = anchA - anchB;
		f64 curDis = relativePos.GetMagnitude();
		f64 offset = distance - curDis;
		if (fabs(offset) >= 0)
		{
			Vector2 offsetDir = relativePos / curDis;
			Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
			Vector2 rb = anchB - (b->transform.GetPosition() + b->transform.GetCOM());
			Vector2 relativeVel = a->velocity + Vector2::Cross(a->angularVelocity, ra) -
				b->velocity - Vector2::Cross(b->angularVelocity, rb);
			f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
			f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(offsetDir));

			f64 constraintMass = termA + termB;
			if (constraintMass > EPSILON)
			{
				f64 vDot = relativeVel.Dot(offsetDir) * (1 - dampingFactor);
				f64 bias = -(biasFactor / dt) * offset;
				f64 lambda = -(vDot + bias) / constraintMass;
				f64 oldLambda = accumulatedLambda;
				accumulatedLambda += lambda;
				lambda = accumulatedLambda - oldLambda;
				a->ApplyImpulse(offsetDir * lambda, ra);
				b->ApplyImpulse(-offsetDir * lambda, rb);
			}
		}
	}

	void DistanceConstraint::WarmStart() noexcept
	{
		if (!a || !b)
			return;
		Vector2 anchA = a->transform.TransformVector(AAnchor);
		Vector2 anchB = b->transform.TransformVector(BAnchor);
		Vector2 relativePos = anchA - anchB;
		f64 curDis = relativePos.GetMagnitude();
		f64 offset = distance - curDis;
		if (fabs(offset) < EPSILON)
			return;
		Vector2 offsetDir = relativePos / curDis;
		Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
		Vector2 rb = anchB - (b->transform.GetPosition() + b->transform.GetCOM());
		
		f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
		f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(offsetDir));
		f64 constraintMass = termA + termB;
		if (constraintMass < EPSILON)
			return;
		a->ApplyImpulse(offsetDir * accumulatedLambda, ra);
		b->ApplyImpulse(-offsetDir * accumulatedLambda, rb);
	}

	HingeConstraint::HingeConstraint(Dynamicbody* A, Dynamicbody* B, const Vector2& AHingePoint, const Vector2& BHinglePoint) noexcept
		: aHingePoint(AHingePoint), bHingePoint(BHinglePoint)
	{
		a = A;
		b = B;

	}

	void HingeConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a || !b)
			return;
		Vector2 aHinge = a->transform.TransformVector(aHingePoint);
		Vector2 bHinge = b->transform.TransformVector(bHingePoint);
		Vector2 relativePos = aHinge - bHinge;
		f64 curDis = relativePos.GetMagnitude();
		f64 offset = -curDis;
		if (fabs(offset) >= 0)
		{
			Vector2 offsetDir = relativePos / curDis;
			Vector2 ra = aHinge - (a->transform.GetPosition() + a->transform.GetCOM());
			Vector2 rb = bHinge - (b->transform.GetPosition() + b->transform.GetCOM());
			Vector2 relativeVel = a->velocity + Vector2::Cross(a->angularVelocity, ra) -
				b->velocity - Vector2::Cross(b->angularVelocity, rb);
			f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
			f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(offsetDir));

			f64 constraintMass = termA + termB;
			if (constraintMass > EPSILON)
			{
				f64 vDot = relativeVel.Dot(offsetDir) * (1 - dampingFactor);
				f64 bias = -(biasFactor / dt) * offset;
				f64 lambda = -(vDot + bias) / constraintMass;
				f64 oldLambda = accumulatedLambda;
				accumulatedLambda += lambda;
				lambda = accumulatedLambda - oldLambda;
				a->ApplyImpulse(offsetDir * lambda, ra);
				b->ApplyImpulse(-offsetDir * lambda, rb);
				if (rotationLock)
				{
					f64 relAngVel = a->angularVelocity - b->angularVelocity;
					f64 rotOffset = angleOffset - (a->transform.GetAngle() - b->transform.GetAngle());
					f64 bias = -(biasFactor / dt) * rotOffset;
					f64 constraintMass = a->GetInvInertia() + b->GetInvInertia();

					f64 angLambda = -(relAngVel + bias) / constraintMass;
					f64 oldAngLambda = accumulatedAngularLambda;
					accumulatedAngularLambda += angLambda;
					angLambda = accumulatedAngularLambda - oldAngLambda;
					a->angularVelocity += angLambda * a->GetInvInertia();
					b->angularVelocity -= angLambda * b->GetInvInertia();
				}
			}
		}
	}


	void HingeConstraint::WarmStart() noexcept
	{
		if (!a || !b)
			return;
		Vector2 aHinge = a->transform.TransformVector(aHingePoint);
		Vector2 bHinge = b->transform.TransformVector(bHingePoint);
		Vector2 relativePos = aHinge - bHinge;
		f64 curDis = relativePos.GetMagnitude();
		f64 offset = -curDis;
		if (fabs(offset) < EPSILON)
			return;
		Vector2 offsetDir = relativePos / curDis;
		Vector2 ra = aHinge - (a->transform.GetPosition() + a->transform.GetCOM());
		Vector2 rb = bHinge - (b->transform.GetPosition() + b->transform.GetCOM());

		f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
		f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(offsetDir));
		f64 constraintMass = termA + termB;
		if (constraintMass < EPSILON)
			return;
		a->ApplyImpulse(offsetDir * accumulatedLambda, ra);
		b->ApplyImpulse(-offsetDir * accumulatedLambda, rb);
		if (rotationLock)
		{
			a->angularVelocity += accumulatedAngularLambda * a->GetInvInertia();
			b->angularVelocity -= accumulatedAngularLambda * b->GetInvInertia();
		}
	}

	RopeConstraint::RopeConstraint(Dynamicbody* A, Dynamicbody* B, const Span& bounds, const Vector2& AAnchor, const Vector2& BAnchor) noexcept
		: bounds(bounds), AAnchor(AAnchor), BAnchor(BAnchor)
	{
		a = A;
		b = B;
	}

	void RopeConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a || !b)
			return;
		Vector2 anchA = a->transform.TransformVector(AAnchor);
		Vector2 anchB = b->transform.TransformVector(BAnchor);
		Vector2 relativePos = anchA - anchB;
		f64 curDis = relativePos.GetMagnitude();
		if (bounds.min <= curDis && curDis <= bounds.max)
			return;
		f64 offset = 0;
		if (curDis < bounds.min)
			offset = bounds.min - curDis;
		if (curDis > bounds.max)
			offset = bounds.max - curDis;
		if (fabs(offset) >= 0)
		{
			Vector2 offsetDir = relativePos / curDis;
			Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
			Vector2 rb = anchB - (b->transform.GetPosition() + b->transform.GetCOM());
			Vector2 relativeVel = a->velocity + Vector2::Cross(a->angularVelocity, ra) -
				b->velocity - Vector2::Cross(b->angularVelocity, rb);
			f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
			f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(offsetDir));

			f64 constraintMass = termA + termB;
			if (constraintMass > EPSILON)
			{
				f64 vDot = relativeVel.Dot(offsetDir);
				f64 bias = -(biasFactor / dt) * offset;
				f64 lambda = -(vDot + bias) / constraintMass;
				f64 oldLambda = accumulatedLambda;
				accumulatedLambda += lambda;
				lambda = accumulatedLambda - oldLambda;
				a->ApplyImpulse(offsetDir * lambda, ra);
				b->ApplyImpulse(-offsetDir * lambda, rb);
			}
		}
	}


	void RopeConstraint::WarmStart() noexcept
	{
		if (!a || !b)
			return;
		Vector2 relativePos = a->transform.TransformVector(AAnchor) - b->transform.TransformVector(BAnchor);
		f64 curDis = relativePos.GetMagnitude();
		if (bounds.min <= curDis && curDis <= bounds.max)
			return;
		f64 offset = 0;
		if (curDis < bounds.min)
			offset = bounds.min - curDis;
		if (curDis > bounds.max)
			offset = bounds.max - curDis;
		Vector2 offsetDir = relativePos / curDis;
		Vector2 ra = a->transform.TransformVector(AAnchor) - (a->transform.GetPosition() + a->transform.GetCOM());
		Vector2 rb = b->transform.TransformVector(BAnchor) - (b->transform.GetPosition() + b->transform.GetCOM());

		f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(offsetDir));
		f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(offsetDir));
		f64 constraintMass = termA + termB;
		if (constraintMass < EPSILON)
			return;
		a->ApplyImpulse(offsetDir * accumulatedLambda, ra);
		b->ApplyImpulse(-offsetDir * accumulatedLambda, rb);
	}

	SliderConstraint::SliderConstraint(Dynamicbody* A, const Vector2& anchor, const Vector2& direction, const Vector2& Origin, const Span& bounds) noexcept
		: anchor(anchor), bounds(bounds), direction(direction.Normalized()), origin(Origin)
	{
		a = A;
	}

	void SliderConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a)
			return;
		Vector2 worldAnchor = a->transform.TransformVector(anchor);
		Vector2 ra = worldAnchor - (a->transform.GetPosition() + a->transform.GetCOM());
		Vector2 normal(-direction.y, direction.x);
		Vector2 relativeVel = a->velocity + Vector2::Cross(a->angularVelocity, ra);
		f64 constraintMass = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(normal));
		if (constraintMass > EPSILON)
		{
			f64 vDot = relativeVel.Dot(normal);
			Vector2 delta = worldAnchor - origin;
			f64 offset = -delta.Dot(normal);
			f64 bias = -(biasFactor / dt) * offset;
			f64 lambda = -(vDot + bias) / constraintMass;
			
			f64 oldLambda = accumulatedLambda;
			accumulatedLambda += lambda;
			lambda = accumulatedLambda - oldLambda;

			a->ApplyImpulse(lambda * normal, ra);
			if (rotationLock)
			{
				f64 relAngVel = a->angularVelocity;
				f64 rotOffset = angleOffset - a->transform.GetAngle();
				f64 bias = -(biasFactor / dt) * rotOffset;

				f64 angLambda = -(relAngVel + bias) / a->GetInvInertia();
				f64 oldAngLambda = accumulatedAngularLambda;
				accumulatedAngularLambda += angLambda;
				angLambda = accumulatedAngularLambda - oldAngLambda;
				a->angularVelocity += angLambda * a->GetInvInertia();
			}
		}

	}

	void SliderConstraint::WarmStart() noexcept
	{
		if (!a)
			return;
		Vector2 worldAnchor = a->transform.TransformVector(anchor);
		Vector2 ra = worldAnchor - a->transform.GetPosition();
		Vector2 normal(-direction.y, direction.x);

		f64 constraintMass = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(normal));
		if (constraintMass < EPSILON)
			return;
		a->ApplyImpulse(normal * accumulatedLambda, ra);
		if (rotationLock)
			a->angularVelocity += accumulatedAngularLambda * a->GetInvInertia();
	}

	SliderContactConstraint::SliderContactConstraint(Dynamicbody* A, Dynamicbody* B, const Vector2& AAnchor, const Vector2& BAnchor, const Vector2& direction, const Span& bounds) noexcept
		: AAnchor(AAnchor), BAnchor(BAnchor), direction(direction), bounds(bounds)
	{
		a = A;
		b = B;
	}

	void SliderContactConstraint::UpdateConstraint(f64 dt) noexcept
	{
		if (!a || !b)
			return;

		Vector2 anchA = a->transform.TransformVector(AAnchor);
		Vector2 anchB = b->transform.TransformVector(BAnchor);
		Vector2 relativePos = anchA - anchB;
		Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
		Vector2 rb = anchB - (b->transform.GetPosition() + b->transform.GetCOM());
		Vector2 norm(-direction.y, direction.x);

		Vector2 relativeVel = a->velocity + Vector2::Cross(a->angularVelocity, ra) -
			b->velocity - Vector2::Cross(b->angularVelocity, rb);
		f64 vDot = relativeVel.Dot(norm);
		f64 termA = a->GetInvMass() + a->GetInvInertia() * SQRD(ra.Cross(norm));
		f64 termB = b->GetInvMass() + b->GetInvInertia() * SQRD(rb.Cross(norm));

		f64 constraintMass = termA + termB;
		f64 offset = (anchB - anchA).Dot(norm);
		f64 bias = -(biasFactor / dt) * offset;
		f64 lambda = -(vDot + bias) / constraintMass;

		f64 oldLambda = accumulatedLambda;
		accumulatedLambda += lambda;
		lambda = accumulatedLambda - oldLambda;
		a->ApplyImpulse(lambda * norm, ra);
		b->ApplyImpulse(-lambda * norm, rb);
		if (rotationLock)
		{
			f64 relAngVel = a->angularVelocity - b->angularVelocity;
			f64 rotOffset = a->transform.GetAngle() - b->transform.GetAngle();
			f64 bias = -(biasFactor / dt) * rotOffset;

			f64 angLambda = -(relAngVel + bias) / (a->GetInvInertia() + b->GetInvInertia());
			f64 oldAngLambda = accumulatedAngularLambda;
			accumulatedAngularLambda += angLambda;
			angLambda = accumulatedAngularLambda - oldAngLambda;
			a->angularVelocity += angLambda * a->GetInvInertia();
			b->angularVelocity -= angLambda * b->GetInvInertia();
		}
	}

	void SliderContactConstraint::WarmStart() noexcept
	{
		if (!a || !b)
			return;
		Vector2 anchA = a->transform.TransformVector(AAnchor);
		Vector2 anchB = b->transform.TransformVector(BAnchor);
		Vector2 relativePos = anchA - anchB;
		Vector2 ra = anchA - (a->transform.GetPosition() + a->transform.GetCOM());
		Vector2 rb = anchB - (b->transform.GetPosition() + b->transform.GetCOM());
		Vector2 norm(-direction.y, direction.x);

		a->ApplyImpulse(accumulatedLambda * norm, ra);
		b->ApplyImpulse(-accumulatedLambda * norm, rb);
		if (rotationLock)
		{
			a->angularVelocity += accumulatedAngularLambda * a->GetInvInertia();
			b->angularVelocity -= accumulatedAngularLambda * b->GetInvInertia();
		}
	}
}