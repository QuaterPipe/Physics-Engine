#include "physics/Collision/PointMatrixCollider.hpp"
#include "physics/Collision/Algo.hpp"

namespace physics
{
	size_t PointMatrixCollider::MAX_WIDTH = 5000;
	size_t PointMatrixCollider::MAX_HEIGHT = 5000;

	PointMatrixCollider::PointMatrixCollider() noexcept
	{

	}

	PointMatrixCollider::PointMatrixCollider(size_t width, size_t height, size_t spacing) noexcept
		: _width(physics::Min(width, MAX_WIDTH)), _height(physics::Min(height, MAX_HEIGHT)), _pointCount(width* height)
	{
		_points = new Vector2[width * height];
		for (size_t i = 0; i < height; i++)
		{
			for (size_t j = 0; j < width; j++)
			{
				_points[i * width + j] = Vector2(j * spacing, i * spacing);
			}
		}
		_min = Vector2::Infinity;
		_max = -Vector2::Infinity;
		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
		}
	}

	PointMatrixCollider::PointMatrixCollider(const PointMatrixCollider& p) noexcept
		: _width(p._width), _height(p._height), _pointCount(p._width* p._height),
		_min(p._min), _max(p._max), _boundingBox(p._boundingBox)
	{
		_points = new Vector2[_width * _height];
		std::memcpy(_points, p._points, _width * _height * sizeof(f64));
	}

	PointMatrixCollider::PointMatrixCollider(PointMatrixCollider&& p) noexcept
		: _width(p._width), _height(p._height), _pointCount(p._width* p._height),
		_min(p._min), _max(p._max), _boundingBox(p._boundingBox)
	{
		_points = p._points;
		p.Release();
	}

	PointMatrixCollider::~PointMatrixCollider() noexcept
	{
		delete[] _points;
	}

	BoxCollider PointMatrixCollider::BoundingBox(const Transform& t) const noexcept
	{
		if (!_points)
			return BoxCollider();
		f64 minx = std::numeric_limits<f64>::max();
		f64 miny = std::numeric_limits<f64>::max();
		f64 maxx = -std::numeric_limits<f64>::max();
		f64 maxy = -std::numeric_limits<f64>::max();
		for (size_t i = 0; i < _pointCount; i++)
		{
			Vector2 tp = t.TransformVector(_points[i]);
			minx = tp.x < minx ? tp.x : minx;
			miny = tp.y < miny ? tp.y : miny;
			maxx = tp.x > maxx ? tp.x : maxx;
			maxy = tp.y > maxy ? tp.y : maxy;
		}
		BoxCollider result;
		result.dimensions.Set(maxx - minx, maxy - miny);
		result.pos = Vector2(minx, miny) + result.dimensions * 0.5;
		return result;
	}

	Collider* PointMatrixCollider::Clone() const noexcept
	{
		return new PointMatrixCollider(*this);
	}

	bool PointMatrixCollider::Contains(const Vector2& point, const Transform& t) const noexcept
	{
		return false;
	}

	f64 PointMatrixCollider::CrossSectionalArea(const Vector2& direction, const Transform& t) const noexcept
	{
		if (!_points)
			return -1;
		Vector2 d(direction.Normalized());
		f64 minP = std::numeric_limits<f64>::infinity(), maxP = -std::numeric_limits<f64>::infinity();
		for (size_t i = 0; i < _pointCount; i++)
		{
			f64 proj = t.TransformVector(_points[i]).Dot(d);
			if (proj < minP)
				minP = proj;
			if (proj > maxP)
				maxP = proj;
		}
		return maxP - minP;
	}

	Vector2 PointMatrixCollider::GetCenter() const noexcept
	{
		Vector2 center(0, 0);
		if (!_points)
			return center;
		for (size_t i = 0; i < _pointCount; i++)
			center += _points[i];
		return center / _pointCount;
	}

	f64 PointMatrixCollider::GetHeight() const noexcept
	{
		return _height;
	}

	f64 PointMatrixCollider::GetWidth() const noexcept
	{
		return _width;
	}

	Vector2 PointMatrixCollider::GetPoint(size_t index) const noexcept
	{
		if (index >= _pointCount);
			return Vector2::Infinity();
		return _points[index];
	}

	const Vector2* PointMatrixCollider::GetPointArray() const noexcept
	{
		return _points;
	}

	size_t PointMatrixCollider::GetPointCount() const noexcept
	{
		return _pointCount;
	}

	bool PointMatrixCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return false;
		if (dynamic_cast<const PointMatrixCollider&>(c)._pointCount != _pointCount)
			return false;
		const PointMatrixCollider& p = dynamic_cast<const PointMatrixCollider&>(c);
		if (!_points && !p._points)
			return true;
		else if (!_points)
			return false;
		for (size_t i = 0; i < _pointCount; i++)
		{
			if (p._points[i] != _points[i])
				return false;
		}
		return true;
	}

	bool PointMatrixCollider::operator!=(const Collider& c) const noexcept
	{
		return !this->operator==(c);
	}

	Vector2 PointMatrixCollider::Max() const noexcept
	{
		return _max;
	}

	Vector2 PointMatrixCollider::Min() const noexcept
	{
		return _min;
	}

	std::vector<Vector2> PointMatrixCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<Vector2> vec(_pointCount);
		if (!_points)
			return vec;
		for (size_t i = 0; i < _pointCount; i++)
			vec.push_back(t.TransformVector(_points[i]));
		return vec;
	}

	void PointMatrixCollider::Release() noexcept
	{
		_points = nullptr;
		_pointCount = 0;
		_width = 0;
		_height = 0;
		_min = Vector2::Infinity;
		_max = -Vector2::Infinity;
	}

	void PointMatrixCollider::SetPoint(size_t index, Vector2 newPosition) noexcept
	{
		if (!_points)
			return;
		_points[index] = newPosition;
		_min = Vector2::Infinity;
		_max = -Vector2::Infinity;
		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
		}
	}

	Manifold PointMatrixCollider::TestCollision(
		const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}
}