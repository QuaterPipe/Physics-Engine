#include "physics/Collision/Collision.hpp"
namespace physics
{
	PolygonCollider::PolygonCollider(const BoxCollider& b) noexcept
		: _pointCount(4), _center(b.pos)
	{
		_points = new Vector2[4];
		_normals = new Vector2[4];
		_points[0] = b.pos - (b.dimensions / 2);
		_points[1] = Vector2(b.x + (b.width / 2), b.y - (b.height / 2));
		_points[2] = b.pos + (b.dimensions / 2);
		_points[3] = Vector2(b.x - (b.width / 2), b.y + (b.height / 2));
		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
			Vector2 norm = _points[(i + 1) % _pointCount] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals[i] = norm;
		}
	}

	PolygonCollider::PolygonCollider(const PolygonCollider& p) noexcept
		: _pointCount(p.GetPointCount()), _center(p.GetCenter())
	{
		_points = new Vector2[_pointCount];
		_normals = new Vector2[_pointCount];
		for (int i = 0; i < _pointCount; i++)
			_points[i] = p.GetPoint(i);
		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
			size_t ind = i == _pointCount - 1 ? 0 : i + 1;
			Vector2 norm = _points[ind] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals[i] = norm;
		}
	}

	PolygonCollider::PolygonCollider(PolygonCollider&& p) noexcept
		: _points(p.GetVectorArray()), _normals(p.GetNormalArray()), _pointCount(p.GetPointCount()), _center(p.GetCenter())
	{
		p.Release();
	}

	PolygonCollider::PolygonCollider(f64 sideLength, size_t count) noexcept
		: _pointCount(physics::Min(count, MAX_POLYGONCOLLIDER_SIZE)), _center(0, 0)
	{
		assert(count >= 3 && sideLength > EPSILON);
		_points = new Vector2[_pointCount];
		_normals = new Vector2[_pointCount];
		f64 rotation = floor((_pointCount + 1) / 4) * (M_PI * 2) / _pointCount + M_PI / _pointCount - (M_PI / 2);
		f64 angle = rotation;
		Vector2 current = GetVectorOnCircle(Vector2(0, 0), (sideLength * (1 / (sin((M_PI * 2) / _pointCount)))) / 2, rotation);
		_points[0] = current;
		for (size_t i = 1; i < _pointCount; i++)
		{
			angle += (M_PI * 2) / _pointCount;
			angle = fmod(angle, M_PI * 2);
			current = GetVectorOnCircle(Vector2(0, 0), (sideLength * (1 / (sin((M_PI * 2) / _pointCount)))) / 2, angle);
			_points[i] = current;
		}
		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
			size_t ind = i == _pointCount - 1 ? 0 : i + 1;
			Vector2 norm = _points[ind] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals[i] = norm;
		}
	}

	PolygonCollider::PolygonCollider() noexcept
		: _pointCount(0)
	{
	}

	PolygonCollider::PolygonCollider(
		const Vector2& a,
		const Vector2& b,
		const Vector2& c,
		std::initializer_list<Vector2> extra) noexcept
		: _pointCount(physics::Min(3 + extra.size(), MAX_POLYGONCOLLIDER_SIZE))
	{
		_points = new Vector2[_pointCount];
		_normals = new Vector2[_pointCount];
		_points[0] = a;
		_points[1] = b;
		_points[2] = c;
		auto beg = extra.begin();
		for (size_t i = 0; i < physics::Min(extra.size(), MAX_POLYGONCOLLIDER_SIZE - 3); i++)
			_points[i + 3] = *(beg++);

		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
			size_t ind = i == _pointCount - 1 ? 0 : i + 1;
			Vector2 norm = _points[ind] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals[i] = norm;
		}
		_center = Centroid(_points, _pointCount);
	}

	PolygonCollider::PolygonCollider(const std::vector<Vector2>& points)
	{
		Set(points);
	}

	PolygonCollider::~PolygonCollider() noexcept
	{
		delete[] _points;
		delete[] _normals;
	}

	bool PolygonCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return false;
		if (dynamic_cast<const PolygonCollider&>(c).GetPointCount() != _pointCount)
			return false;
		const PolygonCollider& p = dynamic_cast<const PolygonCollider&>(c);
		for (size_t i = 0; i < _pointCount; i++)
		{
			if (p.GetPoint(i) != _points[i])
				return false;
		}
		return true;
	}

	bool PolygonCollider::operator!=(const Collider& c) const noexcept
	{
		return !this->operator==(c);
	}

	PolygonCollider& PolygonCollider::operator=(const PolygonCollider& p) noexcept
	{
		delete[] _points;
		delete[] _normals;
		_pointCount = p.GetPointCount();
		_center = p.GetCenter();
		_points = new Vector2[_pointCount];
		_normals = new Vector2[_pointCount];
		for (int i = 0; i < _pointCount; i++)
			_points[i] = p.GetPoint(i);
		for (size_t i = 0; i < _pointCount; i++)
		{
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
			size_t ind = i == _pointCount - 1 ? 0 : i + 1;
			Vector2 norm = _points[ind] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals[i] = norm;
		}
		return *this;
	}
	
	PolygonCollider& PolygonCollider::operator=(PolygonCollider&& p) noexcept
	{
		_center = p.GetCenter();
		_pointCount = p.GetPointCount();
		delete[] _points;
		delete[] _normals;
		_points = p.GetVectorArray();
		_normals = p.GetNormalArray();
		p.Release();
		return *this;
	}

	BoxCollider PolygonCollider::BoundingBox(const Transform& t) const noexcept
	{
		f64 minx = std::numeric_limits<f64>::max();
		f64 miny = std::numeric_limits<f64>::max();
		f64 maxx = -std::numeric_limits<f64>::max();
		f64 maxy = -std::numeric_limits<f64>::max();
		Vector2 transCenter = t.TransformVector(_center);
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

	Collider* PolygonCollider::Clone() const noexcept
	{
		return new PolygonCollider(*this);
	}

	void PolygonCollider::ComputeMass(f64 density, f64* mass, f64* inertia) const noexcept
	{
		Vector2 c;
		f64 area = 0.0;
		f64 I = 0.0;
		const f64 kInv3 = 1.0 / 3.0;
		for (size_t i = 0; i < _pointCount; i++)
		{
			Vector2 p1(_points[i]);
			size_t i2 = i + 1 < _pointCount ? i + 1 : 0;
			Vector2 p2(_points[i2]);
			f64 D = p1.Cross(p2);
			f64 triangleArea = 0.5 * D;
			area += triangleArea;
			c += triangleArea * kInv3 * (p1 + p2);
			f64 intx2 = SQRD(p1.x) + p2.x * p1.x + SQRD(p2.x);
			f64 inty2 = SQRD(p1.y) + p2.y * p1.y + SQRD(p2.y);
			I += (0.25 * kInv3 * D) * (intx2 + inty2);
		}
		c *= 1.0 / area;
		/*for (size_t i = 0; i < _pointCount; i++)
			_points[i] -= c;*/
		//_center.Set(0, 0);
		*mass = density * area;
		*inertia = I * density;
	}

	bool PolygonCollider::Contains(const Vector2& point, const Transform& t) const noexcept
	{
		f64 x = point.x, y = point.y;
		bool inside = false;
		Vector2 p1, p2;
		for (int i = 1; i <= _pointCount; i++)
		{
			p1 = t.TransformVector(_points[i % _pointCount]);
			p2 = t.TransformVector(_points[(i + 1) % _pointCount]);
			if (y > std::min(p1.y, p2.y) && y <= std::max(p1.y, p2.y))
			{
				if (x <= std::max(p1.x, p2.x))
				{
					f64 x_inter = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
					if (p1.x == p2.x || x <= x_inter)
					{
						inside = !inside;
					}
				}
			}
		}
		return inside;
	}

	f64 PolygonCollider::CrossSectionalArea(const Vector2& direction) const noexcept
	{
		Vector2 d(direction.Normalized());
		f64 minP = std::numeric_limits<f64>::infinity(), maxP = -std::numeric_limits<f64>::infinity();
		for (size_t i = 0; i < _pointCount; i++)
		{
			f64 proj = _points[i].Dot(d);
			if (proj < minP)
				minP = proj;
			if (proj > maxP)
				maxP = proj;
		}
		return maxP - minP;
	}

	Vector2 PolygonCollider::GetCenter() const noexcept
	{
		return _center;
	}

	Vector2 PolygonCollider::GetNormal(size_t index) const
	{
		assert(index < _pointCount);
		return _normals[index];
	}

	Vector2* PolygonCollider::GetNormalArray() const noexcept
	{
		return _normals;
	}

	std::vector<Vector2> PolygonCollider::GetNormals() const noexcept
	{
		std::vector<Vector2> v(_pointCount);
		for (int i = 0; i < _pointCount; i++)
			v[i] = _normals[i];
		return v;
	}

	Vector2 PolygonCollider::GetPoint(size_t index) const
	{
		assert(index < _pointCount);
		return _points[index];
	}

	size_t PolygonCollider::GetPointCount() const noexcept
	{
		return _pointCount;
	}

	std::vector<Vector2> PolygonCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<Vector2> v(_pointCount);
		for (int i = 0; i < _pointCount; i++)
			v[i] = t.TransformVector(_points[i]);
		return v;
	}

	Vector2* PolygonCollider::GetVectorArray() const noexcept
	{
		return _points;
	}

	Vector2 PolygonCollider::Max() const noexcept
	{
		if (!_pointCount)
			return Vector2::Origin;
		return _max;
	}

	Vector2 PolygonCollider::Min() const noexcept
	{
		if (!_pointCount)
			return Vector2::Origin;
		return _min;
	}

	void PolygonCollider::Release() noexcept
	{
		_points = NULL;
		_normals = NULL;
	}

	void PolygonCollider::Set(const std::vector<Vector2>& points)
	{
		assert(points.size() > 2);
		_pointCount = physics::Min(points.size(), MAX_POLYGONCOLLIDER_SIZE);
		delete[] _points;
		delete[] _normals;
		_points = new Vector2[_pointCount];
		_normals = new Vector2[_pointCount];
		_min = Vector2::Infinity;
		_max = -Vector2::Infinity;
		for (int i = 0; i < _pointCount; i++)
		{
			_points[i] = points[i];
			_min = _points[i] < _min ? _points[i] : _min;
			_max = _points[i] > _max ? _points[i] : _max;
		}
		for (size_t i = 0; i < _pointCount; i++)
		{
			size_t ind = i == _pointCount - 1 ? 0 : i + 1;
			Vector2 norm = _points[ind] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals[i] = norm;
		}
		_center = Centroid(_points, _pointCount);
	}

	void PolygonCollider::SetPoint(size_t index, const Vector2& point) noexcept
	{
		assert(index < _pointCount);
		_min = _min > point ? point : _min;
		_max = _max < point ? point : _max;
		_points[index] = point;
		size_t prevInd = index ? index - 1 : _pointCount - 1;
		Vector2 norm = _points[index] - _points[prevInd];
		norm.Set(norm.y, -norm.x);
		norm.Normalize();
		_normals[prevInd] = norm;
		norm = _points[(index + 1) % _pointCount] - _points[index];
		norm.Set(norm.y, -norm.x);
		norm.Normalize();
		_normals[index] = norm;
		_center = Centroid(_points, _pointCount);
	}

	Vector2 PolygonCollider::SupportPoint(const Vector2& direction) const noexcept
	{
		f64 bestProj = std::numeric_limits<f64>::min();
		Vector2 bestVertex;
		for (size_t i = 0; i < _pointCount; i++)
		{
			f64 proj = _points[i].Dot(direction);
			if (proj > bestProj)
			{
				bestVertex = _points[i];
				bestProj = proj;
			}
		}
		return bestVertex;
	}
}