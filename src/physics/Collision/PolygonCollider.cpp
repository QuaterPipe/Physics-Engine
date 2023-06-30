#include "physics/Collision/Collision.hpp"
namespace physics
{
	PolygonCollider::PolygonCollider(const BoxCollider& b) noexcept
	: pos(0, 0), _points(4)
	{
		_points[0] = b.pos;
		_points[1] = geo::Vector2(b.x + b.width, b.y);
		_points[2] = geo::Vector2(b.x + b.width, b.y + b.height);
		_points[3] = geo::Vector2(b.x , b.y + b.height);
		for (size_t i = 0; i < _points.size(); i++)
		{
			geo::Vector2 norm = _points[(i + 1) % _points.size()] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals.push_back(norm);
		}
	}

	PolygonCollider::PolygonCollider(const PolygonCollider& p) noexcept
	{
		pos = p.pos;
		_points = p._points;
		for (size_t i = 0; i < _points.size(); i++)
		{
			geo::Vector2 norm = _points[(i + 1) % _points.size()] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals.push_back(norm);
		}
	}

	PolygonCollider::PolygonCollider(const geo::Vector2& pos,
		f64 sideLength, unsigned long count	
	) noexcept : pos(pos)
	{
		if (count < 3 || sideLength < 0)
			return;
		f64 rotation = floor((count + 1) / 4) * (M_PI * 2) / count + M_PI / count - (M_PI / 2);
		f64 angle = rotation;
		geo::Vector2 current = geo::GetVectorOnCircle(geo::Vector2(0, 0), (sideLength * (1 / (sin((M_PI * 2) / count)))) / 2, rotation);
		_points.push_back(current);
		for (ulong i = 1; i < count; i++)
		{
			angle += (M_PI * 2) / count;
			angle = fmod(angle, M_PI * 2);
			current = geo::GetVectorOnCircle(geo::Vector2(0, 0), (sideLength * (1 / (sin((M_PI * 2) / count)))) / 2, angle);
			_points.push_back(current);
		}
		for (size_t i = 0; i < _points.size(); i++)
		{
			geo::Vector2 norm = _points[(i + 1) % _points.size()] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals.push_back(norm);
		}
	}

	PolygonCollider::PolygonCollider()
	{
	}

	PolygonCollider::PolygonCollider(const geo::Vector2& pos,
		const geo::Vector2& a,
		const geo::Vector2& b,
		const geo::Vector2& c,
		std::initializer_list<geo::Vector2> extra) noexcept
	{
		this->pos = pos;
		_points = {extra};
		_points.insert(_points.begin(), c);
		_points.insert(_points.begin(), b);
		_points.insert(_points.begin(), a);
		for (size_t i = 0; i < _points.size(); i++)
		{
			geo::Vector2 norm = _points[(i + 1) % _points.size()] - _points[i];
			norm.Set(norm.y, -norm.x);
			norm.Normalize();
			_normals.push_back(norm);
		}
	}

	PolygonCollider::~PolygonCollider() noexcept {}

	bool PolygonCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return false;
		return pos == dynamic_cast<const PolygonCollider&>(c).pos && _points == dynamic_cast<const PolygonCollider&>(c).GetPoints();
	}

	bool PolygonCollider::operator!=(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return true;
		return pos != dynamic_cast<const PolygonCollider&>(c).pos || _points != dynamic_cast<const PolygonCollider&>(c).GetPoints();
	}
	
	BoxCollider PolygonCollider::BoundingBox(const Transform& t) const noexcept
	{
		f64 minx = std::numeric_limits<f64>::max();
		f64 miny = std::numeric_limits<f64>::max();
		f64 maxx = std::numeric_limits<f64>::min();
		f64 maxy = std::numeric_limits<f64>::min();
		for (geo::Vector2 p: _points)
		{
			geo::Vector2 tp = t.TransformVector(p);
			minx = tp.x < minx ? tp.x : minx;
			miny = tp.y < miny ? tp.y : miny;
			maxx = tp.x > maxx ? tp.x : maxx;
			maxy = tp.y > maxy ? tp.y : maxy;
		}
		BoxCollider result;
		result.pos.Set(minx, miny);
		result.dimensions.Set(maxx - minx, maxy - miny);
		return result;
	}

	geo::Vector2 GetCentroid(std::vector<geo::Vector2> points)
	{
		if (points.size())
		{
			geo::Vector2 first = points.at(0);
			geo::Vector2 last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geo::Vector2 p1, p2;
			// absolutely no clue what this does, it just works lol
			for (size_t i = 0, j = points.size() - 1; i < points.size(); j=i++)
			{
				p1 = points[i]; p2 = points[j];
				f = (p1.y - first.y) * (p2.x - first.x) - (p2.y - first.y) * (p1.x - first.x);
				twiceArea += f;
				x += (p1.x + p2.x - 2 * first.x) * f;
				y += (p1.y + p2.y - 2 * first.y) * f;
			}
			f = twiceArea * 3;
			return geo::Vector2(x / f + first.x, y / f + first.y);
		}
		else
			return geo::Vector2::Origin;
	}

	geo::Vector2 PolygonCollider::GetCenter() const noexcept
	{
		return GetCentroid(_points);
	}

	geo::Vector2 PolygonCollider::GetPoint(size_t index) const
	{
		assert(index < _points.size());
		return _points[index];
	}

	std::vector<geo::Vector2> PolygonCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector2> v;
		for (geo::Vector2 vec: _points)
			v.push_back(t.TransformVector(vec));
		return v;
	}

	geo::Vector2 PolygonCollider::GetNormal(size_t index) const
	{
		assert(index < _normals.size());
		return _normals[index];
	}

	std::vector<geo::Vector2> PolygonCollider::GetNormals() const noexcept
	{
		return _normals;
	}

	Collider* PolygonCollider::Clone() const noexcept
	{
		return new PolygonCollider(*this);
	}
	
	geo::Vector2 PolygonCollider::Max() const noexcept
	{
		if (!_points.size())
			return geo::Vector2::Origin;
		return *std::max(_points.begin(), _points.end()) + pos;
	}

	geo::Vector2 PolygonCollider::Min() const noexcept
	{
		if (!_points.size())
			return geo::Vector2::Origin;
		return *std::min(_points.begin(), _points.end()) + pos;
	}

	geo::Vector2 PolygonCollider::SupportPoint(geo::Vector2 direction) const noexcept
	{
		f64 bestProj = std::numeric_limits<f64>::min();
		geo::Vector2 bestVertex;
		for (size_t i = 0; i < _points.size(); i++)
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
