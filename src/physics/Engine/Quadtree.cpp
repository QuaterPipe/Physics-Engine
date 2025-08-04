#include "physics/Engine/Quadtree.hpp"
#include <iostream>

namespace physics
{
	Quadtree::Quadtree(int level, int capacity, int maxDepth, const BoxCollider& rect, std::vector<CollisionObject*>* objects) noexcept
		: capacity(capacity), level(level), maxDepth(maxDepth), rect(rect), subnodes{nullptr, nullptr, nullptr, nullptr}, objects(objects)
	{
		index.reserve(capacity);
	}

	bool Quadtree::Contains(int id) const noexcept
	{
		Transform& trans = (*objects)[id]->transform;
		BoxCollider b = (*objects)[id]->GetCollider().BoundingBox(trans);
		return ContainsBox(b);
	}

	bool Quadtree::ContainsBox(const BoxCollider& box) const noexcept
	{
		return rect.Overlaps(box);
	}

	void Quadtree::Get(std::vector<std::vector<int>>& container) const noexcept
	{
		if (subnodes[0] != nullptr)
		{
			subnodes[0]->Get(container);
			subnodes[1]->Get(container);
			subnodes[2]->Get(container);
			subnodes[3]->Get(container);
			return;
		}
		if (index.size())
			container.emplace_back(index);
	}

	void Quadtree::Insert(int id) noexcept
	{
		if (!Contains(id))
			return;
		if (subnodes[0] != nullptr)
		{
			if (subnodes[0]->Contains(id)) subnodes[0]->Insert(id);
			if (subnodes[1]->Contains(id)) subnodes[1]->Insert(id);
			if (subnodes[2]->Contains(id)) subnodes[2]->Insert(id);
			if (subnodes[3]->Contains(id)) subnodes[3]->Insert(id);
			return;
		}
		index.emplace_back(id);
		if (index.size() > capacity && level < maxDepth)
		{
			Split();
			for (const auto& index : index)
			{
				for (const auto& subnode : subnodes)
				{
					if (subnode->Contains(index))
						subnode->Insert(index);
				}
			}
			index.clear();
			index.shrink_to_fit();
		}
	}

	void Quadtree::Reset(int width, int height) noexcept
	{
		rect = BoxCollider(width, height);
		index.clear();
		index.shrink_to_fit();
		subnodes[0] = nullptr;
		subnodes[1] = nullptr;
		subnodes[2] = nullptr;
		subnodes[3] = nullptr;
	}

	void Quadtree::Retrieve(std::vector<int>& container, const BoxCollider& rect) const noexcept
	{
		if (subnodes[0] != nullptr)
		{
			if (subnodes[0]->ContainsBox(rect)) subnodes[0]->Retrieve(container, rect);
			if (subnodes[1]->ContainsBox(rect)) subnodes[1]->Retrieve(container, rect);
			if (subnodes[2]->ContainsBox(rect)) subnodes[2]->Retrieve(container, rect);
			if (subnodes[3]->ContainsBox(rect)) subnodes[3]->Retrieve(container, rect);
			return;
		}
		for (const auto& index : index)
			container.emplace_back(index);
	}

	void Quadtree::Split() noexcept
	{
		Vector2 center = rect.GetCenter();
		f64 x = center.x;
		f64 y = center.y;
		f64 width = rect.width * 0.5;
		f64 height = rect.height * 0.5;

		f64 w = width * 0.5;
		f64 h = height * 0.5;

		BoxCollider SW(Vector2(x - w, y - h), Vector2(width, height));
		BoxCollider SE(Vector2(x + w, y - h), Vector2(width, height));
		BoxCollider NW(Vector2(x - w, y + h), Vector2(width, height));
		BoxCollider NE(Vector2(x + w, y + h), Vector2(width, height));
		subnodes[0] = std::make_unique<Quadtree>(level + 1, capacity, maxDepth, SW, objects);
		subnodes[1] = std::make_unique<Quadtree>(level + 1, capacity, maxDepth, SE, objects);
		subnodes[2] = std::make_unique<Quadtree>(level + 1, capacity, maxDepth, NW, objects);
		subnodes[3] = std::make_unique<Quadtree>(level + 1, capacity, maxDepth, NE, objects);
	}

	void Quadtree::Update() noexcept
	{
		index.clear();
		index.shrink_to_fit();
		subnodes[0].reset(nullptr);
		subnodes[1].reset(nullptr);
		subnodes[2].reset(nullptr);
		subnodes[3].reset(nullptr);
		f64 minx = DBL_MAX, miny = DBL_MAX, maxx = DBL_MIN, maxy = DBL_MIN;
		for (size_t i = 0; i < objects->size(); i++)
		{
			Collider& c = (*objects)[i]->GetCollider();
			BoxCollider bound = c.BoundingBox((*objects)[i]->transform);
			Vector2 minV = bound.Min();
			Vector2 maxV = bound.Max();
			maxx = Max(maxx, maxV.x);
			maxy = Max(maxy, maxV.y);
			minx = Min(minx, minV.x);
			miny = Min(miny, minV.y);
		}
		rect = BoxCollider(Vector2((maxx - minx) * 0.5 + minx, (maxy - miny) * 0.5 + miny), Vector2(maxx - minx, maxy - miny));
		for (size_t i = 0; i < objects->size(); i++)
			Insert(i);
	}
}
