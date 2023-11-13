#pragma once
#include "physics/Collision/BoxCollider.hpp"
#include "physics/Collision/CollisionObject.hpp"
#include <memory>

namespace physics
{
	class Quadtree
	{
		public:
			int capacity = 0;
			std::vector<int> index;
			int level = 0;
			int maxDepth = 0;
			std::vector<CollisionObject*>* objects = nullptr;
			BoxCollider rect;
			std::unique_ptr<Quadtree> subnodes[4];
			Quadtree(int _level, int capacity, int maxDepth, const BoxCollider& rect, std::vector<CollisionObject*>* _objects = nullptr) noexcept;
			bool Contains(int id) const noexcept;
			bool ContainsBox(const BoxCollider& box) const noexcept;
			void Get(std::vector<std::vector<int>>& container) const noexcept;
			void Insert(int id) noexcept;
			void Reset(int width, int height) noexcept;
			void Retrieve(std::vector<int>& container, const BoxCollider& box) const noexcept;
			void Split() noexcept;
			void Update() noexcept;

	};
}