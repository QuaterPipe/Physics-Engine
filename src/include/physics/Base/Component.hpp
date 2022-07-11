#pragma once
#include <array>
#include <cstring>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include  <memory>
#include <new>
#include "../../geometry/main.hpp"
#include "Hashable.hpp"
namespace physics
{	
	/*template <typename T>
	class Component : public Hashable
	{
		public:
			const char* const Tag = NULL;
			int64_t id = 0;
			Component();
			Component(int64_t id);
			Component(const Component& c);
			virtual Component* Clone() const noexcept;
			template <const char* tag>
			bool InheritsFrom();
	};*/

	template <typename T, typename ObjBase, typename CompBase, u64 MAX_COMPONENTS>
	struct ComponentManager
	{
		public:
			size_t maxComponents;
			size_t maxObjects;
			struct Object;
			struct ComponentTraits
			{
				u64 num = 0;
				ComponentTraits(u64 n) noexcept: num(n) {}
				inline void AddTrait(u64 n) noexcept
				{
					num |= n;
				}
				inline bool HasTrait(u64 n) const noexcept
				{
					return num & n;
				}
			};

			struct ComponentType
			{
				u64 id;
				size_t size;
				template <typename C>
				static inline ComponentType From(ComponentManager* system, u64 id)
				{
					ComponentType result;
					result.id = id;
					result.size = sizeof(C);
					return result;
				}
			};

			struct BaseComponent : public CompBase
			{
				public:
					static u64 id;
					static ComponentType type;
					ComponentTraits traits;
					BaseComponent() = default;
					BaseComponent(const BaseComponent& other) = delete;
					BaseComponent(BaseComponent&& other) = default;
					BaseComponent& operator=(const BaseComponent& other) = delete;
					BaseComponent& operator=(BaseComponent&& other) = default;
					virtual ~BaseComponent() = default;
					virtual void Init() {};
					virtual void Destroy() {};
					inline u64 GetId() const noexcept
					{ return id; }
					virtual bool operator==(const BaseComponent& rhs) const noexcept {return true;}
					virtual bool operator!=(const BaseComponent& rhs) const noexcept {return false;}
			};

			struct ComponentContainer
			{
				public:
					ComponentContainer() = delete;
					ComponentContainer(Object* parent) noexcept
					: parent(parent) {}
					ComponentContainer(const ComponentContainer& c) = delete;
					ComponentContainer(ComponentContainer&& c) = default;
	
	
					template <typename C>
					C& Add(C&& component) noexcept
					{
						static_assert(!Has<C>());
						this->components.push_back(parent->parent->components[C::id].Append());
						return *this->components[this->components.size() - 1];
					}

					void DestroyAll() noexcept
					{
						for (BaseComponent* c: this->components)
							c->Destroy();
						components.clear();
					}
	
					template <typename C>
					bool Has() const noexcept
					{
						for (BaseComponent* c: this->components)
						{
							if (c->GetId() == C::id)
								return true;
						}
						return false;
					}
	
					template <typename C>
					C& Get() const noexcept
					{
						static_assert(std::is_base_of<BaseComponent, C>());
						static_assert(Has<C>);
						for (BaseComponent* c: this->components)
						{
							if (dynamic_cast<C*>(c))
								return *c;
						}
					}
	
					template <typename C>
					C* Opt() const noexcept
					{
						if (!Has<C>())
							return NULL;
						return &Get<C>();
					}
	
					template <typename C>
					void Remove() noexcept
					{
						if (!Has<C>())
							return;
						for (BaseComponent* c: this->components)
						{
							if (c->GetId() == C::id)
								parent->parent->components[C::id].Remove();
						}
					}

					std::vector<BaseComponent*> components;
					Object* parent;
			};

			struct Object : public ObjBase
			{
				public:
					u64 id = 0;
					Object() = default;
					Object(u64 id, ComponentManager* parent)
					: id(id), parent(parent) {}
					inline operator u64() const noexcept {
						return id;
					}
		
					inline bool operator==(const Object& rhs) const noexcept
					{
						return id == rhs.id;
					}
		
					inline bool operator!=(const Object& rhs) const noexcept
					{
						return id != rhs.id;
					}
				
					template <typename C>
					inline C& Get() const noexcept
					{
						return comp.template Get<C>();
					}
		
					template <typename C>
					inline C* Opt() const noexcept
					{
						return comp.template Opt<C>();
					}
		
					template <typename C>
					inline bool Has() const noexcept
					{
						return comp.template Has<C>();
					}
		
					template <typename C>
					inline C& Add(C&& component = C()) const noexcept
					{
						static_assert(std::is_base_of<BaseComponent, C>());
						return comp.template Add<C>(*this, std::forward<C>(component));
					}
		
					template <typename C>
					inline void Remove() const noexcept
					{
						comp.template Remove<C>();
					}
		
					template <typename _ = T>
					inline void Destroy() const noexcept
					{
						comp.DestroyAll();
					}
				private:
					ComponentContainer comp;
					ComponentManager* parent = nullptr;
			};

			struct ComponentArray
			{
				struct Block
				{
					const ComponentArray* parent = nullptr;
					std::unique_ptr<u8[]> data = nullptr;
					Block() = default;
					
					Block(const ComponentArray* parent, size_t n) noexcept
					: parent(parent)
					{
						const u32 data_size = parent->blockSize * parent->type->size;
						data = std::unique_ptr<u8[]>(
							new (std::align_val_t(16)) u8[data_size]);
						std::memset(data.get(), 0, data_size);
					}
	
					inline BaseComponent* operator[](size_t i) const noexcept
					{
						static_assert(i < parent->blockSize);
						return
							reinterpret_cast<BaseComponent*>(
								reinterpret_cast<u8*>(data.get())
								+ (parent->type->size * i));
					}
	
					inline size_t ToIndex(const void* p) const noexcept
					{
						return static_cast<size_t>(
							(reinterpret_cast<const u8*>(p) - 
							reinterpret_cast<const u8*>(data.get()))
							/ parent->type->size);
					}
				};
	
				ComponentType* type = nullptr;
				std::vector<Block> blocks;
				u32 blockSize = 256;
				u32 maxObjects = 1000;
				u32 numBlocks = (maxObjects / blockSize) + 1;
				size_t n_blocks = 0;
				ComponentArray(ComponentType* type, u32 maxObects)
				: type(type), maxObjects(maxObjects), blockSize(type->size)
				{
					blocks.resize(maxObjects);
				}
	
				void Resize(size_t n) noexcept
				{
					while (n > (n_blocks * blockSize))
					{
						static_assert(n_blocks != numBlocks);
						blocks[n_blocks] = Block(this, n_blocks);
						n_blocks++;
					}
				}
	
				inline u32 block(size_t i) const noexcept
				{
					return i / blockSize;
				}
	
				inline BaseComponent* operator[](size_t i) const noexcept
				{
					return blocks[i / blockSize][i % blockSize];
				}

				template <typename C>
				C* Append(C&& component) noexcept
				{
					static_assert(sizeof(component) <= blockSize);
					if (freedBlocks.size())
					{
						auto p = freedBlocks.front();
						freedBlocks.pop();
						C& b = *p;
						std::move(component, b);
						return p;
					}
					C& b = blocks[index];
					std::move(component, b);
					return blocks[index++]; 
				}

				void Remove(const void* p) noexcept
				{
					for (size_t i = 0; i < n_blocks; i++)
					{
						if (&blocks[i] == p)
						{
							std::memset(blocks[i].data.get(), 0, blockSize);
							return;
						}
					}
				}
	
				inline size_t ToIndex(const void* p) const noexcept
				{
					const auto* c = reinterpret_cast<const BaseComponent*>(p);
					return size_t(c->block * blockSize)
						+ blocks[c->block].ToIndex(p);
				}
				private:
					std::queue<void*> freedBlocks;
					u32 index = 0;
			};
			i64 objIds = 0;
			size_t n_components = 0;
			size_t size = 0;
			std::vector<u64> signatures;
			std::array<ComponentArray, MAX_COMPONENTS> components;
			std::array<ComponentType, MAX_COMPONENTS> componentTypes;
			std::unordered_map<ComponentType, BaseComponent*> componentMap;
			ComponentManager() {}
			ComponentManager(const ComponentManager& other) = delete;
			ComponentManager(ComponentManager&& other) = default;
			ComponentManager& operator=(const ComponentManager& other) = delete;
			ComponentManager& operator=(ComponentManager&& other) = default;
			virtual ~ComponentManager() = default;

			virtual Object CreateObject() noexcept
			{
				return Object(objIds++, this);
			}

			template <typename C>
			void RegisterType() noexcept
			{
				const u64 id = n_components++;
				static_assert(n_components <= MAX_COMPONENTS, "Maximum amount of components reached.");
				C::id = id;
				componentTypes[id] = ComponentType::template From<C>(this, id);
				C::type = componentTypes[id];
					componentMap[componentTypes[id]] = new C();
				components[id] = ComponentArray(&componentTypes[id]);
				components[id].Resize(size);
			}
	};
}