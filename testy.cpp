//#pragma once
#include <array>
#include <cstring>
#include <iostream>
#include <optional>
#include <queue>
#include <string>
#include <map>
#include <memory>
#include <new>
#include <cstdint>
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

template <u64 MAX_COMPONENTS>
struct ComponentManager
{
	public:
		size_t maxComponents;
		size_t maxObjects;
		struct Object;
		struct ComponentTraits
		{
			u64 num = 0;
			ComponentTraits() {}
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
			inline bool operator<(const ComponentType& c) const noexcept
			{return id < c.id;}
			inline bool operator>(const ComponentType& c) const noexcept
			{return id > c.id;}
			inline bool operator==(const ComponentType& c) const noexcept
			{return id == c.id;}
			inline bool operator!=(const ComponentType& c) const noexcept
			{return id != c.id;}
		};

		struct BaseComponent// : public CompBase
		{
			public:
				inline static u64 id;
				inline static ComponentType type;
				ComponentTraits traits;
				BaseComponent() {};
				BaseComponent(const BaseComponent& other) = delete;
				BaseComponent(BaseComponent&& other) = default;
				BaseComponent& operator=(const BaseComponent& other) = delete;
				BaseComponent& operator=(BaseComponent&& other) = default;
				virtual ~BaseComponent() = default;
				virtual void Init() {};
				void Destroy() {};
				inline u64 GetId() const noexcept
				{ return id; }
				virtual bool operator==(const BaseComponent& rhs) const noexcept {return true;}
				virtual bool operator!=(const BaseComponent& rhs) const noexcept {return false;}
		};

		struct ComponentContainer
		{
			public:
				ComponentContainer() = default;
				ComponentContainer(Object* parent) noexcept
				: parent(parent) {}
				ComponentContainer(const ComponentContainer& c) = delete;
				ComponentContainer(ComponentContainer&& c) = default;
				virtual ~ComponentContainer() {DestroyAll();}
				template <typename C>
				C& Add(C&& component) noexcept
				{
					if (Has<C>())
						return component;
					this->components.push_back(parent->parent->components[C::id].template Append<C>(std::forward<C>(component)));
					return (C&)(*this->components[this->components.size() - 1]);
				}

				void DestroyAll() noexcept
				{
					for (BaseComponent* c: this->components)
					{
						c->Destroy();
						this->parent->parent->components[c->GetId()].Remove(c);
					}
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
				C& Get() const
				{
					for (BaseComponent* c: this->components)
					{
						if (c->GetId() == C::id)
							return (C&)(*c);
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
					size_t i = 0;
					for (BaseComponent* c: this->components)
					{
						if (c->GetId() == C::id)
						{
							c->Destroy();
							parent->parent->components[C::id].Remove(c);
							components.erase(components.begin() + i);
						}
						i++;
					}
				}

				std::vector<BaseComponent*> components;
				Object* parent;
		};

		struct Object// : public ObjBase
		{
			public:
				u64 id = 0;
				Object() = default;
				Object(u64 id, ComponentManager* parent)
				: id(id), comp(this), parent(parent) {}

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
				inline C& Add(C&& component = C()) noexcept
				{
					static_assert(std::is_base_of<BaseComponent, C>());
					return comp.template Add<C>(std::forward<C>(component));
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
				inline void Remove() noexcept
				{
					comp.template Remove<C>();
				}

				inline void Destroy() noexcept
				{
					comp.DestroyAll();
					parent = nullptr;
					id = 0;
				}

				ComponentContainer comp;
				ComponentManager* parent = nullptr;
		};

		struct ComponentArray
		{
			struct Block
			{
				const ComponentArray* parent = nullptr;
				std::unique_ptr<u8[]> data;
				Block() = default;

				// Block (const Block& block)
				// : parent(block.parent)
				// {
					
				// }

				Block(Block&& block)
				: parent(block.parent), data(block.data.release())
				{
				}

				Block& operator=(Block&& block)
				{
					delete[] data.release();
					data.reset(block.data.release());
					parent = block.parent;
					return *this;
				}
				
				Block(const ComponentArray* parent, size_t n) noexcept
				: parent(parent)
				{
					const u32 data_size = parent->blockSize * parent->type->size;
					data = std::unique_ptr<u8[]>(
						new (std::align_val_t(16)) u8[data_size]);
					std::memset(data.get(), 0, data_size);
				}

				virtual ~Block()
				{
					if (!parent)
						return;
					delete[] data.release();
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
			u32 maxObjects = 10000;
			u32 blockSize = 256;
			u32 numBlocks = (maxObjects / blockSize) + 1;
			size_t n_blocks = 0;
			std::queue<const void*> freedBlocks;
			u32 index = 0;
			ComponentArray() noexcept {};
			ComponentArray(ComponentType* type, u32 maxObjects) noexcept
			: type(type), maxObjects(maxObjects), blockSize(type->size)
			{
				blocks.resize(maxObjects);
			}

			ComponentArray& operator=(ComponentArray&& c) noexcept
			{
				blocks.clear();
				for (Block& b: c.blocks)
					blocks.push_back(std::move(b));
				maxObjects = c.maxObjects;
				blockSize = c.blockSize;
				numBlocks = c.numBlocks;
				n_blocks = c.n_blocks;
				type = c.type;
				freedBlocks = c.freedBlocks;
				index = c.index;
				return *this;
			}

			virtual ~ComponentArray()
			{
				for (Block& b: blocks)
					b.~Block();
			}

			void Resize(size_t n) noexcept
			{
				while (n > (n_blocks * blockSize))
				{
					//static_assert(n_blocks != numBlocks);
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
				//static_assert(sizeof(component) <= blockSize);
				if (freedBlocks.size())
				{
					auto p = freedBlocks.front();
					freedBlocks.pop();
					C& b = *((C*)p);
					b = std::move(component);
					return (C*)p;
				}
				C& b = (C&)blocks[index];
				b = std::move(component);
				return (C*)&blocks[index++]; 
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
				freedBlocks.push(p);
			}

			inline size_t ToIndex(const void* p) const noexcept
			{
				const auto* c = reinterpret_cast<const BaseComponent*>(p);
				return size_t(c->block * blockSize)
					+ blocks[c->block].ToIndex(p);
			}
		};
		i64 objIds = 0;
		size_t n_components = 0;
		size_t size = 0;
		std::vector<u64> signatures;
		std::array<ComponentArray, MAX_COMPONENTS> components;
		std::array<ComponentType, MAX_COMPONENTS> componentTypes;

		ComponentManager() noexcept
		{
		}

		ComponentManager(const ComponentManager& other) = delete;
		ComponentManager(ComponentManager&& other) = default;
		ComponentManager& operator=(const ComponentManager& other) = delete;
		ComponentManager& operator=(ComponentManager&& other) = default;

		virtual Object CreateObject() noexcept
		{
			return Object(objIds++, this);
		}

		template <typename C>
		void RegisterType(u32 maxObjects = 10000) noexcept
		{
			const u64 id = n_components++;
			C::id = id;
			componentTypes[id] = ComponentType::template From<C>(this, id);
			C::type = componentTypes[id];
			components[id] = ComponentArray(&componentTypes[id], maxObjects);
			components[id].Resize(size);
		}
};

using BaseComponent = ComponentManager<100>::BaseComponent;
struct Thing : public BaseComponent
{
	Thing() {}
	int i;
	void PlusPlus() {i++;}
};

struct Thing2 : public BaseComponent
{
	Thing2() {}
	int i = -20;
	double thingyy[1000];
	void MinusMinus() {i--;}
};

int main()
{
	ComponentManager<100> compManager;
	compManager.RegisterType<Thing>();
	compManager.RegisterType<Thing2>();
	using Object = ComponentManager<100>::Object;
	Object o = compManager.CreateObject();
	o.Add<Thing>(Thing());
	o.Add<Thing2>(Thing2());
	//o.Remove<Thing>();
	o.Get<Thing2>().MinusMinus();
	o.Get<Thing2>().i = 100;
	for (int i = 0; i < 100; i++)
	{
		o.Get<Thing2>().MinusMinus();
	}
	o.Destroy();
	return 0;
}