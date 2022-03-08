#include "../include/physics/Serializer.hpp"

namespace physics::serialization
{
	namespace Transform
	{
		Serializable* Deserialize(std::vector<unsigned char> bytes)
		{
			if (bytes.size() < TotalByteSize())
			{
				throw std::runtime_error("Insufficient amount of bytes for deserialization. ");
			}
			auto iter = bytes.begin();
			physics::Transform* t = new physics::Transform();
			f64 posX = 0;
			f64 posY = 0;
			f64 scaleX = 0;
			f64 scaleY = 0;
			//geometry::Quaternion rot;
			if (BIG_ENDIAN)
			{
				writer c = (writer )&posX;
				for (unsigned i = 0;i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer )&posY;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer )&scaleX;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer )&scaleY;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				// c = (writer)&rot;
				// for (unsigned i = 0; i < sizeof(geometry::Quaternion); i++)
				// {
				// 	c[i] = *iter;
				// 	iter++;
				// }
			}
			else
			{
				writer c = (writer )&posX;
				for (unsigned i = 0;i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1 - i] = *iter;
					iter++;
				}
				c = (writer )&posY;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1 - i] = *iter;
					iter++;
				}
				c = (writer )&scaleX;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1 -i] = *iter;
					iter++;
				}
				c = (writer )&scaleY;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1- i] = *iter;
					iter++;
				}
				// c = (writer)&rot;
				// for (unsigned i = 0; i < sizeof(geometry::Quaternion); i++)
				// {
				// 	c[sizeof(geometry::Quaternion) - 1 - i] = *iter;
				// 	iter++;
				// }
			}
			t->position.x = posX;
			t->position.y = posY;
			//t->scale.x = scaleX;
			//t->scale.y = scaleY;
			//t->rotation = rot;
			return t;
		}

		std::vector<unsigned char> Serialize(const physics::Transform& t)
		{
			std::vector<unsigned char> v;
			if (IS_BIG_ENDIAN)
			{
				reader c = (reader)&t.position.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[i]);
				c = (reader)&t.position.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[i]);
				//c = (reader)&t.scale.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[i]);
				//c = (reader)&t.scale.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[i]);
				c = (reader)&t.rotation;
				for (unsigned i = 0; i < sizeof(t.rotation); i++)
					v.push_back(c[i]);
				v.push_back(0xff);
				v.push_back(0xff);
				v.push_back(0xff);
			}
			else
			{
				reader c = (reader)&t.position.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[sizeof(f64) - 1 - i]);
				c = (reader)&t.position.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[sizeof(f64) - 1 -i]);
				//c = (reader)&t.scale.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[sizeof(f64) - 1 - i]);
				//c = (reader)&t.scale.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
					v.push_back(c[sizeof(f64) - 1 - i]);
				c = (reader)&t.rotation;
				for (unsigned i = 0; i < sizeof(t.rotation); i++)
					v.push_back(c[sizeof(t.rotation) - 1 -i]);
				v.push_back(0xff);
				v.push_back(0xff);
				v.push_back(0xff);
			}
			return v;
		}
		unsigned long TotalByteSize()
		{
			return sizeof(f64) * 4 + sizeof(geometry::Vector) + 3;
		}
	}

	namespace CircleCollider
	{
		Serializable* Deserialize(std::vector<unsigned char> bytes)
		{
			if (bytes.size() < TotalByteSize())
			{
				throw std::runtime_error("Insufficient amount of bytes for deserialization. ");
			}
			physics::CircleCollider* cC = new physics::CircleCollider();
			if (BIG_ENDIAN)
			{
				writer c = (writer)&cC->center.x;
				auto iter = bytes.begin();
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer)&cC->center.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer)&cC->radius;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
			}
			else
			{
				writer c = (writer)&cC->center.x;
				auto iter = bytes.begin();
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer)&cC->center.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer)&cC->radius;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
			}
			return cC;
		}

		std::vector<unsigned char> Serialize(const physics::CircleCollider& c)
		{
			std::vector<unsigned char> v;
			reader C = (reader)&c.center.x;
			for (unsigned i = 0; i < sizeof(f64); i++)
			{
				v.push_back(C[i]);
			}
			C = (reader)&c.center.y;
			for (unsigned i = 0; i < sizeof(f64); i++)
			{
				v.push_back(C[i]);
			}
			C = (reader)&c.radius;
			for (unsigned i = 0; i < sizeof(f64); i++)
			{
				v.push_back(C[i]);
			}
			v.push_back(0xff);
			v.push_back(0xff);
			v.push_back(0xff);
			return v;
		}

		unsigned long TotalByteSize()
		{
			return sizeof(f64) * 3 + 3;
		}
	}

	namespace PolygonCollider
	{
		Serializable* Deserialize(std::vector<unsigned char> bytes)
		{
			physics::PolygonCollider* d = new physics::PolygonCollider();
			if (BIG_ENDIAN)
			{
				auto iter = bytes.begin();
				writer c = (writer)&d->pos.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				c = (writer)&d->pos.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				f64 size = 0;
				c = (writer)&size;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[i] = *iter;
					iter++;
				}
				for (unsigned i = 0; i < size; i++)
				{
					geometry::Vector tmp;
					c = (writer)&tmp.x;
					for (unsigned j = 0; j < sizeof(f64); j++)
					{
						c[j] = *iter;
						iter++;
					}
					c = (writer)&tmp.y;
					for (unsigned j = 0; j < sizeof(f64); j++)
					{
						c[j] = *iter;
						iter++;
					}
					d->points.push_back(tmp);
				}
			}
			else
			{
				auto iter = bytes.begin();
				writer c = (writer)&d->pos.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1 - i] = *iter;
					iter++;
				}
				c = (writer)&d->pos.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1 - i] = *iter;
					iter++;
				}
				f64 size = 0;
				c = (writer)&size;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					c[sizeof(f64) - 1 - i] = *iter;
					iter++;
				}
				for (unsigned i = 0; i < size; i++)
				{
					geometry::Vector tmp;
					c = (writer)&tmp.x;
					for (unsigned j = 0; j < sizeof(f64); j++)
					{
						c[sizeof(f64) - 1 - j] = *iter;
						iter++;
					}
					c = (writer)&tmp.y;
					for (unsigned j = 0; j < sizeof(f64); j++)
					{
						c[sizeof(f64) - 1 - j] = *iter;
						iter++;
					}
					d->points.push_back(tmp);
				}
			}
			return d;
		}

		std::vector<unsigned char> Serialize(const physics::PolygonCollider& d)
		{
			std::vector<unsigned char> v;
			if (BIG_ENDIAN)
			{
				reader c = (reader)&d.pos.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					v.push_back(c[i]);
				}
				c = (reader)&d.pos.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					v.push_back(c[i]);
				}
				f64 db = v.size();
				c = (reader)&db;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					v.push_back(c[i]);
				}
				for (const geometry::Vector& vecin d.points)
				{
					c = (reader)&vec.x;
					for (unsigned i = 0; i < sizeof(f64); i++)
					{
						v.push_back(c[i]);
					}
					c = (reader)&vec.y;
					for (unsigned i = 0; i < sizeof(f64); i++)
					{
						v.push_back(c[i]);
					}
				}
				v.push_back(0xff);
				v.push_back(0xff);
				v.push_back(0xff);
			}
			else
			{
				reader c = (reader)&d.pos.x;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					v.push_back(c[sizeof(f64) - 1 - i]);
				}
				c = (reader)&d.pos.y;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					v.push_back(c[sizeof(f64) - 1 - i]);
				}
				f64 db = v.size();
				c = (reader)&db;
				for (unsigned i = 0; i < sizeof(f64); i++)
				{
					v.push_back(c[sizeof(f64) - 1 - i]);
				}
				for (const geometry::Vector& vecin d.points)
				{
					c = (reader)&vec.x;
					for (unsigned i = 0; i < sizeof(f64); i++)
					{
						v.push_back(c[sizeof(f64) - 1 - i]);
					}
					c = (reader)&vec.y;
					for (unsigned i = 0; i < sizeof(f64); i++)
					{
						v.push_back(c[sizeof(f64) - 1 - i]);
					}
				}
				v.push_back(0xff);
				v.push_back(0xff);
				v.push_back(0xff);
			}
			return v;
		}
		unsigned long TotalByteSize(const physics::PolygonCollider& d)
		{
			return (sizeof(f64) * 2) + (d.points.size() * sizeof(f64)) + 4;
		}
	}
}