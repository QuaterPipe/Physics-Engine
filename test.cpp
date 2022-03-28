#include "src/include/physics/Tools/Notify.hpp"
#include <iostream>
using namespace physics;

struct name
{
	std::string first;
	std::string last;
};
class bruh: public Notifiable
{
	public:
		name x;
		Property<name> xRef;
		bruh() : xRef(*this){
		}
};

int main()
{
	bruh b;
	std::string n = b.operator ->first;
}