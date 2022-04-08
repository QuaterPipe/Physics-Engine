"src/include/geometry/main.hpp"
using namespace geometry;
struct MassPoint
{
	geometry::Vector position;
	geometry::Vector velocity;
	geometry::Vector force;
	f64 mass = 0;
	MassPoint() {};
	MassPoint(geometry::Vector position, geometry::Vector velocity, geometry::Vector force, f64 mass) noexcept;
};


int main()
{
	std::vector<MassPoint> m;
	for (int i = 0; i < 60; i++)
		m.push_back(MassPoint());
	return 0;
}