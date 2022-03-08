all: compilePhysics link run
debug: debugCompileObjects debugLink run

checkSyntax:
	g++ -fsyntax-only -std=c++17 src/physics/*.cpp
	g++ -fsyntax-only -std=c++17 src/geometry/*.cpp
	g++ -fsyntax-only -std=c++17 src/main.cpp

compileObjects:
	g++ -c -Wall -std=c++17 src/physics/Algo.cpp -o bin/o/Algo.o
	g++ -c -Wall -std=c++17 src/physics/Archive.cpp -o bin/o/Archive.o
	g++ -c -Wall -std=c++17 src/physics/BoxCollider.cpp -o bin/o/BoxCollider.o
	g++ -c -Wall -std=c++17 src/physics/CircleCollider.cpp -o bin/o/CircleCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision.cpp -o bin/o/Collision.o
	g++ -c -Wall -std=c++17 src/physics/Collider.cpp -o bin/o/Collider.o
	g++ -c -Wall -std=c++17 src/physics/CollisionObject.cpp -o bin/o/CollisionObject.o
	g++ -c -Wall -std=c++17 src/physics/DataPacket.cpp -o bin/o/DataPacket.o
	g++ -c -Wall -std=c++17 src/physics/Display.cpp -o bin/o/Display.o
	g++ -c -Wall -std=c++17 src/physics/Entity.cpp -o bin/o/Entity.o
	g++ -c -Wall -std=c++17 src/physics/Hashable.cpp -o bin/o/Hashable.o
	g++ -c -Wall -std=c++17 src/physics/MeshCollider.cpp -o bin/o/MeshCollider.o
	g++ -c -Wall -std=c++17 src/physics/OstreamOverloads.cpp -o bin/o/OstreamOverloads.o
	g++ -c -Wall -std=c++17 src/physics/PolygonCollider.cpp -o bin/o/PolygonCollider.o
	g++ -c -Wall -std=c++17 src/physics/Rigidbody.cpp -o bin/o/Rigidbody.o
	g++ -c -Wall -std=c++17 src/physics/Scene.cpp -o bin/o/Scene.o
	g++ -c -Wall -std=c++17 src/physics/Time.cpp -o bin/o/Time.o
	g++ -c -Wall -std=c++17 src/physics/Transform.cpp -o bin/o/Transform.o
	g++ -c -Wall -std=c++17 src/physics/World.cpp -o bin/o/World.o
	g++ -c -Wall -std=c++17 src/geometry/Curve.cpp -o bin/o/Curve.o
	g++ -c -Wall -std=c++17 src/geometry/Line.cpp -o bin/o/Line.o
	g++ -c -Wall -std=c++17 src/geometry/Math.cpp -o bin/o/Math.o
	g++ -c -Wall -std=c++17 src/geometry/Matrix.cpp -o bin/o/Matrix.o
	g++ -c -Wall -std=c++17 src/geometry/Vector.cpp -o bin/o/Vector.o
	ar rcs lib/libphysics.a bin/o/*.o
	ranlib lib/libphysics.a
	rm bin/o/*.o

debugCompileObjects:
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Algo.cpp -o bin/o/Algo.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/BoxCollider.cpp -o bin/o/BoxCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/CircleCollider.cpp -o bin/o/CircleCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision.cpp -o bin/o/Collision.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collider.cpp -o bin/o/Collider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/CollisionObject.cpp -o bin/o/CollisionObject.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Display.cpp -o bin/o/Display.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Entity.cpp -o bin/o/Entity.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Hashable.cpp -o bin/o/Hashable.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/MeshCollider.cpp -o bin/o/MeshCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/OstreamOverloads.cpp -o bin/o/OstreamOverloads.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/PolygonCollider.cpp -o bin/o/PolygonCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Rigidbody.cpp -o bin/o/Rigidbody.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Scene.cpp -o bin/o/Scene.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Time.cpp -o bin/o/Time.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Transform.cpp -o bin/o/Transform.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/World.cpp -o bin/o/World.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/geometry/Curve.cpp -o bin/o/Curve.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/geometry/Line.cpp -o bin/o/Line.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/geometry/Math.cpp -o bin/o/Math.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/geometry/Matrix.cpp -o bin/o/Matrix.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/geometry/Vector.cpp -o bin/o/Vector.o	
	ar rcs lib/libphysics.a bin/o/*.o
	ranlib lib/libphysics.a
	rm bin/o/*.o

link:
	g++ -lX11 -Wl,-rpath,bin -Wall -std=c++17 src/main.cpp -o bin/main lib/physics.a -L lib -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio -lsfml-network

debugLink:
	g++ -g -rdynamic -lX11 -pthread -Wl,-rpath,bin -Wall -std=c++17 src/main.cpp -o bin/main -L lib -lphysics -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio -lsfml-network

run:
	./bin/main