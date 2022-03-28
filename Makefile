all: compilePhysics link run
debug: debugCompileObjects debugLink run
test: compileTest runTest

checkSyntax:
	g++ -fsyntax-only -std=c++17 src/physics/Collision/*.cpp
	g++ -fsyntax-only -std=c++17 src/physics/Engine/*.cpp
	g++ -fsyntax-only -std=c++17 src/physics/Tools/*.cpp
	g++ -fsyntax-only -std=c++17 src/geometry/*.cpp
	g++ -fsyntax-only -std=c++17 src/main.cpp

compileTest:
	g++ -c -Wall -std=c++17 test/TestCollision.cpp -o test/TestCollision.o
	ar rcs test/libtest.a test/*.o
	ranlib test/libtest.a
	rm test/*.o
	g++ -g -rdynamic -lX11 -pthread -Wl,-rpath,test/bin -Wall -std=c++17 test/main.cpp -o test/bin/main test/libtest.a -L lib -lphysics -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio -lsfml-network

runTest:
	./test/bin/main

compileObjects:
	g++ -c -Wall -std=c++17 src/physics/Collision/Algo.cpp -o bin/o/Algo.o
	g++ -c -Wall -std=c++17 src/physics/Tools/Archive.cpp -o bin/o/Archive.o
	g++ -c -Wall -std=c++17 src/physics/Collision/BoxCollider.cpp -o bin/o/BoxCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/CircleCollider.cpp -o bin/o/CircleCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Collision.cpp -o bin/o/Collision.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Collider.cpp -o bin/o/Collider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/CollisionObject.cpp -o bin/o/CollisionObject.o
	g++ -c -Wall -std=c++17 src/physics/Tools/DataPacket.cpp -o bin/o/DataPacket.o
	g++ -c -Wall -std=c++17 src/physics/Engine/Display.cpp -o bin/o/Display.o
	g++ -c -Wall -std=c++17 src/physics/Engine/Entity.cpp -o bin/o/Entity.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Hashable.cpp -o bin/o/Hashable.o
	g++ -c -Wall -std=c++17 src/physics/Collision/MeshCollider.cpp -o bin/o/MeshCollider.o
	g++ -c -Wall -std=c++17 src/physics/Tools/Notify.cpp -o bin/o/Notify.o
	g++ -c -Wall -std=c++17 src/physics/Tools/OstreamOverloads.cpp -o bin/o/OstreamOverloads.o
	g++ -c -Wall -std=c++17 src/physics/Collision/PolygonCollider.cpp -o bin/o/PolygonCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Rigidbody.cpp -o bin/o/Rigidbody.o
	g++ -c -Wall -std=c++17 src/physics/Engine/Scene.cpp -o bin/o/Scene.o
	g++ -c -Wall -std=c++17 src/physics/Engine/Time.cpp -o bin/o/Time.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Transform.cpp -o bin/o/Transform.o
	g++ -c -Wall -std=c++17 src/physics/Engine/World.cpp -o bin/o/World.o
	g++ -c -Wall -std=c++17 src/geometry/Curve.cpp -o bin/o/Curve.o
	g++ -c -Wall -std=c++17 src/geometry/Line.cpp -o bin/o/Line.o
	g++ -c -Wall -std=c++17 src/geometry/Math.cpp -o bin/o/Math.o
	g++ -c -Wall -std=c++17 src/geometry/Matrix.cpp -o bin/o/Matrix.o
	g++ -c -Wall -std=c++17 src/geometry/Vector.cpp -o bin/o/Vector.o
	ar rcs lib/libphysics.a bin/o/*.o
	ranlib lib/libphysics.a
	rm bin/o/*.o

debugCompileObjects:
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/Algo.cpp -o bin/o/Algo.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Tools/Archive.cpp -o bin/o/Archive.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/BoxCollider.cpp -o bin/o/BoxCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/CircleCollider.cpp -o bin/o/CircleCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/Collision.cpp -o bin/o/Collision.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/Collider.cpp -o bin/o/Collider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/CollisionObject.cpp -o bin/o/CollisionObject.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Engine/Display.cpp -o bin/o/Display.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Engine/Entity.cpp -o bin/o/Entity.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/Hashable.cpp -o bin/o/Hashable.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/MeshCollider.cpp -o bin/o/MeshCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Tools/Notify.cpp -o bin/o/Notify.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Tools/OstreamOverloads.cpp -o bin/o/OstreamOverloads.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Engine/PhysicsSolver.cpp -o bin/o/PhysicSolver.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/PolygonCollider.cpp -o bin/o/PolygonCollider.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/Rigidbody.cpp -o bin/o/Rigidbody.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Engine/Scene.cpp -o bin/o/Scene.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Engine/Time.cpp -o bin/o/Time.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Collision/Transform.cpp -o bin/o/Transform.o
	g++ -g -rdynamic -c -Wall -std=c++17 src/physics/Engine/World.cpp -o bin/o/World.o
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