all: compileObjects link run
debug: debugCompileObjects debugLink run
test: compileTest runTest

libs := lib/openal32.dll lib/sfml-audio-2.dll lib/sfml-audio-d-2.dll lib/sfml-graphics-2.dll lib/sfml-graphics-d-2.dll lib/sfml-network-2.dll lib/sfml-network-d-2.dll lib/sfml-system-2.dll lib/sfml-system-d-2.dll lib/sfml-window-2.dll lib/sfml-window-d-2.dll

checkSyntax:
	g++ -Wall -fsyntax-only -std=c++17 src/physics/Collision/*.cpp
	g++ -Wall -fsyntax-only -std=c++17 src/physics/Dynamics/*.cpp
	g++ -Wall -fsyntax-only -std=c++17 src/physics/Engine/*.cpp
	g++ -Wall -fsyntax-only -std=c++17 src/geometry/*.cpp
	g++ -Wall -fsyntax-only -std=c++17 src/main.cpp

compileTest:
	g++ -c -Wall -std=c++17 test/TestCollision.cpp -o test/TestCollision.o
	ar rcs test/libtest.a test/*.o
	ranlib test/libtest.a
	rm test/*.o
	g++ -g -lX11 -pthread -Wl,-rpath,test/bin -Wall -std=c++17 test/main.cpp -o test/bin/main test/libtest.a -L lib -lphysics -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio -lsfml-network

runTest:
	./test/bin/main

compileObjects:
	g++ -c -Wall -std=c++17 src/physics/Collision/Algo.cpp -o bin/o/Algo.o
	g++ -c -Wall -std=c++17 src/physics/Collision/BoxCollider.cpp -o bin/o/BoxCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/CircleCollider.cpp -o bin/o/CircleCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Collision.cpp -o bin/o/Collision.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Collider.cpp -o bin/o/Collider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/CollisionObject.cpp -o bin/o/CollisionObject.o
	g++ -c -Wall -std=c++17 src/physics/Dynamics/Dynamicbody.cpp -o bin/o/Dynamicbody.o
	g++ -c -Wall -std=c++17 src/physics/Dynamics/DynamicWorld.cpp -o bin/o/DynamicWorld.o
	g++ -c -Wall -std=c++17 src/physics/Engine/Entity.cpp -o bin/o/Entity.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Hashable.cpp -o bin/o/Hashable.o
	g++ -c -Wall -std=c++17 src/physics/Collision/MeshCollider.cpp -o bin/o/MeshCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/PointCollider.cpp -o bin/o/PointCollider.o
	g++ -c -Wall -std=c++17 src/physics/Collision/PolygonCollider.cpp -o bin/o/PolygonCollider.o
	g++ -c -Wall -std=c++17 src/physics/Dynamics/Rigidbody.cpp -o bin/o/Rigidbody.o
	g++ -c -Wall -std=c++17 src/physics/Dynamics/Softbody.cpp -o bin/o/Softbody.o
	g++ -c -Wall -std=c++17 src/physics/Engine/Time.cpp -o bin/o/Time.o
	g++ -c -Wall -std=c++17 src/physics/Collision/Transform.cpp -o bin/o/Transform.o
	g++ -c -Wall -std=c++17 src/geometry/Curve.cpp -o bin/o/Curve.o
	g++ -c -Wall -std=c++17 src/geometry/Line.cpp -o bin/o/Line.o
	g++ -c -Wall -std=c++17 src/geometry/Math.cpp -o bin/o/Math.o
	g++ -c -Wall -std=c++17 src/geometry/Matrix.cpp -o bin/o/Matrix.o
	g++ -c -Wall -std=c++17 src/geometry/Matrix2.cpp -o bin/o/Matrix2.o
	g++ -c -Wall -std=c++17 src/geometry/Matrix3.cpp -o bin/o/Matrix3.o
	g++ -c -Wall -std=c++17 src/geometry/Vector.cpp -o bin/o/Vector.o
	g++ -c -Wall -std=c++17 src/geometry/Vector2.cpp -o bin/o/Vector2.o
	g++ -c -Wall -std=c++17 src/geometry/Vector3.cpp -o bin/o/Vector3.o
	ar rcs lib/libphysics.a bin/o/*.o
	ranlib lib/libphysics.a
	rm bin/o/*.o

debugCompileObjects:
	g++ -g -c -Wall -std=c++17 src/physics/Collision/BoxCollider.cpp -o bin/o/BoxCollider.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/BoxCollision.cpp -o bin/o/BoxCollision.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/CircleCollider.cpp -o bin/o/CircleCollider.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/CircleCollision.cpp -o bin/o/CircleCollision.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/Collision.cpp -o bin/o/Collision.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/Collider.cpp -o bin/o/Collider.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/CollisionObject.cpp -o bin/o/CollisionObject.o
	g++ -g -c -Wall -std=c++17 src/physics/Dynamics/Dynamicbody.cpp -o bin/o/Dynamicbody.o
	g++ -g -c -Wall -std=c++17 src/physics/Engine/DynamicsWorld.cpp -o bin/o/DynamicsWorld.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/MeshCollider.cpp -o bin/o/MeshCollider.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/MeshCollision.cpp -o bin/o/MeshCollision.o
	g++ -g -c -Wall -std=c++17 src/physics/Engine/PhysicsSolver.cpp -o bin/o/PhysicSolver.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/PointCollider.cpp -o bin/o/PointCollider.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/PointCollision.cpp -o bin/o/PointCollision.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/PolygonCollider.cpp -o bin/o/PolygonCollider.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/PolygonCollision.cpp -o bin/o/PolygonCollision.o
	g++ -g -c -Wall -std=c++17 src/physics/Dynamics/Rigidbody.cpp -o bin/o/Rigidbody.o
	g++ -g -c -Wall -std=c++17 src/physics/Dynamics/Softbody.cpp -o bin/o/Softbody.o
	g++ -g -c -Wall -std=c++17 src/physics/Engine/Time.cpp -o bin/o/Time.o
	g++ -g -c -Wall -std=c++17 src/physics/Collision/Transform.cpp -o bin/o/Transform.o
	g++ -g -c -Wall -std=c++17 src/geometry/Curve.cpp -o bin/o/Curve.o
	g++ -g -c -Wall -std=c++17 src/geometry/Line.cpp -o bin/o/Line.o
	g++ -g -c -Wall -std=c++17 src/geometry/Math.cpp -o bin/o/Math.o
	g++ -g -c -Wall -std=c++17 src/geometry/Matrix.cpp -o bin/o/Matrix.o
	g++ -g -c -Wall -std=c++17 src/geometry/Matrix2.cpp -o bin/o/Matrix2.o
	g++ -g -c -Wall -std=c++17 src/geometry/Matrix3.cpp -o bin/o/Matrix3.o
	g++ -g -c -Wall -std=c++17 src/geometry/Vector.cpp -o bin/o/Vector.o	
	g++ -g -c -Wall -std=c++17 src/geometry/Vector2.cpp -o bin/o/Vector2.o	
	g++ -g -c -Wall -std=c++17 src/geometry/Vector3.cpp -o bin/o/Vector3.o	
	ar rcs lib/libphysics.a bin/o/*.o
	ranlib lib/libphysics.a
	rm bin/o/*.o

link:
	g++ -lX11 -Wl,-rpath,bin -Wall -std=c++17 src/main.cpp -o bin/main lib/physics.a -L lib -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio -lsfml-network

debugLink:
	g++ -pthread -Wl,-rpath,bin -g -Wall -std=c++17 src/main.cpp -L lib -lphysics $(libs) -o bin/main.exe

run:
	./bin/main