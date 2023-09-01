# Installing
Runs in Visual Studio 2022
Add 'lib' to list in Project > Physics Engine Properties > Configuration Properties > Debugging > Environment

# Physics Engine

This is a simple physics engine meant to make physics easier to implement in games.
The physics engine will be named at some point in the future.

<h2>Functionality</h2>
<p>The physics engine is comprise of a physics module and a geometry module.
The geometry module is independent and can be used without the physics module.
The physics module takes advantage of the math done in the geometry module
to run a physics simulation. The physics simulation is highly customizable and
custom colliders and joints are easy to add.</p>

<h2>Modules</h2>
<p>The geometry module is self explanatory, consisting of vectors, matrices, Lines and some Vector math.</p>
The physics module is comprised of smaller submodules:</p>

> The Collision Module

> The Dynamics Module

> The Engine Module
<p>Each module performs a specific task and most of the API will invoke the Engine module.
The physics engine is currently incomplete and does not even have versioning yet.
Alpha release will come relatively soon.</p>


