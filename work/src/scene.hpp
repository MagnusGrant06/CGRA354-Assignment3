
#pragma once

//std
#include <vector>

// glm
#include <glm/glm.hpp>

// project
#include "cgra/cgra_mesh.hpp"
#include "cgra/cgra_shader.hpp"


// boid class (forward declaration)
class Boid;

class Scene {
private:
	// opengl draw data
	GLuint m_color_shader = 0;
	GLuint m_aabb_shader = 0;
	GLuint m_skymap_shader = 0;
	cgra::gl_mesh m_simple_boid_mesh;
	cgra::gl_mesh m_boid_mesh;
	cgra::gl_mesh m_predator_mesh;
	cgra::gl_mesh m_sphere_mesh;

	// draw status
	bool m_show_aabb = true;
	bool m_show_axis = false;
	bool m_show_skymap = false;
	bool challenge = false;

	// scene data
	glm::vec3 m_bound_hsize = glm::vec3(20);
	//had to change boids to pointers to allow for polymorphism to predator boids
	std::vector<Boid*> m_boids;


	//-------------------------------------------------------------
	// [Assignment 3] :
	// Create variables for keeping track of the boid parameters
	// such as min and max speed etc. These paramters can either be
	// public, or private with getter functions.
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

	int boid_num = 300;
	float max_boid_v = 15.0;
	float min_boid_v = -15.0;
	float valid_radius = 15;

	float max_prey_distance = 30;

	float sphere_radius;
	float boid_radius = 0.5;
	float sphere_size = 3.5f;

	//vector of vertices to store vertices of sphere to calculate radius
	std::vector<cgra::mesh_vertex> sphere_vertices;
public:

	struct Sphere {
		glm::vec3 pos;
		float radius;
	};

	float max_predator_v = 20.0f;
	float object_avoidance_strength = 500.0f;
	float predator_seek_strength = 1.0f;
	float max_predator_acceleration = 30.0f;

	std::vector<Scene::Sphere> spheres;
	float avoidance_weight = 1.0;
	float cohesion_weight = 1.0;
	float alignment_weight = 1.0;
	float flee_weight = 1.0f;
	float max_boid_acceleration = 20.0f;

	std::vector<Boid*> blue_flock;
	std::vector<Boid*> green_flock;
	std::vector<Boid*> red_flock;

	Scene();

	// functions that load the scene
	void loadCore();
	void loadCompletion();
	void loadChallenge();

	// called every frame, with timestep in seconds
	void update(float timestep);

	// called every frame, with the given projection and view matrix
	void draw(const glm::mat4 &proj, const glm::mat4 &view);

	// called every frame (to fill out a ImGui::TreeNode)
	void renderGUI();

	// returns a const reference to the boids vector	
	const std::vector<Boid*> & boids() const { return m_boids; }

	// returns the half-size of the bounding box (centered around the origin)
	glm::vec3 bound() const { return m_bound_hsize; }

	// YOUR CODE GOES HERE
	// ...
	glm::vec3 get_bound_size();

	float get_boid_max_v();

	float get_boid_min_v();

	float get_radius();

	float get_prey_distance();

	std::vector<Boid*>& get_boids();

	std::vector<Scene::Sphere> get_spheres();

	static float get_mesh_radius(std::vector<cgra::mesh_vertex> vertices);
};