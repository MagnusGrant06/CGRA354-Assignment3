
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"

using namespace glm;
using namespace std;


vec3 Boid::color() const {
	return vec3(0, 1, 0);
}


void Boid::calculateForces(Scene *scene) {
	m_acceleration = glm::vec3(0);
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Calculate the forces affecting the boid and update the
	// acceleration (assuming mass = 1).
	// Do NOT update velocity or position in this function.
	// Core : 
	//  - Cohesion
	//  - Alignment
	//  - Avoidance
	//  - Soft Bound (optional)
	// Completion : 
	//  - Cohesion and Alignment with only boids in the same flock
	//  - Predator Avoidance (boids only)
	//  - Predator Chase (predator only)
	// Challenge : 
	//  - Obstacle avoidance
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...
	glm::vec3 total_avoidance = glm::vec3(0);
	for ( const Boid other_boid : scene->get_boids()) {
		//std::cout << this->m_position.x << "    " << this->m_position.y << "    " << this->m_position.z << std::endl;
		total_avoidance  = total_avoidance + calculate_avoidance(this, &other_boid);
	}
	m_acceleration += total_avoidance;
}


void Boid::update(float timestep, Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Integrate the velocity of the boid using the timestep.
	// Update the position of the boid using the new velocity.
	// Take into account the bounds of the scene which may
	// require you to change the velocity (if bouncing) or
	// change the position (if wrapping).
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...
	m_velocity = glm::clamp(m_velocity, scene->get_boid_min_v(), scene->get_boid_max_v());
	m_position = -scene->get_bound_size() + mod(m_position - -scene->get_bound_size(), scene->get_bound_size() - -scene->get_bound_size());
	m_velocity += m_acceleration;
	m_position += m_velocity * timestep;

}

glm::vec3 Boid::calculate_avoidance(const Boid* origin_boid, const Boid* other_boid) {
	//std::cout << other_boid->m_position.x << "    " << other_boid->m_position.y << "    " << other_boid->m_position.z << std::endl;
//	std::cout << origin_boid << "    " << other_boid << std::endl;
	if (origin_boid == other_boid) return glm::vec3(0);

	glm::vec3 displacement = origin_boid->m_position - other_boid->m_position;
	float distance = glm::length(displacement);

//	std::cout << distance << std::endl;
	if (distance > valid_radius || distance < 0.0001f) return glm::vec3(0);

	return displacement / (distance * distance);
}
