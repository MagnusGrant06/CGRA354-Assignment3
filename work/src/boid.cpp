
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"

using namespace glm;
using namespace std;


vec3 Boid::color() const {
	return m_color;
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

	std::vector<Boid>& boids = scene->get_boids();

	glm::vec3 avoidance = calculate_avoidance(boids);
	glm::vec3 cohesion = calculate_cohesion(boids);
	glm::vec3 alignment = calculate_alignment(boids);

	m_acceleration += (avoidance * scene->avoidance_weight) + (cohesion * scene->cohesion_weight) + (alignment * scene->alignment_weight);
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

	valid_radius = scene->get_radius();
	m_position = -scene->get_bound_size() + mod(m_position - -scene->get_bound_size(), scene->get_bound_size() - -scene->get_bound_size());
	m_velocity = glm::clamp(m_velocity += m_acceleration * timestep, scene->get_boid_min_v(), scene->get_boid_max_v());
	m_position += m_velocity * timestep;
}

glm::vec3 Boid::calculate_avoidance(std::vector<Boid>& boids) {

	glm::vec3 avoidance = glm::vec3(0);
	for (const Boid& other_boid : boids) {
		if (this == &other_boid) continue;

		glm::vec3 displacement = this->m_position - other_boid.m_position;
		float distance = glm::length(displacement);
		if (distance > valid_radius || distance < 0.0001f) continue;  // exit if distance is 0 to avoid division by 0 or out of range
		avoidance += displacement / (distance * distance);
	}

	return avoidance;

}

glm::vec3 Boid::calculate_cohesion(std::vector<Boid>& boids) {
	glm::vec3 average_distance = glm::vec3();
	float neighbours = 1.0;
	for (const Boid& other_boid : boids) {
		if (&other_boid == this || other_boid.flock != this->flock)continue;
		
		glm::vec3 displacement = this->m_position - other_boid.m_position;
		float distance = glm::length(displacement);

		if (distance > valid_radius) continue;

		average_distance += other_boid.m_position;
		neighbours+=1.0;
	
	}

	glm::vec3 average_position = average_distance / neighbours;
	return average_position - this->m_position;

}

glm::vec3 Boid::calculate_alignment(std::vector<Boid>& boids) {

	glm::vec3 total_velocity = glm::vec3();
	float neighbours = 1.0;
	for (const Boid& other_boid : boids) {
		if (&other_boid == this || other_boid.flock != this->flock)continue;

		glm::vec3 displacement = this->m_position - other_boid.m_position;
		float distance = glm::length(displacement);
		if (distance > valid_radius) continue;

		total_velocity += other_boid.m_velocity;
		neighbours += 1.0;

	}

	glm::vec3 average_velocity = total_velocity / neighbours;
	return average_velocity - this->m_position;

}
