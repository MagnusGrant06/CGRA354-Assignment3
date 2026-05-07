
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

	std::vector<Boid*>& boids = scene->get_boids();

	glm::vec3 avoidance = clamp_vec_magnitude(calculate_avoidance(boids),15.0f);
	glm::vec3 cohesion = clamp_vec_magnitude(calculate_cohesion(boids), 15.0f);
	glm::vec3 alignment = clamp_vec_magnitude(calculate_alignment(boids), 15.0f);
	glm::vec3 flee_strength = calculate_flee(boids,scene);
	glm::vec3 object_avoidance = calculate_object_avoidance(scene->spheres);

	m_acceleration += object_avoidance + flee_strength + (avoidance * scene->avoidance_weight) + (cohesion * scene->cohesion_weight) + (alignment * scene->alignment_weight);
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
	//clamp velocity
	m_velocity += m_acceleration * timestep;
	if (glm::length(m_velocity) > scene->get_boid_max_v()) {
		if (this->flock == &scene->red_flock) {
			m_velocity = m_velocity / glm::length(m_velocity) * 20.0f;
		}
		else {
			m_velocity = m_velocity / glm::length(m_velocity) * scene->get_boid_max_v();
		}
	}
	m_position += m_velocity * timestep;
	m_position = -scene->get_bound_size() + mod(m_position - -scene->get_bound_size(), scene->get_bound_size() - -scene->get_bound_size());
}

glm::vec3 Boid::calculate_avoidance(std::vector<Boid*>& boids) {

	glm::vec3 avoidance = glm::vec3(0);
	for (const Boid* other_boid : boids) {
		if (this == other_boid) continue;

		glm::vec3 displacement = this->m_position - other_boid->m_position;
		float distance = glm::length(displacement);
		if (distance > valid_radius || distance < 0.0001f) continue;  // exit if distance is 0 to avoid division by 0 or out of range

		avoidance += displacement / (distance * distance);
	}

	return avoidance;

}

glm::vec3 Boid::calculate_cohesion(std::vector<Boid*>& boids) {
	glm::vec3 average_distance = glm::vec3();
	float neighbours = 1.0;
	for (const Boid* other_boid : boids) {
		if (other_boid == this || other_boid->flock != this->flock)continue;

		glm::vec3 displacement = this->m_position - other_boid->m_position;
		float distance = glm::length(displacement);

		if (distance > valid_radius) continue;

		average_distance += other_boid->m_position;
		neighbours += 1.0;

	}

	glm::vec3 average_position = average_distance / neighbours;
	return average_position - this->m_position;

}

glm::vec3 Boid::calculate_alignment(std::vector<Boid*>& boids) {

	glm::vec3 total_velocity = glm::vec3();
	float neighbours = 1.0;
	for (const Boid* other_boid : boids) {
		if (other_boid == this || other_boid->flock != this->flock)continue;

		glm::vec3 displacement = this->m_position - other_boid->m_position;
		float distance = glm::length(displacement);
		if (distance > valid_radius) continue;

		total_velocity += other_boid->m_velocity;
		neighbours += 1.0;

	}

	glm::vec3 average_velocity = total_velocity / neighbours;
	return average_velocity - this->m_position;

}

glm::vec3 Boid::calculate_flee(std::vector<Boid*>& boids, Scene* scene) {
	glm::vec3 flee_strength = glm::vec3(0);
	for (Boid* other_boid : boids) {
		glm::vec3 displacement = this->m_position - other_boid->m_position;
		float distance = glm::length(displacement);
		if (distance > 10.0f) continue;
		if (other_boid->flock == &scene->red_flock) {
			flee_strength += displacement * 2.0f;
		}
	}
	return flee_strength;
}

glm::vec3 Boid::calculate_object_avoidance(std::vector<Scene::Sphere> objects) {
	float boid_radius = 0.5;
	glm::vec3 total_avoidance = glm::vec3(0);

	for (Scene::Sphere sphere : objects) {

		glm::vec3 D = glm::normalize(m_velocity);
		glm::vec3 C = sphere.pos;
		glm::vec3 O = m_position;
		float R = sphere.radius + boid_radius + 2.0f;

		float a = glm::dot(D, D);
		float b = glm::dot(2.0f * (O - C), D);
		float c = glm::dot((O - C), (O - C)) - R * R;

		float discriminant = (b * b) - (4 * a * c);
		if (discriminant < 0) continue;

		float t0 = (-b + glm::sqrt(discriminant)) / (2.0f * a);
		float t1 = (-b - glm::sqrt(discriminant)) / (2.0f * a);

		if (t1 > 0 && t1 < 10.0f){
			glm::vec3 intersection = O + t1 * D;
			glm::vec3 avoidance = glm::normalize(intersection - C) * 10.0f;
			total_avoidance += avoidance*500.0f;
		}


	}
	return total_avoidance;
	
}

glm::vec3 Boid::clamp_vec_magnitude(glm::vec3 v, float magnitude) {
	float leng = glm::length(v);
	if (leng < magnitude) return v;
	return (v / leng) * magnitude;
}
