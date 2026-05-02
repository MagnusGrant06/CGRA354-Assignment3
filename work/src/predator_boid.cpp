#include "predator_boid.hpp"
#include "scene.hpp"

void PredatorBoid::calculateForces(Scene *scene) {
	glm::vec3 displacement = m_position - target_boid->m_position;
	float dist = glm::length(displacement);
	if (dist > scene->get_prey_distance()) change_target(scene->boids());
	calculate_seek(target_boid, dist);
	std::cout << target_boid->m_color.x << target_boid->m_color.y << target_boid->m_color.z << std::endl;
}

glm::vec3 PredatorBoid::calculate_seek(Boid* target, float distance_weight) {
	glm::vec3 target_pos = target->m_position;
	return m_velocity * (target_pos * distance_weight);
	std::cout << "yeh" << std::endl;
}

Boid* PredatorBoid::change_target(const std::vector<Boid*>& boids) {
	float min_dist = FLT_MAX;
	Boid* min_boid = boids[0];
	for (Boid* b : boids) {
		if (this == b) continue;
		glm::vec3 displacement = m_position - b->m_position;
		float dist = glm::length(displacement);

		if (dist < min_dist) {
			min_dist = dist;
			min_boid = b;
		}
	}
	std::cout << "yeh" << std::endl;
	return min_boid;
}