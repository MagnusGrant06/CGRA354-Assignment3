#include "predator_boid.hpp"
#include "scene.hpp"

void PredatorBoid::calculateForces(Scene *scene) {
	m_acceleration = glm::vec3(0);
	glm::vec3 displacement =  target_boid->m_position - m_position;
	float dist = glm::length(displacement);
	if (dist > scene->get_prey_distance()) target_boid = change_target(scene->boids());
	glm::vec3 temp = calculate_seek(target_boid, 1.0, dist);
	m_acceleration += temp;
}

glm::vec3 PredatorBoid::calculate_seek(Boid* target, float distance_weight, float distance) {
	glm::vec3 target_pos = target->m_position;
	glm::vec3 target_v = target->m_velocity;
	float look_ahead = glm::clamp(distance / 30.0f,0.0f,0.5f);

	glm::vec3 future_pos = target_pos + (target_v * look_ahead);

	glm::vec3 desired_force = (future_pos - m_position);
	return desired_force - m_velocity;
}

Boid* PredatorBoid::change_target(const std::vector<Boid*>& boids) {
	float min_dist = FLT_MAX;
	Boid* min_boid = boids[0];
	for (Boid* b : boids) {
		if (this == b) continue;
		glm::vec3 displacement = b->m_position - m_position;
		float dist = glm::length(displacement);
		if (dist < min_dist) {
			min_dist = dist;
			min_boid = b;
		}
	}
	return min_boid;
}