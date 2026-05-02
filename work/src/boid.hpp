
#pragma once

// glm
#include <glm/glm.hpp>
#include <vector>
// project
class Scene;


class Boid {
private:

	//glm::vec3 m_position;
	//glm::vec3 m_velocity;
	//glm::vec3 m_acceleration;

	float valid_radius = 5;

public:
	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;
	glm::vec3 m_color = glm::vec3(0,1,0);
	std::vector<Boid*>* flock = nullptr;

	Boid(glm::vec3 pos, glm::vec3 dir) : m_position(pos), m_velocity(dir) { }
	Boid(glm::vec3 pos, glm::vec3 dir, glm::vec3 col, std::vector<Boid*>*flo) : m_position(pos), m_velocity(dir), m_color(col), flock(flo) {}

	glm::vec3 position() const { return m_position; }
	glm::vec3 velocity() const { return m_velocity; }
	glm::vec3 acceleration() const { return m_acceleration; }

	glm::vec3 color() const;

	virtual void calculateForces(Scene *scene);
	void update(float timestep, Scene *scene);

	virtual glm::vec3 calculate_avoidance(std::vector<Boid*>& boids);

	virtual glm::vec3 calculate_cohesion(std::vector<Boid*>& boids);
	virtual glm::vec3 calculate_alignment(std::vector<Boid*>& boids);


};