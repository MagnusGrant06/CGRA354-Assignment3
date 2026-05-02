#include "boid.hpp"

class PredatorBoid : public Boid {

private:
	Boid* target_boid;

public:

	PredatorBoid(glm::vec3 pos, glm::vec3 dir, glm::vec3 col, std::vector<Boid*>* flo, Boid* target) : Boid(pos,dir,col,flo), target_boid(target) {}

	void calculateForces(Scene *scene) override;
	
	glm::vec3 calculate_seek(Boid* target, float distance_weight);

	Boid* change_target(const std::vector<Boid*>& boids);
	//glm::vec3 calculate_avoidance(std::vector<Boid>& boids) override;
	//glm::vec3 calculate_cohesion(std::vector<Boid>& boids) override;
	//glm::vec3 calculate_alignment(std::vector<Boid>& boids) override;
};