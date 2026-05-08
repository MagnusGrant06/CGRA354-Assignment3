#include "boid.hpp"

//subclass that inherits certain parts from boid to make drawing and basic movement easier
class PredatorBoid : public Boid {

private:
	Boid* target_boid;

public:

	PredatorBoid(glm::vec3 pos, glm::vec3 dir, glm::vec3 col, std::vector<Boid*>* flo, Boid* target) : Boid(pos,dir,col,flo), target_boid(target) {}

	void calculateForces(Scene *scene) override;
	
	glm::vec3 calculate_seek(Boid* target, float distance_weight, float distance);

	Boid* change_target(const std::vector<Boid*>& boids);
};