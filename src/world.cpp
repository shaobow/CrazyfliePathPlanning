#include "world.h"

namespace CF_PLAN {

World::World(const std::string& file_path) { load_world(file_path); }

World::~World() {}

void World::load_world(const std::string& file_path){
    
}

Boundary World::get_bound() const { return world_bound; }
}  // namespace CF_PLAN
