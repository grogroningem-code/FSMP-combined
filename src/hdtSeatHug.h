#pragma once

// Flobrim fork: generic seated "hug" (WorldCollisionAPI.h, interface v3).
// Registry of seated actors + rebuild of their SMP items. The constraints themselves are added by
// SkyrimSystemCreator::addSeatHug while a system is built in the reference pose.

#include "WorldCollisionAPI.h"

#include <cstdint>
#include <string>

namespace hdt
{
	class Generic6DofConstraint;
}

namespace hdt::seathug
{
	// Called by SkyrimSystemCreator for every system it builds. True (and params copied) when the owning
	// actor is seated with the hug on and this physics file is not skipped.
	bool lookup(std::uint32_t actorFormID, const std::string& path, world::HugParams& out);

	// Called by SkyrimSystemCreator after addSeatHug.
	void noteBuilt(std::uint32_t actorFormID, const std::string& path, std::uint32_t constraints, std::string report);

	// Called by SkyrimSystemCreator for every hug constraint it adds (kept for freeze()).
	void noteConstraint(std::uint32_t actorFormID, const RE::BSTSmartPointer<Generic6DofConstraint>& constraint);

	// IWorldCollision::freezeSeatedHug: lock every hug constraint of the actor at its current relative pose.
	std::uint32_t freeze(std::uint32_t actorFormID);

	// IWorldCollision v5: per-outfit parameter overrides (by physics XML path, case-insensitive).
	void setOverride(const char* path, const world::HugParams* params);
	void clearOverrides();

	// IWorldCollision::setSeatedHug.
	std::uint32_t set(std::uint32_t actorFormID, const world::HugParams* params, const char* const* skipPaths, std::uint32_t skipCount);
}
