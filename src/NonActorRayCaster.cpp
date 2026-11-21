#include "NonActorRayCaster.h"

namespace hdt
{
	void NonActorRayCollector::AddRayHit(const RE::hkpCdBody& a_body, const RE::hkpShapeRayCastCollectorOutput& a_hitInfo)
	{
		const RE::hkpCdBody* root = std::addressof(a_body);
		while (root->parent) {
			root = root->parent;
		}
		const auto* collidable = static_cast<const RE::hkpCollidable*>(root);

		switch (collidable->GetCollisionLayer()) {
		case RE::COL_LAYER::kStatic:
		case RE::COL_LAYER::kTerrain:
		case RE::COL_LAYER::kGround:
			break;
		default:
			return;  // actors, clutter, triggers, water, ... never block
		}

		// closest-hit semantics (same contract as hkpClosestRayHitCollector)
		if (a_hitInfo.hitFraction <= earlyOutHitFraction) {
			earlyOutHitFraction = a_hitInfo.hitFraction;
			rayHit.hitFraction = a_hitInfo.hitFraction;
			rayHit.extraInfo = a_hitInfo.extraInfo;
			rayHit.shapeKey = a_hitInfo.shapeKey;
			rayHit.rootCollidable = collidable;
		}
	}

	bool NonActorRayCaster::InFrustum(RE::NiAVObject* a_root, const Params& a_params) const
	{
		if (!camera || !a_root) {
			return true;  // no camera: don't claim hidden
		}
		const auto& bound = a_root->worldBound;
		const float radius = bound.radius * a_params.radiusScale + a_params.radiusMargin;
		return camera->PointInFrustum(bound.center, radius);
	}

	void NonActorRayCaster::GatherBoundPoints(RE::NiAVObject* a_root, std::array<RE::NiPoint3, 6>& a_out) const
	{
		const auto& bound = a_root->worldBound;
		const float r = bound.radius;
		const RE::NiPoint3& c = bound.center;

		// Lateral axis perpendicular to the camera->target direction (horizontal), so the side samples are
		// left/right as seen from the camera rather than along a fixed world axis that may point into a wall.
		RE::NiPoint3 toCam = cameraPos - c;
		RE::NiPoint3 side(-toCam.y, toCam.x, 0.0f);
		const float sideLen = std::sqrt(side.x * side.x + side.y * side.y);
		if (sideLen > 1e-3f)
			side = side * (1.0f / sideLen);
		else
			side = RE::NiPoint3(1.0f, 0.0f, 0.0f);
		const float camLen = std::sqrt(toCam.x * toCam.x + toCam.y * toCam.y + toCam.z * toCam.z);
		const RE::NiPoint3 towardCam = camLen > 1e-3f ? toCam * (1.0f / camLen) : RE::NiPoint3();

		// Banners hang on walls / poles, so the bound center is often inside the mount's collision. Sample
		// the top (the banner's fixing is usually there but the cloth is in front), center, both sides, the
		// bottom, and a point pulled toward the camera (in front of whatever it's mounted on).
		a_out[0] = c + towardCam * (r * 0.5f);
		a_out[1] = RE::NiPoint3(c.x, c.y, c.z + r * 0.6f);
		a_out[2] = c;
		a_out[3] = c + side * (r * 0.6f);
		a_out[4] = c - side * (r * 0.6f);
		a_out[5] = RE::NiPoint3(c.x, c.y, c.z - r * 0.6f);
	}

	NonActorRayCaster::Result NonActorRayCaster::GetResult(RE::TESObjectREFR* a_owner, RE::NiAVObject* a_root, const Params& a_params) const
	{
		if (!a_root) {
			return Result::kOffscreen;
		}

		if (!InFrustum(a_root, a_params)) {
			return Result::kOffscreen;
		}
		if (!a_params.doRaycast) {
			return Result::kVisible;
		}

		auto* cell = a_owner ? a_owner->GetParentCell() : nullptr;
		auto* world = cell ? cell->GetbhkWorld() : nullptr;
		if (!world) {
			return Result::kVisible;  // can't test LOS: don't hide
		}

		std::array<RE::NiPoint3, 6> points;
		GatherBoundPoints(a_root, points);

		const float scale = RE::bhkWorld::GetWorldScale();

		RE::bhkPickData pick{};
		pick.rayInput.from = RE::hkVector4(cameraPos.x * scale, cameraPos.y * scale, cameraPos.z * scale, 0.0f);
		pick.rayInput.enableShapeCollectionFilter = false;
		pick.rayInput.filterInfo.SetCollisionLayer(RE::COL_LAYER::kLOS);

		NonActorRayCollector collector;
		pick.closestRayHitCollector = reinterpret_cast<RE::hkpClosestRayHitCollector*>(std::addressof(collector));

		for (const auto& p : points) {
			collector.Clear();
			pick.rayOutput.Reset();
			pick.rayInput.to = RE::hkVector4(p.x * scale, p.y * scale, p.z * scale, 0.0f);
			world->PickObject(pick);
			if (!collector.HasHit()) {
				return Result::kVisible;  // one clear line is enough
			}
			// Lenient: a blocker at (or just in front of) the target is the banner's own mount / the wall it
			// hangs on, not something between the camera and the banner.
			const RE::NiPoint3 d = p - cameraPos;
			const float targetDist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
			const float hitDist = collector.rayHit.hitFraction * targetDist;
			if (hitDist >= targetDist - (a_root->worldBound.radius + a_params.targetSlack)) {
				return Result::kVisible;
			}
		}
		return Result::kObscured;
	}
}
