#pragma once

#include <array>

// Non-actor (e.g. SMP banner) frustum + havok LOS helper for ActorManager.
//
// Approach mirrors /workspace/actor-visibility/src/RayCaster.{h,cpp} (MIT, our own code):
// padded PointInFrustum on worldBound, then bhkWorld::PickObject rays to a few bound
// sample points with a custom hkpClosestRayHitCollector-compatible collector that only
// counts kStatic / kTerrain / kGround as blockers. Adapted here for TESObjectREFR /
// NiAVObject (no Actor) and for CommonLibVR-ng's bhkPickData::closestRayHitCollector
// and NiCamera::PointInFrustum(point, radius) instance API.

namespace hdt
{
	class NonActorRayCollector : public RE::hkpRayHitCollector
	{
	public:
		NonActorRayCollector() = default;
		~NonActorRayCollector() override = default;

		void AddRayHit(const RE::hkpCdBody& a_body, const RE::hkpShapeRayCastCollectorOutput& a_hitInfo) override;

		void Clear() noexcept
		{
			RE::hkpRayHitCollector::Reset();
			rayHit.Reset();
		}
		[[nodiscard]] bool HasHit() const noexcept { return rayHit.HasHit(); }

		// Same layout as RE::hkpClosestRayHitCollector (0x70): bhkPickData stores the
		// collector as hkpClosestRayHitCollector*.
		RE::hkpWorldRayCastOutput rayHit;  // 10
	};
	static_assert(sizeof(NonActorRayCollector) == sizeof(RE::hkpClosestRayHitCollector));

	class NonActorRayCaster
	{
	public:
		enum class Result
		{
			kOffscreen = 0,
			kObscured,
			kVisible
		};

		struct Params
		{
			float radiusScale = 1.25f;
			float radiusMargin = 64.0f;
			bool  doRaycast = false;
			// A blocker hit within (bound radius + this) of the sample point counts as "at the target"
			// (the wall / pole / beam the banner hangs on), not as an occluder.
			float targetSlack = 48.0f;
		};

		NonActorRayCaster(RE::NiCamera* a_camera, const RE::NiPoint3& a_cameraPos) :
			camera(a_camera), cameraPos(a_cameraPos)
		{}

		// Frustum test with padded bound radius (wake before onscreen).
		[[nodiscard]] bool InFrustum(RE::NiAVObject* a_root, const Params& a_params) const;

		// Frustum, then optional multi-point havok LOS through the owner's cell.
		// a_owner may be null (rays are skipped -> visible if in frustum).
		[[nodiscard]] Result GetResult(RE::TESObjectREFR* a_owner, RE::NiAVObject* a_root, const Params& a_params) const;

	private:
		void GatherBoundPoints(RE::NiAVObject* a_root, std::array<RE::NiPoint3, 6>& a_out) const;

		RE::NiCamera* camera;
		RE::NiPoint3  cameraPos;
	};
}
