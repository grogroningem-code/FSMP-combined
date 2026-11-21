#include "hdtDiscontinuityGuard.h"

#include "hdtSkyrimBone.h"

namespace hdt
{
	namespace
	{
		// A settling system is released once its pose has stayed calm for this long, so that a pose
		// change which starts a moment after the system was built is still waited for.
		constexpr float kCalmTime = 0.25f;
	}

	void DiscontinuityGuard::restart(const btTransform& root)
	{
		m_lastRoot = root;
		m_tracking = true;
	}

	void DiscontinuityGuard::settle()
	{
		m_settling = true;
		m_heldFor = 0.0f;
		m_calmFor = 0.0f;
	}

	DiscontinuityGuard::Finding DiscontinuityGuard::inspect(const Bones& bones, const btTransform& root, float timeStep, const DiscontinuitySettings& settings)
	{
		Finding finding;

		if (!settings.enabled) {
			m_settling = false;
		}
		if (!m_tracking || !settings.enabled || timeStep <= FLT_EPSILON) {
			restart(root);
			return finding;
		}

		// Rigid motion of the root since the previous read.
		const btTransform delta = root * m_lastRoot.inverse();
		const btVector3 shift = root.getOrigin() - m_lastRoot.getOrigin();
		m_lastRoot = root;

		finding.rootSpeed = shift.length() / timeStep;
		finding.turnSpeed = btDegrees(delta.getRotation().getAngleShortestPath()) / timeStep;

		// What is left of each animated bone's motion once the root motion is taken out.
		const float snapDistance = settings.poseSnapSpeedLimit * timeStep;
		const float snapDistance2 = snapDistance * snapDistance;
		float farthest2 = 0.0f;
		for (const auto& bone : bones) {
			if (!bone->m_rig.isStaticOrKinematicObject()) {
				continue;
			}

			++finding.animatedBones;
			const auto target = static_cast<const SkyrimBone*>(bone.get())->animatedRigOrigin();
			const auto carried = delta * bone->m_rig.getWorldTransform().getOrigin();
			const float moved2 = (target - carried).length2();
			farthest2 = std::max(farthest2, moved2);
			if (moved2 > snapDistance2) {
				++finding.snappedBones;
			}
		}
		finding.poseSpeed = std::sqrt(farthest2) / timeStep;

		if (m_settling) {
			m_heldFor += timeStep;
			m_calmFor = finding.poseSpeed <= settings.settleSpeedLimit ? m_calmFor + timeStep : 0.0f;
			finding.heldFor = m_heldFor;

			if (m_calmFor < kCalmTime && m_heldFor < settings.settleTimeLimit) {
				finding.action = Action::kHold;
			} else {
				m_settling = false;
				finding.released = true;
			}
			return finding;
		}

		if (finding.snappedBones > 0 && finding.snappedBones * 2 >= finding.animatedBones) {
			finding.action = Action::kRestart;
		} else if (finding.rootSpeed > settings.teleportSpeedLimit || finding.turnSpeed > settings.teleportTurnLimit) {
			carry(bones, delta);
			finding.action = Action::kCarried;
		}

		return finding;
	}

	void DiscontinuityGuard::carry(const Bones& bones, const btTransform& delta)
	{
		const auto& turn = delta.getBasis();
		for (const auto& bone : bones) {
			auto& rig = bone->m_rig;
			const btTransform moved = delta * rig.getWorldTransform();
			rig.setWorldTransform(moved);
			rig.setInterpolationWorldTransform(moved);

			// Animated bones get their velocity from the read that follows.
			if (!rig.isStaticOrKinematicObject()) {
				const btVector3 linear = turn * rig.getLinearVelocity();
				const btVector3 angular = turn * rig.getAngularVelocity();
				rig.setLinearVelocity(linear);
				rig.setAngularVelocity(angular);
				rig.setInterpolationLinearVelocity(linear);
				rig.setInterpolationAngularVelocity(angular);
			}
		}
	}
}
