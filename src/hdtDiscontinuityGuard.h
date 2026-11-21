#pragma once

#include "hdtSkinnedMesh/hdtBulletHelper.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

namespace hdt
{
	// Limits above which the motion of a skeleton between two physics reads is not something the
	// simulation can follow. Speeds are measured against the simulated time of the step, which is what
	// the solver sees: a long frame squeezed into maxSubSteps ticks counts as fast motion.
	struct DiscontinuitySettings
	{
		bool enabled = true;
		// Skyrim units per second of the skeleton root. Above it the actor was teleported (or a long
		// frame was compressed), and the bodies are carried along instead of being dragged.
		float teleportSpeedLimit = 1500.0f;
		// Degrees per second of the skeleton root, same treatment.
		float teleportTurnLimit = 2000.0f;
		// Skyrim units per second of an animated bone relative to the root. When half of the animated
		// bones exceed it, the pose was swapped and the system restarts from its rest shape.
		float poseSnapSpeedLimit = 1200.0f;
		// A system that was just built (outfit equipped, actor loaded) starts in its rest shape, which
		// only fits a body that is not in the middle of a pose change. It is kept at rest until no
		// animated bone moves faster than this relative to the root (Skyrim units per second)...
		float settleSpeedLimit = 100.0f;
		// ...or until this many seconds have passed, for actors that never stand still.
		float settleTimeLimit = 1.5f;
	};

	// Watches one system's skeleton root and animated bones from read to read. The simulation turns
	// every change of an animated bone into a velocity; a teleport, a placement after a loading screen or
	// a swapped pose would become a sweep of the body through its own cloth, which then stays on the
	// wrong side. The guard removes such a jump before the bones are read, and keeps a new system at
	// rest while the body is still moving into its pose.
	class DiscontinuityGuard
	{
	public:
		enum class Action
		{
			kNone,
			kCarried,  // bodies were moved rigidly with the root; simulation state is kept
			kRestart,  // the caller must reset the system
			kHold      // still settling: the caller must reset the system
		};

		struct Finding
		{
			Action action = Action::kNone;
			float rootSpeed = 0.0f;  // units per second
			float turnSpeed = 0.0f;  // degrees per second
			float poseSpeed = 0.0f;  // fastest animated bone relative to the root, units per second
			int snappedBones = 0;
			int animatedBones = 0;
			bool released = false;  // settling ended with this read
			float heldFor = 0.0f;   // seconds spent settling, valid with kHold or released
		};

		using Bones = std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>;

		// Forgets the previous read: the next one is compared against `root`.
		void restart(const btTransform& root);

		// The system was just built or attached to another scene graph: hold it at rest until its pose
		// has settled.
		void settle();

		// Compares this read against the previous one and carries the bodies along if the root jumped.
		Finding inspect(const Bones& bones, const btTransform& root, float timeStep, const DiscontinuitySettings& settings);

	private:
		static void carry(const Bones& bones, const btTransform& delta);

		btTransform m_lastRoot = btTransform::getIdentity();
		bool m_tracking = false;
		bool m_settling = false;
		float m_heldFor = 0.0f;
		float m_calmFor = 0.0f;
	};
}
