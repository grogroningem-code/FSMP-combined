#pragma once

#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

namespace hdt
{
	class SkyrimBone : public SkinnedMeshBone
	{
	public:
		SkyrimBone(const RE::BSFixedString& name, RE::NiNode* node, RE::NiNode* skeleton, btRigidBody::btRigidBodyConstructionInfo& ci);

		void readTransform(float timeStep) override;
		void writeTransform() override;

		// Where the game's animation currently puts this bone's rigid body, in world space.
		btVector3 animatedRigOrigin() const;

		int m_depth;
		RE::NiPointer<RE::NiNode> m_node;
		RE::NiPointer<RE::NiNode> m_skeleton;

	private:
		int m_forceUpdateType;
	};
}
