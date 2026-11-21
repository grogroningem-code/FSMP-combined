#pragma once

// Flobrim fork: static "world" colliders (e.g. furniture seats) that SMP bodies collide with.
// See WorldCollisionAPI.h for the public contract.

#include "WorldCollisionAPI.h"
#include "hdtSkyrimBody.h"
#include "hdtSkyrimSystem.h"

#include <atomic>
#include <mutex>
#include <unordered_map>

namespace hdt
{
	// One kinematic bone placed at the collider's origin. Never moves.
	class WorldColliderBone : public SkinnedMeshBone
	{
	public:
		WorldColliderBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci, const btVector3& origin);

		void readTransform(float timeStep) override;
		void writeTransform() override {}

		btTransform m_fixed;
	};

	// Kinematic bone that follows a node of another skeleton (attached colliders). Never writes back to the node.
	class AttachedColliderBone : public SkinnedMeshBone
	{
	public:
		AttachedColliderBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci, RE::NiNode* node);

		void readTransform(float timeStep) override;
		void writeTransform() override {}

		RE::NiPointer<RE::NiNode> m_node;
	};

	class WorldColliderBody : public SkyrimBody
	{
	public:
		using SkyrimBody::canCollideWith;
		bool canCollideWith(const SkinnedMeshBody* body) const override;

		// Called by SkyrimBody::canCollideWith for the other side of the pair as well.
		bool allows(const SkinnedMeshBody* other) const;

		std::uint32_t m_ownerFormID = 0;
		// Resolved on the main thread each physics frame (prepareForRead); read on solver threads.
		std::atomic<RE::NiNode*> m_ownerNpc3rd{ nullptr };
		std::atomic<RE::NiNode*> m_ownerNpc1st{ nullptr };
		float m_friction = 0.5f;
	};

	class WorldColliderSystem : public SkyrimSystem
	{
	public:
		WorldColliderSystem();

		// Builds a complete system from the description. Returns null on bad input.
		static RE::BSTSmartPointer<WorldColliderSystem> build(const world::ColliderDesc& desc, std::uint32_t id, RE::NiNode* attachNode = nullptr);

		float prepareForRead(float timeStep) override;

		WorldColliderBody* body() const { return m_body; }

	private:
		WorldColliderBody* m_body = nullptr;
	};

	class WorldColliderManager final : public world::IWorldCollision
	{
	public:
		static WorldColliderManager* get();

		std::uint32_t version() const override { return VERSION; }
		world::Handle addCollider(const world::ColliderDesc& desc) override;
		bool removeCollider(world::Handle handle) override;
		void removeAll() override;
		std::uint32_t count() const override;
		bool swapPhysicsFile(std::uint32_t actorFormID, const char* oldPath, const char* newPath) override;
		std::uint32_t setSeatedHug(std::uint32_t actorFormID, const world::HugParams* params, const char* const* skipPaths, std::uint32_t skipCount) override;
		std::uint32_t freezeSeatedHug(std::uint32_t actorFormID) override;
		void setHugOverride(const char* path, const world::HugParams* params) override;
		void clearHugOverrides() override;
		world::Handle addAttachedCollider(const world::ColliderDesc& desc, std::uint32_t attachActorFormID, const char* nodeName) override;

	private:
		WorldColliderManager() = default;
		world::Handle add(const world::ColliderDesc& desc, RE::NiNode* attachNode);

		mutable std::mutex m_mapLock;
		std::unordered_map<world::Handle, RE::BSTSmartPointer<WorldColliderSystem>> m_colliders;
		world::Handle m_nextHandle = 1;
	};
}
