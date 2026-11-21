#include "hdtWorldCollider.h"

#include "dhdtPapyrusFunctions.h"
#include "hdtSeatHug.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <cmath>

namespace hdt
{
	// Defined in ActorManager.cpp (namespace hdt, external linkage).
	RE::NiNode* getNpcNode(RE::NiNode* skeleton);

	namespace
	{
		btEmptyShape g_worldEmptyShape;

		constexpr std::uint32_t kMaxTriangles = 8192;
		constexpr std::uint32_t kMaxVertices = 3 * kMaxTriangles;
		constexpr float kMaxCoord = 1.0e7f;

		bool finite3(const float* p)
		{
			return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
			       std::abs(p[0]) < kMaxCoord && std::abs(p[1]) < kMaxCoord && std::abs(p[2]) < kMaxCoord;
		}

		RE::NiNode* npcOf(RE::NiAVObject* root)
		{
			auto node = root ? root->AsNode() : nullptr;
			return node ? getNpcNode(node) : nullptr;
		}
	}

	// ---------------------------------------------------------------- bone

	WorldColliderBone::WorldColliderBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci, const btVector3& origin) :
		SkinnedMeshBone(name, ci)
	{
		m_rig.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
		m_fixed.setIdentity();
		m_fixed.setOrigin(origin);
		m_currentTransform = btQsTransform(btQuaternion::getIdentity(), origin, 1.0f);
		m_rig.setWorldTransform(m_fixed);
		m_rig.setInterpolationWorldTransform(m_fixed);
	}

	void WorldColliderBone::readTransform(float)
	{
		// Static: restore the fixed transform every frame (the world applies a temporary
		// translation offset during each step) and keep it motionless.
		static const btVector3 zero(0, 0, 0);
		m_currentTransform = btQsTransform(btQuaternion::getIdentity(), m_fixed.getOrigin(), 1.0f);
		m_rig.setWorldTransform(m_fixed);
		m_rig.setInterpolationWorldTransform(m_fixed);
		m_rig.setLinearVelocity(zero);
		m_rig.setAngularVelocity(zero);
		m_rig.setInterpolationLinearVelocity(zero);
		m_rig.setInterpolationAngularVelocity(zero);
	}

	AttachedColliderBone::AttachedColliderBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci, RE::NiNode* node) :
		SkinnedMeshBone(name, ci), m_node(node)
	{
		m_rig.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
		readTransform(RESET_PHYSICS);
	}

	void AttachedColliderBone::readTransform(float timeStep)
	{
		// Main thread (SkyrimPhysicsWorld::doUpdate). The vertices follow m_currentTransform; the rig (empty shape)
		// is kept on the node with kinematic velocities, like SkyrimBone does for animated bones.
		if (!m_node)
			return;
		const btTransform current = m_rig.getWorldTransform();
		m_currentTransform = convertNi(m_node->world);
		const btTransform dest = m_currentTransform.asTransform();
		static const btVector3 zero(0, 0, 0);
		if (timeStep <= RESET_PHYSICS) {
			m_rig.setLinearVelocity(zero);
			m_rig.setAngularVelocity(zero);
			m_rig.setInterpolationLinearVelocity(zero);
			m_rig.setInterpolationAngularVelocity(zero);
		} else {
			btVector3 linVel, angVel;
			btTransformUtil::calculateVelocity(current, dest, timeStep, linVel, angVel);
			m_rig.setLinearVelocity(linVel);
			m_rig.setAngularVelocity(angVel);
			m_rig.setInterpolationLinearVelocity(linVel);
			m_rig.setInterpolationAngularVelocity(angVel);
		}
		m_rig.setWorldTransform(dest);
		m_rig.setInterpolationWorldTransform(dest);
	}

	// ---------------------------------------------------------------- body

	bool WorldColliderBody::allows(const SkinnedMeshBody* other) const
	{
		if (!other || other == this || other->m_isWorldCollider || other->m_isKinematic)
			return false;

		// All SMP bodies in the world are SkyrimBody (SkyrimBody::canCollideWith relies on the same).
		auto sb = static_cast<const SkyrimBody*>(other);
		if (sb->m_disabled || !sb->m_mesh)
			return false;

		static const RE::BSFixedString worldTag(world::WORLD_TAG);
		if (other->m_noCollideWithTags.contains(worldTag))
			return false;

		if (m_ownerFormID) {
			auto skel = sb->m_mesh->m_skeleton.get();
			if (!skel)
				return false;
			if (skel != m_ownerNpc3rd.load(std::memory_order_relaxed) && skel != m_ownerNpc1st.load(std::memory_order_relaxed))
				return false;
		}
		return true;
	}

	bool WorldColliderBody::canCollideWith(const SkinnedMeshBody* body) const
	{
		return allows(body);
	}

	// ---------------------------------------------------------------- system

	WorldColliderSystem::WorldColliderSystem() :
		SkyrimSystem(nullptr)
	{
		m_windFactor = 0.f;  // applyWind skips the system
		block_resetting = false;
	}

	float WorldColliderSystem::prepareForRead(float timeStep)
	{
		// Main thread. Resolve the owner's SMP skeleton ("NPC" nodes) for the pair filter.
		if (m_body && m_body->m_ownerFormID) {
			RE::NiNode* npc3 = nullptr;
			RE::NiNode* npc1 = nullptr;
			if (auto actor = RE::TESForm::LookupByID<RE::Actor>(m_body->m_ownerFormID)) {
				npc3 = npcOf(actor->Get3D1(false));
				npc1 = npcOf(actor->Get3D1(true));
			}
			m_body->m_ownerNpc3rd.store(npc3, std::memory_order_relaxed);
			m_body->m_ownerNpc1st.store(npc1, std::memory_order_relaxed);
		}
		return timeStep;
	}

	RE::BSTSmartPointer<WorldColliderSystem> WorldColliderSystem::build(const world::ColliderDesc& d, std::uint32_t id, RE::NiNode* attachNode)
	{
		if (!d.positions || !d.indices || d.vertexCount < 3 || d.triangleCount == 0 ||
			d.vertexCount > kMaxVertices || d.triangleCount > kMaxTriangles) {
			logger::warn("[world] collider {}: rejected (vertices {}, triangles {})", id, d.vertexCount, d.triangleCount);
			return nullptr;
		}

		// Origin = centre of the vertex AABB; vertices are stored relative to it (float precision).
		btVector3 mn(FLT_MAX, FLT_MAX, FLT_MAX), mx(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (std::uint32_t i = 0; i < d.vertexCount; ++i) {
			const float* p = d.positions + 3 * i;
			if (!finite3(p)) {
				logger::warn("[world] collider {}: rejected (vertex {} not finite)", id, i);
				return nullptr;
			}
			btVector3 v(p[0], p[1], p[2]);
			mn.setMin(v);
			mx.setMax(v);
		}
		btVector3 origin = (mn + mx) * 0.5f;
		// attached: vertices are stored in the node's frame (inverse of its world transform, scale included)
		btQsTransform toLocal(btQuaternion::getIdentity(), -origin, 1.0f);
		if (attachNode) {
			toLocal = convertNi(attachNode->world).inverse();
			origin = btVector3(0, 0, 0);
		}

		auto system = hdt::make_smart(new WorldColliderSystem());

		std::string nameStr = fmt::format("SMPWorldCollider{}", id);
		RE::BSFixedString name(nameStr.c_str());

		btRigidBody::btRigidBodyConstructionInfo ci(0, nullptr, &g_worldEmptyShape);
		ci.m_friction = d.friction;
		SkinnedMeshBone* bone;
		if (attachNode)
			bone = new AttachedColliderBone(name, ci, attachNode);
		else
			bone = new WorldColliderBone(name, ci, origin);
		bone->m_marginMultipler = 1.0f;
		system->m_bones.push_back(hdt::make_smart<SkinnedMeshBone>(bone));

		auto body = hdt::make_smart(new WorldColliderBody());
		body->m_name = name;
		body->m_isWorldCollider = true;
		body->m_mesh = system.get();
		body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
		body->m_ownerFormID = d.ownerFormID;
		body->m_friction = std::max(0.f, d.friction);
		body->m_tags.push_back(RE::BSFixedString(world::WORLD_TAG));

		float radius = 0.f;
		for (std::uint32_t i = 0; i < d.vertexCount; ++i) {
			const float* p = d.positions + 3 * i;
			radius = std::max(radius, (toLocal * btVector3(p[0], p[1], p[2])).length());
		}
		const btQsTransform identity(btQuaternion::getIdentity(), btVector3(0, 0, 0), 1.0f);
		body->addBone(bone, identity, BoundingSphere(btVector3(0, 0, 0), radius + std::max(d.margin, 0.f) + 1.f));

		body->m_vertices.resize(d.vertexCount);
		for (std::uint32_t i = 0; i < d.vertexCount; ++i) {
			const float* p = d.positions + 3 * i;
			auto& v = body->m_vertices[i];
			v.m_skinPos = toLocal * btVector3(p[0], p[1], p[2]);
			v.m_weight[0] = 1.f;
			v.m_weight[1] = v.m_weight[2] = v.m_weight[3] = 0.f;
			v.m_boneIdx[0] = v.m_boneIdx[1] = v.m_boneIdx[2] = v.m_boneIdx[3] = 0;
		}

		auto shape = RE::make_smart<PerTriangleShape>(body.get());
		shape->m_shapeProp.margin = std::max(d.margin, 0.f);
		shape->m_shapeProp.penetration = d.penetration;

		std::uint32_t added = 0;
		for (std::uint32_t t = 0; t < d.triangleCount; ++t) {
			const auto a = d.indices[3 * t], b = d.indices[3 * t + 1], c = d.indices[3 * t + 2];
			if (a >= d.vertexCount || b >= d.vertexCount || c >= d.vertexCount || a == b || b == c || a == c)
				continue;
			const auto& pa = body->m_vertices[a].m_skinPos;
			if ((body->m_vertices[b].m_skinPos - pa).cross(body->m_vertices[c].m_skinPos - pa).length2() < 1e-8f)
				continue;  // degenerate
			shape->addTriangle(static_cast<int>(a), static_cast<int>(b), static_cast<int>(c));
			++added;
		}
		if (!added) {
			logger::warn("[world] collider {}: rejected (no valid triangles)", id);
			return nullptr;
		}

		body->finishBuild();

		system->m_body = body.get();
		system->m_meshes.push_back(hdt::make_smart<SkinnedMeshBody>(body.get()));

		if (attachNode)
			logger::info("[world] collider {} attached to node '{}'", id, attachNode->name.c_str());
		logger::info("[world] collider {} built: {} triangles, {} vertices, origin ({:.1f}, {:.1f}, {:.1f}), radius {:.1f}, margin {:.2f}, penetration {:.2f}, friction {:.2f}, owner {:08X}",
			id, added, body->m_vertices.size(), origin.x(), origin.y(), origin.z(), radius, d.margin, d.penetration, d.friction, d.ownerFormID);
		return system;
	}

	// ---------------------------------------------------------------- manager

	WorldColliderManager* WorldColliderManager::get()
	{
		static WorldColliderManager s;
		return &s;
	}

	world::Handle WorldColliderManager::addCollider(const world::ColliderDesc& desc)
	{
		return add(desc, nullptr);
	}

	world::Handle WorldColliderManager::addAttachedCollider(const world::ColliderDesc& desc, std::uint32_t attachActorFormID, const char* nodeName)
	{
		if (!attachActorFormID || !nodeName || !*nodeName)
			return 0;
		auto actor = RE::TESForm::LookupByID<RE::Actor>(attachActorFormID);
		auto root = actor ? actor->Get3D1(false) : nullptr;
		auto obj = root ? root->GetObjectByName(RE::BSFixedString(nodeName)) : nullptr;
		auto node = obj ? obj->AsNode() : nullptr;
		if (!node) {
			logger::warn("[world] attached collider: node '{}' not found on {:08X}", nodeName, attachActorFormID);
			return 0;
		}
		return add(desc, node);
	}

	world::Handle WorldColliderManager::add(const world::ColliderDesc& desc, RE::NiNode* attachNode)
	{
		auto world = SkyrimPhysicsWorld::get();
		if (!world || world->disabled)
			return 0;

		world::Handle handle;
		{
			std::lock_guard l(m_mapLock);
			handle = m_nextHandle++;
			if (!m_nextHandle)
				m_nextHandle = 1;
		}

		auto system = WorldColliderSystem::build(desc, handle, attachNode);
		if (!system)
			return 0;

		world->addSkinnedMeshSystem(system.get());

		std::lock_guard l(m_mapLock);
		m_colliders.emplace(handle, std::move(system));
		return handle;
	}

	bool WorldColliderManager::removeCollider(world::Handle handle)
	{
		RE::BSTSmartPointer<WorldColliderSystem> system;
		{
			std::lock_guard l(m_mapLock);
			auto it = m_colliders.find(handle);
			if (it == m_colliders.end())
				return false;
			system = std::move(it->second);
			m_colliders.erase(it);
		}
		if (system->m_world)
			SkyrimPhysicsWorld::get()->removeSkinnedMeshSystem(system.get());
		logger::info("[world] collider {} removed", handle);
		return true;
	}

	void WorldColliderManager::removeAll()
	{
		std::unordered_map<world::Handle, RE::BSTSmartPointer<WorldColliderSystem>> all;
		{
			std::lock_guard l(m_mapLock);
			all.swap(m_colliders);
		}
		for (auto& [handle, system] : all) {
			if (system->m_world)
				SkyrimPhysicsWorld::get()->removeSkinnedMeshSystem(system.get());
		}
		if (!all.empty())
			logger::info("[world] removed all {} colliders", all.size());
	}

	std::uint32_t WorldColliderManager::count() const
	{
		std::lock_guard l(m_mapLock);
		return static_cast<std::uint32_t>(m_colliders.size());
	}

	bool WorldColliderManager::swapPhysicsFile(std::uint32_t actorFormID, const char* oldPath, const char* newPath)
	{
		if (!actorFormID || !oldPath || !newPath || !*oldPath || !*newPath)
			return false;
		const bool ok = papyrus::impl::SwapPhysicsFileImpl(actorFormID, oldPath, newPath, false, false);
		logger::info("[world] swapPhysicsFile {:08X} '{}' -> '{}': {}", actorFormID, oldPath, newPath, ok);
		return ok;
	}

	std::uint32_t WorldColliderManager::setSeatedHug(std::uint32_t actorFormID, const world::HugParams* params, const char* const* skipPaths, std::uint32_t skipCount)
	{
		return seathug::set(actorFormID, params, skipPaths, skipCount);
	}

	std::uint32_t WorldColliderManager::freezeSeatedHug(std::uint32_t actorFormID)
	{
		return seathug::freeze(actorFormID);
	}

	void WorldColliderManager::setHugOverride(const char* path, const world::HugParams* params)
	{
		seathug::setOverride(path, params);
	}

	void WorldColliderManager::clearHugOverrides()
	{
		seathug::clearOverrides();
	}
}
