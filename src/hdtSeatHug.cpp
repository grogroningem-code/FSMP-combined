#include "hdtSeatHug.h"

#include "ActorManager.h"
#include "DynamicHDT.h"
#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtConstraintGroup.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtSkyrimSystem.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt::seathug
{
	namespace
	{
		struct ItemResult
		{
			std::uint32_t constraints = 0;
			std::string report;
		};

		struct ActorState
		{
			bool on = false;
			world::HugParams params;
			std::unordered_set<std::string> skip;  // lower-case physics paths left alone (seat-swapped or ignored)
			std::unordered_map<std::string, ItemResult> built;  // physics path -> last build result while on
			std::vector<RE::BSTSmartPointer<Generic6DofConstraint>> constraints;  // hug constraints built while on
		};

		std::mutex g_lock;
		std::unordered_map<std::uint32_t, ActorState> g_actors;

		// per-outfit parameter overrides, keyed by lower-case physics XML path (IWorldCollision v5)
		std::unordered_map<std::string, world::HugParams> g_overrides;

		std::string lower(std::string s)
		{
			for (auto& ch : s)
				ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
			return s;
		}

		// physics paths that got 0 hug constraints with g_noHugParams: not rebuilt again while those params stay
		std::unordered_set<std::string> g_noHug;
		world::HugParams g_noHugParams;

		bool sameParams(const world::HugParams& a, const world::HugParams& b)
		{
			return a.tMin == b.tMin && a.tMax == b.tMax && a.maxDist == b.maxDist && a.linearLimit == b.linearLimit &&
			       a.angularLimit == b.angularLimit && a.linearStiffness == b.linearStiffness && a.angularStiffness == b.angularStiffness &&
			       a.damping == b.damping && a.backLift == b.backLift && a.pull == b.pull && a.surface == b.surface && a.debug == b.debug &&
			       a.shinMax == b.shinMax && a.shinSurface == b.shinSurface && a.pullFraction == b.pullFraction && a.kneeT == b.kneeT && a.centerBand == b.centerBand && a.pullMode == b.pullMode &&
			       a.frontArc == b.frontArc && a.centerBlend == b.centerBlend && a.longSkirt == b.longSkirt &&
			       a.relaxStiffness == b.relaxStiffness && a.relaxAngle == b.relaxAngle && a.groundOff == b.groundOff;
		}

		// Rebuilds the actor's SMP armor items whose current physics path passes `want`, keeping poses.
		// Same procedure as DynamicHDT.SwapPhysicsFile without the path change.
		std::uint32_t rebuild(std::uint32_t formID, const std::function<bool(const std::string&)>& want, std::vector<std::string>& rebuilt)
		{
			const auto& AM = hdt::ActorManager::instance();
			auto guard = AM->lockGuard();
			std::uint32_t n = 0;
			for (auto& skeleton : AM->getSkeletons()) {
				if (!skeleton.skeleton)
					continue;
				auto owner = skeleton.skeleton->GetUserData();
				if (!owner || owner->formID != formID)
					continue;
				for (auto& armor : skeleton.getArmors()) {
					if (!armor.hasPhysics())
						continue;
					const std::string path = armor.physicsFile.first;
					if (path.empty() || !want(path))
						continue;

					const bool wasActive = armor.state() == hdt::ActorManager::ItemState::e_Active;
					RE::BSTSmartPointer<SkyrimSystem> oldSystem = armor.m_physics;
					armor.clearPhysics();

					auto renameMap = armor.renameMap;
					auto system = SkyrimSystemCreator().createOrUpdateSystem(skeleton.npc.get(), armor.armorWorn.get(), &armor.physicsFile, std::move(renameMap), oldSystem.get());
					if (system) {
						system->block_resetting = true;
						if (oldSystem)
							hdt::util::transferCurrentPosesBetweenSystems(oldSystem.get(), system.get());
						armor.setPhysics(system, wasActive);
						system->block_resetting = false;
					}
					rebuilt.push_back(path);
					++n;
				}
			}
			return n;
		}
	}

	bool lookup(std::uint32_t actorFormID, const std::string& path, world::HugParams& out)
	{
		std::lock_guard l(g_lock);
		auto it = g_actors.find(actorFormID);
		if (it == g_actors.end() || !it->second.on || it->second.skip.contains(lower(path)))
			return false;
		out = it->second.params;
		if (auto o = g_overrides.find(lower(path)); o != g_overrides.end()) {
			auto log = out.log;
			out = o->second;
			out.log = log;  // keep the caller's log sink
		}
		return true;
	}

	void noteBuilt(std::uint32_t actorFormID, const std::string& path, std::uint32_t constraints, std::string report)
	{
		std::lock_guard l(g_lock);
		auto it = g_actors.find(actorFormID);
		if (it == g_actors.end() || !it->second.on)
			return;
		if (!constraints && sameParams(it->second.params, g_noHugParams))
			g_noHug.insert(path);
		it->second.built[path] = { constraints, std::move(report) };
	}

	void setOverride(const char* path, const world::HugParams* params)
	{
		if (!path || !*path)
			return;
		std::lock_guard l(g_lock);
		if (params) {
			world::HugParams p{};
			std::memcpy(&p, params, std::min<std::size_t>(sizeof(world::HugParams), params->size ? params->size : sizeof(world::HugParams)));
			p.size = sizeof(world::HugParams);
			g_overrides[lower(path)] = p;
		} else {
			g_overrides.erase(lower(path));
		}
		g_noHug.clear();
	}

	void clearOverrides()
	{
		std::lock_guard l(g_lock);
		g_overrides.clear();
		g_noHug.clear();
	}

	void noteConstraint(std::uint32_t actorFormID, const RE::BSTSmartPointer<Generic6DofConstraint>& constraint)
	{
		std::lock_guard l(g_lock);
		auto it = g_actors.find(actorFormID);
		if (it != g_actors.end() && it->second.on)
			it->second.constraints.push_back(constraint);
	}

	std::uint32_t freeze(std::uint32_t actorFormID)
	{
		std::vector<RE::BSTSmartPointer<Generic6DofConstraint>> list;
		{
			std::lock_guard l(g_lock);
			auto it = g_actors.find(actorFormID);
			if (it == g_actors.end() || !it->second.on)
				return 0;
			list = it->second.constraints;
		}
		auto world = SkyrimPhysicsWorld::get();
		if (!world)
			return 0;
		auto simLock = world->lockSimulation();  // no physics step while frames change
		std::uint32_t n = 0;
		for (auto& c : list) {
			if (!c || !c->m_boneA || !c->m_boneB)
				continue;
			// Re-anchor at the current relative pose (cloth rig A vs leg rig B), then lock all six axes:
			// the cloth bone stays exactly where it settled relative to the leg and stops moving on its own.
			// Use the rigid bodies' world transforms: outside a step both are in true world space.
			// (m_currentTransform is NOT safe here: the leg's is refreshed by readTransform in world space
			// while the cloth's still holds the last step's translation-offset space -> huge bogus offset.)
			const btTransform& rigA = c->m_boneA->m_rig.getWorldTransform();
			const btTransform& rigB = c->m_boneB->m_rig.getWorldTransform();
			c->setFrames(btTransform::getIdentity(), rigB.inverse() * rigA);
			const btVector3 zero(0, 0, 0);
			c->setLinearLowerLimit(zero);
			c->setLinearUpperLimit(zero);
			c->setAngularLowerLimit(zero);
			c->setAngularUpperLimit(zero);
			for (int i = 0; i < 6; ++i)
				c->setEquilibriumPoint(i, 0.f);
			++n;
		}
		return n;
	}

	std::uint32_t set(std::uint32_t actorFormID, const world::HugParams* params, const char* const* skipPaths, std::uint32_t skipCount)
	{
		if (!actorFormID)
			return 0;

		void (*log)(const char*) = nullptr;
		std::function<bool(const std::string&)> want;

		{
			std::lock_guard l(g_lock);
			auto& st = g_actors[actorFormID];
			if (params) {
				st.on = true;
				st.params = world::HugParams{};
				// copy only the part of the caller's struct both sides know
				std::memcpy(&st.params, params, std::min<std::size_t>(sizeof(world::HugParams), params->size ? params->size : sizeof(world::HugParams)));
				st.params.size = sizeof(world::HugParams);
				st.skip.clear();
				for (std::uint32_t i = 0; skipPaths && i < skipCount; ++i)
					if (skipPaths[i])
						st.skip.insert(lower(skipPaths[i]));
				st.built.clear();
				st.constraints.clear();
				log = st.params.log;
				if (!sameParams(st.params, g_noHugParams)) {
					g_noHug.clear();
					g_noHugParams = st.params;
				}
				auto skip = st.skip;
				auto noHug = g_noHug;
				want = [skip, noHug](const std::string& p) { return !skip.contains(lower(p)) && !noHug.contains(p); };
			} else {
				if (!st.on) {
					g_actors.erase(actorFormID);
					return 0;
				}
				std::unordered_set<std::string> hugged;
				for (auto& [p, r] : st.built)
					if (r.constraints)
						hugged.insert(p);
				log = st.params.log;
				g_actors.erase(actorFormID);  // off before rebuilding, so the rebuild has no hug
				want = [hugged](const std::string& p) { return hugged.contains(p); };
			}
		}

		std::vector<std::string> rebuilt;
		rebuild(actorFormID, want, rebuilt);

		std::uint32_t total = 0;
		std::lock_guard l(g_lock);
		auto it = g_actors.find(actorFormID);
		for (const auto& path : rebuilt) {
			std::string line;
			if (it != g_actors.end()) {
				auto b = it->second.built.find(path);
				if (b != it->second.built.end()) {
					total += b->second.constraints;
					line = fmt::format("hug {:08X} '{}': {}", actorFormID, path, b->second.report);
				} else {
					line = fmt::format("hug {:08X} '{}': rebuilt, no hug (build failed?)", actorFormID, path);
				}
			} else {
				line = fmt::format("hug {:08X} '{}': hug removed", actorFormID, path);
			}
			if (log)
				log(line.c_str());
		}
		return total;
	}
}

namespace hdt
{
	namespace
	{
		bool nodeUnder(RE::NiAVObject* n, const std::unordered_set<std::string>& names, RE::NiAVObject* stop)
		{
			for (int depth = 0; n && depth < 64; ++depth, n = n->parent) {
				if (n == stop)
					return false;
				if (names.contains(n->name.c_str()))
					return true;
			}
			return false;
		}

		// Settle 1.1.1: tails are not skirt cloth (an equippable tail hangs from the pelvis near the thighs and was
		// tied to the legs). "tail" anywhere in a node name, case-insensitive, except detail/retail.
		bool isTailName(const char* name)
		{
			if (!name)
				return false;
			std::string s(name);
			std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			for (auto pos = s.find("tail"); pos != std::string::npos; pos = s.find("tail", pos + 1)) {
				const bool deRe = pos >= 2 && (s.compare(pos - 2, 2, "de") == 0 || s.compare(pos - 2, 2, "re") == 0);
				if (!deRe)
					return true;
			}
			return false;
		}
	}

	std::uint32_t SkyrimSystemCreator::addSeatHug(const world::HugParams& p, std::string& report)
	{
		static const RE::BSFixedString kThigh[2] = { "NPC L Thigh [LThg]", "NPC R Thigh [RThg]" };
		static const RE::BSFixedString kCalf[2] = { "NPC L Calf [LClf]", "NPC R Calf [RClf]" };
		static const RE::BSFixedString kFoot[2] = { "NPC L Foot [Lft ]", "NPC R Foot [Rft ]" };
		static const std::unordered_set<std::string> kLeg = {
			"NPC L Thigh [LThg]", "NPC R Thigh [RThg]", "NPC L Calf [LClf]", "NPC R Calf [RClf]"
		};
		static const std::unordered_set<std::string> kUpper = {
			"NPC Spine1 [Spn1]", "NPC Spine2 [Spn2]", "NPC Neck [Neck]", "NPC Head [Head]",
			"NPC L Clavicle [LClv]", "NPC R Clavicle [RClv]"
		};

		// per side: segment 0 = thigh (hip -> knee), segment 1 = shin (knee -> ankle)
		RE::NiNode* segNode[2][2];
		btVector3 segA[2][2], segB[2][2];
		for (int s = 0; s < 2; ++s) {
			auto thigh = findObjectByName(getRenamedBone(kThigh[s]));
			auto calf = findObjectByName(getRenamedBone(kCalf[s]));
			auto foot = findObjectByName(getRenamedBone(kFoot[s]));
			if (!thigh || !calf || !foot) {
				report = "no thigh/calf/foot nodes, skipped";
				return 0;
			}
			segNode[s][0] = thigh;
			segNode[s][1] = calf;
			segA[s][0] = convertNi(thigh->world.translate);
			segB[s][0] = convertNi(calf->world.translate);
			segA[s][1] = segB[s][0];
			segB[s][1] = convertNi(foot->world.translate);
		}
		const btVector3 fwd = quatRotate(convertNi(m_skeleton->world.rotate), btVector3(0, 1, 0)).normalized();

		// adjacency through every constraint of the system (plain + groups)
		std::unordered_map<SkinnedMeshBone*, std::vector<SkinnedMeshBone*>> adj;
		auto addEdges = [&](const std::vector<RE::BSTSmartPointer<BoneScaleConstraint>>& list) {
			for (auto& c : list) {
				if (!c || !c->m_boneA || !c->m_boneB)
					continue;
				adj[c->m_boneA].push_back(c->m_boneB);
				adj[c->m_boneB].push_back(c->m_boneA);
			}
		};
		addEdges(m_mesh->m_constraints);
		for (auto& g : m_mesh->m_constraintGroups)
			if (g)
				addEdges(g->m_constraints);

		auto isKinematic = [](SkinnedMeshBone* b) { return b->m_rig.isStaticOrKinematicObject(); };

		// true when the dynamic bone's chain reaches kinematic anchors and none of them is upper body
		auto lowerBodyChain = [&](SkinnedMeshBone* start) {
			std::unordered_set<SkinnedMeshBone*> seen{ start };
			std::vector<SkinnedMeshBone*> todo{ start };
			bool anchored = false;
			while (!todo.empty()) {
				auto b = todo.back();
				todo.pop_back();
				auto it = adj.find(b);
				if (it == adj.end())
					continue;
				for (auto nb : it->second) {
					if (!seen.insert(nb).second)
						continue;
					if (isKinematic(nb)) {
						auto node = static_cast<SkyrimBone*>(nb)->m_node.get();
						if (nodeUnder(node, kUpper, m_skeleton))
							return false;
						anchored = true;
					} else {
						todo.push_back(nb);
					}
				}
			}
			return anchored;
		};

		const std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> bones = m_mesh->m_bones;  // copy: leg bones may be appended
		SkyrimBone* segBone[2][2] = { { nullptr, nullptr }, { nullptr, nullptr } };
		auto getSegBone = [&](int s, int g) -> SkyrimBone* {
			if (segBone[s][g])
				return segBone[s][g];
			auto node = segNode[s][g];
			auto b = findBoneFromIndex(node->name);
			if (!b) {
				BoneTemplate kinematic;  // mass 0 -> kinematic
				b = new SkyrimBone(node->name.c_str(), node, m_skeleton, kinematic);
				b->m_localToRig = kinematic.m_centerOfMassTransform;
				b->m_rigToLocal = kinematic.m_centerOfMassTransform.inverse();
				b->readTransform(RESET_PHYSICS);
				m_mesh->m_bones.push_back(hdt::make_smart(b));
				indexBone(b);
			} else if (!b->m_rig.isStaticOrKinematicObject()) {
				return nullptr;  // an outfit simulating the leg bone itself: leave it alone
			}
			segBone[s][g] = b;
			return b;
		};

		// Panels: a cloth bone's column = the chain of dynamic bones above it in the node hierarchy; its root is the
		// topmost dynamic bone. Side (left/right thigh) and the centre test are decided once per column, so a
		// panel is never split between the legs or partly freed.
		std::unordered_map<RE::NiAVObject*, SkyrimBone*> dynByNode;
		for (auto& bp : bones) {
			auto b = static_cast<SkyrimBone*>(bp.get());
			if (b && !isKinematic(b) && b->m_node)
				dynByNode[b->m_node.get()] = b;
		}
		auto columnRoot = [&](SkyrimBone* b) {
			SkyrimBone* root = b;
			RE::NiAVObject* n = b->m_node ? b->m_node->parent : nullptr;
			for (int depth = 0; n && depth < 64; ++depth, n = n->parent) {
				auto it = dynByNode.find(n);
				if (it == dynByNode.end())
					break;
				root = it->second;
			}
			return root;
		};
		auto isTail = [&](SkyrimBone* b) {
			for (RE::NiAVObject* n = b->m_node.get(); n && n != m_skeleton; n = n->parent)
				if (isTailName(n->name.c_str()))
					return true;
			return false;
		};
		auto thighDist = [&](const btVector3& P, int s, float& t, btVector3& on) {
			const btVector3 ax = segB[s][0] - segA[s][0];
			const float len2 = std::max(ax.length2(), 1.f);
			t = (P - segA[s][0]).dot(ax) / len2;
			on = segA[s][0] + ax * std::clamp(t, 0.f, 1.f);
			return (P - on).length();
		};
		std::unordered_map<SkyrimBone*, float> colSum;  // sum over the column of (distance to left thigh - distance to right thigh)
		std::unordered_map<SkyrimBone*, int> colCount;
		std::unordered_map<SkyrimBone*, float> colFwd;  // sum over the column of cos(angle between the offset from the nearer thigh and forward)
		for (auto& [node, b] : dynByNode) {
			float t0, t1;
			btVector3 on0, on1;
			const btVector3 P = b->m_currentTransform.getOrigin();
			auto root = columnRoot(b);
			const float d0 = thighDist(P, 0, t0, on0), d1 = thighDist(P, 1, t1, on1);
			colSum[root] += d0 - d1;
			const btVector3 v = P - (d0 <= d1 ? on0 : on1);
			colFwd[root] += v.dot(fwd) / std::max(v.length(), 0.01f);
			colCount[root] += 1;
		}

		// skirt length: how far below the knee the item's cloth bones reach (0 = knee, 1 = ankle), on the nearer leg
		float reachShin = -1.f;
		for (auto& [node, b] : dynByNode) {
			if (nodeUnder(node, kLeg, m_skeleton) || !lowerBodyChain(b) || isTail(b))
				continue;
			float t0, t1;
			btVector3 on0, on1;
			const btVector3 P = b->m_currentTransform.getOrigin();
			const float d0 = thighDist(P, 0, t0, on0), d1 = thighDist(P, 1, t1, on1);
			const int s = d0 <= d1 ? 0 : 1;
			if ((s == 0 ? t0 : t1) < 0.9f)
				continue;
			const btVector3 ax = segB[s][1] - segA[s][1];
			reachShin = std::max(reachShin, (P - segA[s][1]).dot(ax) / std::max(ax.length2(), 1.f));
		}
		if (p.longSkirt > 0.f && reachShin > p.longSkirt) {
			if (p.relaxStiffness <= 0.f && p.relaxAngle <= 0.f) {
				report = fmt::format("long skirt (cloth reaches {:.2f} down the shin > fHugLongSkirt {:.2f}): no hug, plain physics + seat collision", reachShin, p.longSkirt);
				return 0;
			}
			// Relax: soften the item's own constraints between lower-body cloth bones so gravity can drape the skirt
			// (e.g. over a horse). Undone by the rebuild when the hug is turned off; counted as constraints so it is.
			auto isCloth = [&](SkinnedMeshBone* b) {
				if (!b || isKinematic(b))
					return false;
				auto node = static_cast<SkyrimBone*>(b)->m_node.get();
				return node && !nodeUnder(node, kLeg, m_skeleton) && lowerBodyChain(b);
			};
			std::uint32_t six = 0, cone = 0, other = 0;
			auto relax = [&](const std::vector<RE::BSTSmartPointer<BoneScaleConstraint>>& list) {
				for (auto& c : list) {
					if (!c || !(isCloth(c->m_boneA) || isCloth(c->m_boneB)))
						continue;
					auto tc = c->getConstraint();
					if (!tc) {
						++other;
						continue;
					}
					if (tc->getConstraintType() == D6_SPRING_2_CONSTRAINT_TYPE) {
						auto g = static_cast<btGeneric6DofSpring2Constraint*>(tc);
						if (p.relaxAngle > 0.f) {
							btVector3 lo, hi;
							g->getAngularLowerLimit(lo);
							g->getAngularUpperLimit(hi);
							for (int a = 0; a < 3; ++a) {
								if (lo[a] <= hi[a]) {  // lo > hi = already free
									lo[a] = std::min(lo[a], -p.relaxAngle);
									hi[a] = std::max(hi[a], p.relaxAngle);
								}
							}
							g->setAngularLowerLimit(lo);
							g->setAngularUpperLimit(hi);
						}
						if (p.relaxStiffness > 0.f) {
							for (int a = 0; a < 3; ++a)
								g->setStiffness(3 + a, g->getRotationalLimitMotor(a)->m_springStiffness * p.relaxStiffness);
						}
						++six;
					} else if (tc->getConstraintType() == CONETWIST_CONSTRAINT_TYPE) {
						auto ct = static_cast<btConeTwistConstraint*>(tc);
						if (p.relaxAngle > 0.f) {
							ct->setLimit(std::max(ct->getSwingSpan1(), p.relaxAngle), std::max(ct->getSwingSpan2(), p.relaxAngle),
								std::max(ct->getTwistSpan(), p.relaxAngle), ct->getLimitSoftness(), ct->getBiasFactor(), ct->getRelaxationFactor());
						}
						++cone;
					} else {
						++other;
					}
				}
			};
			relax(m_mesh->m_constraints);
			for (auto& g : m_mesh->m_constraintGroups)
				if (g)
					relax(g->m_constraints);
			report = fmt::format("long skirt (reaches {:.2f} down the shin): no hug, cloth relaxed: {} 6dof + {} cone-twist constraints (stiffness x{:.2f}, angle >= {:.2f} rad), {} other left as is",
				reachShin, six, cone, p.relaxStiffness, p.relaxAngle, other);
			return six + cone;
		}

		std::uint32_t added = 0, dynamic = 0, upper = 0, leg = 0, outOfRange = 0, back = 0, pulled = 0, shin = 0, tails = 0;
		std::string detail;
		std::string tag;
		auto note = [&](SkyrimBone* b, const char* what, const char* seg, float t, float d) {
			if (p.debug)
				detail += fmt::format("\n      {} {} t={:.2f} d={:.1f}: {}", b->m_name.c_str(), seg, t, d, what) + tag;
		};
		for (auto& bp : bones) {
			auto bone = static_cast<SkyrimBone*>(bp.get());
			if (!bone || isKinematic(bone))
				continue;
			++dynamic;
			if (nodeUnder(bone->m_node.get(), kLeg, m_skeleton)) {
				++leg;  // body soft-tissue bones hanging from the legs (e.g. 3BA thighs), not cloth
				note(bone, "under a leg node", "-", 0, 0);
				continue;
			}
			if (isTail(bone)) {
				++tails;
				note(bone, "tail, left free", "-", 0, 0);
				continue;
			}
			if (!lowerBodyChain(bone)) {
				++upper;
				note(bone, "upper-body or unanchored chain", "-", 0, 0);
				continue;
			}

			// side = nearer thigh (distance to the hip->knee line); segment = thigh, or shin when the bone's position
			// along that thigh is beyond kneeT (one cutoff, so a ring of cloth at the knee is not split)
			const btVector3 P = bone->m_currentTransform.getOrigin();
			float thighT[2], thighD[2];
			btVector3 thighOn[2];
			for (int s = 0; s < 2; ++s) {
				const btVector3 ax = segB[s][0] - segA[s][0];
				const float len2 = std::max(ax.length2(), 1.f);
				thighT[s] = (P - segA[s][0]).dot(ax) / len2;
				thighOn[s] = segA[s][0] + ax * std::clamp(thighT[s], 0.f, 1.f);
				thighD[s] = (P - thighOn[s]).length();
			}
			auto root = columnRoot(bone);
			const float colMean = colCount[root] ? colSum[root] / colCount[root] : (thighD[0] - thighD[1]);
			const int side = colMean <= 0.f ? 0 : 1;  // the whole panel follows one thigh
			if (p.centerBand > 0.f && std::fabs(colMean) < p.centerBand) {
				++outOfRange;
				note(bone, "centre panel (centerBand)", "-", thighT[side], thighD[side]);
				continue;
			}
			const float colF = colCount[root] ? colFwd[root] / colCount[root] : 1.f;
			tag = fmt::format(" [col {:+.1f}, fwd {:.2f}]", colMean, colF);
			const bool blend = p.centerBlend > 0.f && std::fabs(colMean) < p.centerBlend;
			auto hugTo = [&](int s, bool soft) -> int {
				int seg = 0;
				float bestT = thighT[s], bestD = thighD[s];
				btVector3 bestAxis = thighOn[s];
				if (thighT[s] > p.kneeT && p.shinMax > 0.f) {
					const btVector3 ax = segB[s][1] - segA[s][1];
					const float len2 = std::max(ax.length2(), 1.f);
					const float t = (P - segA[s][1]).dot(ax) / len2;
					seg = 1;
					bestT = t;
					bestAxis = segA[s][1] + ax * std::clamp(t, 0.f, 1.f);
					bestD = (P - bestAxis).length();
				}
				const char* segName = seg == 0 ? "thigh" : "shin";
				if (seg == 0 && p.frontArc > 0.f && colF < std::cos(p.frontArc * 0.0174533f)) {
					note(bone, "side/back panel left free (frontArc)", segName, bestT, bestD);
					return 0;
				}
				// range: thigh part [tMin, tMax]; shin part [0, shinMax] (shinMax 0 = shin hugging off)
				bool inRange = bestD <= p.maxDist;
				if (inRange && seg == 0)
					inRange = bestT >= p.tMin && bestT <= p.tMax;
				if (inRange && seg == 1)
					inRange = p.shinMax > 0.f && bestT <= p.shinMax;
				if (!inRange) {
					note(bone, bestD > p.maxDist ? "beyond maxDist" : (seg == 0 && bestT < p.tMin) ? "above tMin" : "out of leg range", segName, bestT, bestD);
					return 0;
				}
				auto target = getSegBone(s, seg);
				if (!target) {
					note(bone, "leg bone is simulated by the item", segName, bestT, bestD);
					return 0;
				}

				GenericConstraintTemplate c;
				c.frameType = FrameInA;
				c.frame = btTransform::getIdentity();
				// Rest offset (world): the cloth bone rests `shift` away from its reference-pose spot relative to the leg.
				// Linear diff = (B - A) in A's axes, so the equilibrium is -shift expressed in A's axes.
				btVector3 shift(0, 0, 0);
				const bool isBack = (P - bestAxis).dot(fwd) < 0.f;
				if (p.backLift > 0.f && isBack && seg == 0) {
					shift += p.backLift * fwd;  // v6 back-panel lift (thigh only)
					++back;
				}
				const float surface = seg == 0 ? p.surface : p.shinSurface;
				const bool frontOnly = p.pullMode >= 0.5f;
				const float reach = frontOnly ? (P - bestAxis).dot(fwd) : bestD;  // mode 1: forward offset only
				const float inward = std::min(p.pull, std::max(0.f, reach - surface) * std::max(0.f, p.pullFraction));
				if (inward > 0.01f && bestD > 0.01f) {
					shift += frontOnly ? -fwd * inward : (bestAxis - P) / bestD * inward;  // cloth lies on the leg
					++pulled;
				}
				const btVector3 eq = -quatRotate(bone->m_currentTransform.getBasis().inverse(), shift);
				note(bone, soft ? (s == 0 ? "blend L (centerBlend)" : "blend R (centerBlend)") : inward > 0.01f ? (isBack && seg == 0 ? "hug + pull + back lift" : "hug + pull") : (isBack && seg == 0 ? "hug + back lift" : "hug"), segName, bestT, bestD);

				const float linLim = soft ? std::max(p.linearLimit, 30.f) : p.linearLimit;
				const float angLim = soft ? std::max(p.angularLimit, 1.5f) : p.angularLimit;
				const btVector3 lim(linLim, linLim, linLim);
				c.linearLowerLimit = eq - lim;
				c.linearUpperLimit = eq + lim;
				c.linearEquilibrium = eq;
				c.angularLowerLimit = btVector3(-angLim, -angLim, -angLim);
				c.angularUpperLimit = btVector3(angLim, angLim, angLim);
				c.linearStiffness = btVector3(p.linearStiffness, p.linearStiffness, p.linearStiffness);
				c.angularStiffness = btVector3(p.angularStiffness, p.angularStiffness, p.angularStiffness);
				c.linearDamping = btVector3(p.damping, p.damping, p.damping);
				c.angularDamping = btVector3(p.damping, p.damping, p.damping);

				if (auto constraint = buildGenericConstraint(bone, target, c)) {
					m_mesh->m_constraints.push_back(constraint);
					if (auto* owner = m_skeleton->GetUserData())
						seathug::noteConstraint(owner->formID, constraint);
					++added;
					if (seg == 1)
						++shin;
					return 1;
				}
				return 0;
			};
			if (blend ? (hugTo(0, true) + hugTo(1, true)) == 0 : hugTo(side, false) == 0)
				++outOfRange;
		}

		report = fmt::format("{} hug constraints (skirt reaches {:.2f} down the shin; {} on the shins, {} pulled to the leg, {} with back lift); {} dynamic bones: {} under the legs, {} on upper-body/unanchored chains, {} tail bones, {} outside leg range{}",
			added, reachShin, shin, pulled, back, dynamic, leg, upper, tails, outOfRange, detail);
		return added;
	}
}
