#include "hdtSkyrimPhysicsWorld.h"
#include "NonActorRayCaster.h"
#include "PluginInterfaceImpl.h"
#include "WeatherManager.h"
#include "hdtPhysicsProfiler.h"

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	namespace
	{
		// Per-world-system cull state for systems NOT owned by ActorManager (Banner Physics HDT-SMP
		// banners added via addSkinnedMeshSystem). Kept outside SkyrimPhysicsWorld so that class's
		// sizeof / member offsets stay stock FSMP 4.1.1 (SMPFixes + Banner Physics Detect()).
		struct WorldSystemCullState
		{
			std::uint8_t viewFails = 0;
			bool paused = false;
			std::chrono::steady_clock::time_point lastLog{};
		};

		std::unordered_map<SkinnedMeshSystem*, WorldSystemCullState> g_worldSystemCull;
		// Keep-alive while we have soft-removed the system from m_systems (Banner Physics also holds
		// a ref; this covers any transient gap).
		std::unordered_map<SkinnedMeshSystem*, RE::BSTSmartPointer<SkinnedMeshSystem>> g_pausedWorldSystems;

		constexpr int kMaxWorldSystemRaysPerFrame = 4;

		void clearWorldSystemCull(SkinnedMeshSystem* system)
		{
			if (!system)
				return;
			g_worldSystemCull.erase(system);
			g_pausedWorldSystems.erase(system);
		}

		void zeroBoneMotion(SkinnedMeshSystem* system)
		{
			if (!system)
				return;
			for (auto& bone : system->getBones()) {
				if (!bone)
					continue;
				bone->m_rig.setLinearVelocity(btVector3(0, 0, 0));
				bone->m_rig.setAngularVelocity(btVector3(0, 0, 0));
				bone->m_rig.clearForces();
			}
		}

		RE::NiPoint3 cameraWorldPos()
		{
			if (auto* cam = RE::Main::WorldRootCamera())
				return cam->world.translate;
			return {};
		}

		// Populated in ProcessEvent(FrameEvent) before m_lock is taken (see snapshot note there).
		thread_local std::unordered_set<const SkinnedMeshSystem*> t_trackedSystems;
	}
	static const float* timeStamp = (float*)0x12E355C;

	SkyrimPhysicsWorld::SkyrimPhysicsWorld(void)
	{
		gDisableDeactivation = true;
		setGravity(btVector3(0, 0, -9.8f * scaleSkyrim));

		// https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h

		getSolverInfo().m_friction = 0;

		// This should be enabled by default, but just for clarity I put it here too
		getSolverInfo().m_splitImpulse = true;

		// Set a very low threshold so even micro-penetrations use Split Impulse
		// Too low might cause weird visuals - default is -0.04f
		getSolverInfo().m_splitImpulsePenetrationThreshold = -0.01f;

		// Default ERP2 is 0.2
		// From Bullet: error reduction for non-contact constraints
		getSolverInfo().m_erp2 = 0.15f;

		// constraint force mixing for contacts and non-contacts
		// Adds "sponginess" to collisions to absorb the constant recalculations
		// Default is 0
		getSolverInfo().m_globalCfm = 0.001f;

		// Ignore Bounciness (Restitution) on slow micro-collisions
		// If objects are moving slower than this, they will not bounce at all.
		// The default is 0.2f, but putting this here since it's very noteworthy!
		getSolverInfo().m_restitutionVelocityThreshold = 0.2f;

		// Default is = SOLVER_USE_WARMSTARTING | SOLVER_SIMD;
		// But we don't even use warm starts since we delete the manifolds every frame
		// SOLVER_SIMD nets a small performance uplift
		// SOLVER_RANDMIZE_ORDER is also possible, but I clocked a pretty heavy performance hit. Maybe make it a config option
		getSolverInfo().m_solverMode = SOLVER_SIMD;
		getSolverInfo().m_leastSquaresResidualThreshold = 0.0001f;

		m_averageInterval = m_timeTick;
		m_accumulatedInterval = 0;
	}

	void SkyrimPhysicsWorld::setProfilerCapture(bool a_enabled, std::uint64_t a_sampleFrames, std::uint64_t a_printFrames)
	{
		auto simulationLock = lockSimulation();
		physicsprofiler::setCapture(a_enabled, a_sampleFrames, a_printFrames);
	}

	SkyrimPhysicsWorld::~SkyrimPhysicsWorld(void) noexcept
	{
	}

	//void hdtSkyrimPhysicsWorld::suspend()
	//{
	//	m_suspended++;
	//}

	//void hdtSkyrimPhysicsWorld::resume()
	//{
	//	--m_suspended;
	//}

	//void hdtSkyrimPhysicsWorld::switchToSeperateClock()
	//{
	//	m_lock.lock();
	//	m_useSeperatedClock = true;
	//	m_timeLastUpdate = clock()*0.001;
	//	m_lock.unlock();
	//}

	//void hdtSkyrimPhysicsWorld::switchToInternalClock()
	//{
	//	m_lock.lock();
	//	m_useSeperatedClock = false;
	//	m_timeLastUpdate = *timeStamp;
	//	m_lock.unlock();
	//}

	SkyrimPhysicsWorld* SkyrimPhysicsWorld::get()
	{
		static SkyrimPhysicsWorld g_World;
		return &g_World;
	}

	void SkyrimPhysicsWorld::detachSystemKeepPose(SkinnedMeshSystem* system)
	{
		// Caller must already hold m_lock. Same detach as SkinnedMeshWorld::removeSkinnedMeshSystem
		// (leaves bone/Ni transforms alone so the cloth freezes in place).
		SkinnedMeshWorld::removeSkinnedMeshSystem(system);
	}

	void SkyrimPhysicsWorld::attachSystemKeepPose(SkinnedMeshSystem* system)
	{
		// Caller must already hold m_lock. Re-add without RESET_PHYSICS / without clearing
		// m_initialized — resume from the frozen pose instead of snapping to the animated bind.
		SkinnedMeshWorld::addSkinnedMeshSystemKeepPose(system);
	}

	void SkyrimPhysicsWorld::updateWorldSystemCulling()
	{
		// Caller must already hold m_lock (ProcessEvent -> doUpdate). Uses ActorManager::m_cullNonActorSkeletons.
		auto* am = ActorManager::instance();
		if (!am)
			return;

		if (am->m_cullNonActorSkeletons == 0) {
			// Setting off: resume everything we paused and drop state.
			std::vector<SkinnedMeshSystem*> resume;
			resume.reserve(g_pausedWorldSystems.size());
			for (auto& [system, hold] : g_pausedWorldSystems) {
				(void)hold;
				resume.push_back(system);
			}
			for (auto* system : resume)
				attachSystemKeepPose(system);
			g_pausedWorldSystems.clear();
			g_worldSystemCull.clear();
			return;
		}

		auto* camera = RE::Main::WorldRootCamera();
		const RE::NiPoint3 camPos = cameraWorldPos();
		NonActorRayCaster caster(camera, camPos);
		NonActorRayCaster::Params params;
		params.radiusScale = 1.25f;
		params.radiusMargin = 64.0f;
		int raysLeft = kMaxWorldSystemRaysPerFrame;
		const float minDist = am->m_minCullingDistance;
		const float minDist2 = minDist * minDist;

		// Candidates: active world systems not tracked by ActorManager, plus currently paused ones.
		// t_trackedSystems was snapshotted before m_lock (no nested AM lock here).
		std::vector<SkinnedMeshSystem*> candidates;
		candidates.reserve(m_systems.size() + g_pausedWorldSystems.size());
		for (auto& sp : m_systems) {
			auto* raw = sp.get();
			if (!raw || t_trackedSystems.count(raw))
				continue;
			if (!dynamic_cast<SkyrimSystem*>(raw))
				continue;
			candidates.push_back(raw);
		}
		for (auto& [raw, hold] : g_pausedWorldSystems) {
			(void)hold;
			if (raw && !t_trackedSystems.count(raw) &&
				std::find(candidates.begin(), candidates.end(), raw) == candidates.end())
				candidates.push_back(raw);
		}

		for (auto* raw : candidates) {
			auto* system = dynamic_cast<SkyrimSystem*>(raw);
			if (!system || !system->m_skeleton)
				continue;

			auto& st = g_worldSystemCull[raw];
			RE::NiAVObject* root3D = system->m_skeleton.get();
			RE::TESObjectREFR* owner = nullptr;
			if (auto* ud = system->m_skeleton->GetUserData()) {
				owner = ud;
				if (auto* r3 = ud->Get3D())
					root3D = r3;
			}

			// Near-camera exemption (same idea as ActorManager::isInPlayerView minCullingDistance).
			bool rawVisible = true;
			const char* rayResult = "near";
			bool inFrustum = true;
			if (root3D) {
				const auto& c = root3D->worldBound.center;
				const float dx = c.x - camPos.x;
				const float dy = c.y - camPos.y;
				const float dz = c.z - camPos.z;
				const float dist2 = dx * dx + dy * dy + dz * dz;
				if (dist2 >= minDist2) {
					inFrustum = caster.InFrustum(root3D, params);
					rawVisible = inFrustum;
					rayResult = "skipped";
					if (inFrustum && raysLeft > 0) {
						params.doRaycast = true;
						--raysLeft;
						const auto result = caster.GetResult(owner, root3D, params);
						params.doRaycast = false;
						if (result == NonActorRayCaster::Result::kObscured) {
							rawVisible = false;
							rayResult = "obscured";
						} else {
							rayResult = "clear";
						}
					} else if (!inFrustum) {
						rayResult = "offscreen";
					}
				}
			}

			if (rawVisible)
				st.viewFails = 0;
			else if (st.viewFails < 255)
				++st.viewFails;
			const bool shouldBeActive = st.viewFails < 2;

			if (spdlog::should_log(spdlog::level::debug)) {
				const auto now = std::chrono::steady_clock::now();
				if (now - st.lastLog >= std::chrono::seconds(1)) {
					st.lastLog = now;
					logger::debug("world SMP system {:p}: frustum {} ray {} fails {} -> {}",
						static_cast<const void*>(raw), inFrustum, rayResult, st.viewFails,
						shouldBeActive ? "active" : "culled");
				}
			}

			if (shouldBeActive) {
				if (st.paused) {
					attachSystemKeepPose(raw);
					g_pausedWorldSystems.erase(raw);
					st.paused = false;
				}
			} else if (!st.paused) {
				zeroBoneMotion(raw);
				g_pausedWorldSystems[raw] = hdt::make_smart(raw);
				detachSystemKeepPose(raw);
				st.paused = true;
			}
		}
	}

	void SkyrimPhysicsWorld::doUpdate(float interval)
	{
		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

		// Time passed since last computation
		m_accumulatedInterval += interval;

		// Exponential average - becomes the tick; the tick equals the average interval when the interval is stable.
		m_averageInterval += (interval - m_averageInterval) * .125f;

		// Pause/resume Banner Physics (and other non-ActorManager) world systems before we read
		// transforms. Must run while ProcessEvent holds m_lock; never call the locking add/remove wrappers.
		updateWorldSystemCulling();

		// No need to calculate physics if there is no active skeleton.
		if (!disabled && hdt::ActorManager::instance()->activeSkeletons) {
			// The tick is the given time for each computation substep. We set it to the average fps
			// to have one average computation each frame when everything is usual.
			// In case of poor fps, we set it to the configured minimum engine value (60 Hz),
			// to still allow a physics with max increments of 1/60s.
			const auto tick = std::min(m_averageInterval, m_timeTick);

			// No need to calculate physics when too little time has passed (time exceptionally short since last computation).
			// This magic value directly impacts the number of computations and the time cost of the mod...
			if (m_accumulatedInterval * 2.0f > tick) {
				// The interval is limited to a configurable number of substeps, by default 4.
				// Additional substeps happens when there is a very sudden slowdown, or when fps is lower than min-fps,
				// we have to compute for the passed time we haven't computed.
				// n substeps means that when instant fps is n times lower than usual current fps, we stop computing.
				// So, we guarantee no jitter for fps greater than min-fps / maxSubsteps.
				// For example, if min-fps = 60 and maxSubsteps = 4, we guarantee no jitter for 15+ fps,
				// at the cost of additional simulations.
				const auto remainingTimeStep = std::min(m_accumulatedInterval, tick * m_maxSubSteps);

				readTransform(remainingTimeStep);

				m_resetPc -= m_resetPc > 0;

				m_tasks.run([this, interval, tick, remainingTimeStep] { doUpdate2ndStep(interval, tick, remainingTimeStep); });
			}
		}
	}

	void SkyrimPhysicsWorld::doUpdate2ndStep(float, const float tick, const float remainingTimeStep)
	{
		if (m_suspended)
			return;

		std::lock_guard<decltype(m_lock)> l(m_lock);

		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

		LARGE_INTEGER ticks;
		int64_t startTime = 0;
		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			startTime = ticks.QuadPart;
		}

		g_pluginInterface.onPreStep({ getCollisionObjectArray(), remainingTimeStep });

		{
			BT_PROFILE("HDTSMP_doUpdate2ndStep");
			updateActiveState();
			auto offset = applyTranslationOffset();
			stepSimulation(remainingTimeStep, 0, tick);
			restoreTranslationOffset(offset);
			m_accumulatedInterval = 0;
			m_pendingTransformUpdate = true;
		}

		g_pluginInterface.onPostStep({ getCollisionObjectArray(), remainingTimeStep });

		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			int64_t endTime = ticks.QuadPart;
			QueryPerformanceFrequency(&ticks);
			// float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
			float lastProcessingTime = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
			m_2ndStepAverageProcessingTime = (m_2ndStepAverageProcessingTime + lastProcessingTime) * 0.5f;
		}

		physicsprofiler::advanceFrame();
	}

	std::unique_lock<std::mutex> SkyrimPhysicsWorld::lockSimulation()
	{
		m_tasks.wait();
		return std::unique_lock(m_lock);
	}

	btVector3 SkyrimPhysicsWorld::applyTranslationOffset()
	{
		btVector3 center;
		center.setZero();
		int count = 0;
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto rig = btRigidBody::upcast(m_collisionObjects[i]);
			if (rig) {
				center += rig->getWorldTransform().getOrigin();
				++count;
			}
		}

		if (count > 0) {
			center /= static_cast<btScalar>(count);
			for (int i = 0; i < m_collisionObjects.size(); ++i) {
				auto rig = btRigidBody::upcast(m_collisionObjects[i]);
				if (rig)
					rig->getWorldTransform().getOrigin() -= center;
			}
		}
		return center;
	}

	void SkyrimPhysicsWorld::restoreTranslationOffset(const btVector3& offset)
	{
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto rig = btRigidBody::upcast(m_collisionObjects[i]);
			if (rig) {
				rig->getWorldTransform().getOrigin() += offset;
			}
		}
	}

	void SkyrimPhysicsWorld::setWind(const RE::NiPoint3& a_point, float a_scale, uint32_t a_smoothingSamples)
	{
		if (a_smoothingSamples == 0) {
			logger::error("setWind a_smoothingSamples must be > 0; values ignored");
			return;
		}
		const auto oldValueWeight = a_smoothingSamples - 1;
		if (!btFuzzyZero((m_windSpeed - btVector3(a_point.x, a_point.y, a_point.z)).length())) {
			m_windSpeed.setValue((oldValueWeight * m_windSpeed.getX() + a_point.x * a_scale) / a_smoothingSamples, (oldValueWeight * m_windSpeed.getY() + a_point.y * a_scale) / a_smoothingSamples, (oldValueWeight * m_windSpeed.getZ() + a_point.z * a_scale) / a_smoothingSamples);
			logger::debug(
				"Wind Speed now ({:.2f}, {:.2f}, {:.2f}), target ({:.2f}, {:.2f}, {:.2f}) using {} samples.",
				m_windSpeed.getX(),
				m_windSpeed.getY(),
				m_windSpeed.getZ(),
				a_point.x * a_scale,
				a_point.y * a_scale,
				a_point.z * a_scale,
				a_smoothingSamples);
		}
	}

	void SkyrimPhysicsWorld::updateActiveState()
	{
		struct Group
		{
			std::unordered_set<RE::BSFixedString> tags;
			std::unordered_map<RE::BSFixedString, std::vector<SkyrimBody*>> list;
		};

		std::unordered_map<RE::NiNode*, Group> maps;

		RE::BSFixedString invalidString;
		for (auto& i : m_systems) {
			auto system = static_cast<SkyrimSystem*>(i.get());
			auto& map = maps[system->m_skeleton.get()];
			for (auto& j : system->meshes()) {
				auto shape = static_cast<SkyrimBody*>(j.get());
				if (!shape)
					continue;

				if (shape->m_disableTag == invalidString) {
					for (auto& k : shape->m_tags)
						map.tags.insert(k);
				} else {
					map.list[shape->m_disableTag].push_back(shape);
				}
			}
		}

		for (auto& i : maps) {
			for (auto& j : i.second.list) {
				if (i.second.tags.find(j.first) != i.second.tags.end()) {
					for (auto& k : j.second)
						k->m_disabled = true;
				} else if (j.second.size()) {
					std::sort(j.second.begin(), j.second.end(), [](SkyrimBody* a, SkyrimBody* b) {
						if (a->m_disablePriority != b->m_disablePriority)
							return a->m_disablePriority > b->m_disablePriority;
						return a < b;
					});

					for (auto& k : j.second)
						k->m_disabled = true;
					j.second[0]->m_disabled = false;
				}
			}
		}
	}

	void SkyrimPhysicsWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		auto s = dynamic_cast<SkyrimSystem*>(system);
		if (!s)
			return;

		// A real add (Banner Physics / ActorManager) wins over our soft-pause.
		clearWorldSystemCull(system);

		s->m_initialized = false;
		SkinnedMeshWorld::addSkinnedMeshSystem(system);
	}

	void SkyrimPhysicsWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

		clearWorldSystemCull(system);
		SkinnedMeshWorld::removeSkinnedMeshSystem(system);
	}

	void SkyrimPhysicsWorld::removeSystemByNode(void* root)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

		for (int i = 0; i < m_systems.size();) {
			RE::BSTSmartPointer<SkyrimSystem> s = hdt::make_smart(dynamic_cast<SkyrimSystem*>(m_systems[i].get()));
			if (s && s->m_skeleton == root) {
				clearWorldSystemCull(s.get());
				SkinnedMeshWorld::removeSkinnedMeshSystem(s.get());
			}

			else
				++i;
		}
		// Also drop any soft-paused world system whose skeleton matches.
		std::vector<SkinnedMeshSystem*> dropPaused;
		for (auto& [sys, hold] : g_pausedWorldSystems) {
			(void)hold;
			auto* s = dynamic_cast<SkyrimSystem*>(sys);
			if (s && s->m_skeleton == root)
				dropPaused.push_back(sys);
		}
		for (auto* sys : dropPaused)
			clearWorldSystemCull(sys);
	}

	void SkyrimPhysicsWorld::resetSystems()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		for (auto& i : m_systems)
			i->readTransform(i->prepareForRead(RESET_PHYSICS));
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*)
	{
		auto mm = RE::UI::GetSingleton();

		if ((e->gamePaused || mm->GameIsPaused()) && !m_suspended) {
			suspend();
		} else if (!(e->gamePaused || mm->GameIsPaused()) && m_suspended) {
			resume();
		}

		if (m_enableWind) {
			WeatherManager::runWeatherTick(RE::GetSecondsSinceLastFrame());
		}

		LARGE_INTEGER ticks;
		int64_t startTime = 0;
		int64_t endTime = 0;
		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			startTime = ticks.QuadPart;
		}

		// Normally each frame's background step is already drained by the intervening FrameSync, so this
		// is a no-op. But if two frame events ever fire before a frame-sync event (observed once in the
		// wild, likely a mod-induced event-order anomaly), the previous step could still be running when
		// we start writing bone transforms below in readTransform -- a data race that corrupts the step's
		// memory. Wait for the queue to drain here, BEFORE taking m_lock, so the still-running step (which
		// takes m_lock itself) can finish rather than deadlocking against us.
		m_tasks.wait();

		// Snapshot ActorManager-tracked systems BEFORE taking m_lock to avoid lock-order inversion
		// (ActorManager holds its lock then calls add/removeSkinnedMeshSystem which take m_lock).
		t_trackedSystems.clear();
		if (auto* am = ActorManager::instance()) {
			std::vector<const SkinnedMeshSystem*> tracked;
			{
				auto lock = am->lockGuard();
				am->snapshotTrackedSystems(tracked);
			}
			t_trackedSystems.insert(tracked.begin(), tracked.end());
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);

		float interval = (m_useRealTime ? RE::BSTimer::GetSingleton()->realTimeDelta : RE::BSTimer::GetSingleton()->delta);

		if (interval > FLT_EPSILON && !m_suspended && !m_systems.empty()) {
			doUpdate(interval);
		} else if (m_suspended && !m_loading) {
			writeTransform();
		}

		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			endTime = ticks.QuadPart;
			QueryPerformanceFrequency(&ticks);
			// float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
			m_SMPProcessingTimeInMainLoop = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::FrameSyncEvent*, RE::BSTEventSource<Events::FrameSyncEvent>*)
	{
		if (m_doMetrics) {
			LARGE_INTEGER ticks, freq;
			QueryPerformanceCounter(&ticks);
			int64_t t0 = ticks.QuadPart;

			m_tasks.wait();

			QueryPerformanceCounter(&ticks);
			int64_t t1 = ticks.QuadPart;

			if (m_pendingTransformUpdate) {
				std::lock_guard<decltype(m_lock)> l(m_lock);
				writeTransform();
				m_pendingTransformUpdate = false;
			}

			QueryPerformanceCounter(&ticks);
			int64_t t2 = ticks.QuadPart;
			QueryPerformanceFrequency(&freq);
			float f = static_cast<float>(freq.QuadPart);

			float instWaitTime = (t1 - t0) / f * 1000.0f;
			float instWriteTime = (t2 - t1) / f * 1000.0f;
			float instSetupTime = m_SMPProcessingTimeInMainLoop;

			float instFpsImpact = instSetupTime + instWaitTime + instWriteTime;

			m_averageSMPProcessingTimeInMainLoop = (m_averageSMPProcessingTimeInMainLoop * (m_sampleSize - 1) + instFpsImpact) / m_sampleSize;

			// Smooth the individual components for logging so the math adds up perfectly visually. Kept in
			// members (not function-local static variables) so the in-game Perf tab / overlay can read the live breakdown.
			float& avgSetupTime = m_avgSetupMs;
			float& avgWaitTime = m_avgWaitMs;
			float& avgWriteTime = m_avgWriteMs;

			avgSetupTime = (avgSetupTime * (m_sampleSize - 1) + instSetupTime) / m_sampleSize;
			avgWaitTime = (avgWaitTime * (m_sampleSize - 1) + instWaitTime) / m_sampleSize;
			avgWriteTime = (avgWriteTime * (m_sampleSize - 1) + instWriteTime) / m_sampleSize;

			// The background thread's math time (this is already smoothed in doUpdate2ndStep)
			float avgBackgroundCalcTime = m_2ndStepAverageProcessingTime;

			// How much of that background math was successfully hidden?
			float avgHiddenTime = std::max(0.0f, avgBackgroundCalcTime - avgWaitTime);

			float avgTotalCpuWork = avgSetupTime + avgBackgroundCalcTime + avgWriteTime;

			logger::info(
				"[SMP Metrics] Avg Frame-time Impact: {:.2f}ms (Setup: {:.2f}, Wait: {:.2f}, Apply: {:.2f}) | Avg Hidden Time: {:.2f}ms | Avg Total CPU Work: {:.2f}ms",
				m_averageSMPProcessingTimeInMainLoop,  // This will exactly equal Setup + Wait + Apply
				avgSetupTime,
				avgWaitTime,
				avgWriteTime,
				avgHiddenTime,
				avgTotalCpuWork);
		} else {
			m_tasks.wait();
			if (m_pendingTransformUpdate) {
				std::lock_guard<decltype(m_lock)> l(m_lock);
				writeTransform();
				m_pendingTransformUpdate = false;
			}
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*)
	{
		while (m_systems.size()) {
			SkinnedMeshWorld::removeSkinnedMeshSystem(m_systems.back().get());
		}
		g_pausedWorldSystems.clear();
		g_worldSystemCull.clear();

		m_tasks.wait();

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const SKSE::CameraEvent* evn, RE::BSTEventSource<SKSE::CameraEvent>*)
	{
		if (evn && evn->oldState && evn->newState) {
			if (evn->oldState->id == RE::CameraState::kFirstPerson && evn->newState->id == RE::CameraState::kThirdPerson) {
				m_resetPc = 3;
			}
		}

		return RE::BSEventNotifyControl::kContinue;
	}
}
