#pragma once

// Flobrim fork of Faster HDT-SMP: world-collision API (v3).
// v1: static world colliders. v2: swapPhysicsFile. v3: setSeatedHug (generic seated "hug").
// v4: freezeSeatedHug. v5: per-outfit hug overrides.
// Functions are only ever appended, so older consumers keep working.
//
// Lets another SKSE plugin register static triangle meshes ("world colliders", e.g. a chair's seat
// collision) that SMP cloth and hair collide with. World colliders:
//   - are kinematic: they push SMP bodies, nothing pushes them;
//   - ignore outfit can-collide-with-tag whitelists and the <shared> setting;
//   - honour an explicit <no-collide-with-tag>smp-world</no-collide-with-tag> on an SMP body (opt-out);
//   - never collide with kinematic SMP bodies (body collision shapes) or with each other.
//
// How to get the interface:
//   In SKSEPlugin_Load, call SKSE::GetMessagingInterface()->RegisterListener("hdtsmp64", handler).
//   During kPostPostLoad hdtsmp64 dispatches a message with type == MSG_WORLD_COLLISION_INTERFACE and
//   data == IWorldCollision*. Check version() before use. The pointer stays valid until shutdown.
//
// Threading: call every function from the game's main thread only (e.g. inside SKSE::GetTaskInterface()->AddTask).
//
// This header has no dependency on FSMP internals; copy it into the consumer plugin as is.

#include <cstdint>

namespace hdt::world
{
	// 'WCOL'. Chosen far away from PluginInterface::MSG_STARTUP (0).
	inline constexpr std::uint32_t MSG_WORLD_COLLISION_INTERFACE = 0x57434F4C;

	// Body tag a mesh can name in <no-collide-with-tag> to opt out of world collision.
	inline constexpr const char* WORLD_TAG = "smp-world";

	using Handle = std::uint32_t;  // 0 = invalid

	// v3: generic seated "hug". While set for an actor, every SMP item of that actor is built with extra
	// spring constraints tying its lower-skirt cloth bones to the same-side thigh, measured in the
	// skeleton's reference pose, so the cloth follows the legs when seated. No per-outfit files needed.
	// Selection, per dynamic (non-kinematic) bone of the item:
	//  - its constraint chain must reach a kinematic bone that is NOT under the spine (Spine1/Spine2),
	//    neck, head or clavicles (rejects capes, cloaks and hair);
	//  - its position along the nearer thigh (hip = 0, knee = 1) must be within [tMin, tMax];
	//  - its distance from that thigh's hip-knee axis must be <= maxDist (game units).
	// Bones behind the thigh axis get a rest offset of backLift units toward the actor's front.
	struct HugParams
	{
		std::uint32_t size = sizeof(HugParams);  // for future extension; set by the caller's header
		float tMin = 0.25f;
		float tMax = 1.15f;
		float maxDist = 20.0f;
		float linearLimit = 3.0f;      // +/- slack around the rest position (game units)
		float angularLimit = 0.6f;     // +/- radians
		float linearStiffness = 20.0f;
		float angularStiffness = 5.0f;
		float damping = 0.5f;
		float backLift = 3.5f;         // units; 0 = off
		// Optional: receives one human-readable line per rebuilt item (main thread, during the call).
		void (*log)(const char* line) = nullptr;
		// appended (fork build wc4):
		float pull = 4.0f;             // rest position moved this many units toward the thigh axis (0 = off) ...
		float surface = 6.5f;          // ... but never closer to the axis than this (approx. thigh radius)
		std::uint32_t debug = 0;       // 1 = the log line lists every dynamic bone with t / distance / decision
		// appended (fork build wc5): bones nearest the shin (knee -> ankle) are tied to the calf
		float shinMax = 1.0f;          // along the shin, 0 = knee, 1 = ankle; 0 = no shin hugging
		float shinSurface = 4.5f;      // like `surface`, for the shin
		// appended (fork build wc6):
		float pullFraction = 1.0f;     // pull = min(pull, (distance - surface) * pullFraction): <1 pulls wide cloth only partway
		float kneeT = 1.0f;            // a bone goes to the shin only when its position along the thigh is > kneeT
		float centerBand = 0.0f;       // bones whose distances to the two thighs differ by less than this (units) are left free
		// appended (fork build wc8):
		float pullMode = 0.0f;         // 0 = pull toward the leg axis; 1 = remove only the forward offset (front cloth lies on the lap, sides hang as modelled)
		// appended (fork build wc9):
		float frontArc = 0.0f;         // degrees; >0 = only panels within this angle of straight ahead are tied to the thigh, side/back panels hang freely
		float centerBlend = 0.0f;      // units; panels whose left/right thigh distances differ by less than this are tied softly to BOTH thighs (spans the gap)
		// appended (fork build wc10):
		float longSkirt = 0.0f;        // >0 = when the item's cloth bones reach further down the shin than this (0 = knee, 1 = ankle), no hug: long skirts keep plain physics + seat collision
		// appended (fork build wc11): for items skipped as long skirts, soften their own cloth constraints so gravity can drape them
		float relaxStiffness = 0.0f;   // >0 = multiply the angular spring stiffness of the skirt's constraints by this (0.1 = 10%); 0 = off
		float relaxAngle = 0.0f;       // radians: every angular limit of those constraints opens to at least +/- this
		// appended (fork build wc12):
		float groundOff = 0.0f;        // >0 = the item's "ground" shapes (VirtualGround planes on NPC Root) are switched off while this is on.
		                               // Riding lifts NPC Root to the saddle, so that plane would hold the skirt up as a flat disc.
	};

	struct ColliderDesc
	{
		// Vertex positions as x,y,z triples in Skyrim world units (game world space).
		const float* positions = nullptr;
		std::uint32_t vertexCount = 0;

		// Triangle vertex indices as triples into positions.
		const std::uint32_t* indices = nullptr;
		std::uint32_t triangleCount = 0;

		// Collision margin around each triangle, in game units (SMP's per-triangle-shape <margin>).
		float margin = 1.0f;

		// SMP's per-triangle-shape <penetration>:
		//   0  : two-sided (a point is pushed to whichever side of the triangle it is on),
		//   > 0: one-sided along the normal (v1 - v0) x (v2 - v0); points up to this deep behind the
		//        triangle are pushed out to the front,
		//   < 0: same, with the normal reversed.
		float penetration = 0.0f;

		// Contact friction used for every contact with this collider (replaces the usual
		// product of both bones' friction, so cloth whose bones declare friction 0 still grips).
		float friction = 0.5f;

		// 0: collides with the SMP bodies of every actor.
		// Otherwise: only with the SMP bodies of this actor (both its 1st- and 3rd-person skeletons).
		std::uint32_t ownerFormID = 0;
	};

	class IWorldCollision
	{
	public:
		static constexpr std::uint32_t VERSION = 6;

		virtual std::uint32_t version() const = 0;

		// Builds and adds a collider. Returns 0 on bad input or when physics is unavailable.
		virtual Handle addCollider(const ColliderDesc& desc) = 0;

		// Removes a collider. Returns false if the handle is unknown (already removed, or never existed).
		virtual bool removeCollider(Handle handle) = 0;

		// Removes every collider this interface added.
		virtual void removeAll() = 0;

		// Number of live colliders.
		virtual std::uint32_t count() const = 0;

		// ---- v2 (check version() >= 2 before calling) ----

		// Same as Papyrus DynamicHDT.SwapPhysicsFile(actor, oldPath, newPath, persist=false, verbose=false):
		// every SMP item of the actor whose current physics XML path equals oldPath exactly (as written in
		// the NIF / by a previous swap) is rebuilt from newPath, keeping its current pose.
		// Returns true if an item was swapped (or already used newPath). Not saved in the savegame.
		virtual bool swapPhysicsFile(std::uint32_t actorFormID, const char* oldPath, const char* newPath) = 0;

		// ---- v3 (check version() >= 3 before calling) ----

		// params != nullptr: turn the seated hug on for the actor (or update its params) and rebuild the
		// actor's SMP items, except items whose current physics XML path is in skipPaths (case-insensitive since Settle 1.1.1, e.g.
		// items already swapped to a hand-made seated XML). params == nullptr: turn it off and rebuild the
		// items that got a hug. Items' current poses are kept across the rebuild. Not saved in the savegame.
		// Returns the number of hug constraints now active on the actor.
		virtual std::uint32_t setSeatedHug(std::uint32_t actorFormID, const HugParams* params, const char* const* skipPaths, std::uint32_t skipCount) = 0;

		// ---- v4 (check version() >= 4 before calling) ----

		// Locks every hug constraint of the actor at the cloth's current pose relative to the legs, so the
		// settled cloth stops moving on its own and just follows the legs. Undone by setSeatedHug(actor, nullptr)
		// (or any rebuild). Returns the number of constraints locked.
		virtual std::uint32_t freezeSeatedHug(std::uint32_t actorFormID) = 0;

		// ---- v5 (check version() >= 5 before calling) ----

		// Per-outfit hug parameters: items whose physics XML path equals `path` (case-insensitive) use these
		// params instead of the actor's. nullptr removes the override. Applies to systems built afterwards.
		virtual void setHugOverride(const char* path, const HugParams* params) = 0;
		virtual void clearHugOverrides() = 0;

		// v6: like addCollider, but the collider follows a node of another actor's skeleton (e.g. a horse's spine)
		// every physics frame. desc.positions are world space at the time of the call (main thread); the node is
		// looked up by name in attachActorFormID's third-person 3D. ownerFormID still selects whose cloth collides.
		// Returns 0 if the actor or node is not found. Remove with removeCollider.
		virtual Handle addAttachedCollider(const ColliderDesc& desc, std::uint32_t attachActorFormID, const char* nodeName) = 0;

	protected:
		~IWorldCollision() = default;
	};
}
