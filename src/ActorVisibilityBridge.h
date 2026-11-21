#pragma once

#include <cstdint>

namespace hdt
{
	// Optional runtime link to ActorVisibility.dll (separate MIT-licensed SKSE plugin). Only its two plain C
	// exports are resolved with GetProcAddress, so there is no header or link dependency. When the DLL is
	// absent or its API version differs, GetState() always returns kUnknown and FSMP's stock culling applies.
	namespace ActorVisibilityBridge
	{
		enum class State : std::uint32_t
		{
			kUnknown = 0,
			kVisible = 1,
			kOffscreen = 2,
			kOccluded = 3,
		};

		// Called once at SKSE kPostLoad.
		void Load();

		bool IsAvailable();

		// Thread-safe, cheap (binary search in ActorVisibility's published snapshot).
		State GetState(std::uint32_t formID);
	}
}
