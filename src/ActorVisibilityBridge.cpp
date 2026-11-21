#include "ActorVisibilityBridge.h"

namespace hdt::ActorVisibilityBridge
{
	namespace
	{
		using GetApiVersion_t = std::uint32_t (*)();
		using GetState_t = std::uint32_t (*)(std::uint32_t);

		constexpr std::uint32_t kExpectedApiVersion = 1;

		GetState_t g_getState = nullptr;
	}

	void Load()
	{
		g_getState = nullptr;

		const HMODULE module = ::GetModuleHandleA("ActorVisibility.dll");
		if (!module) {
			logger::info("ActorVisibility.dll not loaded; smp.actorVisibilityMode has no effect.");
			return;
		}

		const auto getApiVersion = reinterpret_cast<GetApiVersion_t>(::GetProcAddress(module, "AV_GetApiVersion"));
		const auto getState = reinterpret_cast<GetState_t>(::GetProcAddress(module, "AV_GetState"));
		if (!getApiVersion || !getState) {
			logger::warn("ActorVisibility.dll is missing AV_GetApiVersion/AV_GetState; integration disabled.");
			return;
		}

		const auto version = getApiVersion();
		if (version != kExpectedApiVersion) {
			logger::warn("ActorVisibility API version {} (expected {}); integration disabled.", version, kExpectedApiVersion);
			return;
		}

		g_getState = getState;
		logger::info("ActorVisibility API v{} connected.", version);
	}

	bool IsAvailable()
	{
		return g_getState != nullptr;
	}

	State GetState(std::uint32_t formID)
	{
		if (!g_getState)
			return State::kUnknown;
		const auto s = g_getState(formID);
		return s <= 3 ? static_cast<State>(s) : State::kUnknown;
	}
}
