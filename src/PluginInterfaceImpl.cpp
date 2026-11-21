#include "PluginInterfaceImpl.h"
#include "hdtWorldCollider.h"

hdt::PluginInterfaceImpl hdt::g_pluginInterface;

void hdt::PluginInterfaceImpl::addListener(IPreStepListener* l)
{
	if (l) {
		m_preStepDispatcher.AddEventSink(l);
	}
}

void hdt::PluginInterfaceImpl::removeListener(IPreStepListener* l)
{
	if (l) {
		m_preStepDispatcher.RemoveEventSink(l);
	}
}

void hdt::PluginInterfaceImpl::addListener(IPostStepListener* l)
{
	if (l) {
		m_postStepDispatcher.AddEventSink(l);
	}
}

void hdt::PluginInterfaceImpl::removeListener(IPostStepListener* l)
{
	if (l) {
		m_postStepDispatcher.RemoveEventSink(l);
	}
}

void hdt::PluginInterfaceImpl::onPostPostLoad()
{
	// Send ourselves to any plugin that registered during the PostLoad event
	if (m_skseMessagingInterface) {
		m_skseMessagingInterface->Dispatch(PluginInterface::MSG_STARTUP, static_cast<PluginInterface*>(this), 0, nullptr);

		// Flobrim fork: world-collision interface (WorldCollisionAPI.h)
		m_skseMessagingInterface->Dispatch(world::MSG_WORLD_COLLISION_INTERFACE,
			static_cast<world::IWorldCollision*>(WorldColliderManager::get()), sizeof(void*), nullptr);
		logger::info("[world] world-collision interface v{} dispatched", world::IWorldCollision::VERSION);
	}
}

void hdt::PluginInterfaceImpl::init(const SKSE::LoadInterface* skse)
{
	//We need to have our SKSE plugin handle and the messaging interface in order to reach our plugins later
	if (skse) {
		m_sksePluginHandle = skse->GetPluginHandle();
		m_skseMessagingInterface = reinterpret_cast<SKSE::MessagingInterface*>(skse->QueryInterface(SKSE::LoadInterface::kMessaging));
	}
	if (!m_skseMessagingInterface) {
		logger::warn("Failed to get a messaging interface. Plugins will not work.");
	}
}
