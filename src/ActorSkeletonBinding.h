#pragma once

#include "LifecycleEvents.h"

#include <vector>

namespace RE
{
	class Actor;
	class BipedAnim;
	class NiAVObject;
}

namespace Smp::ActorSkeletonBinding
{
	std::vector<RE::NiAVObject*> CaptureTrustedActorSkeletonNodesBeforeAttach(
		RE::Actor* a_actor,
		RE::BipedAnim* a_biped,
		RE::NiAVObject* a_sourceObject,
		bool a_firstPerson);
	void PruneTrustedActorSkeletonNodesBySourceParents(RE::NiAVObject* a_sourceObject, std::vector<RE::NiAVObject*>& a_trustedNodes);
	std::vector<MergeParentBinding> BuildPreAttachMergeParentBindings(
		RE::NiAVObject* a_sourceObject,
		const std::vector<RE::NiAVObject*>& a_trustedActorSkeletonNodes);
}
