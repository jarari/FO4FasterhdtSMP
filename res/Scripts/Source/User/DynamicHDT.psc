Scriptname DynamicHDT Native Hidden

; Replace the physics XML used by an equipped ArmorAddon on an Actor.
; The path is relative to Data, matching the path stored in the NIF.
Bool Function ReloadPhysicsFile(Actor on_actor, ArmorAddon on_ARMA_item, String new_physics_file_path, Bool persist, Bool verbose_log = false) Native Global

; Replace the first active physics system using old_physics_file_path on an Actor.
Bool Function SwapPhysicsFile(Actor on_actor, String old_physics_file_path, String new_physics_file_path, Bool persist, Bool verbose_log = false) Native Global

; Return the current physics XML path for an equipped ArmorAddon.
String Function QueryCurrentPhysicsFile(Actor on_actor, ArmorAddon on_ARMA_item, Bool verbose_log = false) Native Global

; Set matching physics bones dynamic (on = true) or kinematic/frozen (on = false).
; Each return entry is that bone's previous state; false also means not found.
Bool[] Function TogglePhysics(Actor actor, String[] boneNames, Bool on) Native Global

; Full rebuilds from the source/reference pose. Soft rebuilds retain the live pose.
Function ResetPhysics(Actor actor, Bool full) Native Global
