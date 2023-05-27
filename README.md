# physics

Positional-based rigid-body simulation in Bevy

Eventually to be merged into my other [constraint solver project](https://github.com/jim-ec/constraint_solver).

Differences to [Rapier](https://rapier.rs/):
- 3D single-precision only
- Built directly with Bevy's ECS, no own allocator implementation
- Position-based approach instead of a traditional impulse- and force-based approach for unconditional stability
