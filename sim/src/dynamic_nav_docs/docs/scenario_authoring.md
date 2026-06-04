# Scenario Authoring

Scenario YAML files live in `dynamic_nav_worlds/config/scenarios`.

Required fields:

- `name`
- `robot_start`
- `goals`
- `density_profiles`
- `pedestrian_routes`
- `cart_routes`

Routes are waypoint lists in the world frame. The actor manager samples these
deterministically from the configured seed, so the same scenario and seed produce
the same crowd.

Density profiles control actor counts:

```yaml
density_profiles:
  low: {pedestrians: 8, carts: 1}
  medium: {pedestrians: 18, carts: 2}
```

World files live separately under `worlds/classic` and `worlds/ignition`. Keep
the walkable coordinate frame consistent between backends so benchmark configs
can be shared.
