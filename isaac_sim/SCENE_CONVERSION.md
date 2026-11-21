## Converting a world

Convert Tomas's `src/lwa4p_gazebo/worlds/lwa4p_<world>_gz.sdf` to USD, then clean the result in place:

```bash
ISAACSIM_PATH=<Isaac Sim directory> isaac_sim/tools/clean_world.sh \
  isaac_sim/scenes/lwa4p_<world>.usd src/lwa4p_gazebo/worlds/lwa4p_<world>_gz.sdf
```

It uses the USD libraries shipped with Isaac Sim and does not start the simulator; any Python with `pxr` (for example `usd-core`) runs `clean_world.py` as well.
It refuses a USD whose models differ from the SDF, and running it on its own output changes nothing.
It removes or fixes what the conversion and Gazebo left behind:

| Artifact | Why it is wrong | Fix |
| --- | --- | --- |
| A `PhysicsScene` in the world | `SimulationManager` adds its own, and PhysX steps the two at different rates | removed; `sim.yaml` sets the step |
| `RigidBodyAPI` on a model and on its link | PhysX adds a massless body per model | kept on the link only |
| Physics materials | restitution 1 everywhere; density 1 kg/m³; ODE's two friction directions `mu`, `mu2` written as dynamic and static friction; Gazebo grip workarounds (friction 2–3, ground 100) | removed; `sim.yaml` binds the contact materials |
| Lost `collide_bitmask` | tubes collide with the rack mesh that Tomas filtered out | collision groups |
| Materials in a second root prim `/Looks`, no default prim | the file cannot be referenced | moved under the root prim, which becomes the default prim |
| Bindings without `MaterialBindingAPI` | deprecated in newer USD | API applied |
| Black emission enabled on every material | the SDF has no `<emissive>` | removed |
| Light: intensity 100, direction dropped, deprecated attributes | dim, and straight down instead of the SDF's direction | 3000 (`--light-intensity`), the SDF's direction and colour |
| Animation time codes | the world has no animation | removed |

Tomas's SDF changes do not reach the committed USD: convert and clean the world again when his `worlds/` change.