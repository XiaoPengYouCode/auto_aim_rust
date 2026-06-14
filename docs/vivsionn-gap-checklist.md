# vivsionn Gap Checklist

This checklist tracks the functional gaps found while comparing this Rust workspace with `/Users/flamingo/Projects/robomaster/vivsionn`.

| feature | target repository | current repository | notes |
|---|---|---|---|
| [x] P0 true CAN TX/RX loop | `Serial + SocketCAN` reads `0x203/0x204` feedback and sends `0x100` control frames | SocketCAN runtime task opens `can0`, pairs feedback frames, feeds `SensData`, and sends serialized `CtrlData` | Implemented in `rbt_comm_device` and wired into `auto_aim_async` |
| [x] P0 armor pitch ballistic control | Armor fire control computes gravity-compensated pitch and sends it with yaw | Armor route now computes ballistic pitch from planner target position and sends it with yaw | Implemented in armor fire-control controller |
| [x] P0 YPD geometry recovery | Tracker recovers after armor jump/mismatch by gating windows and inflating geometry covariance | Rust tracker now opens a recovery window after multi-armor observations and inflates `dr/h` covariance after consecutive geometry mismatches | Implemented in YPD tracker with configurable thresholds |
| [x] P0 outpost specialization | Outpost path has height phase lock, radius prior, and outpost yaw recovery | Rust tracker now converts outpost observed/radial yaw, locks height phase, freezes locked height offsets, applies radius prior, and gates rejected updates | Implemented in YPD tracker with outpost-specific tests |
| [ ] P0 energy mechanism R center / switch gate | Buff detector corrects R center, gates target switching, and has contour/template fallback | Rust energy mechanism decode still needs R center correction and switching gates | Required for stable energy mechanism detection |
