# vivsionn Gap Checklist

This checklist tracks the functional gaps found while comparing this Rust workspace with `/Users/flamingo/Projects/robomaster/vivsionn`.

| feature | target repository | current repository | notes |
|---|---|---|---|
| [x] P0 true CAN TX/RX loop | `Serial + SocketCAN` reads `0x203/0x204` feedback and sends `0x100` control frames | SocketCAN runtime task opens `can0`, pairs feedback frames, feeds `SensData`, and sends serialized `CtrlData` | Implemented in `rbt_comm_device` and wired into `auto_aim_async` |
| [ ] P0 armor pitch ballistic control | Armor fire control computes gravity-compensated pitch and sends it with yaw | Armor route still needs target pitch output instead of holding feedback pitch | Required for full 3D armor aiming |
| [ ] P0 YPD geometry recovery | Tracker recovers after armor jump/mismatch by gating windows and inflating geometry covariance | Rust tracker still needs online geometry recovery logic | Required for robust reacquisition |
| [ ] P0 outpost specialization | Outpost path has height phase lock, radius prior, and outpost yaw recovery | Rust outpost handling still needs target-specific recovery and yaw logic | Required for stable outpost mode |
| [ ] P0 energy mechanism R center / switch gate | Buff detector corrects R center, gates target switching, and has contour/template fallback | Rust energy mechanism decode still needs R center correction and switching gates | Required for stable energy mechanism detection |
