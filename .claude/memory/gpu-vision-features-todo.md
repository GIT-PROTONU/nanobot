---
name: gpu-vision-features-todo
description: Two planned GPU (Mali-450/GLES2) vision features the user wants built — blob-tracking bearing and a motion-diff wake trigger
metadata: 
  node_type: memory
  type: project
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

User approved 2026-07-10 (session `0511c092`): two vision features to build using the Mali-450 GPU (GLES2 fragment shaders, not OpenCV — see [[h5-gpu-only-webcam-use]]), once the GPU is un-blacklisted (`sudo rm /etc/modprobe.d/blacklist-mali-gpu.conf && reboot`, since that blacklist just got deployed).

**1. Color-threshold blob tracking → a bearing.** Shader thresholds camera frame by hue, downsample-reduces the mask to a centroid on-GPU, reads back only `(x, y, confidence)` — a few bytes, not an image. Convert x-position to a bearing via the camera FOV, publish on something like `/target`. A small CPU-side proportional controller turns/drives toward it. Unlocks: chase-a-colored-ball play skill, camera-based dock approach (put a colored marker at the charging dock — currently there's no vision-based return-to-dock, only lidar/SLAM pose), orient-before-commenting for the "looking" beat.

**2. Motion/frame-diff wake trigger.** Shader diffs current vs. previous camera frame, downsample-reduces to one "how much changed" scalar, polled cheaply every frame. Near-zero for a while = skip the autonomous "looking" beat's LLM call (nothing to comment on). A spike = something entered the frame → *that's* the trigger to actually wake up and narrate. Turns "looking" from blind-timer-driven into an actual presence/motion reflex, same shape as the existing pickup-detection reflex in `mood_node`.

**Design constraints from the cost discussion (see this session for full reasoning):**
- Both must fold into an **existing hub** (`app_hub`, which already owns the camera via `web_control`) — a standalone new rclpy process would cost ~80-150 MB just to exist, dwarfing everything else. This is the single highest-leverage decision, learned the hard way already in this codebase (it's why `sensor_hub`/`app_hub` exist at all).
- Poll/analyze at 5-10 Hz, well below the camera's ~15 fps cap — neither feature needs full frame rate, and it bounds worst-case CPU (~5-15% of one core estimated, unmeasured).
- Biggest unmeasured unknown: the Mesa/lima GLES driver's RAM footprint once loaded into a process (textures themselves are trivial, a few hundred KB-2.4 MB). Recommended first step before building either feature: a small standalone GLES2 spike script on the board measuring actual RSS before/after context creation + one threshold pass, to replace the estimate with a real number.

Not started — this is a backlog item, not in progress. No code exists yet for either feature.
