# Restaurant Bringup Debug

## Stage1 Real Integration

Primary launch:

```bash
roslaunch seu_restaurant_bringup restaurant_stage1_real.launch use_mock_perception:=false launch_camera:=false
```

If `launch_camera:=false`, start D435i first in another terminal:

```bash
roslaunch realsense2_camera rs_camera.launch enable_color:=true enable_depth:=true align_depth:=true color_width:=640 color_height:=480 color_fps:=30 depth_width:=640 depth_height:=480 depth_fps:=30
```

## Perception Debug

The dedicated perception debug launch keeps the experiment to two terminals by default.

Launch command:

```bash
roslaunch seu_restaurant_bringup restaurant_perception_debug.launch launch_camera:=false use_mock_perception:=false enable_visualizer:=true enable_smach:=false image_topic:=/camera/color/image_raw
```

Second terminal:

```bash
rqt_image_view /restaurant/perception/debug_image
```

If `image_view` is preferred, set `enable_image_view:=true` in the launch and keep everything in one launch terminal plus one optional monitor terminal.

## Test Steps

Single person:

1. Stand inside the camera FOV.
2. Raise one hand above the shoulder and wave laterally for several frames.
3. Check that the panel shows one tracked person, `waving`, increasing `wave_score`, and a `Top-2` / `published` transition when thresholds are met.

Two people:

1. Place two people at different lateral positions.
2. Let one person wave first, then both wave.
3. Check that queue order, `kept_track_ids`, `Top-2`, and memory `waiting/selected/serving/served` stay readable and stable.

Multiple people:

1. Put three or more people in view.
2. Wave with different amplitudes and timing.
3. Confirm only the best current Top-2 tracks are marked as kept, while others remain visible with their track state and suppression information.

## Notes

- Default mode does not start the full SMACH pipeline and does not publish any autonomous navigation goal.
- If `customer_memory_debug` is absent, the visualizer still draws candidates, Top-2, published state, and shows `memory=unavailable` / `customer_memory_debug unavailable` in the side panel instead of failing.

## Multi-Person Tracking Stability Validation

Use the same debug launch:

```bash
roslaunch seu_restaurant_bringup restaurant_perception_debug.launch launch_camera:=false use_mock_perception:=false enable_visualizer:=true enable_smach:=false image_topic:=/camera/color/image_raw
```

Monitor with:

```bash
rqt_image_view /restaurant/perception/debug_image
rostopic echo /restaurant/perception/wave_candidates
rostopic echo /restaurant/perception/selection_debug
```

Recommended scenarios:

1. Two people standing still, both waving.
   Check that `track_id`, `state=active`, `hits`, `miss_count`, and `kept_track_ids` remain stable for both.
2. Two people waving while moving slowly.
   Check that `smoothed_center`, `smoothed_bbox`, `smoothed_wave_score`, and `top2_scores` change smoothly without frequent ID swaps.
3. Three people in view, with two waving steadily.
   Check that only the two strongest stable tracks stay in Top-2, and the third is visible but listed in `suppressed_reasons`.
4. Two people briefly crossing or partially occluding each other.
   Check that one or both tracks may become `lost` briefly, then reconnect with the same `track_id` before deletion.
5. Short miss / short re-entry.
   Check that `miss_count` increases, `state` changes to `lost`, and the same `track_id` returns if the person reappears within `track_lost_timeout_sec`.

Primary fields to watch:

- `/restaurant/perception/wave_candidates`
  Watch `tracks[].track_id`, `tracks[].state`, `tracks[].hits`, `tracks[].miss_count`, `tracks[].smoothed_center`, `tracks[].smoothed_wave_score`, and `track_counts`.
- `/restaurant/perception/selection_debug`
  Watch `kept_track_ids`, `candidate_track_states`, `top2_scores`, `kept_reasons`, `suppressed_reasons`, and `replacement_events`.
- `/restaurant/perception/debug_image`
  Watch bbox labels for `state`, `hits`, `miss`, `hold`, `top2`, and the temporary replacement panel when Top-2 changes.

Practical success criteria:

- Dual-person waving no longer causes frequent `track_id` swaps.
- Short occlusion or short missed detection does not immediately delete a good track.
- Existing Top-2 members are not replaced unless the new candidate is clearly stronger.
- `replacement_events` become occasional and explainable, instead of frequent and noisy.
