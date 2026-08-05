# gst_gzimgsrc Implementation Plan

This plan breaks the work needed to implement `docs/requirements.md` into
milestones that can be completed and verified independently.

## Milestone 1: Baseline Plugin Validation

Goal: Confirm the current plugin builds, loads, and exposes the required public
interface before changing behavior.

Tasks:

- Build the plugin with CMake.
- Verify `gst-inspect-1.0 gzimgsrc` finds the plugin from the local `build`
  directory.
- Confirm the element exposes the `topic` property.
- Confirm the default topic is `/camera/image`.
- Confirm the source pad advertises `video/x-raw` caps.

Validation:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
GST_PLUGIN_PATH="$PWD/build" gst-inspect-1.0 gzimgsrc
```

Exit criteria:

- The plugin builds successfully.
- `gst-inspect-1.0` shows the `gzimgsrc` element.
- The `topic` property is visible and configurable.

## Milestone 2: Gazebo Topic Subscription

Goal: Make `gzimgsrc` subscribe reliably to the configured Gazebo image topic.

Tasks:

- Use the `topic` property value when starting the source.
- Fall back to `/camera/image` when no topic is configured.
- Report a clear GStreamer error if the subscription fails.
- Stop the subscription cleanly when the element stops or is finalized.

Validation:

- Start a pipeline with the default topic.
- Start a pipeline with an explicit topic, such as `/camera`.
- Confirm logs show the selected Gazebo topic.
- Confirm stopping the pipeline releases the source without hanging.

Exit criteria:

- The element subscribes to the configured topic.
- Failed subscriptions stop the pipeline clearly.
- Shutdown does not deadlock or leave a running source thread.

## Milestone 3: Frame Conversion and Buffer Output

Goal: Convert received Gazebo image messages into downstream GStreamer buffers.

Tasks:

- Copy the Gazebo image payload into a `GstBuffer`.
- Preserve the message width and height in the output caps.
- Preserve the source pixel format when a matching GStreamer raw format exists.
- Push one output buffer for each accepted input frame, unless frames are
  intentionally dropped to minimize latency.
- Avoid pushing buffers when required image metadata is invalid.

Validation:

- Publish Gazebo image messages on the configured topic.
- Run the test pipeline and confirm buffers flow downstream.
- Confirm caps contain the expected width, height, and format.

Exit criteria:

- Received Gazebo frames appear as `GstBuffer` objects downstream.
- Output caps match the received image data.
- Invalid frames are rejected without producing incorrect caps.

## Milestone 4: Timing and Latency

Goal: Make the source behave as a live video source with low latency and timing
that follows the incoming Gazebo stream.

Tasks:

- Mark the element as a live source.
- Preserve source frame cadence as closely as possible.
- Timestamp output buffers consistently.
- Keep the buffering strategy low-latency, preferring the latest frame when the
  pipeline cannot keep up.
- Avoid blocking the Gazebo transport callback longer than necessary.

Validation:

- Run the plugin against a live Gazebo camera stream.
- Confirm the displayed video updates at the expected cadence.
- Inspect GStreamer logs for timestamp and caps negotiation issues.
- Confirm latency remains low when the downstream sink is active.

Exit criteria:

- The source behaves correctly in a live GStreamer pipeline.
- Frame timing follows the source stream closely enough for display use.
- Slow downstream processing does not cause unbounded queue growth.

## Milestone 5: Format Support

Goal: Support every Gazebo image format that can be represented as GStreamer
raw video caps, and handle unsupported formats explicitly.

Tasks:

- Inventory Gazebo `PixelFormatType` values used by the target simulator.
- Map supported Gazebo formats to GStreamer `video/x-raw` formats.
- Update the source pad template to advertise all supported formats.
- Skip or fail unsupported formats with a clear warning.
- Add checks so caps are never set to a format that does not match the buffer
  payload.

Validation:

- Test with RGB, BGR, RGBA, BGRA, and grayscale streams.
- Test at least one unsupported or unmapped format.
- Confirm supported formats negotiate successfully through `videoconvert`.
- Confirm unsupported formats produce a clear warning and no invalid output.

Exit criteria:

- Supported Gazebo image formats display through the test pipeline.
- Unsupported formats are visible in logs and do not corrupt downstream data.
- The pad template matches the actual supported output formats.

## Milestone 6: Test Pipeline and Documentation

Goal: Document how to install dependencies, build the plugin, inspect it, and
run a display pipeline.

Tasks:

- Update `README.md` with Gazebo and GStreamer dependency installation steps.
- Add the build commands.
- Add the plugin inspection command.
- Add the `autovideosink` test pipeline.
- Add a debug pipeline for caps negotiation and plugin logs.
- Keep `docs/requirements.md` aligned with the README examples.

Test pipeline:

```sh
GST_PLUGIN_PATH=/home/user/projects/bt_ws/gst_gzimgsrc/build \
gst-launch-1.0 -v \
  gzimgsrc topic=/camera \
  ! videoconvert \
  ! autovideosink
```

Debug pipeline:

```sh
GST_DEBUG=gzimgsrc:6,GST_CAPS:5 \
GST_PLUGIN_PATH=/home/user/projects/bt_ws/gst_gzimgsrc/build \
gst-launch-1.0 -v \
  gzimgsrc topic=/camera \
  ! videoconvert \
  ! autovideosink
```

Exit criteria:

- A developer can install dependencies, build, inspect, and run the plugin using
  only the README.
- The documented test pipeline matches the requirements document.
- The pipeline uses `autovideosink` and works with the configured Gazebo topic.

## Final Acceptance Checklist

- `topic` configures the Gazebo image topic.
- Gazebo image frames are pushed downstream as `GstBuffer` objects.
- Output width, height, and caps match the incoming image data.
- Frame timing follows the Gazebo stream.
- Latency is minimized from message receipt to buffer output.
- Unsupported formats are skipped or failed clearly.
- `README.md` documents dependencies, build, inspection, and test pipelines.
