# gst_gzimgsrc Requirements

## Goal

`gzimgsrc` is a GStreamer source element that subscribes to a Gazebo image
topic, converts each received Gazebo image message into a `GstBuffer`, and
pushes the frame into the downstream GStreamer pipeline.

## Inputs

- The element must expose a `topic` property.
- The `topic` property selects the Gazebo image topic to subscribe to.
- The default topic is `/camera/image`.
- The source image data comes from Gazebo image messages.

## Output Behavior

- The element must output GStreamer buffers containing the image payload.
- The output resolution must match the source message width and height.
- The output caps must describe the produced video frames using valid
  GStreamer `video/x-raw` caps.
- The element must preserve the source image format when a matching GStreamer
  raw video format exists.

## Timing and Latency

- The element must preserve the source frame rate as closely as possible.
- The element must minimize latency between receiving a Gazebo image message
  and pushing the corresponding GStreamer buffer.
- The source should behave as a live source.

## Format Support

- The element must support Gazebo image formats that can be represented as
  GStreamer raw video caps.
- Unsupported Gazebo image formats must be handled explicitly, without pushing
  buffers with incorrect caps.

## Test Pipeline

Use the built plugin from the local `build` directory and display incoming
frames with `autovideosink`:

```sh
GST_PLUGIN_PATH=/home/user/projects/bt_ws/gst_gzimgsrc/build \
gst-launch-1.0 -v \
  gzimgsrc topic=/camera \
  ! videoconvert \
  ! autovideosink
```

Change `/camera` if the Gazebo image topic is different.

For caps negotiation and plugin debugging, run:

```sh
GST_DEBUG=gzimgsrc:6,GST_CAPS:5 \
GST_PLUGIN_PATH=/home/user/projects/bt_ws/gst_gzimgsrc/build \
gst-launch-1.0 -v \
  gzimgsrc topic=/camera \
  ! videoconvert \
  ! autovideosink
```

## Acceptance Criteria

- A user can configure the Gazebo topic through the `topic` property.
- Frames received from Gazebo are pushed downstream as GStreamer buffers.
- Width, height, and caps match the received image data.
- Frame timing follows the incoming Gazebo stream.
- Unsupported formats fail or are skipped clearly instead of producing invalid
  output.

## documentation
- update readme.md with gazebo dependencies to install 
- add the test pipe