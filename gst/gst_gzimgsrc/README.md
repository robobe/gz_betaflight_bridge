# gst_gzimgsrc

`gst_gzimgsrc` provides the `gzimgsrc` GStreamer source element. The element
subscribes to a Gazebo image topic and publishes incoming frames as raw video
`GstBuffer` objects.

## Dependencies

Install the build tools, GStreamer development packages, and Gazebo transport
and message libraries before building.

On Ubuntu with Gazebo Harmonic-compatible packages:

```sh
sudo apt install \
  build-essential \
  cmake \
  pkg-config \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  libgz-transport13-dev \
  libgz-msgs10-dev
```

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

## Inspect

```sh
GST_PLUGIN_PATH="$PWD/build" gst-inspect-1.0 gzimgsrc
```

The element exposes one plugin-specific property:

- `topic`: Gazebo image topic to subscribe to. Defaults to `/camera/image`.

## Test Pipeline

Run a Gazebo camera that publishes image messages, then launch:

```sh
GST_PLUGIN_PATH="$PWD/build" \
gst-launch-1.0 -v \
  gzimgsrc topic=/camera \
  ! videoconvert \
  ! glimagesink
```

> [!WARNING]
> Use `glimagesink` for this test pipeline instead of `autovideosink`.
> `autovideosink` chooses a video sink automatically, and on some systems it
> may select a sink that does not negotiate correctly with the live raw video
> stream from `gzimgsrc`. `glimagesink` uses OpenGL video upload/rendering
> directly and has worked reliably with this plugin after `videoconvert`.

Change `/camera` if your Gazebo image topic is different.

## Python OpenCV Viewer

Use a Python environment whose OpenCV build includes GStreamer and GUI support:

```sh
python -c 'import cv2; print(cv2.getBuildInformation())' | grep GStreamer
```

With Gazebo publishing the front camera, run from this directory:

```sh
../../scripts/tools/view_camera.py
```

The viewer finds the plugin in `build/` automatically and opens this low-latency
pipeline:

```text
gzimgsrc ! videoconvert ! video/x-raw,format=BGR ! appsink
```

It defaults to `/X3/front_camera/image`. Select another topic or plugin build:

```sh
../../scripts/tools/view_camera.py --topic /camera --plugin-path /path/to/gst_gzimgsrc/build
```

The camera and recording controls share one window. Press `q` or Escape, close
the window, or press Ctrl-C to exit. The displayed FPS is calculated over the
latest second. Enter a full output path or select the directory and filename
with Browse.

Record clean frames without the FPS overlay using OpenCV `VideoWriter`:

```sh
../../scripts/tools/view_camera.py
```

Use Start Recording and Stop Recording to control capture. Recording uses MP4V,
writes clean frames at 30 FPS without the FPS overlay, and overwrites an
existing output file. The active Gazebo topic is used as the window title.
The Help button explains how to install OpenCV with GStreamer support and how
to trim recordings with FFmpeg.

For caps negotiation and plugin debugging:

```sh
GST_DEBUG=gzimgsrc:6,GST_CAPS:5 \
GST_PLUGIN_PATH="$PWD/build" \
gst-launch-1.0 -v \
  gzimgsrc topic=/camera \
  ! videoconvert \
  ! autovideosink
```

## Supported Formats

The plugin currently maps these Gazebo image formats to GStreamer raw video
caps:

- `RGB_INT8` -> `RGB`
- `BGR_INT8` -> `BGR`
- `RGBA_INT8` -> `RGBA`
- `BGRA_INT8` -> `BGRA`
- `L_INT8` -> `GRAY8`
- `L_INT16` -> `GRAY16_LE`

Unsupported formats are skipped with a warning instead of producing buffers
with incorrect caps.
