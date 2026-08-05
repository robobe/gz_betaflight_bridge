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
