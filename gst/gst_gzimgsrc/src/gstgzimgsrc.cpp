#include <gst/base/gstpushsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr const char *kDefaultTopic = "/camera/image";

struct Frame
{
  std::vector<std::uint8_t> data;
  guint width = 0;
  guint height = 0;
  guint stride = 0;
  std::string gstFormat;
  GstClockTime pts = GST_CLOCK_TIME_NONE;
  GstClockTime duration = GST_CLOCK_TIME_NONE;
};

struct SourceState
{
  gz::transport::Node node;
  std::mutex mutex;
  std::condition_variable frameAvailable;
  std::optional<Frame> frame;
  bool running = false;
  GstClockTime lastFrameTime = GST_CLOCK_TIME_NONE;
  guint activeUsers = 0;
};

struct GstGzImgSrc
{
  GstPushSrc parent;
  gchar *topic = nullptr;
  SourceState *state = nullptr;
  GMutex stateLock;
  GCond stateIdle;
};

struct GstGzImgSrcClass
{
  GstPushSrcClass parentClass;
};

enum
{
  PROP_0,
  PROP_TOPIC,
};

GType gst_gz_img_src_get_type();

#define GST_TYPE_GZ_IMG_SRC (gst_gz_img_src_get_type())
#define GST_GZ_IMG_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_GZ_IMG_SRC, GstGzImgSrc))

G_DEFINE_TYPE(GstGzImgSrc, gst_gz_img_src, GST_TYPE_PUSH_SRC)

struct FormatInfo
{
  const char *gstFormat;
  guint bytesPerPixel;
};

std::optional<FormatInfo> formatInfoFromGazebo(
  gz::msgs::PixelFormatType pixelFormat)
{
  switch (pixelFormat)
  {
    case gz::msgs::PixelFormatType::RGB_INT8:
      return FormatInfo{"RGB", 3};
    case gz::msgs::PixelFormatType::BGR_INT8:
      return FormatInfo{"BGR", 3};
    case gz::msgs::PixelFormatType::RGBA_INT8:
      return FormatInfo{"RGBA", 4};
    case gz::msgs::PixelFormatType::BGRA_INT8:
      return FormatInfo{"BGRA", 4};
    case gz::msgs::PixelFormatType::L_INT8:
      return FormatInfo{"GRAY8", 1};
    case gz::msgs::PixelFormatType::L_INT16:
      return FormatInfo{"GRAY16_LE", 2};
    default:
      return std::nullopt;
  }
}

bool checkedMultiply(guint a, guint b, std::size_t *result)
{
  const auto max = std::numeric_limits<std::size_t>::max();
  if (a != 0 && b > max / a)
  {
    return false;
  }
  *result = static_cast<std::size_t>(a) * static_cast<std::size_t>(b);
  return true;
}

bool validateFrameShape(
  GstGzImgSrc *self,
  const gz::msgs::Image &msg,
  const FormatInfo &formatInfo)
{
  const guint width = msg.width();
  const guint height = msg.height();
  const guint stride = msg.step();

  if (width == 0 || height == 0)
  {
    GST_WARNING_OBJECT(
      self,
      "Ignoring image with invalid dimensions %ux%u",
      width,
      height);
    return false;
  }

  std::size_t minimumStride = 0;
  if (!checkedMultiply(width, formatInfo.bytesPerPixel, &minimumStride))
  {
    GST_WARNING_OBJECT(self, "Ignoring image with overflowing row size");
    return false;
  }

  if (stride < minimumStride)
  {
    GST_WARNING_OBJECT(
      self,
      "Ignoring image with stride %u smaller than expected row size %" G_GSIZE_FORMAT,
      stride,
      static_cast<gsize>(minimumStride));
    return false;
  }

  std::size_t expectedSize = 0;
  if (!checkedMultiply(stride, height, &expectedSize))
  {
    GST_WARNING_OBJECT(self, "Ignoring image with overflowing payload size");
    return false;
  }

  if (msg.data().size() < expectedSize)
  {
    GST_WARNING_OBJECT(
      self,
      "Ignoring image with payload size %" G_GSIZE_FORMAT
      " smaller than expected %" G_GSIZE_FORMAT,
      static_cast<gsize>(msg.data().size()),
      static_cast<gsize>(expectedSize));
    return false;
  }

  return true;
}

SourceState *acquireState(GstGzImgSrc *self)
{
  g_mutex_lock(&self->stateLock);
  SourceState *state = self->state;
  if (state != nullptr)
  {
    ++state->activeUsers;
  }
  g_mutex_unlock(&self->stateLock);
  return state;
}

void releaseState(GstGzImgSrc *self, SourceState *state)
{
  g_mutex_lock(&self->stateLock);
  if (state != nullptr)
  {
    --state->activeUsers;
    if (state->activeUsers == 0)
    {
      g_cond_signal(&self->stateIdle);
    }
  }
  g_mutex_unlock(&self->stateLock);
}

gboolean updateCaps(GstGzImgSrc *self, const Frame &frame)
{
  GstCaps *caps = gst_caps_new_simple(
    "video/x-raw",
    "format", G_TYPE_STRING, frame.gstFormat.c_str(),
    "width", G_TYPE_INT, frame.width,
    "height", G_TYPE_INT, frame.height,
    "framerate", GST_TYPE_FRACTION, 0, 1,
    nullptr);

  const gboolean ok = gst_base_src_set_caps(GST_BASE_SRC(self), caps);
  gst_caps_unref(caps);
  return ok;
}

void onGazeboImage(GstGzImgSrc *self, const gz::msgs::Image &msg)
{
  SourceState *state = acquireState(self);
  if (state == nullptr)
  {
    return;
  }

  const auto formatInfo = formatInfoFromGazebo(msg.pixel_format_type());
  if (!formatInfo.has_value())
  {
    GST_WARNING_OBJECT(
      self,
      "Ignoring unsupported Gazebo image pixel format %d",
      msg.pixel_format_type());
    releaseState(self, state);
    return;
  }

  if (!validateFrameShape(self, msg, *formatInfo))
  {
    releaseState(self, state);
    return;
  }

  GstClockTime now = gst_util_get_timestamp();
  GstClockTime duration = GST_CLOCK_TIME_NONE;

  Frame frame;
  frame.width = msg.width();
  frame.height = msg.height();
  frame.stride = msg.step();
  frame.gstFormat = formatInfo->gstFormat;
  frame.data.assign(msg.data().begin(), msg.data().end());
  frame.pts = now;

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!state->running)
    {
      releaseState(self, state);
      return;
    }

    if (GST_CLOCK_TIME_IS_VALID(state->lastFrameTime) &&
        now > state->lastFrameTime)
    {
      duration = now - state->lastFrameTime;
    }
    state->lastFrameTime = now;
    frame.duration = duration;

    state->frame = std::move(frame);
  }

  state->frameAvailable.notify_one();
  releaseState(self, state);
}

void gst_gz_img_src_set_property(
  GObject *object,
  guint propId,
  const GValue *value,
  GParamSpec *pspec)
{
  auto *self = GST_GZ_IMG_SRC(object);

  switch (propId)
  {
    case PROP_TOPIC:
      g_free(self->topic);
      self->topic = g_value_dup_string(value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, propId, pspec);
      break;
  }
}

void gst_gz_img_src_get_property(
  GObject *object,
  guint propId,
  GValue *value,
  GParamSpec *pspec)
{
  auto *self = GST_GZ_IMG_SRC(object);

  switch (propId)
  {
    case PROP_TOPIC:
      g_value_set_string(value, self->topic);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, propId, pspec);
      break;
  }
}

gboolean gst_gz_img_src_start(GstBaseSrc *baseSrc)
{
  auto *self = GST_GZ_IMG_SRC(baseSrc);

  g_mutex_lock(&self->stateLock);
  if (self->state != nullptr)
  {
    g_mutex_unlock(&self->stateLock);
    return TRUE;
  }
  g_mutex_unlock(&self->stateLock);

  auto *state = new SourceState();
  state->running = true;
  state->lastFrameTime = GST_CLOCK_TIME_NONE;

  g_mutex_lock(&self->stateLock);
  self->state = state;
  g_mutex_unlock(&self->stateLock);

  const std::string topic =
    (self->topic != nullptr && self->topic[0] != '\0') ? self->topic
                                                       : kDefaultTopic;

  std::function<void(const gz::msgs::Image &)> callback =
    [self](const gz::msgs::Image &msg) { onGazeboImage(self, msg); };
  const bool subscribed = state->node.Subscribe(topic, callback);

  if (!subscribed)
  {
    GST_ERROR_OBJECT(self, "Failed to subscribe to Gazebo image topic: %s",
                     topic.c_str());
    g_mutex_lock(&self->stateLock);
    self->state = nullptr;
    g_mutex_unlock(&self->stateLock);
    delete state;
    return FALSE;
  }

  GST_INFO_OBJECT(self, "Subscribed to Gazebo image topic: %s", topic.c_str());
  return TRUE;
}

gboolean gst_gz_img_src_stop(GstBaseSrc *baseSrc)
{
  auto *self = GST_GZ_IMG_SRC(baseSrc);

  g_mutex_lock(&self->stateLock);
  SourceState *state = self->state;
  self->state = nullptr;
  g_mutex_unlock(&self->stateLock);

  if (state != nullptr)
  {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->running = false;
      state->frame.reset();
    }
    state->frameAvailable.notify_all();

    g_mutex_lock(&self->stateLock);
    while (state->activeUsers > 0)
    {
      g_cond_wait(&self->stateIdle, &self->stateLock);
    }
    g_mutex_unlock(&self->stateLock);

    delete state;
  }

  return TRUE;
}

GstFlowReturn gst_gz_img_src_create(GstPushSrc *pushSrc, GstBuffer **buffer)
{
  auto *self = GST_GZ_IMG_SRC(pushSrc);
  SourceState *state = acquireState(self);
  if (state == nullptr)
  {
    return GST_FLOW_FLUSHING;
  }

  std::unique_lock<std::mutex> lock(state->mutex);
  state->frameAvailable.wait(lock, [state] {
    return !state->running || state->frame.has_value();
  });

  if (!state->running)
  {
    releaseState(self, state);
    return GST_FLOW_FLUSHING;
  }

  Frame frame = std::move(*state->frame);
  state->frame.reset();
  lock.unlock();

  if (!updateCaps(self, frame))
  {
    GST_ERROR_OBJECT(self, "Failed to set output caps");
    releaseState(self, state);
    return GST_FLOW_NOT_NEGOTIATED;
  }

  GstBuffer *outBuffer = gst_buffer_new_allocate(nullptr, frame.data.size(), nullptr);
  if (outBuffer == nullptr)
  {
    releaseState(self, state);
    return GST_FLOW_ERROR;
  }

  gst_buffer_fill(outBuffer, 0, frame.data.data(), frame.data.size());
  GST_BUFFER_PTS(outBuffer) = frame.pts;
  GST_BUFFER_DTS(outBuffer) = frame.pts;
  GST_BUFFER_DURATION(outBuffer) = frame.duration;

  *buffer = outBuffer;
  releaseState(self, state);
  return GST_FLOW_OK;
}

void gst_gz_img_src_finalize(GObject *object)
{
  auto *self = GST_GZ_IMG_SRC(object);

  if (self->state != nullptr)
  {
    gst_gz_img_src_stop(GST_BASE_SRC(self));
  }

  g_free(self->topic);
  self->topic = nullptr;
  g_cond_clear(&self->stateIdle);
  g_mutex_clear(&self->stateLock);

  G_OBJECT_CLASS(gst_gz_img_src_parent_class)->finalize(object);
}

void gst_gz_img_src_class_init(GstGzImgSrcClass *klass)
{
  auto *objectClass = G_OBJECT_CLASS(klass);
  auto *baseSrcClass = GST_BASE_SRC_CLASS(klass);
  auto *pushSrcClass = GST_PUSH_SRC_CLASS(klass);

  objectClass->set_property = gst_gz_img_src_set_property;
  objectClass->get_property = gst_gz_img_src_get_property;
  objectClass->finalize = gst_gz_img_src_finalize;

  g_object_class_install_property(
    objectClass,
    PROP_TOPIC,
    g_param_spec_string(
      "topic",
      "Topic",
      "Gazebo image topic to subscribe to",
      kDefaultTopic,
      static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  baseSrcClass->start = GST_DEBUG_FUNCPTR(gst_gz_img_src_start);
  baseSrcClass->stop = GST_DEBUG_FUNCPTR(gst_gz_img_src_stop);
  pushSrcClass->create = GST_DEBUG_FUNCPTR(gst_gz_img_src_create);

  gst_element_class_set_static_metadata(
    GST_ELEMENT_CLASS(klass),
    "Gazebo image source",
    "Source/Video",
    "Reads Gazebo image messages and outputs raw video buffers",
    "Betaloop");

  GstCaps *caps = gst_caps_from_string(
    "video/x-raw,format=(string){ RGB, BGR, RGBA, BGRA, GRAY8, GRAY16_LE },"
    "width=(int)[ 1, MAX ],height=(int)[ 1, MAX ],framerate=(fraction)[ 0/1, MAX ]");
  gst_element_class_add_pad_template(
    GST_ELEMENT_CLASS(klass),
    gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
  gst_caps_unref(caps);
}

void gst_gz_img_src_init(GstGzImgSrc *self)
{
  self->topic = g_strdup(kDefaultTopic);
  self->state = nullptr;
  g_mutex_init(&self->stateLock);
  g_cond_init(&self->stateIdle);

  gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
  gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
}

gboolean pluginInit(GstPlugin *plugin)
{
  return gst_element_register(
    plugin,
    "gzimgsrc",
    GST_RANK_NONE,
    GST_TYPE_GZ_IMG_SRC);
}

}  // namespace

GST_PLUGIN_DEFINE(
  GST_VERSION_MAJOR,
  GST_VERSION_MINOR,
  gzimgsrc,
  "Gazebo image source",
  pluginInit,
  "0.1.0",
  "MIT",
  "gst_gzimgsrc",
  "https://example.invalid/gst_gzimgsrc")
