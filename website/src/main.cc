#include <drogon/drogon.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtc/rtc.h>
#include <sensor_msgs/msg/image.hpp>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{
constexpr const char *kSessionCookieName = "aerosentinel_session";
constexpr const char *kDefaultCameraTopic = "/front_camera/image";
constexpr int kDefaultJpegQuality = 85;
constexpr int kCameraStreamFps = 60;
constexpr int kCameraStreamFramePeriodUs = 1000000 / kCameraStreamFps;
constexpr int kWebRtcAnswerTimeoutMs = 5000;
constexpr int kDefaultWebRtcMaxBufferedBytes = 256 * 1024;
constexpr int kDefaultH264Bitrate = 6000000;
constexpr int kH264PayloadType = 102;
constexpr uint32_t kH264Ssrc = 42;
constexpr uint32_t kH264ClockRate = 90000;
constexpr unsigned kRtcpNackCachePackets = 512;
constexpr size_t kMaxWebRtcSessions = 16;

struct AuthConfig
{
    std::string username;
    std::string password;
    bool secureCookies{false};
};

struct ImageFormat
{
    std::string colorOrder;
    int channels{0};
};

struct CameraFrame
{
    std::vector<unsigned char> h264;
    std::chrono::steady_clock::time_point timestamp;
    uint32_t width{0};
    uint32_t height{0};
    std::string encoding;
    uint64_t sequence{0};
    std::string topic;
    bool keyframe{false};
};

struct CameraJpegFrame
{
    std::vector<unsigned char> jpeg;
    std::chrono::steady_clock::time_point timestamp;
    uint32_t width{0};
    uint32_t height{0};
    std::string encoding;
    uint64_t sequence{0};
    std::string topic;
};

std::string randomHexToken(size_t byteCount)
{
    std::vector<unsigned char> bytes(byteCount);
    std::random_device random;
    for (auto &byte : bytes)
    {
        byte = static_cast<unsigned char>(random());
    }

    std::ostringstream token;
    token << std::hex << std::setfill('0');
    for (const auto byte : bytes)
    {
        token << std::setw(2) << static_cast<int>(byte);
    }
    return token.str();
}

class SessionStore
{
  public:
    std::string create()
    {
        const auto token = makeToken();
        const auto expiresAt = std::chrono::steady_clock::now() + ttl_;

        std::lock_guard<std::mutex> lock(mutex_);
        sessions_[token] = expiresAt;
        return token;
    }

    bool isValid(const std::string &token)
    {
        if (token.empty())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto match = sessions_.find(token);
        if (match == sessions_.end())
        {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (match->second <= now)
        {
            sessions_.erase(match);
            return false;
        }

        match->second = now + ttl_;
        return true;
    }

    void destroy(const std::string &token)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions_.erase(token);
    }

  private:
    static std::string makeToken()
    {
        return randomHexToken(32);
    }

    std::mutex mutex_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> sessions_;
    std::chrono::hours ttl_{8};
};

std::string envOrDefault(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || *value == '\0')
    {
        return fallback;
    }
    return value;
}

bool envFlag(const char *name)
{
    std::string value = envOrDefault(name, "");
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    return value == "1" || value == "true" || value == "yes" || value == "on";
}

double envDouble(const char *name, double fallback)
{
    try
    {
        return std::stod(envOrDefault(name, ""));
    }
    catch (...)
    {
        return fallback;
    }
}

int envInt(const char *name, int fallback)
{
    try
    {
        return std::stoi(envOrDefault(name, ""));
    }
    catch (...)
    {
        return fallback;
    }
}

int cameraJpegQuality()
{
    return std::clamp(envInt("AEROSENTINEL_JPEG_QUALITY", kDefaultJpegQuality), 1, 100);
}

std::filesystem::path executableDir(const char *argv0)
{
    if (argv0 == nullptr || *argv0 == '\0')
    {
        return std::filesystem::current_path();
    }

    std::error_code ec;
    auto path = std::filesystem::absolute(argv0, ec);
    if (ec)
    {
        return std::filesystem::current_path();
    }

    return path.parent_path();
}

std::filesystem::path findPublicDir(const char *argv0)
{
    const auto exeDir = executableDir(argv0);
    std::vector<std::filesystem::path> candidates;
    const auto configuredPublicDir = std::getenv("AEROSENTINEL_PUBLIC_DIR");
    if (configuredPublicDir != nullptr && *configuredPublicDir != '\0')
    {
        candidates.emplace_back(configuredPublicDir);
    }

    candidates.insert(candidates.end(), {
        std::filesystem::current_path() / "public",
        exeDir / "public",
        exeDir.parent_path() / "public"});

    for (const auto &candidate : candidates)
    {
        std::error_code ec;
        if (std::filesystem::exists(candidate / "index.html", ec))
        {
            return std::filesystem::weakly_canonical(candidate, ec);
        }
    }

    return std::filesystem::current_path() / "public";
}

uint16_t portFromEnvironment()
{
    const char *portValue = std::getenv("PORT");
    if (portValue == nullptr)
    {
        return 8080;
    }

    try
    {
        const int parsed = std::stoi(portValue);
        if (parsed > 0 && parsed <= 65535)
        {
            return static_cast<uint16_t>(parsed);
        }
    }
    catch (...)
    {
    }

    return 8080;
}

bool constantTimeEquals(const std::string &left, const std::string &right)
{
    unsigned char diff = static_cast<unsigned char>(left.size() ^ right.size());
    const auto length = left.size() > right.size() ? left.size() : right.size();
    for (size_t index = 0; index < length; ++index)
    {
        const auto leftChar = index < left.size() ? static_cast<unsigned char>(left[index]) : 0;
        const auto rightChar = index < right.size() ? static_cast<unsigned char>(right[index]) : 0;
        diff = static_cast<unsigned char>(diff | (leftChar ^ rightChar));
    }
    return diff == 0;
}

int hexValue(char ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }
    if (ch >= 'a' && ch <= 'f')
    {
        return 10 + ch - 'a';
    }
    if (ch >= 'A' && ch <= 'F')
    {
        return 10 + ch - 'A';
    }
    return -1;
}

std::string urlDecode(std::string_view value)
{
    std::string decoded;
    decoded.reserve(value.size());

    for (size_t index = 0; index < value.size(); ++index)
    {
        if (value[index] == '+')
        {
            decoded.push_back(' ');
            continue;
        }

        if (value[index] == '%' && index + 2 < value.size())
        {
            const int high = hexValue(value[index + 1]);
            const int low = hexValue(value[index + 2]);
            if (high >= 0 && low >= 0)
            {
                decoded.push_back(static_cast<char>((high << 4) | low));
                index += 2;
                continue;
            }
        }

        decoded.push_back(value[index]);
    }

    return decoded;
}

std::string formValue(std::string_view body, std::string_view key)
{
    size_t start = 0;
    while (start <= body.size())
    {
        const auto ampersand = body.find('&', start);
        const auto end = ampersand == std::string_view::npos ? body.size() : ampersand;
        const auto pair = body.substr(start, end - start);
        const auto equals = pair.find('=');

        const auto name = urlDecode(equals == std::string_view::npos ? pair : pair.substr(0, equals));
        if (name == key)
        {
            return urlDecode(equals == std::string_view::npos ? std::string_view{} : pair.substr(equals + 1));
        }

        if (ampersand == std::string_view::npos)
        {
            break;
        }
        start = ampersand + 1;
    }

    return {};
}

std::string requestFormValue(const drogon::HttpRequestPtr &request,
                             std::string_view body,
                             const std::string &key)
{
    auto value = request->getParameter(key);
    if (!value.empty())
    {
        return value;
    }

    return formValue(body, key);
}

std::string trim(std::string_view value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
    {
        value.remove_prefix(1);
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
    {
        value.remove_suffix(1);
    }
    return std::string(value);
}

std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::vector<std::string> splitCsv(std::string_view value)
{
    std::vector<std::string> fields;
    size_t start = 0;
    while (start <= value.size())
    {
        const auto comma = value.find(',', start);
        const auto end = comma == std::string_view::npos ? value.size() : comma;
        auto field = trim(value.substr(start, end - start));
        if (!field.empty())
        {
            fields.push_back(std::move(field));
        }

        if (comma == std::string_view::npos)
        {
            break;
        }
        start = comma + 1;
    }
    return fields;
}

std::vector<std::string> webRtcIceServers()
{
    return splitCsv(envOrDefault("AEROSENTINEL_WEBRTC_ICE_SERVERS", ""));
}

int webRtcMaxBufferedBytes()
{
    return std::max(0, envInt("AEROSENTINEL_WEBRTC_MAX_BUFFERED_BYTES",
                              kDefaultWebRtcMaxBufferedBytes));
}

int h264Bitrate()
{
    return std::max(100000, envInt("AEROSENTINEL_H264_BITRATE", kDefaultH264Bitrate));
}

int h264GopFrames()
{
    return std::max(1, envInt("AEROSENTINEL_H264_GOP_FRAMES", kCameraStreamFps));
}

std::string ffmpegError(int code)
{
    std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
    av_strerror(code, buffer.data(), buffer.size());
    return std::string(buffer.data());
}

std::string h264Fmtp()
{
    const auto configuredFmtp = envOrDefault("AEROSENTINEL_H264_FMTP", "");
    if (!configuredFmtp.empty())
    {
        return configuredFmtp;
    }

    return "profile-level-id=" +
           envOrDefault("AEROSENTINEL_H264_PROFILE_ID", "42e01f") +
           ";packetization-mode=1;level-asymmetry-allowed=1";
}

std::string cookieValue(const drogon::HttpRequestPtr &request, const std::string &name)
{
    const auto cookie = request->getCookie(name);
    if (!cookie.empty())
    {
        return cookie;
    }

    auto header = request->getHeader("cookie");
    if (header.empty())
    {
        header = request->getHeader("Cookie");
    }

    size_t start = 0;
    while (start <= header.size())
    {
        const auto semicolon = header.find(';', start);
        const auto end = semicolon == std::string::npos ? header.size() : semicolon;
        const auto pair = std::string_view(header).substr(start, end - start);
        const auto equals = pair.find('=');

        if (equals != std::string_view::npos && trim(pair.substr(0, equals)) == name)
        {
            return trim(pair.substr(equals + 1));
        }

        if (semicolon == std::string::npos)
        {
            break;
        }
        start = semicolon + 1;
    }

    return {};
}

std::string sessionCookie(const std::string &token, bool secure)
{
    std::string cookie = std::string(kSessionCookieName) + "=" + token +
                         "; Path=/; HttpOnly; SameSite=Strict; Max-Age=28800";
    if (secure)
    {
        cookie += "; Secure";
    }
    return cookie;
}

std::string expiredSessionCookie(bool secure)
{
    std::string cookie = std::string(kSessionCookieName) +
                         "=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0";
    if (secure)
    {
        cookie += "; Secure";
    }
    return cookie;
}

drogon::HttpResponsePtr noStore(const drogon::HttpResponsePtr &response)
{
    response->addHeader("Cache-Control", "no-store");
    return response;
}

drogon::HttpResponsePtr redirectTo(const std::string &location,
                                   drogon::HttpStatusCode status = drogon::k302Found)
{
    auto response = drogon::HttpResponse::newHttpResponse();
    response->setStatusCode(status);
    response->addHeader("Location", location);
    return response;
}

drogon::HttpResponsePtr jsonError(const std::string &code,
                                  drogon::HttpStatusCode status,
                                  const std::string &detail = "")
{
    Json::Value error;
    error["error"] = code;
    if (!detail.empty())
    {
        error["detail"] = detail;
    }

    auto response = drogon::HttpResponse::newHttpJsonResponse(error);
    response->setStatusCode(status);
    return noStore(response);
}

bool isAuthenticated(const drogon::HttpRequestPtr &request,
                     const std::shared_ptr<SessionStore> &sessions)
{
    return sessions->isValid(cookieValue(request, kSessionCookieName));
}

ImageFormat imageEncodingFormat(const std::string &encoding)
{
    const auto normalized = lowerCopy(encoding);
    if (normalized == "rgb8" || normalized == "r8g8b8" || normalized == "8uc3")
    {
        return {"rgb", 3};
    }
    if (normalized == "bgr8" || normalized == "b8g8r8")
    {
        return {"bgr", 3};
    }
    if (normalized == "rgba8" || normalized == "r8g8b8a8" || normalized == "8uc4")
    {
        return {"rgba", 4};
    }
    if (normalized == "bgra8" || normalized == "b8g8r8a8")
    {
        return {"bgra", 4};
    }
    if (normalized == "mono8" || normalized == "8uc1")
    {
        return {"mono", 1};
    }

    throw std::runtime_error("Unsupported camera image encoding: " + encoding);
}

cv::Mat rosImageToCvImage(const sensor_msgs::msg::Image &msg)
{
    if (msg.width == 0 || msg.height == 0)
    {
        throw std::runtime_error("Camera image has invalid dimensions.");
    }

    const auto format = imageEncodingFormat(msg.encoding);
    const auto rowSize = static_cast<size_t>(msg.width) * static_cast<size_t>(format.channels);
    const auto step = msg.step > 0 ? static_cast<size_t>(msg.step) : rowSize;
    if (step < rowSize || msg.data.size() < step * msg.height)
    {
        throw std::runtime_error("Camera image payload is smaller than expected.");
    }

    const int cvType = CV_MAKETYPE(CV_8U, format.channels);
    cv::Mat source(static_cast<int>(msg.height),
                   static_cast<int>(msg.width),
                   cvType,
                   const_cast<unsigned char *>(msg.data.data()),
                   step);

    cv::Mat compact;
    if (step != rowSize)
    {
        compact = source.clone();
    }
    else
    {
        compact = source;
    }

    cv::Mat converted;
    if (format.colorOrder == "rgb")
    {
        cv::cvtColor(compact, converted, cv::COLOR_RGB2BGR);
        return converted;
    }
    if (format.colorOrder == "rgba")
    {
        cv::cvtColor(compact, converted, cv::COLOR_RGBA2BGR);
        return converted;
    }
    if (format.colorOrder == "bgra")
    {
        cv::cvtColor(compact, converted, cv::COLOR_BGRA2BGR);
        return converted;
    }

    return compact;
}

struct EncodedH264Frame
{
    std::vector<unsigned char> payload;
    bool keyframe{false};
};

class H264Encoder
{
  public:
    ~H264Encoder()
    {
        reset();
    }

    void requestKeyframe()
    {
        forceKeyframe_.store(true);
    }

    std::vector<EncodedH264Frame> encode(const cv::Mat &bgr)
    {
        if (bgr.empty() || bgr.channels() != 3)
        {
            throw std::runtime_error("H.264 encoder requires a non-empty BGR image.");
        }
        if (bgr.cols < 2 || bgr.rows < 2 ||
            (bgr.cols % 2) != 0 || (bgr.rows % 2) != 0)
        {
            throw std::runtime_error("H.264 encoder requires even image dimensions.");
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (codecContext_ == nullptr || width_ != bgr.cols || height_ != bgr.rows)
        {
            configure(bgr.cols, bgr.rows);
        }

        int result = av_frame_make_writable(frame_);
        if (result < 0)
        {
            throw std::runtime_error("FFmpeg could not make the H.264 frame writable: " +
                                     ffmpegError(result));
        }

        const uint8_t *sourceData[] = {bgr.data};
        const int sourceLineSize[] = {static_cast<int>(bgr.step)};
        sws_scale(swsContext_,
                  sourceData,
                  sourceLineSize,
                  0,
                  bgr.rows,
                  frame_->data,
                  frame_->linesize);

        frame_->pts = pts_++;
        frame_->pict_type = forceKeyframe_.exchange(false) ? AV_PICTURE_TYPE_I
                                                           : AV_PICTURE_TYPE_NONE;
        result = avcodec_send_frame(codecContext_, frame_);
        if (result < 0)
        {
            throw std::runtime_error("FFmpeg failed to encode a H.264 frame: " +
                                     ffmpegError(result));
        }

        return drainPackets();
    }

  private:
    void reset()
    {
        if (packet_ != nullptr)
        {
            av_packet_free(&packet_);
        }
        if (frame_ != nullptr)
        {
            av_frame_free(&frame_);
        }
        if (codecContext_ != nullptr)
        {
            avcodec_free_context(&codecContext_);
        }
        if (swsContext_ != nullptr)
        {
            sws_freeContext(swsContext_);
            swsContext_ = nullptr;
        }

        width_ = 0;
        height_ = 0;
        pts_ = 0;
    }

    void configure(int width, int height)
    {
        reset();

        const auto configuredEncoder = envOrDefault("AEROSENTINEL_H264_ENCODER", "libx264");
        const AVCodec *codec = avcodec_find_encoder_by_name(configuredEncoder.c_str());
        if (codec == nullptr)
        {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        }
        if (codec == nullptr)
        {
            throw std::runtime_error("No FFmpeg H.264 encoder is available.");
        }

        codecContext_ = avcodec_alloc_context3(codec);
        if (codecContext_ == nullptr)
        {
            throw std::runtime_error("FFmpeg failed to allocate the H.264 encoder context.");
        }

        codecContext_->width = width;
        codecContext_->height = height;
        codecContext_->pix_fmt = AV_PIX_FMT_YUV420P;
        codecContext_->time_base = AVRational{1, kCameraStreamFps};
        codecContext_->framerate = AVRational{kCameraStreamFps, 1};
        codecContext_->bit_rate = h264Bitrate();
        codecContext_->rc_max_rate = h264Bitrate();
        codecContext_->rc_buffer_size = h264Bitrate();
        codecContext_->gop_size = h264GopFrames();
        codecContext_->max_b_frames = 0;
        codecContext_->flags |= AV_CODEC_FLAG_LOW_DELAY;

        if (codecContext_->priv_data != nullptr)
        {
            const auto defaultX264Params =
                "keyint=" + std::to_string(h264GopFrames()) +
                ":min-keyint=" + std::to_string(h264GopFrames()) +
                ":scenecut=0:repeat-headers=1:annexb=1";
            const auto x264Params = envOrDefault("AEROSENTINEL_X264_PARAMS",
                                                 defaultX264Params.c_str());
            av_opt_set(codecContext_->priv_data, "preset", "ultrafast", 0);
            av_opt_set(codecContext_->priv_data, "tune", "zerolatency", 0);
            av_opt_set(codecContext_->priv_data,
                       "profile",
                       envOrDefault("AEROSENTINEL_H264_PROFILE", "baseline").c_str(),
                       0);
            av_opt_set(codecContext_->priv_data,
                       "x264-params",
                       x264Params.c_str(),
                       0);
        }

        int result = avcodec_open2(codecContext_, codec, nullptr);
        if (result < 0)
        {
            throw std::runtime_error("FFmpeg failed to open the H.264 encoder: " +
                                     ffmpegError(result));
        }

        frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        if (frame_ == nullptr || packet_ == nullptr)
        {
            throw std::runtime_error("FFmpeg failed to allocate H.264 frame buffers.");
        }

        frame_->format = codecContext_->pix_fmt;
        frame_->width = codecContext_->width;
        frame_->height = codecContext_->height;
        result = av_frame_get_buffer(frame_, 32);
        if (result < 0)
        {
            throw std::runtime_error("FFmpeg failed to allocate the H.264 frame image: " +
                                     ffmpegError(result));
        }

        swsContext_ = sws_getContext(width,
                                     height,
                                     AV_PIX_FMT_BGR24,
                                     width,
                                     height,
                                     AV_PIX_FMT_YUV420P,
                                     SWS_FAST_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr);
        if (swsContext_ == nullptr)
        {
            throw std::runtime_error("FFmpeg failed to create the H.264 color converter.");
        }

        width_ = width;
        height_ = height;
        pts_ = 0;
        LOG_WARN << "AeroSentinel camera using FFmpeg H.264 encoder '"
                 << codec->name << "' at " << width << "x" << height << ".";
    }

    std::vector<EncodedH264Frame> drainPackets()
    {
        std::vector<EncodedH264Frame> frames;
        while (true)
        {
            const int result = avcodec_receive_packet(codecContext_, packet_);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
            {
                break;
            }
            if (result < 0)
            {
                throw std::runtime_error("FFmpeg failed to receive a H.264 packet: " +
                                         ffmpegError(result));
            }

            EncodedH264Frame frame;
            frame.payload.assign(packet_->data, packet_->data + packet_->size);
            frame.keyframe = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
            frames.push_back(std::move(frame));
            av_packet_unref(packet_);
        }
        return frames;
    }

    std::mutex mutex_;
    AVCodecContext *codecContext_{nullptr};
    SwsContext *swsContext_{nullptr};
    AVFrame *frame_{nullptr};
    AVPacket *packet_{nullptr};
    int width_{0};
    int height_{0};
    int64_t pts_{0};
    std::atomic_bool forceKeyframe_{true};
};

class CameraFrameStore
{
  public:
    void requestKeyframe()
    {
        h264Encoder_.requestKeyframe();
    }

    void updateFromRosImage(const sensor_msgs::msg::Image &msg, const std::string &topic)
    {
        auto image = rosImageToCvImage(msg);
        cv::Mat bgr;
        if (image.channels() == 1)
        {
            cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
        }
        else
        {
            bgr = image;
        }
        if ((bgr.cols % 2) != 0 || (bgr.rows % 2) != 0)
        {
            const auto evenWidth = bgr.cols & ~1;
            const auto evenHeight = bgr.rows & ~1;
            if (evenWidth < 2 || evenHeight < 2)
            {
                throw std::runtime_error("Camera image is too small for H.264 encoding.");
            }
            bgr = bgr(cv::Rect(0, 0, evenWidth, evenHeight));
        }

        auto encodedFrames = h264Encoder_.encode(bgr);
        if (encodedFrames.empty())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        latestBgr_ = bgr.clone();
        width_ = static_cast<uint32_t>(bgr.cols);
        height_ = static_cast<uint32_t>(bgr.rows);
        encoding_ = msg.encoding;
        topic_ = topic;

        for (auto &encoded : encodedFrames)
        {
            h264_ = std::move(encoded.payload);
            keyframe_ = encoded.keyframe;
            timestamp_ = std::chrono::steady_clock::now();
            ++sequence_;
        }
        condition_.notify_all();
    }

    std::optional<CameraJpegFrame> getJpeg() const
    {
        cv::Mat image;
        CameraJpegFrame snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (latestBgr_.empty())
            {
                return std::nullopt;
            }

            image = latestBgr_.clone();
            snapshot.timestamp = timestamp_;
            snapshot.width = width_;
            snapshot.height = height_;
            snapshot.encoding = encoding_;
            snapshot.sequence = sequence_;
            snapshot.topic = topic_;
        }

        if (!cv::imencode(".jpg",
                          image,
                          snapshot.jpeg,
                          {cv::IMWRITE_JPEG_QUALITY, cameraJpegQuality()}))
        {
            throw std::runtime_error("OpenCV failed to encode camera snapshot as JPEG.");
        }
        return snapshot;
    }

    std::optional<CameraFrame> waitForFrame(uint64_t lastSequence,
                                            std::chrono::milliseconds timeout) const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto hasNewFrame = condition_.wait_for(lock, timeout, [&] {
            return !h264_.empty() && sequence_ != lastSequence;
        });
        if (!hasNewFrame)
        {
            return std::nullopt;
        }
        return snapshotLocked();
    }

  private:
    std::optional<CameraFrame> snapshotLocked() const
    {
        if (h264_.empty())
        {
            return std::nullopt;
        }

        return CameraFrame{
            h264_, timestamp_, width_, height_, encoding_, sequence_, topic_, keyframe_};
    }

    mutable std::mutex mutex_;
    mutable std::condition_variable condition_;
    H264Encoder h264Encoder_;
    std::vector<unsigned char> h264_;
    cv::Mat latestBgr_;
    std::chrono::steady_clock::time_point timestamp_{};
    uint32_t width_{0};
    uint32_t height_{0};
    std::string encoding_;
    uint64_t sequence_{0};
    bool keyframe_{false};
    std::string topic_{kDefaultCameraTopic};
};

struct WebRtcAnswer
{
    std::string id;
    std::string type;
    std::string sdp;
};

class WebRtcCameraSession : public std::enable_shared_from_this<WebRtcCameraSession>
{
  public:
    WebRtcCameraSession(std::shared_ptr<CameraFrameStore> cameraFrames,
                        std::vector<std::string> iceServers,
                        int maxBufferedBytes)
        : id_(randomHexToken(12)),
          cameraFrames_(std::move(cameraFrames)),
          iceServers_(std::move(iceServers)),
          maxBufferedBytes_(maxBufferedBytes),
          h264Fmtp_(h264Fmtp())
    {
        iceServerPointers_.reserve(iceServers_.size());
        for (const auto &server : iceServers_)
        {
            iceServerPointers_.push_back(server.c_str());
        }

        rtcConfiguration config{};
        config.iceServers = iceServerPointers_.empty() ? nullptr : iceServerPointers_.data();
        config.iceServersCount = static_cast<int>(iceServerPointers_.size());

        pc_ = rtcCreatePeerConnection(&config);
        if (pc_ <= 0)
        {
            throw std::runtime_error("Failed to create WebRTC peer connection.");
        }

        rtcSetUserPointer(pc_, this);
        rtcSetLocalDescriptionCallback(pc_, &WebRtcCameraSession::onLocalDescription);
        rtcSetGatheringStateChangeCallback(pc_, &WebRtcCameraSession::onGatheringStateChange);
        rtcSetStateChangeCallback(pc_, &WebRtcCameraSession::onStateChange);
        createVideoTrack();
    }

    ~WebRtcCameraSession()
    {
        close();
        if (track_ > 0)
        {
            rtcDeleteTrack(track_);
        }
        if (pc_ > 0)
        {
            rtcDeletePeerConnection(pc_);
        }
    }

    const std::string &id() const
    {
        return id_;
    }

    bool isClosed() const
    {
        return closed_.load();
    }

    void acceptOffer(const std::string &sdp, const std::string &type)
    {
        const auto descriptionType = type.empty() ? "offer" : type.c_str();
        const auto result = rtcSetRemoteDescription(pc_, sdp.c_str(), descriptionType);
        if (result < 0)
        {
            throw std::runtime_error("Failed to set remote WebRTC offer.");
        }
    }

    std::optional<WebRtcAnswer> waitForAnswer(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto ready = condition_.wait_for(lock, timeout, [this] {
            return closed_.load() || (localDescriptionReady_ && gatheringComplete_);
        });
        if (!ready || closed_.load())
        {
            return std::nullopt;
        }
        lock.unlock();

        auto sdp = getLocalDescription();
        if (sdp.empty())
        {
            return std::nullopt;
        }

        auto type = getLocalDescriptionType();
        if (type.empty())
        {
            type = "answer";
        }

        return WebRtcAnswer{id_, type, sdp};
    }

    void close()
    {
        if (closed_.exchange(true))
        {
            return;
        }

        int track = -1;
        int pc = -1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            trackOpen_ = false;
            track = track_;
            pc = pc_;
            condition_.notify_all();
        }

        if (track > 0)
        {
            rtcClose(track);
        }
        if (pc > 0)
        {
            rtcClosePeerConnection(pc);
        }
    }

  private:
    static void onLocalDescription(int, const char *, const char *, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(session->mutex_);
        session->localDescriptionReady_ = true;
        session->condition_.notify_all();
    }

    static void onGatheringStateChange(int, rtcGatheringState state, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }

        if (state == RTC_GATHERING_COMPLETE)
        {
            std::lock_guard<std::mutex> lock(session->mutex_);
            session->gatheringComplete_ = true;
            session->condition_.notify_all();
        }
    }

    static void onStateChange(int, rtcState state, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }

        if (state == RTC_FAILED || state == RTC_CLOSED)
        {
            session->markClosed();
        }
    }

    static void onTrackOpen(int, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }
        session->markTrackOpen();
    }

    static void onTrackClosed(int, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }
        session->markClosed();
    }

    static void onTrackError(int, const char *error, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }
        if (error != nullptr)
        {
            LOG_WARN << "WebRTC H.264 track error: " << error;
        }
        session->markClosed();
    }

    static void onPictureLossIndication(int, void *userPtr)
    {
        auto *session = static_cast<WebRtcCameraSession *>(userPtr);
        if (session == nullptr)
        {
            return;
        }
        session->cameraFrames_->requestKeyframe();
    }

    void createVideoTrack()
    {
        rtcTrackInit trackInit{};
        trackInit.direction = RTC_DIRECTION_SENDONLY;
        trackInit.codec = RTC_CODEC_H264;
        trackInit.payloadType = kH264PayloadType;
        trackInit.ssrc = kH264Ssrc;
        trackInit.mid = "video";
        trackInit.name = "front-camera";
        trackInit.msid = "aerosentinel-camera";
        trackInit.trackId = "front-camera";
        trackInit.profile = h264Fmtp_.c_str();

        track_ = rtcAddTrackEx(pc_, &trackInit);
        if (track_ <= 0)
        {
            throw std::runtime_error("Failed to create WebRTC H.264 video track.");
        }
        rtcSetUserPointer(track_, this);

        std::random_device random;
        rtpTimestamp_ = static_cast<uint32_t>(random());

        rtcPacketizerInit packetizer{};
        packetizer.ssrc = kH264Ssrc;
        packetizer.cname = "aerosentinel-camera";
        packetizer.payloadType = kH264PayloadType;
        packetizer.clockRate = kH264ClockRate;
        packetizer.sequenceNumber = static_cast<uint16_t>(random());
        packetizer.timestamp = rtpTimestamp_;
        packetizer.maxFragmentSize = 0;
        packetizer.nalSeparator = RTC_NAL_SEPARATOR_START_SEQUENCE;

        if (rtcSetH264Packetizer(track_, &packetizer) < 0)
        {
            throw std::runtime_error("Failed to configure WebRTC H.264 packetizer.");
        }
        rtcChainRtcpSrReporter(track_);
        rtcChainRtcpNackResponder(track_, kRtcpNackCachePackets);
        rtcChainPliHandler(track_, &WebRtcCameraSession::onPictureLossIndication);

        rtcSetOpenCallback(track_, &WebRtcCameraSession::onTrackOpen);
        rtcSetClosedCallback(track_, &WebRtcCameraSession::onTrackClosed);
        rtcSetErrorCallback(track_, &WebRtcCameraSession::onTrackError);
        rtcSetBufferedAmountLowThreshold(track_, maxBufferedBytes_ / 2);
    }

    void markTrackOpen()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_.load())
            {
                return;
            }
            trackOpen_ = true;
        }
        cameraFrames_->requestKeyframe();
        startStreaming();
    }

    void markClosed()
    {
        closed_.store(true);
        std::lock_guard<std::mutex> lock(mutex_);
        trackOpen_ = false;
        condition_.notify_all();
    }

    int videoTrack() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return trackOpen_ ? track_ : -1;
    }

    std::string getLocalDescription() const
    {
        const auto size = rtcGetLocalDescription(pc_, nullptr, 0);
        if (size <= 0)
        {
            return {};
        }

        std::vector<char> buffer(static_cast<size_t>(size));
        if (rtcGetLocalDescription(pc_, buffer.data(), size) < 0)
        {
            return {};
        }
        return std::string(buffer.data());
    }

    std::string getLocalDescriptionType() const
    {
        const auto size = rtcGetLocalDescriptionType(pc_, nullptr, 0);
        if (size <= 0)
        {
            return {};
        }

        std::vector<char> buffer(static_cast<size_t>(size));
        if (rtcGetLocalDescriptionType(pc_, buffer.data(), size) < 0)
        {
            return {};
        }
        return std::string(buffer.data());
    }

    void startStreaming()
    {
        if (streamStarted_.exchange(true))
        {
            return;
        }

        auto self = shared_from_this();
        std::thread([self] {
            self->streamLoop();
        }).detach();
    }

    bool sendFrame(int track, const CameraFrame &frame)
    {
        if (frame.h264.empty() ||
            frame.h264.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
        {
            return true;
        }

        if (!rtpStartTime_)
        {
            rtpStartTime_ = frame.timestamp;
        }
        uint64_t elapsedUs = 0;
        if (frame.timestamp > *rtpStartTime_)
        {
            elapsedUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    frame.timestamp - *rtpStartTime_)
                    .count());
        }
        const auto timestampOffset =
            static_cast<uint32_t>((elapsedUs * kH264ClockRate) / 1000000);

        if (rtcSetTrackRtpTimestamp(track, rtpTimestamp_ + timestampOffset) < 0)
        {
            return false;
        }

        const auto result = rtcSendMessage(
            track,
            reinterpret_cast<const char *>(frame.h264.data()),
            static_cast<int>(frame.h264.size()));
        if (result < 0)
        {
            return false;
        }

        return true;
    }

    void streamLoop()
    {
        uint64_t lastSequence = 0;

        while (!closed_.load())
        {
            auto frame = cameraFrames_->waitForFrame(lastSequence, std::chrono::seconds(1));
            if (!frame)
            {
                continue;
            }
            lastSequence = frame->sequence;

            const auto track = videoTrack();
            if (track < 0)
            {
                break;
            }

            const auto bufferedAmount = rtcGetBufferedAmount(track);
            if (bufferedAmount > maxBufferedBytes_)
            {
                continue;
            }

            if (!sendFrame(track, *frame))
            {
                markClosed();
                break;
            }
        }
    }

    std::string id_;
    std::shared_ptr<CameraFrameStore> cameraFrames_;
    std::vector<std::string> iceServers_;
    std::vector<const char *> iceServerPointers_;
    int maxBufferedBytes_{kDefaultWebRtcMaxBufferedBytes};
    std::string h264Fmtp_;
    int pc_{-1};
    int track_{-1};
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool localDescriptionReady_{false};
    bool gatheringComplete_{false};
    bool trackOpen_{false};
    uint32_t rtpTimestamp_{0};
    std::optional<std::chrono::steady_clock::time_point> rtpStartTime_;
    std::atomic_bool closed_{false};
    std::atomic_bool streamStarted_{false};
};

class WebRtcCameraManager
{
  public:
    explicit WebRtcCameraManager(std::shared_ptr<CameraFrameStore> cameraFrames)
        : cameraFrames_(std::move(cameraFrames))
    {
    }

    std::vector<std::string> iceServers() const
    {
        return webRtcIceServers();
    }

    std::optional<WebRtcAnswer> answerOffer(const std::string &sdp,
                                            const std::string &type,
                                            std::string &error)
    {
        reapClosedSessions();

        std::shared_ptr<WebRtcCameraSession> session;
        try
        {
            session = std::make_shared<WebRtcCameraSession>(
                cameraFrames_,
                webRtcIceServers(),
                webRtcMaxBufferedBytes());
            session->acceptOffer(sdp, type);
        }
        catch (const std::exception &exception)
        {
            error = exception.what();
            if (session)
            {
                session->close();
            }
            return std::nullopt;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            sessions_[session->id()] = session;
        }

        auto answer = session->waitForAnswer(
            std::chrono::milliseconds(kWebRtcAnswerTimeoutMs));
        if (!answer)
        {
            error = "Timed out while gathering WebRTC answer candidates.";
            removeSession(session->id());
            session->close();
            return std::nullopt;
        }

        pruneSessionLimit();
        return answer;
    }

    void closeAll()
    {
        std::vector<std::shared_ptr<WebRtcCameraSession>> sessions;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto &entry : sessions_)
            {
                sessions.push_back(entry.second);
            }
            sessions_.clear();
        }

        for (const auto &session : sessions)
        {
            session->close();
        }
    }

  private:
    void removeSession(const std::string &id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions_.erase(id);
    }

    void reapClosedSessions()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto iterator = sessions_.begin(); iterator != sessions_.end();)
        {
            if (iterator->second->isClosed())
            {
                iterator = sessions_.erase(iterator);
            }
            else
            {
                ++iterator;
            }
        }
    }

    void pruneSessionLimit()
    {
        std::vector<std::shared_ptr<WebRtcCameraSession>> removed;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            while (sessions_.size() > kMaxWebRtcSessions)
            {
                auto iterator = sessions_.begin();
                removed.push_back(iterator->second);
                sessions_.erase(iterator);
            }
        }

        for (const auto &session : removed)
        {
            session->close();
        }
    }

    std::shared_ptr<CameraFrameStore> cameraFrames_;
    std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<WebRtcCameraSession>> sessions_;
};

class RosBridge
{
  public:
    explicit RosBridge(std::shared_ptr<CameraFrameStore> cameraFrames)
        : cameraFrames_(std::move(cameraFrames))
    {
    }

    void start(int argc, char **argv)
    {
        cameraTopic_ = envOrDefault("AEROSENTINEL_CAMERA_TOPIC", kDefaultCameraTopic);
        cameraEnabled_ = !envFlag("AEROSENTINEL_DISABLE_CAMERA");
        try
        {
            if (!rclcpp::ok())
            {
                rclcpp::init(argc, argv);
            }
        }
        catch (const std::exception &error)
        {
            setError(std::string("ROS control bridge failed to start: ") + error.what());
            LOG_ERROR << "ROS bridge failed to start: " << error.what();
            return;
        }

        thread_ = std::thread([this] {
            spin();
        });
    }

    void stop()
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    std::pair<bool, std::string> publishCmdVel(double linearX, double angularZ)
    {
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            publisher = publisher_;
            if (!ready_ || !publisher)
            {
                return {false, lastError_};
            }
        }

        geometry_msgs::msg::Twist message;
        message.linear.x = linearX;
        message.angular.z = angularZ;
        publisher->publish(message);
        return {true, ""};
    }

  private:
    void setError(const std::string &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_ = false;
        lastError_ = message;
    }

    void configure(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &publisher)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        publisher_ = publisher;
        ready_ = true;
        lastError_.clear();
    }

    void spin()
    {
        try
        {
            auto node = rclcpp::Node::make_shared("aerosentinel_web");
            configure(node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10));

            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscription;
            if (cameraEnabled_)
            {
                const auto logger = node->get_logger();
                cameraSubscription = node->create_subscription<sensor_msgs::msg::Image>(
                    cameraTopic_,
                    rclcpp::SensorDataQoS(),
                    [this, logger](const sensor_msgs::msg::Image::SharedPtr msg) {
                        try
                        {
                            cameraFrames_->updateFromRosImage(*msg, cameraTopic_);
                        }
                        catch (const std::exception &error)
                        {
                            RCLCPP_WARN(logger, "%s", error.what());
                        }
                    });
                RCLCPP_INFO(node->get_logger(),
                            "AeroSentinel web feed subscribed to %s",
                            cameraTopic_.c_str());
            }

            RCLCPP_INFO(node->get_logger(), "AeroSentinel web controls publishing to /cmd_vel");
            rclcpp::spin(node);
        }
        catch (const std::exception &error)
        {
            setError(std::string("ROS control bridge stopped unexpectedly: ") + error.what());
            LOG_ERROR << "ROS bridge stopped unexpectedly: " << error.what();
        }
        catch (...)
        {
            setError("ROS control bridge stopped unexpectedly.");
            LOG_ERROR << "ROS bridge stopped unexpectedly.";
        }
    }

    std::shared_ptr<CameraFrameStore> cameraFrames_;
    std::thread thread_;
    std::mutex mutex_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    bool ready_{false};
    bool cameraEnabled_{true};
    std::string cameraTopic_{kDefaultCameraTopic};
    std::string lastError_{"ROS bridge has not started."};
};

bool jsonNumber(const Json::Value &json, const char *name, double &value)
{
    if (!json.isMember(name))
    {
        value = 0.0;
        return true;
    }

    const auto &field = json[name];
    if (field.isNumeric())
    {
        value = field.asDouble();
        return std::isfinite(value);
    }

    if (field.isString())
    {
        try
        {
            value = std::stod(field.asString());
            return std::isfinite(value);
        }
        catch (...)
        {
            return false;
        }
    }

    return false;
}

drogon::HttpResponsePtr loginPage(bool showError)
{
    const std::string error = showError
                                  ? "<p class=\"auth-error\">Invalid username or password.</p>"
                                  : "";

    const std::string html =
        "<!doctype html>"
        "<html lang=\"en\">"
        "<head>"
        "<meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>AeroSentinel Login</title>"
        "<link rel=\"stylesheet\" href=\"/styles.css\">"
        "</head>"
        "<body class=\"auth-body\">"
        "<main class=\"auth-shell\">"
        "<section class=\"auth-panel\">"
        "<div class=\"auth-brand\">"
        "<div class=\"brand-mark\" aria-hidden=\"true\"><span></span></div>"
        "<div><strong>AeroSentinel</strong><small>CONTROL CENTER</small></div>"
        "</div>"
        "<p class=\"auth-kicker\">Secure mission access</p>"
        "<h1>Operator Sign In</h1>"
        "<form class=\"auth-form\" method=\"post\" action=\"/login\">"
        "<label>Username<input type=\"text\" name=\"username\" autocomplete=\"username\" required autofocus></label>"
        "<label>Password<input type=\"password\" name=\"password\" autocomplete=\"current-password\" required></label>" +
        error +
        "<button type=\"submit\">Unlock Dashboard</button>"
        "</form>"
        "</section>"
        "</main>"
        "</body>"
        "</html>";

    auto response = drogon::HttpResponse::newHttpResponse();
    response->addHeader("Content-Type", "text/html; charset=utf-8");
    response->setBody(html);
    return noStore(response);
}

void registerProtectedPage(const std::string &path,
                           const std::filesystem::path &indexPath,
                           const std::shared_ptr<SessionStore> &sessions)
{
    drogon::app().registerHandler(
        path,
        [indexPath, sessions](const drogon::HttpRequestPtr &request,
                              std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(redirectTo("/login"));
                return;
            }

            callback(drogon::HttpResponse::newFileResponse(indexPath.string()));
        },
        {drogon::Get});
}

void registerStaticFile(const std::string &route, const std::filesystem::path &filePath)
{
    drogon::app().registerHandler(
        route,
        [filePath](const drogon::HttpRequestPtr &,
                   std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            callback(drogon::HttpResponse::newFileResponse(filePath.string()));
        },
        {drogon::Get});
}

void addCameraHeaders(const drogon::HttpResponsePtr &response, const CameraJpegFrame &frame)
{
    const auto age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - frame.timestamp)
                         .count();

    response->addHeader("X-Camera-Topic", frame.topic);
    response->addHeader("X-Camera-Width", std::to_string(frame.width));
    response->addHeader("X-Camera-Height", std::to_string(frame.height));
    response->addHeader("X-Camera-Encoding", frame.encoding);
    response->addHeader("X-Camera-Age", std::to_string(age));
}

drogon::HttpResponsePtr cameraJpegResponse(const CameraJpegFrame &frame)
{
    auto response = drogon::HttpResponse::newHttpResponse();
    response->setContentTypeCodeAndCustomString(drogon::CT_CUSTOM, "image/jpeg");
    response->setBody(std::string(reinterpret_cast<const char *>(frame.jpeg.data()),
                                  frame.jpeg.size()));
    addCameraHeaders(response, frame);
    return noStore(response);
}

void registerApiHandlers(const std::shared_ptr<SessionStore> &sessions,
                         const std::shared_ptr<RosBridge> &rosBridge,
                         const std::shared_ptr<CameraFrameStore> &cameraFrames,
                         const std::shared_ptr<WebRtcCameraManager> &webrtcCamera)
{
    drogon::app().registerHandler(
        "/api/mission/alpha-0426",
        [sessions](const drogon::HttpRequestPtr &request,
                   std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(jsonError("authentication_required", drogon::k401Unauthorized));
                return;
            }

            Json::Value mission;
            mission["id"] = "ALPHA-0426";
            mission["status"] = "ACTIVE";
            mission["profile"] = "Search & Inspect";
            mission["survey"] = "Ridge Line Survey";
            mission["drone"] = "Sentinel-7B";
            mission["battery"] = 78;
            mission["altitude_m"] = 512;
            mission["speed_ms"] = 15.2;
            mission["distance_km"] = 1.2;
            mission["signal_percent"] = 94;
            mission["version"] = AEROSENTINEL_VERSION;

            callback(drogon::HttpResponse::newHttpJsonResponse(mission));
        },
        {drogon::Get});

    drogon::app().registerHandler(
        "/api/control/cmd_vel",
        [sessions, rosBridge](const drogon::HttpRequestPtr &request,
                              std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(jsonError("authentication_required", drogon::k401Unauthorized));
                return;
            }

            const auto json = request->getJsonObject();
            if (!json || !json->isObject())
            {
                callback(jsonError("invalid_velocity", drogon::k400BadRequest));
                return;
            }

            double linearX = 0.0;
            double angularZ = 0.0;
            if (!jsonNumber(*json, "linear_x", linearX) ||
                !jsonNumber(*json, "angular_z", angularZ))
            {
                callback(jsonError("invalid_velocity", drogon::k400BadRequest));
                return;
            }

            const auto maxLinear = std::abs(envDouble("AEROSENTINEL_MAX_LINEAR", 2.5));
            const auto maxAngular = std::abs(envDouble("AEROSENTINEL_MAX_ANGULAR", 1.8));
            linearX = std::clamp(linearX, -maxLinear, maxLinear);
            angularZ = std::clamp(angularZ, -maxAngular, maxAngular);

            const auto [ok, error] = rosBridge->publishCmdVel(linearX, angularZ);
            if (!ok)
            {
                callback(jsonError("ros_control_unavailable",
                                   drogon::k503ServiceUnavailable,
                                   error));
                return;
            }

            Json::Value result;
            result["linear_x"] = linearX;
            result["angular_z"] = angularZ;
            result["max_linear"] = maxLinear;
            result["max_angular"] = maxAngular;
            callback(noStore(drogon::HttpResponse::newHttpJsonResponse(result)));
        },
        {drogon::Post});

    drogon::app().registerHandler(
        "/api/camera/webrtc/config",
        [sessions, webrtcCamera](const drogon::HttpRequestPtr &request,
                                 std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(jsonError("authentication_required", drogon::k401Unauthorized));
                return;
            }

            Json::Value result;
            result["transport"] = "webrtc-video";
            result["codec"] = "H264";
            result["payload_type"] = kH264PayloadType;
            result["ssrc"] = Json::UInt64(kH264Ssrc);
            result["bitrate"] = h264Bitrate();
            result["fmtp"] = h264Fmtp();
            result["max_buffered_bytes"] = webRtcMaxBufferedBytes();
            result["iceServers"] = Json::Value(Json::arrayValue);
            for (const auto &server : webrtcCamera->iceServers())
            {
                result["iceServers"].append(server);
            }

            callback(noStore(drogon::HttpResponse::newHttpJsonResponse(result)));
        },
        {drogon::Get});

    drogon::app().registerHandler(
        "/api/camera/webrtc/offer",
        [sessions, webrtcCamera](const drogon::HttpRequestPtr &request,
                                 std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(jsonError("authentication_required", drogon::k401Unauthorized));
                return;
            }

            const auto json = request->getJsonObject();
            if (!json || !json->isObject() ||
                !json->isMember("sdp") || !(*json)["sdp"].isString())
            {
                callback(jsonError("invalid_webrtc_offer", drogon::k400BadRequest));
                return;
            }

            const auto type = json->isMember("type") && (*json)["type"].isString()
                                  ? (*json)["type"].asString()
                                  : "offer";
            if (type != "offer")
            {
                callback(jsonError("invalid_webrtc_offer", drogon::k400BadRequest));
                return;
            }

            std::string error;
            const auto answer = webrtcCamera->answerOffer((*json)["sdp"].asString(), type, error);
            if (!answer)
            {
                callback(jsonError("webrtc_answer_failed",
                                   drogon::k503ServiceUnavailable,
                                   error));
                return;
            }

            Json::Value result;
            result["id"] = answer->id;
            result["type"] = answer->type;
            result["sdp"] = answer->sdp;
            callback(noStore(drogon::HttpResponse::newHttpJsonResponse(result)));
        },
        {drogon::Post});

    drogon::app().registerHandler(
        "/api/camera/frame.jpg",
        [sessions, cameraFrames](const drogon::HttpRequestPtr &request,
                                 std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(jsonError("authentication_required", drogon::k401Unauthorized));
                return;
            }

            const auto frame = cameraFrames->getJpeg();
            if (!frame)
            {
                callback(jsonError("camera_frame_unavailable", drogon::k503ServiceUnavailable));
                return;
            }

            callback(cameraJpegResponse(*frame));
        },
        {drogon::Get});

}
} // namespace

int main(int argc, char *argv[])
{
    const auto publicDir = findPublicDir(argc > 0 ? argv[0] : nullptr);
    const auto indexPath = publicDir / "index.html";
    const auto port = portFromEnvironment();
    const auto bindAddress = envOrDefault("AEROSENTINEL_BIND_ADDRESS", "0.0.0.0");
    const AuthConfig auth{
        envOrDefault("AEROSENTINEL_USER", "admin"),
        envOrDefault("AEROSENTINEL_PASSWORD", "admin"),
        envFlag("AEROSENTINEL_SECURE_COOKIES")};
    auto sessions = std::make_shared<SessionStore>();
    auto cameraFrames = std::make_shared<CameraFrameStore>();
    auto rosBridge = std::make_shared<RosBridge>(cameraFrames);
    auto webrtcCamera = std::make_shared<WebRtcCameraManager>(cameraFrames);

    if (std::getenv("AEROSENTINEL_PASSWORD") == nullptr)
    {
        LOG_WARN << "AEROSENTINEL_PASSWORD is not set; using development credentials "
                 << "username='" << auth.username << "', password='admin'.";
    }

    rosBridge->start(argc, argv);
    rtcPreload();

    auto &app = drogon::app();
    app.setDocumentRoot(publicDir.string());
    app.setLogLevel(trantor::Logger::kWarn);
    app.addListener(bindAddress, port);

    registerProtectedPage("/", indexPath, sessions);
    registerProtectedPage("/mission/alpha-0426", indexPath, sessions);
    registerProtectedPage("/index.html", indexPath, sessions);

    registerStaticFile("/styles.css", publicDir / "styles.css");
    registerStaticFile("/app.js", publicDir / "app.js");
    registerStaticFile("/assets/drone.png", publicDir / "assets" / "drone.png");
    registerStaticFile("/assets/flight-map.png", publicDir / "assets" / "flight-map.png");
    registerStaticFile("/assets/live-feed.png", publicDir / "assets" / "live-feed.png");
    registerStaticFile("/assets/obstacle.png", publicDir / "assets" / "obstacle.png");

    app.registerHandler(
        "/login",
        [sessions](const drogon::HttpRequestPtr &request,
                   std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (isAuthenticated(request, sessions))
            {
                callback(redirectTo("/mission/alpha-0426"));
                return;
            }

            callback(loginPage(request->getParameter("error") == "1"));
        },
        {drogon::Get});

    app.registerHandler(
        "/login",
        [auth, sessions](const drogon::HttpRequestPtr &request,
                         std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            const auto body = std::string_view(request->body().data(), request->body().size());
            const auto username = requestFormValue(request, body, "username");
            const auto password = requestFormValue(request, body, "password");

            if (constantTimeEquals(username, auth.username) &&
                constantTimeEquals(password, auth.password))
            {
                auto response = redirectTo("/mission/alpha-0426", drogon::k303SeeOther);
                response->addHeader("Cache-Control", "no-store");
                response->addHeader("Set-Cookie", sessionCookie(sessions->create(), auth.secureCookies));
                callback(response);
                return;
            }

            LOG_WARN << "Failed login attempt for user '" << username << "'";
            callback(redirectTo("/login?error=1", drogon::k303SeeOther));
        },
        {drogon::Post});

    app.registerHandler(
        "/logout",
        [auth, sessions](const drogon::HttpRequestPtr &request,
                         std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            sessions->destroy(cookieValue(request, kSessionCookieName));
            auto response = redirectTo("/login");
            response->addHeader("Set-Cookie", expiredSessionCookie(auth.secureCookies));
            callback(response);
        },
        {drogon::Get, drogon::Post});

    registerApiHandlers(sessions, rosBridge, cameraFrames, webrtcCamera);

    LOG_WARN << "AeroSentinel Control Center listening on http://" << bindAddress << ":" << port
             << " with document root " << publicDir.string();
    app.run();
    webrtcCamera->closeAll();
    rtcCleanup();
    rosBridge->stop();
    return 0;
}
