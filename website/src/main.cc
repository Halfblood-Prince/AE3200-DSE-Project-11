#include <drogon/drogon.h>
#include <drogon/WebSocketController.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
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
    std::vector<unsigned char> jpeg;
    std::chrono::steady_clock::time_point timestamp;
    uint32_t width{0};
    uint32_t height{0};
    std::string encoding;
    uint64_t sequence{0};
    std::string topic;
};

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
        std::array<unsigned char, 32> bytes{};
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

std::string cookieValue(const drogon::HttpRequestPtr &request, const std::string &name)
{
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

std::vector<unsigned char> rosImageToJpeg(const sensor_msgs::msg::Image &msg, int quality)
{
    const auto image = rosImageToCvImage(msg);
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".jpg", image, encoded, {cv::IMWRITE_JPEG_QUALITY, quality}))
    {
        throw std::runtime_error("OpenCV failed to encode camera frame as JPEG.");
    }
    return encoded;
}

class CameraFrameStore
{
  public:
    void updateFromRosImage(const sensor_msgs::msg::Image &msg, const std::string &topic)
    {
        auto jpeg = rosImageToJpeg(msg, cameraJpegQuality());

        std::lock_guard<std::mutex> lock(mutex_);
        jpeg_ = std::move(jpeg);
        timestamp_ = std::chrono::steady_clock::now();
        width_ = msg.width;
        height_ = msg.height;
        encoding_ = msg.encoding;
        ++sequence_;
        topic_ = topic;
        condition_.notify_all();
    }

    std::optional<CameraFrame> get() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshotLocked();
    }

    std::optional<CameraFrame> waitForFrame(uint64_t lastSequence,
                                            std::chrono::milliseconds timeout) const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto hasNewFrame = condition_.wait_for(lock, timeout, [&] {
            return !jpeg_.empty() && sequence_ != lastSequence;
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
        if (jpeg_.empty())
        {
            return std::nullopt;
        }

        return CameraFrame{jpeg_, timestamp_, width_, height_, encoding_, sequence_, topic_};
    }

    mutable std::mutex mutex_;
    mutable std::condition_variable condition_;
    std::vector<unsigned char> jpeg_;
    std::chrono::steady_clock::time_point timestamp_{};
    uint32_t width_{0};
    uint32_t height_{0};
    std::string encoding_;
    uint64_t sequence_{0};
    std::string topic_{kDefaultCameraTopic};
};

class CameraStreamSocket : public drogon::WebSocketController<CameraStreamSocket, false>
{
  public:
    CameraStreamSocket(std::shared_ptr<SessionStore> sessions,
                       std::shared_ptr<CameraFrameStore> cameraFrames)
        : sessions_(std::move(sessions)), cameraFrames_(std::move(cameraFrames))
    {
    }

    void handleNewMessage(const drogon::WebSocketConnectionPtr &,
                          std::string &&,
                          const drogon::WebSocketMessageType &) override
    {
    }

    void handleNewConnection(const drogon::HttpRequestPtr &request,
                             const drogon::WebSocketConnectionPtr &connection) override
    {
        if (!isAuthenticated(request, sessions_))
        {
            connection->shutdown();
            return;
        }

        auto cameraFrames = cameraFrames_;
        std::thread([cameraFrames, connection] {
            uint64_t lastSequence = 0;
            auto nextFrameAt = std::chrono::steady_clock::now();

            while (connection->connected())
            {
                auto frame = cameraFrames->waitForFrame(lastSequence, std::chrono::seconds(1));
                if (!frame)
                {
                    continue;
                }
                lastSequence = frame->sequence;

                const auto now = std::chrono::steady_clock::now();
                if (now < nextFrameAt)
                {
                    std::this_thread::sleep_until(nextFrameAt);
                    if (!connection->connected())
                    {
                        break;
                    }
                }

                connection->send(
                    reinterpret_cast<const char *>(frame->jpeg.data()),
                    static_cast<uint64_t>(frame->jpeg.size()),
                    drogon::WebSocketMessageType::Binary);

                nextFrameAt = std::chrono::steady_clock::now() +
                              std::chrono::microseconds(kCameraStreamFramePeriodUs);
            }
        }).detach();
    }

    void handleConnectionClosed(const drogon::WebSocketConnectionPtr &) override
    {
    }

    WS_PATH_LIST_BEGIN
    WS_PATH_ADD("/api/camera/live");
    WS_PATH_LIST_END

  private:
    std::shared_ptr<SessionStore> sessions_;
    std::shared_ptr<CameraFrameStore> cameraFrames_;
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

void addCameraHeaders(const drogon::HttpResponsePtr &response, const CameraFrame &frame)
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

drogon::HttpResponsePtr cameraJpegResponse(const CameraFrame &frame)
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
                         const std::shared_ptr<CameraFrameStore> &cameraFrames)
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
        "/api/camera/frame.jpg",
        [sessions, cameraFrames](const drogon::HttpRequestPtr &request,
                                 std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
            if (!isAuthenticated(request, sessions))
            {
                callback(jsonError("authentication_required", drogon::k401Unauthorized));
                return;
            }

            const auto frame = cameraFrames->get();
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

    if (std::getenv("AEROSENTINEL_PASSWORD") == nullptr)
    {
        LOG_WARN << "AEROSENTINEL_PASSWORD is not set; using development credentials "
                 << "username='" << auth.username << "', password='admin'.";
    }

    rosBridge->start(argc, argv);

    auto &app = drogon::app();
    app.setDocumentRoot(publicDir.string());
    app.setLogLevel(trantor::Logger::kWarn);
    app.addListener(bindAddress, port);
    app.registerController(std::make_shared<CameraStreamSocket>(sessions, cameraFrames));

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

    registerApiHandlers(sessions, rosBridge, cameraFrames);

    LOG_WARN << "AeroSentinel Control Center listening on http://" << bindAddress << ":" << port
             << " with document root " << publicDir.string();
    app.run();
    rosBridge->stop();
    return 0;
}
