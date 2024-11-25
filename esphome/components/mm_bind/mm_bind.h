#pragma once

#include "esphome/components/web_server_base/web_server_base.h"
#ifdef USE_WEBSERVER
#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"

#include <map>
#include <vector>
#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <deque>
#endif

namespace esphome {
namespace mm_bind {

/// Internal helper struct that is used to parse incoming URLs
struct UrlMatch {
  std::string domain;  ///< The domain of the component, for example "sensor"
  std::string id;      ///< The id of the device that's being accessed, for example "living_room_fan"
  std::string method;  ///< The method that's being called, for example "turn_on"
  bool valid;          ///< Whether this match is valid
};

enum JsonDetail { DETAIL_ALL, DETAIL_STATE };

/** This class allows users to create a web server with their ESP nodes.
 *
 * Behind the scenes it's using AsyncWebServer to set up the server. It exposes 3 things:
 * an index page under '/' that's used to show a simple web interface (the css/js is hosted
 * by esphome.io by default), an event source under '/events' that automatically sends
 * all state updates in real time + the debug log. Lastly, there's an REST API available
 * under the '/light/...', '/sensor/...', ... URLs. A full documentation for this API
 * can be found under https://esphome.io/web-api/index.html.
 */
class MMBind : public Component, public AsyncWebHandler {
 public:
  MMBind(web_server_base::WebServerBase *base);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Setup the internal web server and register handlers.
  void setup() override;
  void loop() override;

  void dump_config() override;

  /// MQTT setup priority.
  float get_setup_priority() const override;

  /// Handle an index request under '/'.
  void handle_index_request(AsyncWebServerRequest *request);

  void handle_id_request(AsyncWebServerRequest *request);
  void handle_bind_get_request(AsyncWebServerRequest *request);
  void handle_bind_set_request(AsyncWebServerRequest *request);
  void handle_reboot_request(AsyncWebServerRequest *request);
  /// Return the webserver configuration as JSON.
  std::string get_config_json();

  /// Override the web handler's canHandle method.
  bool canHandle(AsyncWebServerRequest *request) override;
  /// Override the web handler's handleRequest method.
  void handleRequest(AsyncWebServerRequest *request) override;
  /// This web handle is not trivial.
  bool isRequestHandlerTrivial() override;  // NOLINT(readability-identifier-naming)

 protected:
  void schedule_(std::function<void()> &&f);
  web_server_base::WebServerBase *base_;
  bool expose_log_{true};
  bool can_run_{false};
#ifdef USE_ESP32
  std::deque<std::function<void()>> to_schedule_;
  SemaphoreHandle_t to_schedule_lock_;
#endif
};

}  // namespace mm_bind
}  // namespace esphome
#endif
