#include "mm_bind.h"
#ifdef USE_WEBSERVER
#include "esphome/components/json/json_util.h"
#include "esphome/components/meshmesh/meshmesh.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

#include <cstdlib>
#include <cstdio>
#include <string>

namespace esphome {
namespace mm_bind {

static const char *const TAG = "mm_bind";

MMBind::MMBind(web_server_base::WebServerBase *base) : base_(base) {
#ifdef USE_ESP32
  to_schedule_lock_ = xSemaphoreCreateMutex();
#endif
}

std::string MMBind::get_config_json() {
  return json::build_json([this](JsonObject root) {
    root["title"] = App.get_friendly_name().empty() ? App.get_name() : App.get_friendly_name();
    root["comment"] = App.get_comment();
    root["lang"] = "en";
  });
}

void MMBind::setup() {
  ESP_LOGCONFIG(TAG, "Setting up web server with bind value %06X",
                meshmesh::MeshmeshComponent::getInstance()->bindedServer());
  if (!meshmesh::MeshmeshComponent::getInstance()->isDisabled()) {
    return;
  }
  this->can_run_ = true;
  this->base_->init();
  this->base_->add_handler(this);
}

void MMBind::loop() {
  if (!can_run_)
    return;
#ifdef USE_ESP32
  if (xSemaphoreTake(this->to_schedule_lock_, 0L)) {
    std::function<void()> fn;
    if (!to_schedule_.empty()) {
      // scheduler execute things out of order which may lead to incorrect state
      // this->defer(std::move(to_schedule_.front()));
      // let's execute it directly from the loop
      fn = std::move(to_schedule_.front());
      to_schedule_.pop_front();
    }
    xSemaphoreGive(this->to_schedule_lock_);
    if (fn) {
      fn();
    }
  }
#endif
}

void MMBind::dump_config() {
  ESP_LOGCONFIG(TAG, "Web Server:");
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", network::get_use_address().c_str(), this->base_->get_port());
}

float MMBind::get_setup_priority() const { return setup_priority::WIFI - 1.0f; }

void MMBind::handle_index_request(AsyncWebServerRequest *request) {
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", "{}");
  // No gzip header here because the HTML file is so small
  request->send(response);
}

void MMBind::handle_id_request(AsyncWebServerRequest *request) {
  JsonObject root;
  std::string data = json::build_json([this](JsonObject root) {
    char idhex[12];
    std::sprintf(idhex, "%X", system_get_chip_id());
    root["id"] = idhex;
  });
  request->send(200, "application/json", data.c_str());
}

void MMBind::handle_bind_get_request(AsyncWebServerRequest *request) {
  JsonObject root;
  auto val = meshmesh::MeshmeshComponent::getInstance()->bindedServer();
  std::string data = json::build_json([this, val](JsonObject root) {
    char idhex[12];
    std::sprintf(idhex, "%X", val);
    root["id"] = idhex;
  });
  request->send(200, "application/json", data.c_str());
}

void MMBind::handle_bind_set_request(AsyncWebServerRequest *request) {
  int params = request->params();
  for (int i = 0; i < params; i++) {
    AsyncWebParameter *p = request->getParam(i);
    // ESP_LOGW(TAG, "POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    if (p->name() == "id") {
      int val = std::stoi(p->value().c_str(), 0, 16);
      meshmesh::MeshmeshComponent::getInstance()->setBindedServer(val);
    }
  }

  JsonObject root;
  auto val = meshmesh::MeshmeshComponent::getInstance()->bindedServer();
  std::string data = json::build_json([this, val](JsonObject root) {
    char idhex[12];
    std::sprintf(idhex, "%X", val);
    root["id"] = idhex;
  });
  request->send(200, "application/json", data.c_str());
}

void MMBind::handle_reboot_request(AsyncWebServerRequest *request) {
  // App
  bool confirm{true};
  int params = request->params();
  for (int i = 0; i < params; i++) {
    AsyncWebParameter *p = request->getParam(i);
    if (p->name() == "confirm") {
      int val = std::stoi(p->value().c_str(), 0, 16);
      confirm = val == 1;
    }
  }
  char js[64];
  std::sprintf(js, "{\"confirm\":\"%d\"}", confirm);
  request->send(200, "application/json", js);
  if (confirm) {
    App.safe_reboot();
  }
}

bool MMBind::canHandle(AsyncWebServerRequest *request) {
  if (request->url() == "/")
    return true;
  if (request->method() == HTTP_GET && request->url() == "/id")
    return true;
  if ((request->method() == HTTP_GET || request->method() == HTTP_POST) && request->url() == "/bind")
    return true;
  if (request->method() == HTTP_POST && request->url() == "/reboot")
    return true;
  return false;
}
void MMBind::handleRequest(AsyncWebServerRequest *request) {
  if (request->url() == "/") {
    this->handle_index_request(request);
    return;
  }
  if (request->url() == "/id") {
    this->handle_id_request(request);
    return;
  }
  if (request->method() == HTTP_POST && request->url() == "/reboot") {
    this->handle_reboot_request(request);
    return;
  }
  if (request->method() == HTTP_GET && request->url() == "/bind") {
    this->handle_bind_get_request(request);
    return;
  }
  if (request->method() == HTTP_POST && request->url() == "/bind") {
    this->handle_bind_set_request(request);
    return;
  }
}

bool MMBind::isRequestHandlerTrivial() { return false; }

void MMBind::schedule_(std::function<void()> &&f) {
#ifdef USE_ESP32
  xSemaphoreTake(this->to_schedule_lock_, portMAX_DELAY);
  to_schedule_.push_back(std::move(f));
  xSemaphoreGive(this->to_schedule_lock_);
#else
  this->defer(std::move(f));
#endif
}

}  // namespace mm_bind
}  // namespace esphome
#endif
