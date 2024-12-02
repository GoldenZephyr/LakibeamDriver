#include "../include/remote.h"
#include "../thirdparty/rapidjson/document.h"
#include "../thirdparty/rapidjson/prettywriter.h"
#include <curl/curl.h>
#include <stdio.h>

using namespace rapidjson;
static size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                            void *userp) {
  ((std::string *)userp)->append((char *)contents, size * nmemb);
  return size * nmemb;
}

static size_t dummy_callback(void *buffer, size_t size, size_t nmemb,
                             void *userp) {
  return size * nmemb;
}

int sensor_config(std::string sensor_ipaddr, std::string parameter,
                  std::string value) {
  std::cout << "URL_RESTFUL_API" << std::endl;

  long http_code;
  CURL *curl = curl_easy_init();
  std::string URL_RESTFUL_API = "http://" + sensor_ipaddr + parameter;
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, URL_RESTFUL_API.c_str());
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, value.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, dummy_callback);
    if (curl_easy_perform(curl) == CURLE_OK) {
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
      if (http_code == 200) {

        std::cout << "Set " << URL_RESTFUL_API.c_str()
                  << ", Value: " << value.c_str() << " ... done" << std::endl;

      } else {

        std::cout << "Set " << URL_RESTFUL_API.c_str()
                  << "Value: " << value.c_str() << " ... failed!";
      }
    } else {
      std::cout << "http put error! please check lidar connection!";
    }
  }
  curl_easy_cleanup(curl);
  curl_global_cleanup();

  return 0;
}

int get_telemetry_data(std::string sensor_ipaddr) {
  CURL *curl;
  std::string readBuffer;
  std::string URL_API_FIRMWARE =
      "http://" + sensor_ipaddr + "/api/v1/system/firmware";
  std::string URL_API_MONITOR =
      "http://" + sensor_ipaddr + "/api/v1/system/monitor";
  std::string URL_API_OVERVIEW =
      "http://" + sensor_ipaddr + "/api/v1/sensor/overview";

  curl = curl_easy_init();
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);
  if (curl) {
    readBuffer = "";
    curl_easy_setopt(curl, CURLOPT_URL, URL_API_FIRMWARE.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    const char *json = const_cast<char *>(readBuffer.c_str());
    Document jsondoc;
    jsondoc.Parse(json);
    assert(jsondoc.IsObject());
    std::cout << "-------------------------------------------------"
              << std::endl;
    std::cout << "model:		" << jsondoc["model"].GetString()
              << std::endl;
    std::cout << "sn:		" << jsondoc["sn"].GetString() << std::endl;
    std::cout << "ver hw:		" << jsondoc["hw"].GetString()
              << std::endl;
    std::cout << "ver fpga:	" << jsondoc["fpga"].GetString() << std::endl;
    std::cout << "ver core:	" << jsondoc["core"].GetString() << std::endl;
    std::cout << "ver aux:	" << jsondoc["aux"].GetString() << std::endl;
  }

  curl = curl_easy_init();
  if (curl) {
    readBuffer = "";
    curl_easy_setopt(curl, CURLOPT_URL, URL_API_MONITOR.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    const char *json = const_cast<char *>(readBuffer.c_str());
    Document jsondoc;
    jsondoc.Parse(json);
    assert(jsondoc.IsObject());
    std::cout << "load average:	" << jsondoc["load_average"].GetDouble()
              << std::endl;
    std::cout << "men useage:	" << jsondoc["mem_useage"].GetDouble()
              << std::endl;
    std::cout << "uptime:		" << jsondoc["uptime"].GetDouble()
              << std::endl;
  }

  curl = curl_easy_init();
  if (curl) {
    readBuffer = "";
    curl_easy_setopt(curl, CURLOPT_URL, URL_API_OVERVIEW.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    const char *json = const_cast<char *>(readBuffer.c_str());
    Document jsondoc;
    jsondoc.Parse(json);
    assert(jsondoc.IsObject());
    std::cout << "scanfreq:	" << jsondoc["scanfreq"].GetInt() << "hz"
              << std::endl;

    std::cout << "motor rpm:	" << jsondoc["motor_rpm"].GetInt() << " "
              << (jsondoc["motor_rpm"].GetInt() / 60.f) << " hz" << std::endl;

    std::cout << "laser enable:	" << jsondoc["laser_enable"].GetBool()
              << std::endl;

    std::cout << "scan start:	" << jsondoc["scan_range"]["start"].GetInt()
              << " deg" << std::endl;

    std::cout << "scan stop:	" << jsondoc["scan_range"]["stop"].GetInt()
              << " deg" << std::endl;

    std::cout << "flt level:	" << jsondoc["filter"]["level"].GetInt()
              << std::endl;

    std::cout << "flt min_angle:	"
              << jsondoc["filter"]["min_angle"].GetInt() << std::endl;

    std::cout << "flt max_angle:	"
              << jsondoc["filter"]["max_angle"].GetInt() << std::endl;

    std::cout << "flt window:	" << jsondoc["filter"]["window"].GetInt()
              << std::endl;

    std::cout << "flt neighbors:    " << jsondoc["filter"]["neighbors"].GetInt()
              << std::endl;
    std::cout << "host ip:	" << jsondoc["host"]["ip"].GetString()
              << std::endl;
    std::cout << "host port:	" << jsondoc["host"]["port"].GetInt()
              << std::endl;
    std::cout << "-------------------------------------------------"
              << std::endl;
  }

  return 0;
}
