#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <unistd.h>

#include "../include/data_type.h"
#include "../include/remote.h"
#include "cnpy.h"
#include "yaml-cpp/yaml.h"
#include <arpa/inet.h>
#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define DEG2RAD(x) ((x)*M_PI / 180.f)

struct LidarConfig {
  std::string frame_id;
  std::string port;
  std::string hostip;
  std::string sensorip;
  std::string scanfreq;
  std::string filter;
  std::string laser_enable;
  std::string scan_range_start;
  std::string scan_range_stop;
  bool inverted;
  int angle_offset;
};

void declare_config(LidarConfig &config) {
  config::name("LidarConfig");

  config::field(config.frame_id, "frame_id");
  config::field(config.port, "port");
  config::field(config.hostip, "hostip");
  config::field(config.sensorip, "sensorip");
  config::field(config.scanfreq, "scanfreq");
  config::field(config.filter, "filter");
  config::field(config.laser_enable, "laser_enable");
  config::field(config.scan_range_start, "scan_range_start");
  config::field(config.scan_range_stop, "scan_range_stop");
  config::field(config.inverted, "inverted");
  config::field(config.angle_offset, "angle_offset");
}

class lakibeam1_scan {
public:
  lakibeam1_scan(std::string config_path) {
    get_parameters(config_path);
    scan_config();
    create_socket();
  }

  void scan_publish() {
    std::cout << "scan_publish" << std::endl;
    // rclcpp::sleep_for(std::chrono::milliseconds(2000));
    // get_telemetry_data(sensorip);
    while (1) {
      if (scan_vec_ready == 0) {
        while (1) {
          if (j == 12) {
            unsigned int len = sizeof(clent_addr);
            recvfrom(sockfd, &MSOP_Data, sizeof(MSOP_Data), 0,
                     (struct sockaddr *)&clent_addr, &len);
            if (MSOP_Data.BlockID[0].Azimuth == 0) {
              // scan_end = scan_begin; // TODO
              // scan_begin = rclcpp::Clock().now(); // TODO
            }
            if ((MSOP_Data.BlockID[1].Azimuth - MSOP_Data.BlockID[0].Azimuth) >
                0) {
              resolution = (MSOP_Data.BlockID[1].Azimuth -
                            MSOP_Data.BlockID[0].Azimuth) /
                           16;
            }
            j = 0;
          }

          for (; j < 12; j++) {
            for (i = 0; i < 16; i++) {
              bm_response_scan_t response_ptr;
              response_ptr.angle =
                  (MSOP_Data.BlockID[j].Azimuth + (resolution * i));
              if (MSOP_Data.BlockID[j].DataFlag == 0xEEFF) {
                if (response_ptr.angle == 0) {
                  if (!scan_vec.empty() & (scan_vec_ready == 0)) {
                    scan_vec_ready = 1;
                    if (scan_vec.size() < 1200) {
                      j = 12;
                    }
                    break;
                  }
                }
                response_ptr.dist = MSOP_Data.BlockID[j].Result[i].Dist_1;
                response_ptr.rssi = MSOP_Data.BlockID[j].Result[i].RSSI_1;
                scan_vec.push_back(response_ptr);
              }
            }
            if (scan_vec_ready == 1) {
              break;
            }
          }
          if (scan_vec_ready == 1) {
            break;
          }
        }
      }

      if (scan_vec_ready == 1) {
        uint16_t num_readings;
        // float duration = (scan_begin - scan_end).seconds();

        num_readings = scan_vec.size();
        std::chrono::milliseconds time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());
        // auto time = std::chrono::system_clock::now();
        //  scan.header.stamp = scan_begin;
        //  scan.angle_min = DEG2RAD(-180 + angle_offset);
        //  scan.angle_max = DEG2RAD(180 + angle_offset);
        //  scan.angle_increment = 2.0 * M_PI / num_readings;
        //  scan.scan_time = duration;
        //  scan.time_increment = duration / (float)num_readings;
        //  scan.range_min = 0.0;
        //  scan.range_max = 100.0;

        std::vector<double> scan_ranges;
        std::vector<double> scan_intensities;
        scan_ranges.resize(num_readings);
        scan_intensities.resize(num_readings);
        for (int idx = 0; idx < num_readings; idx++) {
          if (!inverted) {
            scan_ranges[idx] = scan_vec[idx].dist / 1000.0;
            scan_intensities[idx] = scan_vec[idx].rssi;
          } else {
            scan_ranges[num_readings - idx - 1] = scan_vec[idx].dist / 1000.0;
            scan_intensities[num_readings - idx - 1] = scan_vec[idx].rssi;
          }
        }

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "timestamp";
        out << YAML::Value << time_ms.count();
        out << YAML::EndMap;

        std::string prefix = "output/";
        std::ofstream fout(prefix + "metadata_" + std::to_string(scan_num) + ".yaml");
        fout << out.c_str();

        cnpy::npy_save(prefix + "intensities_" + std::to_string(scan_num) + ".npy",
                       &scan_intensities[0], {num_readings}, "w");
        cnpy::npy_save(prefix + "ranges_" + std::to_string(scan_num) + ".npy",
                       &scan_ranges[0], {num_readings}, "w");
        ++scan_num;
        scan_vec_ready = 0;
        scan_vec.clear();
      }
    }
    close(sockfd);
  }

protected:
  void get_parameters(std::string config_path) {
    config_ = config::fromYamlFile<LidarConfig>(config_path);
    std::cout << config::toString(config_) << std::endl;
  };

  void scan_config() {
    std::cout << "scan_config" << std::endl;
    sensor_config(config_.sensorip, "/api/v1/sensor/scanfreq",
                  config_.scanfreq);
    sensor_config(config_.sensorip, "/api/v1/sensor/laser_enable",
                  config_.laser_enable);
    sensor_config(config_.sensorip, "/api/v1/sensor/scan_range/start",
                  config_.scan_range_start);
    sensor_config(config_.sensorip, "/api/v1/sensor/scan_range/stop",
                  config_.scan_range_stop);
    std::cout << "scan_config1" << std::endl;
  };
  int create_socket() {
    std::cout << "create_socket" << std::endl;
    // rclcpp::sleep_for(std::chrono::milliseconds(2000));
    // get_telemetry_data(sensorip);
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
      std::cout << "Failed to create socket" << std::endl;
      return -1;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(config_.hostip.c_str());
    ser_addr.sin_port = htons(atoi(config_.port.c_str()));

    if (bind(sockfd, (struct sockaddr *)&ser_addr, sizeof(ser_addr)) < 0) {
      std::cout << "Socket bind error!" << std::endl;
      return -1;
    }
    return 0;
  };

private:
  LidarConfig config_;
  int resolution = 25, scan_vec_ready = 0, angle_offset;
  bool inverted;
  struct sockaddr_in ser_addr, clent_addr;
  int i = 0, j = 12;
  int sockfd;
  std::vector<bm_response_scan_t> scan_vec;
  int scan_num = 0;
};

int main(int argc, char **argv) {
  std::string config_path;
  if (argc < 2) {
    // std::cout << "Usage: ./lakibeam1_scan_node <path_to_config>" <<
    // std::endl;
    config_path = "config/default.yaml";
  } else {
    config_path = std::string(argv[1]);
  }
  auto node = std::make_shared<lakibeam1_scan>(config_path);
  node->scan_publish();

  return 0;
}
