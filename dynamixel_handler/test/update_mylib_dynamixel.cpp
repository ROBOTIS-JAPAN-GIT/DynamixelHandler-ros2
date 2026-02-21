#include "dynamixel_communicator.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum class Mode {
  Ping,
  BroadcastPing,
  BulkRead,
  FastBulkRead,
  ReadSingle,
  ReadMulti,
  SyncReadSingle,
  SyncReadMulti,
  FastSyncReadSingle,
  FastSyncReadMulti,
};

struct Options {
  std::string port = "/dev/ttyUSB0";
  int baudrate = 1000000;
  int latency_timer = 1;
  int scan_min_id = 0;
  int scan_max_id = 20;
  std::vector<uint8_t> ids;
  int iterations = 500;
  int warmup = 50;
  int retry_num = 1;
  int retry_interval_ms = 1;
  bool verbose = false;
  Mode mode = Mode::SyncReadMulti;
};

struct IterResult {
  bool full_success = false;
  size_t success_items = 0;
  size_t packets = 0;
};

void print_usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [options]\n"
      << "Options:\n"
      << "  --port <path>              serial port (default: /dev/ttyUSB0)\n"
      << "  --baud <int>               baudrate (default: 1000000)\n"
      << "  --latency <int>            latency timer ms (default: 1)\n"
      << "  --ids <csv>                target ids, example: 1,2,3\n"
      << "  --scan-min <int>           scan lower id when --ids omitted\n"
      << "  --scan-max <int>           scan upper id when --ids omitted\n"
      << "  --iter <int>               benchmark iterations (default: 500)\n"
      << "  --warmup <int>             warmup iterations (default: 50)\n"
      << "  --retry-num <int>          retry count config (default: 1)\n"
      << "  --retry-interval <int>     retry interval ms (default: 1)\n"
      << "  --mode <name>              mode name:\n"
      << "                             ping\n"
      << "                             broadcast_ping\n"
      << "                             bulk_read\n"
      << "                             fast_bulk_read\n"
      << "                             read_single\n"
      << "                             read_multi\n"
      << "                             sync_read_single\n"
      << "                             sync_read_multi\n"
      << "                             fast_sync_read_single\n"
      << "                             fast_sync_read_multi\n"
      << "  --verbose                  enable communicator verbose logs\n";
}

bool parse_int(const std::string& s, int* value) {
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') {
    return false;
  }
  *value = static_cast<int>(v);
  return true;
}

bool parse_ids(const std::string& csv, std::vector<uint8_t>* ids) {
  std::stringstream ss(csv);
  std::string item;
  std::vector<uint8_t> out;

  while (std::getline(ss, item, ',')) {
    if (item.empty()) {
      continue;
    }
    int v = 0;
    if (!parse_int(item, &v) || v < 0 || v > 252) {
      return false;
    }
    out.push_back(static_cast<uint8_t>(v));
  }

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  *ids = std::move(out);
  return true;
}

bool parse_mode(const std::string& s, Mode* mode) {
  if (s == "ping") {
    *mode = Mode::Ping;
  } else if (s == "broadcast_ping") {
    *mode = Mode::BroadcastPing;
  } else if (s == "bulk_read") {
    *mode = Mode::BulkRead;
  } else if (s == "fast_bulk_read") {
    *mode = Mode::FastBulkRead;
  } else if (s == "read_single") {
    *mode = Mode::ReadSingle;
  } else if (s == "read_multi") {
    *mode = Mode::ReadMulti;
  } else if (s == "sync_read_single") {
    *mode = Mode::SyncReadSingle;
  } else if (s == "sync_read_multi") {
    *mode = Mode::SyncReadMulti;
  } else if (s == "fast_sync_read_single") {
    *mode = Mode::FastSyncReadSingle;
  } else if (s == "fast_sync_read_multi") {
    *mode = Mode::FastSyncReadMulti;
  } else {
    return false;
  }
  return true;
}

std::string mode_name(Mode mode) {
  switch (mode) {
    case Mode::Ping:
      return "ping";
    case Mode::BroadcastPing:
      return "broadcast_ping";
    case Mode::BulkRead:
      return "bulk_read";
    case Mode::FastBulkRead:
      return "fast_bulk_read";
    case Mode::ReadSingle:
      return "read_single";
    case Mode::ReadMulti:
      return "read_multi";
    case Mode::SyncReadSingle:
      return "sync_read_single";
    case Mode::SyncReadMulti:
      return "sync_read_multi";
    case Mode::FastSyncReadSingle:
      return "fast_sync_read_single";
    case Mode::FastSyncReadMulti:
      return "fast_sync_read_multi";
  }
  return "unknown";
}

bool is_read_mode(Mode mode) {
  switch (mode) {
    case Mode::BulkRead:
    case Mode::FastBulkRead:
    case Mode::ReadSingle:
    case Mode::ReadMulti:
    case Mode::SyncReadSingle:
    case Mode::SyncReadMulti:
    case Mode::FastSyncReadSingle:
    case Mode::FastSyncReadMulti:
      return true;
    case Mode::Ping:
    case Mode::BroadcastPing:
      return false;
  }
  return false;
}

size_t count_values(const std::map<uint8_t, std::vector<int64_t>>& id_values_map) {
  size_t count = 0;
  for (const auto& id_values : id_values_map) {
    count += id_values.second.size();
  }
  return count;
}

void print_read_values(const std::map<uint8_t, std::vector<int64_t>>& id_values_map) {
  std::cout << "read_values=";
  bool first_id = true;
  for (const auto& id_values : id_values_map) {
    if (!first_id) {
      std::cout << ' ';
    }
    first_id = false;
    std::cout << static_cast<int>(id_values.first) << ":[";
    for (size_t i = 0; i < id_values.second.size(); ++i) {
      if (i) {
        std::cout << ',';
      }
      std::cout << id_values.second[i];
    }
    std::cout << "]";
  }
  if (id_values_map.empty()) {
    std::cout << "(empty)";
  }
  std::cout << "\n";
}

void print_ids(const std::vector<uint8_t>& ids) {
  std::cout << "ids=";
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i) {
      std::cout << ',';
    }
    std::cout << static_cast<int>(ids[i]);
  }
  std::cout << "\n";
}

std::vector<uint8_t> scan_ids(DynamixelCommunicator& comm, int min_id, int max_id) {
  std::vector<uint8_t> ids;
  min_id = std::max(min_id, 0);
  max_id = std::min(max_id, 252);
  (void)comm.Ping_broadcast();
  const auto id_model_map_all = comm.ping_id_model_map_last_read();
  for (const auto& id_model : id_model_map_all) {
    if (id_model.first < static_cast<uint8_t>(min_id) || id_model.first > static_cast<uint8_t>(max_id)) {
      continue;
    }
    ids.push_back(id_model.first);
  }
  if (!ids.empty()) {
    return ids;
  }

  // fallback for environments where broadcast ping is unavailable
  for (int id = min_id; id <= max_id; ++id) {
    if (comm.Ping(static_cast<uint8_t>(id))) {
      ids.push_back(static_cast<uint8_t>(id));
    }
  }
  return ids;
}

IterResult run_iteration(
    DynamixelCommunicator& comm,
    Mode mode,
    const std::vector<uint8_t>& ids,
    const std::vector<DynamixelAddress>& multi_addrs,
    std::map<uint8_t, std::vector<int64_t>>* read_values = nullptr) {
  IterResult result;
  if (read_values) {
    read_values->clear();
  }

  switch (mode) {
    case Mode::Ping: {
      result.packets = ids.size();
      for (uint8_t id : ids) {
        if (comm.Ping(id)) {
          ++result.success_items;
        }
      }
      result.full_success = (result.success_items == ids.size());
      return result;
    }

    case Mode::BroadcastPing: {
      if (ids.empty()) {
        result.full_success = true;
        return result;
      }
      result.packets = 1;
      const bool ping_ok = !comm.Ping_broadcast().empty();
      const auto id_model_map = comm.ping_id_model_map_last_read();
      for (uint8_t id : ids) {
        if (id_model_map.count(id)) {
          ++result.success_items;
        }
      }
      result.full_success = ping_ok && (result.success_items == ids.size());
      return result;
    }

    case Mode::BulkRead: {
      result.packets = 1;
      std::map<uint8_t, DynamixelAddress> id_addr_map;
      for (uint8_t id : ids) id_addr_map.emplace(id, AddrCommon::model_number);
      const auto values = comm.BulkRead(id_addr_map);
      if (read_values) {
        for (const auto& id_value : values) {
          (*read_values)[id_value.first] = {id_value.second};
        }
      }
      result.success_items = values.size();
      result.full_success = !comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == ids.size();
      return result;
    }

    case Mode::FastBulkRead: {
      result.packets = 1;
      std::map<uint8_t, DynamixelAddress> id_addr_map;
      for (uint8_t id : ids) id_addr_map.emplace(id, AddrCommon::model_number);
      const auto values = comm.BulkRead_fast(id_addr_map);
      if (read_values) {
        for (const auto& id_value : values) {
          (*read_values)[id_value.first] = {id_value.second};
        }
      }
      result.success_items = values.size();
      result.full_success = !comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == ids.size();
      return result;
    }

    case Mode::ReadSingle: {
      result.packets = ids.size();
      for (uint8_t id : ids) {
        const int64_t value = comm.Read(AddrCommon::model_number, id);
        if (!comm.timeout_last_read() && !comm.comm_error_last_read()) {
          ++result.success_items;
          if (read_values) {
            (*read_values)[id] = {value};
          }
        }
      }
      result.full_success = (result.success_items == ids.size());
      return result;
    }

    case Mode::ReadMulti: {
      result.packets = ids.size();
      for (uint8_t id : ids) {
        const auto values = comm.Read(multi_addrs, id);
        if (!comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == multi_addrs.size()) {
          ++result.success_items;
          if (read_values) {
            (*read_values)[id] = values;
          }
        }
      }
      result.full_success = (result.success_items == ids.size());
      return result;
    }

    case Mode::SyncReadSingle: {
      result.packets = 1;
      const auto values = comm.SyncRead(AddrCommon::model_number, ids);
      if (read_values) {
        for (const auto& id_value : values) {
          (*read_values)[id_value.first] = {id_value.second};
        }
      }
      result.success_items = values.size();
      result.full_success = !comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == ids.size();
      return result;
    }

    case Mode::SyncReadMulti: {
      result.packets = 1;
      const auto values = comm.SyncRead(multi_addrs, ids);
      if (read_values) {
        *read_values = values;
      }
      result.success_items = values.size();
      result.full_success = !comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == ids.size();
      return result;
    }

    case Mode::FastSyncReadSingle: {
      result.packets = 1;
      const auto values = comm.SyncRead_fast(AddrCommon::model_number, ids);
      if (read_values) {
        for (const auto& id_value : values) {
          (*read_values)[id_value.first] = {id_value.second};
        }
      }
      result.success_items = values.size();
      result.full_success = !comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == ids.size();
      return result;
    }

    case Mode::FastSyncReadMulti: {
      result.packets = 1;
      const auto values = comm.SyncRead_fast(multi_addrs, ids);
      if (read_values) {
        *read_values = values;
      }
      result.success_items = values.size();
      result.full_success = !comm.timeout_last_read() && !comm.comm_error_last_read() && values.size() == ids.size();
      return result;
    }
  }

  return result;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        print_usage(argv[0]);
        std::exit(2);
      }
      return argv[++i];
    };

    if (arg == "--port") {
      opt.port = need_value("--port");
    } else if (arg == "--baud") {
      if (!parse_int(need_value("--baud"), &opt.baudrate)) {
        std::cerr << "invalid --baud\n";
        return 2;
      }
    } else if (arg == "--latency") {
      if (!parse_int(need_value("--latency"), &opt.latency_timer)) {
        std::cerr << "invalid --latency\n";
        return 2;
      }
    } else if (arg == "--scan-min") {
      if (!parse_int(need_value("--scan-min"), &opt.scan_min_id)) {
        std::cerr << "invalid --scan-min\n";
        return 2;
      }
    } else if (arg == "--scan-max") {
      if (!parse_int(need_value("--scan-max"), &opt.scan_max_id)) {
        std::cerr << "invalid --scan-max\n";
        return 2;
      }
    } else if (arg == "--ids") {
      if (!parse_ids(need_value("--ids"), &opt.ids)) {
        std::cerr << "invalid --ids\n";
        return 2;
      }
    } else if (arg == "--iter") {
      if (!parse_int(need_value("--iter"), &opt.iterations)) {
        std::cerr << "invalid --iter\n";
        return 2;
      }
    } else if (arg == "--warmup") {
      if (!parse_int(need_value("--warmup"), &opt.warmup)) {
        std::cerr << "invalid --warmup\n";
        return 2;
      }
    } else if (arg == "--retry-num") {
      if (!parse_int(need_value("--retry-num"), &opt.retry_num)) {
        std::cerr << "invalid --retry-num\n";
        return 2;
      }
    } else if (arg == "--retry-interval") {
      if (!parse_int(need_value("--retry-interval"), &opt.retry_interval_ms)) {
        std::cerr << "invalid --retry-interval\n";
        return 2;
      }
    } else if (arg == "--mode") {
      if (!parse_mode(need_value("--mode"), &opt.mode)) {
        std::cerr << "invalid --mode\n";
        print_usage(argv[0]);
        return 2;
      }
    } else if (arg == "--verbose") {
      opt.verbose = true;
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::cerr << "unknown arg: " << arg << "\n";
      print_usage(argv[0]);
      return 2;
    }
  }

  if (opt.iterations <= 0) {
    std::cerr << "--iter must be > 0\n";
    return 2;
  }
  if (opt.warmup < 0) {
    std::cerr << "--warmup must be >= 0\n";
    return 2;
  }

  DynamixelCommunicator comm(opt.port.c_str(), opt.baudrate, opt.latency_timer);
  comm.set_verbose(opt.verbose);
  comm.set_retry_config(opt.retry_num, opt.retry_interval_ms);

  if (!comm.OpenPort()) {
    std::cerr << "failed to open port: " << opt.port << "\n";
    return 1;
  }

  if (opt.ids.empty()) {
    opt.ids = scan_ids(comm, opt.scan_min_id, opt.scan_max_id);
    if (opt.ids.empty()) {
      std::cerr << "no servo found in scan range\n";
      comm.ClosePort();
      return 1;
    }
  }

  const std::vector<DynamixelAddress> multi_addrs = {
      AddrCommon::model_number,
      AddrCommon::id,
  };

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "mode=" << mode_name(opt.mode)
            << " port=" << opt.port
            << " baud=" << opt.baudrate
            << " latency=" << opt.latency_timer
            << " iter=" << opt.iterations
            << " warmup=" << opt.warmup
            << " retry=" << opt.retry_num << "@" << opt.retry_interval_ms << "ms\n";
  print_ids(opt.ids);

  for (int i = 0; i < opt.warmup; ++i) {
    (void)run_iteration(comm, opt.mode, opt.ids, multi_addrs);
  }

  size_t full_success_count = 0;
  size_t success_items_total = 0;
  size_t packets_total = 0;

  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < opt.iterations; ++i) {
    const IterResult r = run_iteration(comm, opt.mode, opt.ids, multi_addrs);
    full_success_count += static_cast<size_t>(r.full_success);
    success_items_total += r.success_items;
    packets_total += r.packets;
  }
  const auto t1 = std::chrono::steady_clock::now();

  const auto total_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  const double total_ms = static_cast<double>(total_us) / 1000.0;
  const double per_iter_us = static_cast<double>(total_us) / static_cast<double>(opt.iterations);
  const double iter_per_sec = (total_us > 0) ? (1e6 * opt.iterations / static_cast<double>(total_us)) : 0.0;
  const double packet_per_sec = (total_us > 0) ? (1e6 * packets_total / static_cast<double>(total_us)) : 0.0;
  const double item_per_sec = (total_us > 0) ? (1e6 * success_items_total / static_cast<double>(total_us)) : 0.0;

  std::cout << "result total_ms=" << total_ms
            << " per_iter_us=" << per_iter_us
            << " iter_per_sec=" << iter_per_sec
            << " packet_per_sec=" << packet_per_sec
            << " success_item_per_sec=" << item_per_sec
            << " full_success=" << full_success_count << "/" << opt.iterations
            << "\n";

  if (is_read_mode(opt.mode)) {
    std::map<uint8_t, std::vector<int64_t>> read_values;
    const IterResult snapshot = run_iteration(comm, opt.mode, opt.ids, multi_addrs, &read_values);
    std::cout << "read_snapshot id_count=" << read_values.size()
              << " value_count=" << count_values(read_values)
              << " success_item=" << snapshot.success_items
              << " full_success=" << (snapshot.full_success ? 1 : 0)
              << "\n";
    print_read_values(read_values);
  }

  comm.ClosePort();
  return 0;
}
