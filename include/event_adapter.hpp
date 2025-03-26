#pragma once

#include <vector>
#include <event_camera_codecs/event_processor.h>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>

struct DvsEvent {
  int64_t ts;     // ns
  uint16_t x;
  uint16_t y;
  uint8_t polarity;
};

// 在类定义前添加文件流声明
// static std::ofstream debugFile_("time_debug_integrator.txt");

class EventBatchProcessor : public event_camera_codecs::EventProcessor {
public:
  explicit EventBatchProcessor(size_t batch_size = 1000) 
    : batch_size_(batch_size) {
    buffer_.reserve(batch_size_);  // 预分配空间
  }

  // 核心回调接口
  void eventCD(uint64_t ts, uint16_t x, uint16_t y, uint8_t polarity) override {
    // std::cout << "eventCD---ts(ns): " << shorten_time(ts)* 1000 << ", x: " << x << ", y: " << y << ", polarity: " << static_cast<int>(polarity) << std::endl;
    // std::cout << "ts: " << ts << ", x: " << x << ", y: " << y << ", polarity: " << polarity << std::endl;
    // debugFile_ << "sensor_time: " << ts << "\n";

    buffer_.emplace_back(DvsEvent{shorten_time(ts)* 1000 , x, y, polarity});
    if (buffer_.size() >= batch_size_) {
      flush();
    }
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override{}; // called after no more events decoded in this packet
  void rawData(const char *, size_t) override{};  // passthrough of raw data

  // 获取批处理事件
  const std::vector<DvsEvent>& events() const { 
    return accumulated_events_; 
  }

  // 手动刷新缓冲区
  void flush() {
    if (!buffer_.empty()) {
      // 将当前缓冲区事件追加到累计列表
      accumulated_events_.insert(accumulated_events_.end(), 
                               buffer_.begin(), buffer_.end());
      buffer_.clear();
    }
  }

  void clear() {
    accumulated_events_.clear();
    buffer_.clear();
  }

  void setHasSensorTimeSinceEpoch(bool b) { hasSensorTimeSinceEpoch_ = b; }
  int64_t shorten_time(uint64_t t)
  {
    if (hasSensorTimeSinceEpoch_) {
      if (!hasStartTime_) {
        startTime_ = t;
        hasStartTime_ = true;
      }
      // debugFile_ << "hasSensorTimeSinceEpoch: " << hasSensorTimeSinceEpoch_
      //            << ", startTime: " << startTime_
      //            << ", Before: " << t << ", After: " << static_cast<int32_t>(t - startTime_) << "\n";
      return (static_cast<int64_t>(t - startTime_));
    }
    // debugFile_ << "hasSensorTimeSinceEpoch: " << hasSensorTimeSinceEpoch_
    //            << ", startTime: " << startTime_
    //            << ", Before: " << t << ", After: " << static_cast<int32_t>(t - startTime_) << "\n";
    return (static_cast<int64_t>(t - startTime_));
  }




private:
  std::vector<DvsEvent> buffer_;
  std::vector<DvsEvent> accumulated_events_;  // 累计所有批次
  size_t batch_size_;


  bool hasStartTime_{false};
  bool hasSensorTimeSinceEpoch_{false};
  uint64_t startTime_{0};

  

}; 