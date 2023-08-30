// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "moteus_pi3hat/pi3hat.h"
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/realtime.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <iostream>



namespace mjbots {
namespace moteus {

/// This class represents the interface to the moteus controllers.
/// Internally it uses a background thread to operate the pi3hat,
/// enabling the main thread to perform work while servo communication
/// is taking place.
class Pi3HatMoteusInterface {
 public:
  struct Options {
    int cpu = -1;
    Options set_cpu(int i)
    {
      Options a;
      a.cpu = i;
      return a;
    }
  };

  Pi3HatMoteusInterface()
      : options_{},//(options),
        thread_{}//(std::bind(&Pi3HatMoteusInterface::CHILD_Run, this)),
        // main_timeout_(m_tmout)
        {}

  ~Pi3HatMoteusInterface() {
    
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // RCLCPP_INFO(rclcpp::get_logger("DESC"),"DEL LOCK ");
      done_ = true;
    }
    condition_.notify_one();  
    // RCLCPP_INFO(rclcpp::get_logger("DESC"),"DEL  PASS");
    
  //  

    if(thread_.joinable())
    {
      // RCLCPP_INFO(rclcpp::get_logger("END"),"JOIN");
      thread_.join();
    }
    // RCLCPP_INFO(rclcpp::get_logger("END"),"PASSED JOIN");
  }

  struct ServoCommand {
    int id = 0;
    int bus = 1;

    moteus::Mode mode = moteus::Mode::kStopped;

    // For mode = kPosition or kZeroVelocity
    moteus::PositionCommand position;
    moteus::PositionResolution resolution;

    moteus::QueryCommandV2 query;
  };

  struct ServoReply {
    int id = 0;
    int bus = 0;
    moteus::QueryResultV2 result;
  };

  // This describes what you would like to do in a given control cycle
  // in terms of sending commands or querying data.
  struct Data {
    pi3hat::Span<ServoCommand> commands;

    pi3hat::Span<ServoReply> replies;
  };

  struct Output {
    size_t query_result_size = 0;
  };

  using CallbackFunction = std::function<void (const Output&)>;

  /// When called, this will schedule a cycle of communication with
  /// the servos.  The callback will be invoked from an arbitrary
  /// thread when the communication cycle has completed.
  ///
  /// All memory pointed to by @p data must remain valid until the
  /// callback is invoked.
  void set_options(int cpu, uint32_t m_to, uint32_t c_to, uint32_t r_to , bool att)
  {
    if(!std::isnan(cpu) && cpu >0 && cpu < 4)
      options_.cpu = cpu;
    else 
      throw std::logic_error("CPU value is wrong, choose a value between 0 and 3");
    if(m_to > 0 && c_to >0 && r_to > 0)
    {
      main_timeout_ = m_to;
      can_extra_timeout_ = c_to;
      rx_extra_timeout_ = r_to;
      attitude_ = att;
    }
    else  
      throw std::logic_error("All timeout should be greater than zero");
  }
  void start_communication()
  {
    thread_ = std::thread(std::bind(&Pi3HatMoteusInterface::CHILD_Run, this));
  }
  void Cycle(const Data& data, CallbackFunction callback) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {
        throw std::logic_error(
            "Cycle cannot be called until the previous has completed");
      }
    callback_ = std::move(callback);
    active_ = true;
    data_ = data;
    }
    condition_.notify_one();
    // RCLCPP_INFO(rclcpp::get_logger("CYCLE"),"PASS");

     
  }

 private:
  void CHILD_Run() {
    ConfigureRealtime(options_.cpu);

    pi3hat_.reset(new pi3hat::Pi3Hat({}));

    while (true)
    {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        //  RCLCPP_INFO(rclcpp::get_logger("RUN"),"RUN LOCK");
        // if (!active_) {
         
      
        condition_.wait(lock, [this]{return(active_ || done_);});
        
        // RCLCPP_INFO(rclcpp::get_logger("RUN"),"WAIT ENTER");
        if (done_) { 
          // RCLCPP_INFO(rclcpp::get_logger("END"),"The done input is call");

          RCLCPP_INFO(rclcpp::get_logger("DONE"),"EXIT THREAD ");
          return; }

        // if (!active_) { continue; }
          
        // }
        // RCLCPP_INFO(rclcpp::get_logger("RUN"),"RUN RELASE");
      }
       

      auto output = CHILD_Cycle(main_timeout_, rx_extra_timeout_, can_extra_timeout_, attitude_);
      CallbackFunction callback_copy;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // std::unique_lock<std::mutex> lock(mutex_);
        // RCLCPP_INFO(rclcpp::get_logger("DONE"),"EXEC LOCK");
        active_ = false;
        std::swap(callback_copy, callback_);
        // RCLCPP_INFO(rclcpp::get_logger("DONE"),"EXEC RELASE");
      }
     // RCLCPP_INFO(rclcpp::get_logger("Child_run"),"CALL CALLBACK");
      callback_copy(output);
    }
    
  }

  Output CHILD_Cycle(uint32_t main_to,uint32_t can_to, uint32_t rec_to, bool att) {
    tx_can_.resize(data_.commands.size());
    int out_idx = 0;
    for (const auto& cmd : data_.commands) {
      const auto& query = cmd.query;
      auto& can = tx_can_[out_idx++];
      can.expect_reply = query.any_set();
      can.id = cmd.id | (can.expect_reply ? 0x8000 : 0x0000);
      can.bus = cmd.bus;
      can.size = 0;

      moteus::WriteCanFrame write_frame(can.data, &can.size);
      switch (cmd.mode) {
        case Mode::kStopped: {
          moteus::EmitStopCommand(&write_frame);
          break;
        }
        case Mode::kPosition:
        case Mode::kZeroVelocity: {
          moteus::EmitPositionCommand(&write_frame, cmd.position, cmd.resolution);
          break;
        }
        default: {
          throw std::logic_error("unsupported mode");
        }
      }
      moteus::EmitQueryCommandV2(&write_frame, cmd.query);
    }

    rx_can_.resize(data_.commands.size() * 2);

    pi3hat::Pi3Hat::Input input;
    input.tx_can = { tx_can_.data(), tx_can_.size() };
    input.rx_can = { rx_can_.data(), rx_can_.size() };
    input.timeout_ns = main_timeout_; 
    input.rx_extra_wait_ns = this->rx_extra_timeout_;
    input.min_tx_wait_ns = this->can_extra_timeout_;
    input.request_attitude = attitude_;
    Options option_;
    Output result;

    const auto output = pi3hat_->Cycle(input);
    for (size_t i = 0; i < output.rx_can_size && i < data_.replies.size(); i++) {
      const auto& can = rx_can_[i];

      data_.replies[i].id = (can.id & 0x7f00) >> 8;
      data_.replies[i].bus = can.bus;
      data_.replies[i].result = moteus::ParseQueryResultV2(can.data, can.size);
      result.query_result_size = i + 1;
    }

    return result;
  }

  Options options_;


  /// This block of variables are all controlled by the mutex.
  std::mutex mutex_;
  std::condition_variable condition_;

  bool active_ = false;
  bool done_ = false;
  CallbackFunction callback_;
  Data data_;

  std::thread thread_;
  uint32_t main_timeout_;
  uint32_t rx_extra_timeout_;
  uint32_t can_extra_timeout_;
  bool attitude_;


  /// All further variables are only used from within the child thread.

  std::unique_ptr<pi3hat::Pi3Hat> pi3hat_;

  // These are kept persistently so that no memory allocation is
  // required in steady state.
  std::vector<pi3hat::CanFrame> tx_can_;
  std::vector<pi3hat::CanFrame> rx_can_;
};


}
}
