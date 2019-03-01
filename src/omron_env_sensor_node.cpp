#include <string>

#include <omron_env_sensor_msgs/DataShort.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/crc.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/states.hpp>
#include <boost/ref.hpp>
#include <boost/system/error_code.hpp>

namespace ba = boost::asio;
namespace bm = boost::msm;
namespace bmf = bm::front;
namespace bmb = bm::back;
namespace bp = boost::posix_time;
namespace bs = boost::system;

//
// context manipulated by the state machine
//

struct Context {
  Context(const std::string &_port, ba::io_service &ios, const ros::NodeHandle &_nh,
          const ros::NodeHandle &_pnh)
      : port(_port), serial(ios), timer(ios), nh(_nh), pnh(_pnh) {}

  std::string port;
  ba::serial_port serial;
  ba::deadline_timer timer;

  ros::NodeHandle nh, pnh;
};

//
// events to be processed by the state machine
//

struct Success {};
struct Error {};

//
// the state machine
//

struct MachineDef : bmf::state_machine_def< MachineDef > {

  //
  // states and sub-machines
  //

  struct WorkModeDef : bmf::state_machine_def< WorkModeDef > {

    // sub-states

    struct Ok : bmf::state<> {};

    struct Exit : bmf::exit_pseudo_state< Error > {};

    struct Opening : bmf::state<> {
      template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) const {
        try {
          typedef ba::serial_port Serial;

          if (fsm.ctx->serial.is_open()) {
            fsm.ctx->serial.close();
          }

          fsm.ctx->serial.open(fsm.ctx->port);
          fsm.ctx->serial.set_option(Serial::baud_rate(115200));
          fsm.ctx->serial.set_option(Serial::character_size(8));
          fsm.ctx->serial.set_option(Serial::stop_bits(Serial::stop_bits::one));
          fsm.ctx->serial.set_option(Serial::parity(Serial::parity::none));
          fsm.ctx->serial.set_option(Serial::flow_control(Serial::flow_control::none));

          // schedule an event processing after this entry action.
          // this is because the event processing has to be executed
          // after completing the transition to the Opening state
          fsm.ctx->serial.get_io_service().post(boost::bind(
              &bmb::state_machine< MachineDef >::process_event< Success >, fsm.parent, Success()));
        } catch (const bs::system_error &error) {
          ROS_ERROR_STREAM(error.what());
          fsm.ctx->serial.get_io_service().post(boost::bind(
              &bmb::state_machine< MachineDef >::process_event< Error >, fsm.parent, Error()));
        }
      }
    };

    struct RequestingData : bmf::state<> {
      RequestingData() {
        typedef boost::crc_optimal< 16, 0x8005, 0xFFFF, 0, true, true > CRC;

        cmd_[0] = 0x52;                   // header 1
        cmd_[1] = 0x42;                   // header 2
        cmd_[2] = 5 & 0x00ff;             // length L
        cmd_[3] = (5 & 0xff00) >> 8;      // length H
        cmd_[4] = 0x01;                   // command (0x01: read)
        cmd_[5] = 0x5022 & 0x00ff;        // address L (0x5022: latest data short)
        cmd_[6] = (0x5022 & 0xff00) >> 8; // address H

        CRC crc;
        crc.process_bytes(cmd_, 7);
        cmd_[7] = crc.checksum() & 0x00ff;        // CRC L
        cmd_[8] = (crc.checksum() & 0xff00) >> 8; // CRC H
      }

      template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) const {
        // set timeout
        fsm.ctx->timer.expires_from_now(bp::seconds(1));
        fsm.ctx->timer.async_wait(
            boost::bind(&RequestingData::handle_wait< FSM >, this, _1, boost::ref(fsm)));

        // start writing
        ba::async_write(
            fsm.ctx->serial, ba::buffer(cmd_),
            boost::bind(&RequestingData::handle_write< FSM >, this, _1, boost::ref(fsm)));
      }

      template < class FSM > void handle_wait(const bs::error_code &error, FSM &fsm) const {
        // do nothing if this timeout operation has been canceled
        if (error == ba::error::operation_aborted) {
          return;
        }
        // cancel simultaneous operation to launch no more events
        fsm.ctx->serial.cancel();
        // launch an event
        if (error) {
          ROS_ERROR_STREAM(error.message());
          fsm.parent->process_event(Error());
        } else {
          ROS_ERROR("Write timeout");
          fsm.parent->process_event(Error());
        }
      }

      template < class FSM > void handle_write(const bs::error_code &error, FSM &fsm) const {
        if (error == ba::error::operation_aborted) {
          return;
        }

        fsm.ctx->timer.cancel();

        if (error) {
          ROS_ERROR_STREAM(error.message());
          fsm.parent->process_event(Error());
        } else {
          fsm.parent->process_event(Success());
        }
      }

    private:
      uint8_t cmd_[9];
    };

    struct ReceivingData : bmf::state<> {
      template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) {
        if (!pub_) {
          pub_ = fsm.ctx->nh.template advertise< omron_env_sensor_msgs::DataShort >("data", true);
        }

        // set timeout
        fsm.ctx->timer.expires_from_now(bp::seconds(1));
        fsm.ctx->timer.async_wait(
            boost::bind(&ReceivingData::handle_wait< FSM >, this, _1, boost::ref(fsm)));

        // start reading
        ba::async_read(fsm.ctx->serial, ba::buffer(res_),
                       boost::bind(&ReceivingData::handle_read< FSM >, this, _1, boost::ref(fsm)));
      }

      template < class FSM > void handle_wait(const bs::error_code &error, FSM &fsm) const {
        if (error == ba::error::operation_aborted) {
          return;
        }

        fsm.ctx->serial.cancel();

        if (error) {
          ROS_ERROR_STREAM(error.message());
          fsm.parent->process_event(Error());
        } else {
          ROS_ERROR("Read timeout");
          fsm.parent->process_event(Error());
        }
      }

      template < class FSM > void handle_read(const bs::error_code &error, FSM &fsm) {
        if (error == ba::error::operation_aborted) {
          return;
        }

        fsm.ctx->timer.cancel();

        if (error) {
          ROS_ERROR_STREAM(error.message());
          fsm.parent->process_event(Error());
        } else {
          if (pub_.getNumSubscribers() > 0) {
            omron_env_sensor_msgs::DataShortPtr msg(new omron_env_sensor_msgs::DataShort());
            msg->header.stamp = ros::Time::now();
            msg->temperature = ((0x00ff & res_[8]) + ((0x00ff & res_[9]) << 8)) / 100.;
            msg->relative_humidity = ((0x00ff & res_[10]) + ((0x00ff & res_[11]) << 8)) / 100.;
            msg->ambient_light = (0x00ff & res_[12]) + ((0x00ff & res_[13]) << 8);
            msg->barometric_pressure = ((0x00ff & res_[14]) + ((0x00ff & res_[15]) << 8) +
                                        ((0x00ff & res_[16]) << 16) + ((0x00ff & res_[17]) << 24)) /
                                       1000.;
            msg->sound_noise = ((0x00ff & res_[18]) + ((0x00ff & res_[19]) << 8)) / 100.;
            msg->etvoc = (0x00ff & res_[20]) + ((0x00ff & res_[21]) << 8);
            msg->eco2 = (0x00ff & res_[22]) + ((0x00ff & res_[23]) << 8);
            msg->discomfort_index = ((0x00ff & res_[24]) + ((0x00ff & res_[25]) << 8)) / 100.;
            msg->heat_stroke = ((0x00ff & res_[26]) + ((0x00ff & res_[27]) << 8)) / 100.;
            pub_.publish(msg);
          }
          fsm.parent->process_event(Success());
        }
      }

    private:
      ros::Publisher pub_;
      uint8_t res_[30];
    };

    struct Sleeping : bmf::state<> {
      Sleeping() : start_(bp::min_date_time), cycle_(bp::neg_infin) {}

      template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) {
        if (cycle_.is_negative()) {
          cycle_ = bp::milliseconds(fsm.ctx->pnh.param("cycle", 1000));
        }

        // set the end time of sleeping. sleep duration must be in [0, cycle_].
        const bp::ptime now(bp::microsec_clock::universal_time());
        const bp::ptime end(boost::algorithm::clamp(start_ + cycle_, now, now + cycle_));

        // set the sleep timer
        fsm.ctx->timer.expires_at(end);
        fsm.ctx->timer.async_wait(
            boost::bind(&Sleeping::handle_wait< FSM >, this, _1, boost::ref(fsm)));

        // remember this end time as the next start time
        start_ = end;
      }

      template < class FSM > void handle_wait(const bs::error_code &error, FSM &fsm) const {
        if (error == ba::error::operation_aborted) {
          return;
        }

        if (error) {
          ROS_ERROR_STREAM(error.message());
          fsm.parent->process_event(Error());
        } else {
          fsm.parent->process_event(Success());
        }
      }

    private:
      bp::ptime start_;
      bp::time_duration cycle_;
    };

    // transitions

    typedef boost::mpl::vector< Ok, Opening > initial_state;

    // clang-format off
    typedef boost::mpl::vector<
        //        Start            Event     Next             Action      Guard
        //      +----------------+---------+----------------+-----------+-----------+
        bmf::Row< Ok             , Error   , Exit           , bmf::none , bmf::none >,
        //      +----------------+---------+----------------+-----------+-----------+
        bmf::Row< Opening        , Success , RequestingData , bmf::none , bmf::none >,
        bmf::Row< RequestingData , Success , ReceivingData  , bmf::none , bmf::none >,
        bmf::Row< ReceivingData  , Success , Sleeping       , bmf::none , bmf::none >,
        bmf::Row< Sleeping       , Success , RequestingData , bmf::none , bmf::none > >
        //      +----------------+---------+----------------+-----------+-----------+
        transition_table;
    // clang-format on

    // entry action of sub-machine

    template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) {
      parent = &fsm;
      ctx = parent->ctx;
    }

  private:
    bmb::state_machine< MachineDef > *parent;
    Context *ctx;
  };
  typedef bmb::state_machine< WorkModeDef > WorkMode;
  typedef WorkMode::exit_pt< WorkModeDef::Exit > WorkModeExit;

  struct IdleMode : bmf::state<> {
    template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) const {
      fsm.ctx->timer.expires_from_now(bp::seconds(1));
      fsm.ctx->timer.async_wait(
          boost::bind(&IdleMode::handle_wait< FSM >, this, _1, boost::ref(fsm)));
    }

    template < class FSM > void handle_wait(const bs::error_code &error, FSM &fsm) const {
      if (error == ba::error::operation_aborted) {
        return;
      }

      if (error) {
        ROS_ERROR_STREAM(error.message());
        fsm.process_event(Error());
      } else {
        fsm.process_event(Success());
      }
    }
  };

  struct TerminateMode : bmf::terminate_state<> {
    template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) const {
      // no event processed here because this is a terminate state
      try {
        if (fsm.ctx->serial.is_open()) {
          fsm.ctx->serial.close();
        }
      } catch (const bs::system_error &error) {
        ROS_ERROR_STREAM(error.what());
      }
    }
  };

  //
  // transitions
  //

  typedef WorkMode initial_state;

  // clang-format off
  typedef boost::mpl::vector<
      //        Start          Event     Next            Action      Guard
      //      +--------------+---------+---------------+-----------+-----------+
      bmf::Row< WorkModeExit , Error   , IdleMode      , bmf::none , bmf::none >,
      bmf::Row< IdleMode     , Success , WorkMode      , bmf::none , bmf::none >,
      bmf::Row< IdleMode     , Error   , TerminateMode , bmf::none , bmf::none > >
      //      +--------------+---------+---------------+-----------+-----------+
      transition_table;
  // clang-format on

  //
  // reference to context
  //

  MachineDef(Context *_ctx) : ctx(_ctx) {}

private:
  Context *ctx;
};
typedef bmb::state_machine< MachineDef > Machine;

//
//
//

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "omron_env_sensor_node");

  ba::io_service ios;
  ros::NodeHandle nh, pnh("~");
  Context ctx(pnh.param< std::string >("device", "/dev/ttyUSB0"), ios, nh, pnh);

  Machine m(&ctx);

  m.start();

  while (ros::ok()) {
    ios.run_one();
  }

  return 0;
}