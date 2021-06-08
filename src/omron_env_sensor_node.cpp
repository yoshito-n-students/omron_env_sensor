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
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/crc.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/endian/conversion.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/states.hpp>
#include <boost/ref.hpp>
#include <boost/system/error_code.hpp>

#if BOOST_VERSION >= 107000
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#else
#include <boost/asio/io_service.hpp>
#endif

namespace ba = boost::asio;
namespace be = boost::endian;
namespace bm = boost::msm;
namespace bmf = bm::front;
namespace bmb = bm::back;
namespace bp = boost::posix_time;
namespace bs = boost::system;

#if BOOST_VERSION >= 107000
namespace boost {
namespace asio {
using io_service = io_context;
}
} // namespace boost
#endif

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

    using CRC = boost::crc_optimal< 16, 0x8005, 0xFFFF, 0, true, true >;

    // sub-states

    struct Ok : bmf::state<> {};

    struct Exit : bmf::exit_pseudo_state< Error > {};

    struct Opening : bmf::state<> {
      template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) const {
        try {
          using Serial = ba::serial_port;

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
#if BOOST_VERSION >= 107000
          ba::post(fsm.ctx->serial.get_executor(),
                   boost::bind(&bmb::state_machine< MachineDef >::process_event< Success >,
                               fsm.parent, Success()));
#else
          fsm.ctx->serial.get_io_service().post(boost::bind(
              &bmb::state_machine< MachineDef >::process_event< Success >, fsm.parent, Success()));
#endif
        } catch (const bs::system_error &error) {
          ROS_ERROR_STREAM(error.what());
#if BOOST_VERSION >= 107000
          ba::post(fsm.ctx->serial.get_executor(),
                   boost::bind(&bmb::state_machine< MachineDef >::process_event< Error >,
                               fsm.parent, Error()));
#else
          fsm.ctx->serial.get_io_service().post(boost::bind(
              &bmb::state_machine< MachineDef >::process_event< Error >, fsm.parent, Error()));
#endif
        }
      }
    };

    struct RequestingData : bmf::state<> {
      RequestingData() {
        cmd_[0] = 0x52;                       // header 1
        cmd_[1] = 0x42;                       // header 2
        encode< uint16_t >(5, &cmd_[2]);      // length (2 bytes)
        cmd_[4] = 0x01;                       // command (0x01: read)
        encode< uint16_t >(0x5022, &cmd_[5]); // address (2 bytes, 0x5022: latest data short)

        CRC crc;
        crc.process_bytes(cmd_, 7);
        encode< uint16_t >(crc.checksum(), &cmd_[7]); // CRC (2 bytes)
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
      template < typename T > static void encode(const T &src, uint8_t *const dst) {
        *reinterpret_cast< T * >(dst) = be::native_to_little(src);
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
          return;
        }

        // check recieved data format
        if (res_[0] != 0x52 || res_[1] != 0x42) {
          ROS_ERROR("Invalid header");
          fsm.parent->process_event(Error());
          return;
        }
        if (decode< uint16_t >(&res_[2]) != 26) {
          ROS_ERROR("Invalid length");
          fsm.parent->process_event(Error());
          return;
        }
        if (!checkCRC()) {
          ROS_ERROR("CRC error");
          fsm.parent->process_event(Error());
          return;
        }

        // publish data if subscriber exists
        if (pub_.getNumSubscribers() > 0) {
          const omron_env_sensor_msgs::DataShortPtr msg(new omron_env_sensor_msgs::DataShort());
          msg->header.stamp = ros::Time::now();
          msg->temperature = decode< uint16_t >(&res_[8]) / 100.;
          msg->relative_humidity = decode< uint16_t >(&res_[10]) / 100.;
          msg->ambient_light = decode< uint16_t >(&res_[12]);
          msg->barometric_pressure = decode< uint32_t >(&res_[14]) / 1000.;
          msg->sound_noise = decode< uint16_t >(&res_[18]) / 100.;
          msg->etvoc = decode< uint16_t >(&res_[20]);
          msg->eco2 = decode< uint16_t >(&res_[22]);
          msg->discomfort_index = decode< uint16_t >(&res_[24]) / 100.;
          msg->heat_stroke = decode< uint16_t >(&res_[26]) / 100.;
          pub_.publish(msg);
        }
        fsm.parent->process_event(Success());
      }

    private:
      template < typename T > static T decode(const uint8_t *const src) {
        return be::little_to_native(*reinterpret_cast< const T * >(src));
      }

      bool checkCRC() {
        CRC crc;
        crc.process_bytes(res_, 28);
        return crc.checksum() == decode< uint16_t >(&res_[28]);
      }

    private:
      ros::Publisher pub_;
      uint8_t res_[30];
    };

    struct Sleeping : bmf::state<> {
      Sleeping() : end_(bp::min_date_time), cycle_(bp::neg_infin) {}

      template < class Event, class FSM > void on_entry(const Event &, FSM &fsm) {
        if (cycle_.is_negative()) {
          cycle_ = bp::milliseconds(fsm.ctx->pnh.param("cycle", 1000));
        }

        // update the end time of sleeping. sleep duration must be in [0, cycle_].
        const bp::ptime now(bp::microsec_clock::universal_time());
        end_ = boost::algorithm::clamp(end_ + cycle_, now, now + cycle_);

        // set the sleep timer
        fsm.ctx->timer.expires_at(end_);
        fsm.ctx->timer.async_wait(
            boost::bind(&Sleeping::handle_wait< FSM >, this, _1, boost::ref(fsm)));
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
      bp::ptime end_;
      bp::time_duration cycle_;
    };

    // transitions

    using initial_state = boost::mpl::vector< Ok, Opening >;

    // clang-format off
    using transition_table = boost::mpl::vector<
        //        Start            Event     Next             Action      Guard
        //      +----------------+---------+----------------+-----------+-----------+
        bmf::Row< Ok             , Error   , Exit           , bmf::none , bmf::none >,
        //      +----------------+---------+----------------+-----------+-----------+
        bmf::Row< Opening        , Success , RequestingData , bmf::none , bmf::none >,
        bmf::Row< RequestingData , Success , ReceivingData  , bmf::none , bmf::none >,
        bmf::Row< ReceivingData  , Success , Sleeping       , bmf::none , bmf::none >,
        bmf::Row< Sleeping       , Success , RequestingData , bmf::none , bmf::none > >;
        //      +----------------+---------+----------------+-----------+-----------+
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
  using WorkMode = bmb::state_machine< WorkModeDef >;
  using WorkModeExit = WorkMode::exit_pt< WorkModeDef::Exit >;

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

  using initial_state = WorkMode;

  // clang-format off
  using transition_table = boost::mpl::vector<
      //        Start          Event     Next            Action      Guard
      //      +--------------+---------+---------------+-----------+-----------+
      bmf::Row< WorkModeExit , Error   , IdleMode      , bmf::none , bmf::none >,
      bmf::Row< IdleMode     , Success , WorkMode      , bmf::none , bmf::none >,
      bmf::Row< IdleMode     , Error   , TerminateMode , bmf::none , bmf::none > >;
      //      +--------------+---------+---------------+-----------+-----------+
  // clang-format on

  //
  // reference to context
  //

  MachineDef(Context *_ctx) : ctx(_ctx) {}

private:
  Context *ctx;
};
using Machine = bmb::state_machine< MachineDef >;

//
//
//

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "omron_env_sensor_node");
  ros::NodeHandle nh, pnh("~");

  ba::io_service ios;
  Context ctx(pnh.param< std::string >("device", "/dev/ttyUSB0"), ios, nh, pnh);

  Machine m(&ctx);

  m.start();

  while (ros::ok()) {
    ios.run_one();
  }

  return 0;
}