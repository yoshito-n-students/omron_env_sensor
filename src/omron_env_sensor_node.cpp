#include <iostream>
#include <string>

#include <omron_env_sensor_msgs/DataShort.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>

#include <boost/asio/buffer.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/crc.hpp>
//#include <boost/current_function.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
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
// events
//

struct Success {};
struct Error {};

//
// the machine
//

struct MachineDef;
typedef bmb::state_machine< MachineDef > Machine;

struct MachineDef : bmf::state_machine_def< MachineDef > {

  //
  // states and sub-machines
  //

  struct WorkingDef;
  typedef bmb::state_machine< WorkingDef > Working;

  struct WorkingDef : bmf::state_machine_def< WorkingDef > {

    // states

    struct Opening : bmf::state<> {
      template < class Event > void on_entry(const Event &, Working &fsm) const {
        // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;
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

          fsm.process_event(Success());
        } catch (const bs::system_error &error) {
          std::cerr << error.what() << std::endl;
          fsm.super->process_event(Error());
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

      template < class Event > void on_entry(const Event &, Working &fsm) const {
        // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;

        // set 1s timeout
        fsm.ctx->timer.expires_from_now(bp::seconds(1));
        fsm.ctx->timer.async_wait(
            boost::bind(&RequestingData::handle_wait, this, _1, boost::ref(fsm)));

        // start writing
        ba::async_write(fsm.ctx->serial, ba::buffer(cmd_),
                        boost::bind(&RequestingData::handle_write, this, _1, boost::ref(fsm)));
      }

      void handle_wait(const bs::error_code &error, Working &fsm) const {
        if (error == ba::error::operation_aborted) {
          return; // do nothing when the timer is canceled
        }
        fsm.ctx->serial.cancel();
      }

      void handle_write(const bs::error_code &error, Working &fsm) const {
        // unset timeout
        fsm.ctx->timer.cancel();

        // handle error
        if (error) {
          fsm.super->process_event(Error());
        } else {
          fsm.process_event(Success());
        }
      }

    private:
      uint8_t cmd_[9];
    };

    struct ReadingAndPublishingData : bmf::state<> {
      template < class Event > void on_entry(const Event &, Working &fsm) {
        // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;
        if (!pub_) {
          pub_ = fsm.ctx->nh.advertise< omron_env_sensor_msgs::DataShort >("data", true);
        }

        // set 1s timeout
        fsm.ctx->timer.expires_from_now(bp::seconds(1));
        fsm.ctx->timer.async_wait(
            boost::bind(&ReadingAndPublishingData::handle_wait, this, _1, boost::ref(fsm)));

        // start reading
        ba::async_read(
            fsm.ctx->serial, ba::buffer(res_),
            boost::bind(&ReadingAndPublishingData::handle_read, this, _1, boost::ref(fsm)));
      }

      void handle_wait(const bs::error_code &error, Working &fsm) const {
        if (error == ba::error::operation_aborted) {
          return; // do nothing when the timer is canceled
        }
        fsm.ctx->serial.cancel();
      }

      void handle_read(const bs::error_code &error, Working &fsm) {
        // unset timeout
        fsm.ctx->timer.cancel();

        // process read data
        if (error) {
          fsm.super->process_event(Error());
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
            msg->sound_noise = ((0x00ff & res_[12]) + ((0x00ff & res_[13]) << 8)) / 100.;
            msg->etvoc = (0x00ff & res_[20]) + ((0x00ff & res_[21]) << 8);
            msg->eco2 = (0x00ff & res_[22]) + ((0x00ff & res_[23]) << 8);
            msg->discomfort_index = ((0x00ff & res_[24]) + ((0x00ff & res_[25]) << 8)) / 100.;
            msg->heat_stroke = ((0x00ff & res_[26]) + ((0x00ff & res_[27]) << 8)) / 100.;
            pub_.publish(msg);
          }

          fsm.process_event(Success());
        }
      }

    private:
      ros::Publisher pub_;
      uint8_t res_[30];
    };

    struct Sleeping : bmf::state<> {
      Sleeping() : interval_(-1) {}

      template < class Event > void on_entry(const Event &, Working &fsm) {
        // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;
        if (interval_ < 0) {
          interval_ = fsm.ctx->pnh.param("interval", 1000);
        }
        fsm.ctx->timer.expires_from_now(bp::milliseconds(interval_));
        fsm.ctx->timer.async_wait(boost::bind(&Sleeping::handle_wait, this, _1, boost::ref(fsm)));
      }

      void handle_wait(const bs::error_code &error, Working &fsm) const {
        if (error) {
          fsm.super->process_event(Error());
        } else {
          fsm.process_event(Success());
        }
      }

    private:
      int interval_;
    };

    // transitions

    typedef Opening initial_state;

    typedef boost::mpl::vector<
        //        Start    Event   Next     Action     Guard
        bmf::Row< Opening, Success, RequestingData, bmf::none, bmf::none >,
        bmf::Row< RequestingData, Success, ReadingAndPublishingData, bmf::none, bmf::none >,
        bmf::Row< ReadingAndPublishingData, Success, Sleeping, bmf::none, bmf::none >,
        bmf::Row< Sleeping, Success, RequestingData, bmf::none, bmf::none > >
        transition_table;

    // entry action of sub-machine

    template < class Event > void on_entry(const Event &, Machine &fsm) {
      // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;
      super = &fsm;
      ctx = super->ctx;
    }

    // reference to context

    Machine *super;
    Context *ctx;
  };

  struct Idling : bmf::state<> {
    template < class Event > void on_entry(const Event &, Machine &fsm) const {
      // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;
      fsm.ctx->timer.expires_from_now(bp::seconds(1));
      fsm.ctx->timer.async_wait(boost::bind(&Idling::handle_wait, this, _1, boost::ref(fsm)));
    }

    void handle_wait(const bs::error_code &error, Machine &fsm) const {
      if (error) {
        fsm.process_event(Error());
      } else {
        fsm.process_event(Success());
      }
    }
  };

  struct Closing : bmf::terminate_state<> {
    template < class Event > void on_entry(const Event &, Machine &fsm) const {
      // std::cout << "\n" << BOOST_CURRENT_FUNCTION << std::endl;
      // no event processed here because this is a terminate state
      try {
        if (fsm.ctx->serial.is_open()) {
          fsm.ctx->serial.close();
        }
      } catch (const bs::system_error &error) {
        std::cerr << error.what() << std::endl;
      }
    }
  };

  //
  // transitions
  //

  typedef Working initial_state;

  typedef boost::mpl::vector<
      //        Start    Event   Next     Action     Guard
      bmf::Row< Working, Error, Idling, bmf::none, bmf::none >,
      bmf::Row< Idling, Success, Working, bmf::none, bmf::none >,
      bmf::Row< Idling, Error, Closing, bmf::none, bmf::none > >
      transition_table;

  //
  // reference to context
  //

  MachineDef(Context *_ctx) : ctx(_ctx) {}

  Context *ctx;
};

//
//
//

int main(int argc, char *argv[]) {
  // TODO: handle SIGINT to stop io_service
  ros::init(argc, argv, "omron_env_sensor_node");

  ba::io_service ios;
  ros::NodeHandle nh, pnh("~");
  Context ctx("/dev/ttyUSB0", ios, nh, pnh);

  Machine m(&ctx);

  m.start();

  ios.run();

  return 0;
}