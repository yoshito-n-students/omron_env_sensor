#include <iostream>
#include <vector>

#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/crc.hpp>

int main(int argc, char *argv[]) {
  namespace ba = boost::asio;
  typedef ba::serial_port Serial;
  typedef boost::crc_optimal< 16, 0x8005, 0xFFFF, 0, true, true > CRC;

  ba::io_service service;
  Serial serial(service);

  serial.open("/dev/ttyUSB0");
  serial.set_option(Serial::baud_rate(115200));
  serial.set_option(Serial::character_size(8));
  serial.set_option(Serial::stop_bits(Serial::stop_bits::one));
  serial.set_option(Serial::parity(Serial::parity::none));
  serial.set_option(Serial::flow_control(Serial::flow_control::none));

  std::vector< uint8_t > read_cmd;
  read_cmd.push_back(0x52);                   // header 1
  read_cmd.push_back(0x42);                   // header 2
  read_cmd.push_back(5 & 0x00ff);             // length L
  read_cmd.push_back((5 & 0xff00) >> 8);      // length H
  read_cmd.push_back(0x01);                   // 1:read, 2:write
  read_cmd.push_back(0x5022 & 0x00ff);        // address L
  read_cmd.push_back((0x5022 & 0xff00) >> 8); // address H
                                              // data if any

  CRC crc;
  crc.process_bytes(&read_cmd[0], read_cmd.size());
  read_cmd.push_back(crc.checksum() & 0x00ff);        // CRC L
  read_cmd.push_back((crc.checksum() & 0xff00) >> 8); // CRC H

  const std::size_t size_cmd(ba::write(serial, ba::buffer(read_cmd)));
  std::cout << std::dec << size_cmd << " bytes wrote" << std::endl;

  // according to the manual, the maximum response time is 1s
  sleep(1);

  //
  std::vector< uint8_t > read_res(30);
  const std::size_t size_res(ba::read(serial, ba::buffer(read_res)));

  std::cout << size_res << " bytes read" << std::endl;
  for (int i = 0; i < size_res; ++i) {
    std::cout << "0x" << std::hex << static_cast< int >(read_res[i]) << std::endl;
  }

  std::cout << "temp: " << std::dec
            << ((0x00ff & read_res[8]) + ((0x00ff & read_res[9]) << 8)) / 100. << " C" << std::endl;
  std::cout << "hum: " << std::dec
            << ((0x00ff & read_res[10]) + ((0x00ff & read_res[11]) << 8)) / 100. << " %"
            << std::endl;
  std::cout << "co2: " << std::dec
            << ((0x00ff & read_res[22]) + ((0x00ff & read_res[23]) << 8)) / 100. << " ppm"
            << std::endl;

  return 0;
}