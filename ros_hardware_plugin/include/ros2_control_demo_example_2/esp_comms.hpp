#ifndef DIFFDRIVE_ESP32_COMMS_HPP
#define DIFFDRIVE_ESP32_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <chrono>
#include <thread>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
    return LibSerial::BaudRate::BAUD_115200;
  }
}

class EspComms
{
public:
  EspComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg_to_send);
    std::string response;

    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // non-fatal
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  // === HIGH-LEVEL COMMANDS for your new firmware ===

  void enterROSMode()
  {
    send_msg("13\n", true);
  }

  void setMotorSpeeds(float motor1, float motor2)
  {
    std::stringstream ss;
    ss << "CMD_VEL " << motor1 << " " << motor2 << "\n";
    send_msg(ss.str(), true);
  }

  void stopMotors()
  {
    send_msg("STOP\n", true);
  }

  void startStream()
  {
    send_msg("START_STREAM\n", true);
  }

  void stopStream()
  {
    send_msg("STOP_STREAM\n", true);
  }

  void exitROS()
  {
    send_msg("EXIT_ROS\n", true);
  }

  std::string getStatus()
  {
    return send_msg("GET_STATUS\n", true);
  }

  std::string readLine(char terminator = '\n', int timeout_ms = 100)
  {
    std::string line;
    auto start_time = std::chrono::steady_clock::now();

    while (true)
    {
      char c;
      try
      {
        serial_conn_.ReadByte(c, 10);
        line += c;
        if (c == terminator)
          break;
      }
      catch (const LibSerial::ReadTimeout &)
      {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start_time)
                           .count();
        if (elapsed >= timeout_ms)
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
      line.pop_back();

    return line;
  }

  void readEncoderValues(int &left_ticks, int &right_ticks)
  {
    std::string line = readLine('\n', timeout_ms_);
    std::stringstream ss(line);
    std::vector<int> values;
    int val;

    // Extract all integers from the line
    while (ss >> val)
    {
      values.push_back(val);
    }

    if (values.size() >= 2)
    {
      // Assign the last two integers to the referenced variables
      left_ticks = values[values.size() - 2];
      right_ticks = values[values.size() - 1];
    }
    else
    {
      // If parsing fails, set both to zero
      left_ticks = 0;
      right_ticks = 0;
    }
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // DIFFDRIVE_ESP32_COMMS_HPP