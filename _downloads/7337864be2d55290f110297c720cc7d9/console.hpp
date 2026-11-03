
#pragma once

#include <ostream>
#include <string>

/// \brief Output a message
#define dtmsg (::crobot::common::colorMsg("Msg", 32))

/// \brief Output a debug message
#define dtdbg (::crobot::common::colorMsg("Dbg", 36))

/// \brief Output a warning message
#define dtwarn (::crobot::common::colorErr("Warning", __FILE__, __LINE__, 33))

/// \brief Output an error message
#define dterr (::crobot::common::colorErr("Error", __FILE__, __LINE__, 31))

namespace crobot {
namespace common {

/// \brief
std::ostream& colorMsg(const std::string& _msg, int _color);

/// \brief
std::ostream& colorErr(
    const std::string& _msg,
    const std::string& _file,
    unsigned int _line,
    int _color);

} // namespace common

template <class T>
auto operator<<(std::ostream& os, const T& t) -> decltype(t.print(os), os) {
  t.print(os);
  return os;
}

} // namespace crobot
