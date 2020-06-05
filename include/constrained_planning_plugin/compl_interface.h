#ifndef COMPL_INTERFACE_H
#define COMPL_INTERFACE_H

#include <moveit/macros/class_forward.h>

namespace compl_interface
{
MOVEIT_CLASS_FORWARD(COMPLInterface);

class COMPLInterface
{
public:
  COMPLInterface();

  bool solve();
};
}  // namespace compl_interface

#endif  // COMPL_INTERFACE_H