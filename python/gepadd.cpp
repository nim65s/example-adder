#include "example-adder/python.hpp"

namespace gepetto {
namespace example {
void exposeExampleAdder() {
  boost::python::def("add", add);
  boost::python::def("sub", sub);
  boost::python::def("show", show);
}
}  // namespace example
}  // namespace gepetto
