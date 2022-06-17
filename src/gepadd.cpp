#include <iostream>
#include "example-adder/gepadd.hpp"

namespace gepetto {
namespace example {
long add(const long a, const long b) { return a + b; }

long sub(const long a, const long b) { return a - b; }

void show(const std::string & something) {
  std::cout << "Hi ! Somebody want me do display something for you:" <<std::endl;
  std::cout << "vvv" << std::endl;
  std::cout << something << std::endl;
  std::cout << "^^^" << std::endl;
}
}  // namespace example
}  // namespace gepetto
