#define BOOST_TEST_MODULE example test boost
#include <boost/test/included/unit_test.hpp>

#include "example-adder/gepadd.hpp"

BOOST_AUTO_TEST_CASE(test_boost_add) {
  BOOST_CHECK(gepetto::example::add(1, 2) == 3);
  BOOST_CHECK(gepetto::example::add(5, -1) == 4);
  BOOST_CHECK(gepetto::example::add(-3, 1) == -2);
}
