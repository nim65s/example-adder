import unittest

import example_adder as exa


class TestAdder(unittest.TestCase):
    def test_adder_integers(self):
        self.assertEqual(exa.add(4, 3), 7)
        self.assertEqual(exa.sub(4, 3), 1)

        exa.show("WAZAAAAA")

        # This will make the test fail, allowing you to see the output of the previous
        # function call
        self.assertTrue(False)


if __name__ == "__main__":
    unittest.main()
