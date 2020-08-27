# Example project

[![Pipeline status](https://gitlab.laas.fr/gepetto/example-adder/badges/master/pipeline.svg)](https://gitlab.laas.fr/gepetto/example-adder/commits/master)
[![Coverage report](https://gitlab.laas.fr/gepetto/example-adder/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/gepetto/example-adder/master/coverage/)

This is an example project, to show how to use Gepetto's tools. It has been modified to work with torchscript.

The example-add app is based on [Pytorch-TorchScript](https://pytorch.org/tutorials/advanced/cpp_export.html)

First, get libtorch:
```bash
wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip
unzip libtorch-shared-with-deps-latest.zip
```
We can now run the following commands to build the application from within the example-app/ folder:
```bash
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=/path/to/libtorch ..
cmake --build
```
