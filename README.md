# xsens_ll

[Xsens low-level communication](https://mtidocs.xsens.com/introduction-4) driver written in C++17
without utilizing the STL or dynamic memory allocation.  This makes it suitable for a variety of
applications including with low power embedded devices.

## Usage

### WORKSPACE

To incorporate `xsens_ll` into your project copy the following into your `WORKSPACE` file.

```Starlark
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "xsens_ll",
    # See release page for latest version url and sha.
)
```
