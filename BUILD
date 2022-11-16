load("@bazel_lint//bazel:buildifier.bzl", "buildifier")
load("@bazel_lint//cpp:clang.bzl", "clang_format")

buildifier(
    name = "format_bazel",
    srcs = ["WORKSPACE"],
    glob = [
        "**/*BUILD",
        "**/*.bzl",
    ],
    glob_exclude = [
        "bazel-*/**",
    ],
)

clang_format(
    name = "format_cc",
    glob = [
        "**/*.c",
        "**/*.cc",
        "**/*.h",
    ],
    glob_exclude = [
        "bazel-*/**",
    ],
    style_file = ".clang-format",
)

cc_library(
    name = "msg_id",
    hdrs = ["msg_id.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "xbus_parser",
    hdrs = ["xbus_parser.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":msg_id",
    ],
)

cc_library(
    name = "xsens_manager",
    hdrs = ["xsens_manager.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":xbus_parser",
    ],
)

cc_library(
    name = "linux_xsens_manager",
    hdrs = ["linux_xsens_manager.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":xsens_manager",
    ],
)

cc_test(
    name = "test_xbus_parser",
    srcs = ["test_xbus_parser.cc"],
    deps = [
        ":xbus_parser",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "test_xsens_manager",
    srcs = ["test_xsens_manager.cc"],
    deps = [
        ":xsens_manager",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)

cc_binary(
    name = "test_linux_xsens_manager",
    srcs = ["test_linux_xsens_manager.cc"],
    deps = [
        ":linux_xsens_manager",
        "@argparse",
    ],
)
