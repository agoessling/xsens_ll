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

cc_test(
    name = "test_xbus_parser",
    srcs = ["test_xbus_parser.cc"],
    deps = [
        ":xbus_parser",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)
