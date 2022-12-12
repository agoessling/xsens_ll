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
    name = "xsens_ll",
    includes = ["src"],
    visibility = ["//visibility:public"],
    deps = [
        "//src:data_packet",
        "//src:msg_id",
        "//src:xbus_parser",
        "//src:xsens_manager",
        "//src:xsens_types",
    ],
)

cc_library(
    name = "xsens_ll_linux",
    includes = ["src"],
    visibility = ["//visibility:public"],
    deps = [
        ":xsens_ll",
        "//src:linux_xsens_manager",
    ],
)
