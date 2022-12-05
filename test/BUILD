cc_test(
    name = "test_xbus_parser",
    srcs = ["test_xbus_parser.cc"],
    deps = [
        "//src:xbus_parser",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "test_data_packet",
    srcs = ["test_data_packet.cc"],
    deps = [
        "//src:bitfield",
        "//src:data_packet",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "test_xsens_manager",
    srcs = ["test_xsens_manager.cc"],
    deps = [
        "//src:xsens_manager",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)

cc_binary(
    name = "test_linux_xsens_manager",
    srcs = ["test_linux_xsens_manager.cc"],
    deps = [
        "//src:data_packet",
        "//src:linux_xsens_manager",
        "@argparse",
    ],
)
