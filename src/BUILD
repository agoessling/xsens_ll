cc_library(
    name = "msg_id",
    hdrs = ["msg_id.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "bitfield",
    hdrs = ["bitfield.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "xsens_types",
    hdrs = ["xsens_types.h"],
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
    name = "data_packet",
    hdrs = ["data_packet.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":bitfield",
        ":xbus_parser",
    ],
)

cc_library(
    name = "xsens_manager",
    hdrs = ["xsens_manager.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":bitfield",
        ":msg_id",
        ":xbus_parser",
        ":xsens_types",
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
