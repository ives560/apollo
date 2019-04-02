load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_yema_protocol",
    srcs = [
        "brake_60.cc",
        "gear_66.cc",
        "steering_64.cc",
        "throttle_62.cc",
        "turnsignal_68.cc",
    ],
    hdrs = [
        "brake_60.h",
        "gear_66.h",
        "steering_64.h",
        "throttle_62.h",
        "turnsignal_68.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_60_test",
    size = "small",
    srcs = [
        "brake_60_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/yema/protocol:canbus_yema_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "steering_64_test",
    size = "small",
    srcs = [
        "steering_64_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/yema/protocol:canbus_yema_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "throttle_62_test",
    size = "small",
    srcs = [
        "throttle_62_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/yema/protocol:canbus_yema_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "turnsignal_68_test",
    size = "small",
    srcs = [
        "turnsignal_68_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/yema/protocol:canbus_yema_protocol",
        "@gtest//:main",
    ],
)

cpplint()
