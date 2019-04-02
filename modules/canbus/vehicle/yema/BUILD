load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "yema_vehicle_factory",
    srcs = [
        "yema_vehicle_factory.cc",
    ],
    hdrs = [
        "yema_vehicle_factory.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":yema_controller",
        ":yema_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "yema_message_manager",
    srcs = [
        "yema_message_manager.cc",
    ],
    hdrs = [
        "yema_message_manager.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/yema/protocol:canbus_yema_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "yema_controller",
    srcs = [
        "yema_controller.cc",
    ],
    hdrs = [
        "yema_controller.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":yema_message_manager",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/yema/protocol:canbus_yema_protocol",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest",
    ],
)

cc_test(
    name = "yema_controller_test",
    size = "small",
    srcs = [
        "yema_controller_test.cc",
    ],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        "//modules/canbus/vehicle/yema:yema_controller",
        "@gtest//:main",
    ],
)

cc_test(
    name = "yema_message_manager_test",
    size = "small",
    srcs = [
        "yema_message_manager_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/yema:yema_message_manager",
        "@gtest//:main",
    ],
)

cc_test(
    name = "yema_vehicle_factory_test",
    size = "small",
    srcs = ["yema_vehicle_factory_test.cc"],
    deps = [
        ":yema_vehicle_factory",
        "@gtest//:main",
    ],
)

cpplint()
