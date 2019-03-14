load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ins_localization_component_lib",
    srcs = [
        "ins_localization.cc",
        "ins_localization_component.cc",
    ],
    hdrs = [
        "ins_localization.h",
        "ins_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_proto",
        "//modules/common/time",
        "//modules/common/util:message_util",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:ins_config_proto",
        "//modules/transform:transform_broadcaster_lib",
        "@eigen",
        "@gtest",
    ],
)

cc_binary(
    name = "libins_localization_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":ins_localization_component_lib"],
)

cc_test(
    name = "ins_localization_test",
    size = "small",
    srcs = [
        "ins_localization_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/ins550D:ins_localization_component_lib",
        "@gtest//:main",
    ],
)

cpplint()
