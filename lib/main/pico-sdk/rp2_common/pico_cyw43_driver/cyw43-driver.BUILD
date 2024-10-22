load("@pico-sdk//bazel:defs.bzl", "compatible_with_pico_w")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cyw43_driver",
    srcs = [
        "src/cyw43_ctrl.c",
        "src/cyw43_ll.c",
        "src/cyw43_lwip.c",
        "src/cyw43_stats.c",
    ],
    hdrs = glob(["**/*.h"]),
    defines = select({
        "@pico-sdk//bazel/constraint:pico_btstack_config_unset": [
            "CYW43_ENABLE_BLUETOOTH=0",
        ],
        "//conditions:default": [
            "CYW43_ENABLE_BLUETOOTH=1",
        ],
    }),
    includes = [
        "firmware",
        "src",
    ],
    target_compatible_with = compatible_with_pico_w(),
    deps = [
        "@pico-sdk//src/rp2_common/pico_cyw43_driver:cyw43_configport",
        "@pico-sdk//src/rp2_common/pico_lwip",
    ],
)
