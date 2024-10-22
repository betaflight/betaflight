load("@pico-sdk//bazel:defs.bzl", "incompatible_with_config")

package(default_visibility = ["//visibility:public"])

# Some of the LWIP sys_arch.h and the lwip headers depend circularly on one
# another. Include them all in the same target.
cc_library(
    name = "pico_lwip_headers",
    hdrs = glob(["**/*.h"]),
    includes = [
        "contrib/ports/freertos/include/arch",
        "src/include",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "@pico-sdk//bazel/config:PICO_LWIP_CONFIG",
        "@pico-sdk//src/rp2_common/pico_lwip:pico_lwip_config",
    ],
)

cc_library(
    name = "pico_lwip_core",
    srcs = glob(["src/core/*.c"]),
    target_compatible_with = incompatible_with_config(
        "@pico-sdk//bazel/constraint:pico_lwip_config_unset",
    ),
    deps = [
        ":pico_lwip_headers",
    ] + select({
        "@pico-sdk//bazel/constraint:pico_freertos_unset": [],
        "//conditions:default": [
            ":pico_lwip_contrib_freertos",
        ],
    }),
)

cc_library(
    name = "pico_lwip_core4",
    srcs = glob(["src/core/ipv4/*.c"]),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_core6",
    srcs = glob(["src/core/ipv6/*.c"]),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_api",
    srcs = glob(["src/api/*.c"]),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_netif",
    srcs = [
        "src/netif/bridgeif.c",
        "src/netif/bridgeif_fdb.c",
        "src/netif/ethernet.c",
        "src/netif/slipif.c",
    ],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_sixlowpan",
    srcs = [
        "src/netif/lowpan6.c",
        "src/netif/lowpan6_ble.c",
        "src/netif/lowpan6_common.c",
        "src/netif/zepif.c",
    ],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_ppp",
    srcs = glob(["src/netif/ppp/*/*.c"]),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_snmp",
    srcs = glob(
        ["src/apps/snmp/*.c"],
        # mbedtls is provided through pico_lwip_mbedtls.
        exclude = ["*mbedtls.c"],
    ),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_http",
    srcs = glob(["src/apps/http/*.c"]),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_makefsdata",
    srcs = ["src/apps/http/makefsdata/makefsdata.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_iperf",
    srcs = ["src/apps/lwiperf/lwiperf.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_smtp",
    srcs = ["src/apps/smtp/smtp.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_sntp",
    srcs = ["src/apps/sntp/sntp.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_mdns",
    srcs = glob(["src/apps/mdns/*.c"]),
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_netbios",
    srcs = ["src/apps/netbiosns/netbiosns.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_tftp",
    srcs = ["src/apps/tftp/tftp.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_mbedtls",
    srcs = [
        "src/apps/altcp_tls/altcp_tls_mbedtls.c",
        "src/apps/altcp_tls/altcp_tls_mbedtls_mem.c",
        "src/apps/snmp/snmpv3_mbedtls.c",
    ],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip_mqttt",
    srcs = ["src/apps/mqtt/mqtt.c"],
    deps = [":pico_lwip_core"],
)

cc_library(
    name = "pico_lwip",
    deps = [
        ":pico_lwip_api",
        ":pico_lwip_core",
        ":pico_lwip_core4",
        ":pico_lwip_core6",
        ":pico_lwip_netif",
        ":pico_lwip_ppp",
        ":pico_lwip_sixlowpan",
    ],
)

cc_library(
    name = "pico_lwip_contrib_freertos",
    srcs = ["contrib/ports/freertos/sys_arch.c"],
    includes = ["contrib/ports/freertos/include"],
    target_compatible_with = incompatible_with_config(
        "@pico-sdk//bazel/constraint:pico_freertos_unset",
    ),
    deps = [
        ":pico_lwip_headers",
        "@pico-sdk//bazel/config:PICO_FREERTOS_LIB",
    ],
)
