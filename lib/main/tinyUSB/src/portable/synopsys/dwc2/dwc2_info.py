#!/usr/bin/env python3

import ctypes
import argparse
import click
import pandas as pd

# hex value for register: guid, gsnpsid, ghwcfg1, ghwcfg2, ghwcfg3, ghwcfg4
# Note: FS is FullSpeed, HS is HighSpeed
dwc2_reg_list = ['GUID', 'GSNPSID', 'GHWCFG1', 'GHWCFG2', 'GHWCFG3', 'GHWCFG4']
dwc2_reg_value = {
    'BCM2711 (Pi4)': [0x2708A000, 0x4F54280A, 0, 0x228DDD50, 0xFF000E8, 0x1FF00020],
    'EFM32GG': [0, 0x4F54330A, 0, 0x228F5910, 0x01F204E8, 0x1BF08030],
    'ESP32-S2/S3': [0, 0x4F54400A, 0, 0x224DD930, 0x0C804B5, 0xD3F0A030],
    'ESP32-P4': [0, 0x4F54400A, 0, 0x215FFFD0, 0x03805EB5, 0xDFF1A030],
    'ST F207/F407/411/429 FS': [0x1200, 0x4F54281A, 0, 0x229DCD20, 0x020001E8, 0x0FF08030],
    'ST F407/429 HS': [0x1100, 0x4F54281A, 0, 0x229ED590, 0x03F403E8, 0x17F00030],
    'ST F412/76x FS': [0x2000, 0x4F54320A, 0, 0x229ED520, 0x0200D1E8, 0x17F08030],
    'ST F723/L4P5 FS': [0x3000, 0x4F54330A, 0, 0x229ED520, 0x0200D1E8, 0x17F08030],
    'ST F723 HS': [0x3100, 0x4F54330A, 0, 0x229FE1D0, 0x03EED2E8, 0x23F00030],
    'ST F76x HS': [0x2100, 0x4F54320A, 0, 0x229FE190, 0x03EED2E8, 0x23F00030],
    'ST H743/H750': [0x2300, 0x4F54330A, 0, 0x229FE190, 0x03B8D2E8, 0xE3F00030],
    'ST L476 FS': [0x2000, 0x4F54310A, 0, 0x229ED520, 0x0200D1E8, 0x17F08030],
    'ST U5A5 HS': [0x5000, 0x4F54411A, 0, 0x228FE052, 0x03B882E8, 0xE2103E30],
    'XMC4500': [0xAEC000, 0x4F54292A, 0, 0x228F5930, 0x027A01E5, 0xDBF08030],
    'GD32VF103': [0x1000, 0, 0, 0, 0, 0],
}

# Combine dwc2_info with dwc2_reg_list
# dwc2_info = {
#     'BCM2711 (Pi4)': {
#         'GUID': 0x2708A000,
#         'GSNPSID': 0x4F54280A,
#         'GHWCFG1': 0,
#         'GHWCFG2': 0x228DDD50,
#         'GHWCFG3': 0xFF000E8,
#         'GHWCFG4': 0x1FF00020
#     },
dwc2_info = {key: {field: value for field, value in zip(dwc2_reg_list, values)} for key, values in dwc2_reg_value.items()}


class GHWCFG2(ctypes.LittleEndianStructure):
    _fields_ = [
        ("op_mode", ctypes.c_uint32, 3),
        ("arch", ctypes.c_uint32, 2),
        ("single_point", ctypes.c_uint32, 1),
        ("hs_phy_type", ctypes.c_uint32, 2),
        ("fs_phy_type", ctypes.c_uint32, 2),
        ("num_dev_ep", ctypes.c_uint32, 4),
        ("num_host_ch", ctypes.c_uint32, 4),
        ("period_channel_support", ctypes.c_uint32, 1),
        ("enable_dynamic_fifo", ctypes.c_uint32, 1),
        ("mul_proc_intrpt", ctypes.c_uint32, 1),
        ("reserved21", ctypes.c_uint32, 1),
        ("nptx_q_depth", ctypes.c_uint32, 2),
        ("ptx_q_depth", ctypes.c_uint32, 2),
        ("token_q_depth", ctypes.c_uint32, 5),
        ("otg_enable_ic_usb", ctypes.c_uint32, 1)
    ]


class GHWCFG3(ctypes.LittleEndianStructure):
    _fields_ = [
        ("xfer_size_width", ctypes.c_uint32, 4),
        ("packet_size_width", ctypes.c_uint32, 3),
        ("otg_enable", ctypes.c_uint32, 1),
        ("i2c_enable", ctypes.c_uint32, 1),
        ("vendor_ctrl_itf", ctypes.c_uint32, 1),
        ("optional_feature_removed", ctypes.c_uint32, 1),
        ("synch_reset", ctypes.c_uint32, 1),
        ("otg_adp_support", ctypes.c_uint32, 1),
        ("otg_enable_hsic", ctypes.c_uint32, 1),
        ("battery_charger_support", ctypes.c_uint32, 1),
        ("lpm_mode", ctypes.c_uint32, 1),
        ("dfifo_depth", ctypes.c_uint32, 16)
    ]


class GHWCFG4(ctypes.LittleEndianStructure):
    _fields_ = [
        ("num_dev_period_in_ep", ctypes.c_uint32, 4),
        ("partial_powerdown", ctypes.c_uint32, 1),
        ("ahb_freq_min", ctypes.c_uint32, 1),
        ("hibernation", ctypes.c_uint32, 1),
        ("extended_hibernation", ctypes.c_uint32, 1),
        ("reserved8", ctypes.c_uint32, 1),
        ("enhanced_lpm_support1", ctypes.c_uint32, 1),
        ("service_interval_flow", ctypes.c_uint32, 1),
        ("ipg_isoc_support", ctypes.c_uint32, 1),
        ("acg_support", ctypes.c_uint32, 1),
        ("enhanced_lpm_support", ctypes.c_uint32, 1),
        ("phy_data_width", ctypes.c_uint32, 2),
        ("ctrl_ep_num", ctypes.c_uint32, 4),
        ("iddg_filter", ctypes.c_uint32, 1),
        ("vbus_valid_filter", ctypes.c_uint32, 1),
        ("a_valid_filter", ctypes.c_uint32, 1),
        ("b_valid_filter", ctypes.c_uint32, 1),
        ("session_end_filter", ctypes.c_uint32, 1),
        ("dedicated_fifos", ctypes.c_uint32, 1),
        ("num_dev_in_eps", ctypes.c_uint32, 4),
        ("dma_desc_enable", ctypes.c_uint32, 1),
        ("dma_desc_dynamic", ctypes.c_uint32, 1)
    ]

# mapping for specific fields in GHWCFG2
GHWCFG2_field = {
    'op_mode': {
        0: "HNP SRP",
        1: "SRP",
        2: "noHNP noSRP",
        3: "SRP Device",
        4: "noOTG Device",
        5: "SRP Host",
        6: "noOTG Host"
    },
    'arch': {
        0: "Slave only",
        1: "DMA external",
        2: "DMA internal"
    },
    'single_point': {
        0: "hub",
        1: "n/a"
    },
    'hs_phy_type': {
        0: "n/a",
        1: "UTMI+",
        2: "ULPI",
        3: "UTMI+/ULPI"
    },
    'fs_phy_type': {
        0: "n/a",
        1: "Dedicated",
        2: "Shared UTMI+",
        3: "Shared ULPI"
    },
    'nptx_q_depth': {
        0: "2",
        1: "4",
        2: "8",
    },
    'ptx_q_depth': {
        0: "2",
        1: "4",
        2: "8",
        3: "16"
    },
}

# mapping for specific fields in GHWCFG4
GHWCFG4_field = {
    'phy_data_width': {
        0: "8 bit",
        1: "16 bit",
        2: "8/16 bit",
        3: "Reserved"
    },
    }

def main():
    """Render dwc2_info to Markdown table"""

    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    # Create an empty list to hold the dictionaries
    md_table = []

    # Iterate over the dwc2_info dictionary and extract fields
    for device, reg_values in dwc2_info.items():
        md_item = {"Device": device}
        for r_name, r_value in reg_values.items():
            md_item[r_name] = f"0x{r_value:08X}"

            if r_name == 'GSNPSID':
                # Get dwc2 specs version
                major = ((r_value >> 8) >> 4) & 0x0F
                minor = (r_value >> 4) & 0xFF
                patch = chr((r_value & 0x0F) + ord('a') - 0xA)
                md_item[f' - specs version'] = f"{major:X}.{minor:02X}{patch}"
            elif r_name in globals():
                # Get bit-field values which exist as ctypes structures
                class_hdl = globals()[r_name]
                ghwcfg = class_hdl.from_buffer_copy(r_value.to_bytes(4, byteorder='little'))
                for field_name, field_type, _ in class_hdl._fields_:
                    field_value = getattr(ghwcfg, field_name)
                    if class_hdl == GHWCFG2 and field_name in GHWCFG2_field:
                        field_value = GHWCFG2_field[field_name].get(field_value, f"Unknown ({field_value})")
                    if class_hdl == GHWCFG4 and field_name in GHWCFG4_field:
                        field_value = GHWCFG4_field[field_name].get(field_value, f"Unknown ({field_value})")

                    md_item[f' - {field_name}'] = field_value

        md_table.append(md_item)

    # Create a Pandas DataFrame from the list of dictionaries
    df = pd.DataFrame(md_table).set_index('Device')

    # Transpose the DataFrame to switch rows and columns
    df = df.T
    #print(df)

    # Write the Markdown table to a file
    with open('dwc2_info.md', 'w') as md_file:
        md_file.write(df.to_markdown())
        md_file.write('\n')


if __name__ == '__main__':
    main()
