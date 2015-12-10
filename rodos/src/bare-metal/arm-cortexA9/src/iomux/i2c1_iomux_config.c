/*
 * Copyright (C) 2012, [Your Company Here] All Rights Reserved.
 * IT IS EXPECTED THAT THIS TEXT BE REPLACED WITH THE BOARD SOFTWARE
 * PROVIDER'S COPYRIGHT INFORMATION. THIS TEXT WILL BE DISPLAYED AT 
 * THE TOP OF ALL SOURCE FILES GENERATED FOR THIS BOARD DESIGN.
*/

// File: i2c1_iomux_config.c

/* ------------------------------------------------------------------------------
 * <auto-generated>
 *     This code was generated by a tool.
 *     Runtime Version:3.4.0.0
 *
 *     Changes to this file may cause incorrect behavior and will be lost if
 *     the code is regenerated.
 * </auto-generated>
 * ------------------------------------------------------------------------------
*/

#include "iomux_config.h"
#include "iomux_define.h"
#include "registers/regsiomuxc.h"

// Function to configure IOMUXC for i2c1 module.
void i2c1_iomux_config(void)
{
    // Config i2c1.I2C1_SCL to pad EIM_DATA21(H20)
    // HW_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21_WR(0x00000016);
    // HW_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_WR(0x0001F8B0);
    // HW_IOMUXC_I2C1_SCL_IN_SELECT_INPUT_WR(0x00000000);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21(0x020E00A4)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: eim signal: EIM_DATA21
    //     ALT1 (1) - Select instance: ecspi4 signal: ECSPI4_SCLK
    //     ALT2 (2) - Select instance: ipu1 signal: IPU1_DI0_PIN17
    //     ALT3 (3) - Select instance: ipu2 signal: IPU2_CSI1_DATA11
    //     ALT4 (4) - Select instance: usb signal: USB_OTG_OC
    //     ALT5 (5) - Select instance: gpio3 signal: GPIO3_IO21
    //     ALT6 (6) - Select instance: i2c1 signal: I2C1_SCL
    //     ALT7 (7) - Select instance: spdif signal: SPDIF_IN
    HW_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21_MUX_MODE_V(ALT6));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21(0x020E03B8)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_PUS_V(22K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_ODE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21_SRE_V(SLOW));
    // Pad EIM_DATA21 is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_I2C1_SCL_IN_SELECT_INPUT(0x020E0898)
    //   DAISY [0] - MUX Mode Select Field Reset: EIM_DATA21_ALT6
    //               Selecting Pads Involved in Daisy Chain.
    //     EIM_DATA21_ALT6 (0) - Select signal i2c1 I2C1_SCL as input from pad EIM_DATA21(ALT6).
    //     CSI0_DATA09_ALT4 (1) - Select signal i2c1 I2C1_SCL as input from pad CSI0_DATA09(ALT4).
    HW_IOMUXC_I2C1_SCL_IN_SELECT_INPUT_WR(
            BF_IOMUXC_I2C1_SCL_IN_SELECT_INPUT_DAISY_V(EIM_DATA21_ALT6));

    // Config i2c1.I2C1_SDA to pad EIM_DATA28(G23)
    // HW_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28_WR(0x00000011);
    // HW_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_WR(0x0001F8B0);
    // HW_IOMUXC_I2C1_SDA_IN_SELECT_INPUT_WR(0x00000000);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28(0x020E00C4)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: eim signal: EIM_DATA28
    //     ALT1 (1) - Select instance: i2c1 signal: I2C1_SDA
    //     ALT2 (2) - Select instance: ecspi4 signal: ECSPI4_MOSI
    //     ALT3 (3) - Select instance: ipu2 signal: IPU2_CSI1_DATA12
    //     ALT4 (4) - Select instance: uart2 signal: UART2_CTS_B
    //     ALT5 (5) - Select instance: gpio3 signal: GPIO3_IO28
    //     ALT6 (6) - Select instance: ipu1 signal: IPU1_EXT_TRIG
    //     ALT7 (7) - Select instance: ipu1 signal: IPU1_DI0_PIN13
    HW_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28_MUX_MODE_V(ALT1));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28(0x020E03D8)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_PUS_V(22K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_ODE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28_SRE_V(SLOW));
    // Pad EIM_DATA28 is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_I2C1_SDA_IN_SELECT_INPUT(0x020E089C)
    //   DAISY [0] - MUX Mode Select Field Reset: EIM_DATA28_ALT1
    //               Selecting Pads Involved in Daisy Chain.
    //     EIM_DATA28_ALT1 (0) - Select signal i2c1 I2C1_SDA as input from pad EIM_DATA28(ALT1).
    //     CSI0_DATA08_ALT4 (1) - Select signal i2c1 I2C1_SDA as input from pad CSI0_DATA08(ALT4).
    HW_IOMUXC_I2C1_SDA_IN_SELECT_INPUT_WR(
            BF_IOMUXC_I2C1_SDA_IN_SELECT_INPUT_DAISY_V(EIM_DATA28_ALT1));
}
