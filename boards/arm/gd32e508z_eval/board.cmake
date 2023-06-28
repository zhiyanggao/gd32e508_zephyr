# Copyright (c) 2022, Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(pyocd "--target=gd32e508ze" "--tool-opt=--pack=${ZEPHYR_HAL_GIGADEVICE_MODULE_DIR}/${CONFIG_SOC_SERIES}/support/GigaDevice.GD32E50x_DFP.1.3.3.pack")
board_runner_args(
  jlink
  "--device=GD32E508ZE" "--iface=jtag" "--tool-opt=-JTAGConf -1,-1" "--speed=4000"
)

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
