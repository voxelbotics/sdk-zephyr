# Copyright (c) 2023 Trackunit Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(gnss_publish.c)
zephyr_library_sources_ifdef(CONFIG_GNSS_DUMP gnss_dump.c)
zephyr_library_sources_ifdef(CONFIG_GNSS_PARSE gnss_parse.c)
zephyr_library_sources_ifdef(CONFIG_GNSS_NMEA0183 gnss_nmea0183.c)
zephyr_library_sources_ifdef(CONFIG_GNSS_NMEA0183_MATCH gnss_nmea0183_match.c)
zephyr_library_sources_ifdef(CONFIG_GNSS_QUECTEL_LCX6G gnss_quectel_lcx6g.c)
