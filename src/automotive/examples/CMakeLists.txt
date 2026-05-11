# ============================================================================
# Research Project: Data communication in the environment of
# intelligent cars
# Author: Kirill Komarov
# Date: 2026
# 
# Description:
# This file contains source code developed (or modified) as part of the 
# research for the paper: "Data communication in the environment of
# intelligent cars".
# 
# DISCLAIMER & ACKNOWLEDGEMENT:
# Please note that this file contains or may contain code fragments, 
# algorithms, or architectural solutions that were previously implemented 
# in the "VaN3Twin" project https://github.com/DriveX-devs/VaN3Twin.git.
# 
# The borrowed code has been adapted and is used strictly for academic 
# and research purposes. All rights to the original code segments belong 
# to their respective original authors.
# ============================================================================
build_lib_example(
    NAME v2i-areaSpeedAdvisor-lte 
    SOURCE_FILES v2i-areaSpeedAdvisor-lte.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${liblte}
    ${libtraci}
)


 build_lib_example(
    NAME v2i-greenWindow-lte
    SOURCE_FILES v2i-greenWindow-lte.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${liblte}
    ${libtraci}
 )

 build_lib_example(
    NAME v2i-greenWindow-80211p
    SOURCE_FILES v2i-greenWindow-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
 )


 build_lib_example(
    NAME v2i-bGlosa-lte
    SOURCE_FILES v2i-bGlosa-lte.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${liblte}
    ${libtraci}
 )

 build_lib_example(
    NAME v2i-bGlosa-80211p
    SOURCE_FILES v2i-bGlosa-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
 )

 build_lib_example(
    NAME v2i-ecoDriving-lte
    SOURCE_FILES v2i-ecoDriving-lte.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${liblte}
    ${libtraci}
 )

 build_lib_example(
    NAME v2i-ecoDriving-80211p
    SOURCE_FILES v2i-ecoDriving-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
 )

build_lib_example(
    NAME v2i-areaSpeedAdvisor-80211p
    SOURCE_FILES v2i-areaSpeedAdvisor-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
)


build_lib_example(
    NAME v2i-emergencyVehicleWarning-80211p
    SOURCE_FILES v2i-emergencyVehicleWarning-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
)

build_lib_example(
    NAME v2i-trafficManager-80211p
    SOURCE_FILES v2i-trafficManager-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
)

build_lib_example(
    NAME v2i-trafficManager-LTE
    SOURCE_FILES v2i-trafficManager-LTE.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${liblte}
    ${libtraci}
)

build_lib_example(
    NAME v2v-emergencyVehicleAlert-ltev2x
    SOURCE_FILES v2v-emergencyVehicleAlert-ltev2x.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libcv2x}
    ${libtraci}
    )

build_lib_example(
    NAME v2v-emergencyVehicleAlert-80211p
    SOURCE_FILES v2v-emergencyVehicleAlert-80211p.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libtraci}
)

build_lib_example(
    NAME v2v-emergencyVehicleAlert-nrv2x
    SOURCE_FILES v2v-emergencyVehicleAlert-nrv2x.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libnr}
    ${libtraci}
)

build_lib_example(
    NAME v2x-emulator
    SOURCE_FILES v2x-emulator.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libfd-net-device}
    ${libtraci}
)

build_lib_example(
    NAME v2v-80211p-gps-tc-example
    SOURCE_FILES v2v-80211p-gps-tc-example.cc
    LIBRARIES_TO_LINK
    ${libcarla}
    ${libautomotive}
    ${libwave}
    ${libgps-tc}
    ${libtraci}
)

build_lib_example(
        NAME v2v-80211p-gps-tc-dcc
        SOURCE_FILES v2v-80211p-gps-tc-dcc.cc
        LIBRARIES_TO_LINK
        ${libcarla}
        ${libautomotive}
        ${libwave}
        ${libgps-tc}
        ${libtraci}
)

build_lib_example(
        NAME v2p-vam-80211p
        SOURCE_FILES v2p-vam-80211p.cc
        LIBRARIES_TO_LINK
        ${libcarla}
        ${libautomotive}
        ${libwave}
        ${libtraci}
)

build_lib_example(
        NAME v2v-simple-cam-exchange-80211p
        SOURCE_FILES v2v-simple-cam-exchange-80211p.cc
        LIBRARIES_TO_LINK
        ${libcarla}
        ${libautomotive}
        ${libwave}
        ${libtraci}
)

build_lib_example(
        NAME v2v-congestion-80211p
        SOURCE_FILES v2v-congestion-80211p.cc
        LIBRARIES_TO_LINK
        ${libcarla}
        ${libautomotive}
        ${libwave}
        ${libtraci}
)

#build_lib_example(
#        NAME v2v-congestion-nrv2x
#        SOURCE_FILES v2v-congestion-nrv2x.cc
#        LIBRARIES_TO_LINK
#        ${libautomotive}
#        ${libnr}
#        ${libtraci}
#)

 build_lib_example(
 NAME v2v-carla-nrv2x
 SOURCE_FILES v2v-carlaMobility-nrv2x.cc
 LIBRARIES_TO_LINK
 ${libcarla}
 ${libautomotive}
 ${libnr}
 ${libtraci}
 )

 build_lib_example(
 NAME v2v-carla-80211p
 SOURCE_FILES v2v-carlaMobility-80211p.cc
 LIBRARIES_TO_LINK
 ${libcarla}
 ${libautomotive}
 ${libwave}
 ${libtraci}
 )

build_lib_example(
       NAME v2v-coexistence-80211p-nrv2x
       SOURCE_FILES v2v-coexistence-80211p-nrv2x.cc
       LIBRARIES_TO_LINK
       ${libautomotive}
)

#build_lib_example(
#        NAME v2v-coexistence-80211p-ltev2x
#        SOURCE_FILES v2v-coexistence-80211p-ltev2x.cc
#        LIBRARIES_TO_LINK
#        ${libautomotive}
#)

build_lib_example(
        NAME v2v-cam-exchange-sionna-80211p
        SOURCE_FILES v2v-cam-exchange-sionna-80211p.cc
        LIBRARIES_TO_LINK
        ${libautomotive}
)

build_lib_example(
        NAME v2v-cam-exchange-sionna-nrv2x
        SOURCE_FILES v2v-cam-exchange-sionna-nrv2x.cc
        LIBRARIES_TO_LINK
        ${libautomotive}
)

build_lib_example(
        NAME v2v-cam-exchange-sionna-ltev2x
        SOURCE_FILES v2v-cam-exchange-sionna-ltev2x.cc
        LIBRARIES_TO_LINK
        ${libautomotive}
)
