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
build_lib(
  LIBNAME propagation
  SOURCE_FILES
    model/channel-condition-model.cc
    model/cost231-propagation-loss-model.cc
    model/itu-r-1411-los-propagation-loss-model.cc
    model/itu-r-1411-nlos-over-rooftop-propagation-loss-model.cc
    model/jakes-process.cc
    model/jakes-propagation-loss-model.cc
    model/kun-2600-mhz-propagation-loss-model.cc
    model/okumura-hata-propagation-loss-model.cc
    model/probabilistic-v2v-channel-condition-model.cc
    model/propagation-delay-model.cc
    model/propagation-loss-model.cc
    model/three-gpp-propagation-loss-model.cc
    model/three-gpp-v2v-propagation-loss-model.cc
    model/cni-urbanmicrocell-propagation-loss-model.cc
  HEADER_FILES
    model/channel-condition-model.h
    model/cost231-propagation-loss-model.h
    model/itu-r-1411-los-propagation-loss-model.h
    model/itu-r-1411-nlos-over-rooftop-propagation-loss-model.h
    model/jakes-process.h
    model/jakes-propagation-loss-model.h
    model/kun-2600-mhz-propagation-loss-model.h
    model/okumura-hata-propagation-loss-model.h
    model/probabilistic-v2v-channel-condition-model.h
    model/propagation-cache.h
    model/propagation-delay-model.h
    model/propagation-environment.h
    model/propagation-loss-model.h
    model/three-gpp-propagation-loss-model.h
    model/three-gpp-v2v-propagation-loss-model.h
    model/cni-urbanmicrocell-propagation-loss-model.h
  LIBRARIES_TO_LINK ${libnetwork}
                    ${libmobility}
  TEST_SOURCES
    test/channel-condition-model-test-suite.cc
    test/itu-r-1411-los-test-suite.cc
    test/itu-r-1411-nlos-over-rooftop-test-suite.cc
    test/kun-2600-mhz-test-suite.cc
    test/okumura-hata-test-suite.cc
    test/probabilistic-v2v-channel-condition-model-test.cc
    test/propagation-loss-model-test-suite.cc
    test/three-gpp-propagation-loss-model-test-suite.cc
    test/three-gpp-propagation-loss-model-test-suite.cc
)
