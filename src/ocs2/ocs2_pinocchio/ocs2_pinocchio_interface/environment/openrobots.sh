#!/usr/bin/env sh

# Make the robotpkg Pinocchio install visible at runtime and during follow-up builds.
if [ -d "/opt/openrobots" ]; then
  ament_prepend_unique_value CMAKE_PREFIX_PATH "/opt/openrobots"
  ament_prepend_unique_value LD_LIBRARY_PATH "/opt/openrobots/lib"
fi
