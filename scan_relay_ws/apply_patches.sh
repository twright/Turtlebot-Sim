#!/usr/bin/bash

# Base relative paths off script dir
cd "$(dirname "${BASH_SOURCE[0]}")"
cd navigation2

git apply ../nav_parameters.patch