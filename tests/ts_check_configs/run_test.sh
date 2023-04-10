#!/bin/bash -e

full_graphics_config_files=(
    "tsimx6_defconfig"
)

min_graphics_config_files=(
    "tsimx6_minimal_defconfig"
)

full_headless_config_files=(
    "tsa38x_defconfig"
    "tsimx6ul_defconfig"
    "tsimx28_defconfig"
)

min_headless_config_files=(
    "tsa38x_minimal_defconfig"
    "tsimx6ul_minimal_defconfig"
    "tsimx28_minimal_defconfig"
)

for config_file in "${full_graphics_config_files[@]}"; do
    echo "Testing ${config_file}..."
    make "${config_file}" > /dev/null 2>&1
    tests/ts_check_configs/config_check \
        tests/ts_check_configs/full_graphics_config_requirements \
        .config
done

for config_file in "${min_graphics_config_files[@]}"; do
    echo "Testing ${config_file}..."
    make "${config_file}" > /dev/null 2>&1
    tests/ts_check_configs/config_check \
        tests/ts_check_configs/min_graphics_config_requirements \
        .config
done

for config_file in "${full_headless_config_files[@]}"; do
    echo "Testing ${config_file}..."
    make "${config_file}" > /dev/null 2>&1
    tests/ts_check_configs/config_check \
        tests/ts_check_configs/full_headless_config_requirements \
        .config
done

for config_file in "${min_headless_config_files[@]}"; do
    echo "Testing ${config_file}..."
    make "${config_file}" > /dev/null 2>&1
    tests/ts_check_configs/config_check \
        tests/ts_check_configs/min_headless_config_requirements \
        .config
done
