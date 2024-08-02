#!/bin/bash

echo "Running build.bash"

echo "=========================================================================================="

if [[ -z "${VISIS_ROOT}" ]]; then
	echo "Env. var. VISIS_ROOT unset! Using \$(pwd) as root_dir."
	root_dir="$(pwd)"
else
	root_dir="${VISIS_ROOT}"
fi

if [[ -z "${VISIS_BUILD_TYPE}" ]]; then
	echo "Env. var. VISIS_BUILD_TYPE unset! Using Release as build_type."
	build_type="Release"
else
	build_type="${VISIS_BUILD_TYPE}"
fi

if [[ -z "${VISIS_BUILD_DIR}" ]]; then
	echo "Env. var. VISIS_BUILD_DIR unset! Using \$(pwd)/build-\${build_type} as build_dir."
	build_dir="$(pwd)/build-${build_type}"
else
	build_dir="${VISIS_BUILD_DIR}"
fi

echo "=========================================================================================="

echo "root_dir: ${root_dir}"
echo "build_type: ${build_type}"
echo "build_dir: ${build_dir}"

echo "=========================================================================================="

mkdir -p "${build_dir}"
cd "${build_dir}" || exit 1
cmake -DCMAKE_BUILD_TYPE="${build_type}" "${root_dir}/visis"
make -j8 

echo "=========================================================================================="

echo "DONE"

exit 0
