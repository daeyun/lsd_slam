#!/usr/bin/env bash

set -ex

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

cd ${DIR}

mkdir -p ./build
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
make -Cbuild VERBOSE=1
