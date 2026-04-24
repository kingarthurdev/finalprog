#!/usr/bin/env bash

rm -rf build
cmake -S . -B build
make -C build
