# Frequently Asked Questions

## Installing

## C++ Library

## Python Library

## General

## Errors

### Build or Dependency Errors
If you see CMake errors upon trying to run or build Obelisk code, there are a few things to try:
* make sure you have the most updated commands and aliases by running the `setup.sh` script with the desired flags on your local filesystem in the Obelisk root
* run `obk-clean` and try again
* if developing in `pixi`, delete your `.pixi` and `pixi.lock` directories/files and try rebuilding from scratch
* if developing in `docker` or on a local filesystem, make sure you run the `install_sys_deps.sh` script under the `scripts` directory
    * if that doesn't work in `docker`, try clearing out cached Docker images and rebuilding the image from scratch
