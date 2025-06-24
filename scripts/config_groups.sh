#!/bin/bash

# --- script flags --- #
leap=false
zed=false
fr3=false

for arg in "$@"; do
    case $arg in
        # by hardware type
        --leap)
            leap=true
            shift  # Enables LEAP hand dependencies
            ;;
        --zed)
            zed=true
            shift  # Enables ZED SDK
            ;;
        --fr3)
            fr3=true
            shift  # Enables FR3 dependencies
            ;;

        # help
        --help)
            echo "Usage: bash config_groups.sh [OPTIONS]

Options:
  --leap    Adds user to the dialout group
  --zed     Adds user to the video group
  --fr3     Adds user to the realtime group
  --help    Display this help message and exit
"
            shift
            return
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg. Run 'source config_groups.sh --help' for more information."
            return
            ;;
    esac
done

if [ "$leap" = true ]; then
    sudo usermod -a -G dialout $USER
fi

if [ "$zed" = true ]; then
    sudo usermod -a -G video $USER
fi

if [ "$fr3" = true ]; then
    sudo addgroup realtime
    sudo usermod -a -G realtime $USER
fi
