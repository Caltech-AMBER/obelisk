#!/bin/bash

# --- script flags --- #
leap=false
zed=false

for arg in "$@"; do
    case $arg in
        # by hardware type
        --zed)
            zed=true
            shift  # Enables ZED SDK
            ;;

        # help
        --help)
            echo "Usage: bash config_groups.sh [OPTIONS]

Options:
  --zed     Adds user to the video group
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

if [ "$zed" = true ]; then
    sudo usermod -a -G video $USER
fi
