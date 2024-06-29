#!/bin/bash

OPTIONS=$(getopt -o 'h:' --long mujoco-dir: -- "$@")

eval set -- "$OPTIONS"

# process inputs
while true; do
  case "$1" in
    -h)
      if [[ -n "$2" ]] && [[ "$2" != "" ]] && [[ "${2:0:1}" != "-" ]]; then
        hash="$2"
        shift 2
      elif [[ "$2" == "" ]]; then
        hash=""
        shift 2
      else
        hash=""
        shift
      fi
      ;;
    --mujoco-dir)
      if [[ -n "$2" ]] && [[ "$2" != "" ]] && [[ "${2:0:1}" != "-" ]]; then
        mujoco_dir="$2"
        shift 2
      elif [[ "$2" == "" ]]; then
        mujoco_dir=""
        shift 2
      else
        mujoco_dir=""
        shift
      fi
      ;;
    --mujoco-dir=*)
      mujoco_dir="${1#*=}"
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "$1"
      echo "[ERROR] Open an issue, this should never occur if you call install.sh."
      exit 1
      ;;
  esac
done

echo -e "\n[NOTE] Installing mujoco from source..."

# the script directory and mujoco directory
if ! [[ -n "$mujoco_dir" ]] ; then
    mujoco_dir="$HOME/mujoco"
fi

# check whether we already have the most recent release cached to save build time
latest_git_hash=$(git ls-remote https://github.com/google-deepmind/mujoco HEAD | awk '{ print $1}')
if [ -d "$mujoco_dir/python/dist" ]; then
    tar_path=$(find "$mujoco_dir/python/dist" -name "mujoco-*${latest_git_hash}.tar.gz" 2>/dev/null)
    whl_path=$(find "$mujoco_dir/python/dist" -name "mujoco-*.whl" 2>/dev/null)

    if [ -f "$tar_path" ]; then
        echo -e "[NOTE] Found cached mujoco tar.gz file..."
        if [ -f "$whl_path" ]; then
            echo -e "[NOTE] Wheel found! Installing from wheel..."
            cd "$mujoco_dir/python/dist"

            # [Nov 10, 2023] we install with --no-deps because this would upgrade numpy to a version incompatible with cmeel-boost 1.82.0
            MUJOCO_PATH="$mujoco_dir/mujoco_install" MUJOCO_PLUGIN_PATH="$mujoco_dir/plugin" pip install --no-deps --force-reinstall "$whl_path"
            cd "$mujoco_dir/mjx"
            pip install --no-deps --force-reinstall .
            exit 0
        else
            echo -e "[NOTE] No wheel found! Building it and installing..."
            cd "$mujoco_dir/python/dist"
            MUJOCO_PATH="$mujoco_dir/mujoco_install" MUJOCO_PLUGIN_PATH="$mujoco_dir/plugin" pip wheel -w $(dirname "whl_path") "$tar_path"

            # [Nov 10, 2023] we install with --no-deps because this would upgrade numpy to a version incompatible with cmeel-boost 1.82.0
            MUJOCO_PATH="$mujoco_dir/mujoco_install" MUJOCO_PLUGIN_PATH="$mujoco_dir/plugin" pip install --no-deps --force-reinstall "$whl_path"
            cd "$mujoco_dir/mjx"
            pip install --no-deps --force-reinstall .
            exit 0
        fi
    fi
fi

# Check if CMake and a compiler are installed
command -v cmake &> /dev/null || { echo "CMake is not installed. Please install CMake before proceeding."; exit 1; }
command -v g++ &> /dev/null || { echo "C++ compiler is not available. Please install a C++ compiler before proceeding."; exit 1; }


# Clone the Mujoco repo to the directory above where this script is located
# If it exists already, then just git pull new changes
if [ -d "$mujoco_dir" ]; then
    echo "Mujoco exists already - running git pull to update it!"
    cd "$mujoco_dir"
    git pull origin main
    if [ ! -z "$hash" ]; then
        echo "Checking out with provided commit hash!"
        git checkout "$hash"
    fi
else
    echo "Mujoco does not exist - cloning it!"
    git clone https://github.com/google-deepmind/mujoco.git "$mujoco_dir"
    if [ ! -z "$hash" ]; then
        echo "Checking out to provided hash!"
        (cd "$mujoco_dir" && git checkout "$hash")
    fi
fi

# Configure and build
export MAKEFLAGS="-j$(nproc)"  # allow the max number of processors when building
cd "$mujoco_dir"
if [ -z "$hash" ]; then
    saved_git_hash=$(git rev-parse HEAD)  # getting the git hash
else
    saved_git_hash="$hash"
fi
cmake . && cmake --build .

# Install Mujoco
(cd "$mujoco_dir" && cmake . -DCMAKE_INSTALL_PREFIX="./mujoco_install" && cmake --install .)

# Generate source distribution required for Python bindings
cd "$mujoco_dir/python"
./make_sdist.sh
tar_path=$(find "$mujoco_dir/python/dist" -name 'mujoco-*.tar.gz' 2>/dev/null)

# Renaming the tar file by appending the commit hash
new_tar_path="$(dirname "$tar_path")/$(basename "$tar_path" .tar.gz)-$saved_git_hash.tar.gz"
mv "$tar_path" "$new_tar_path"

# manually building wheel so we can cache it
cd "$mujoco_dir/python/dist"
MUJOCO_PATH="$mujoco_dir/mujoco_install" MUJOCO_PLUGIN_PATH="$mujoco_dir/plugin" pip wheel -w $(dirname "new_tar_path") "$new_tar_path"

# installing mujoco from wheel and then finally mjx
whl_path=$(find "$mujoco_dir/python/dist" -name "mujoco-*.whl" 2>/dev/null)
MUJOCO_PATH="$mujoco_dir/mujoco_install" MUJOCO_PLUGIN_PATH="$mujoco_dir/plugin" pip install --no-deps --force-reinstall "$whl_path"
cd "$mujoco_dir/mjx"
pip wheel -w "$mujoco_dir/mjx" --no-deps .
pip install --no-deps --force-reinstall .
