#!/bin/bash

# workarounds to fix certain features while waiting for dev updates
uv pip install pytest==8.0.0  # pytest+humble broken: github.com/prefix-dev/pixi/issues/1529
