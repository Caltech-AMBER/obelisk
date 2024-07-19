#!/bin/bash
# resolves issue with rosidl typesupport libraries not being found
# see: https://github.com/prefix-dev/pixi/issues/1635
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PIXI_PROJECT_ROOT/.pixi/envs/$PIXI_ENVIRONMENT_NAME/lib
