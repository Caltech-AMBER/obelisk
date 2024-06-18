#!/bin/bash

# symlink the home directory in the container to the placeholder
if [ -n "$USERNAME" ]; then
    ln -s /home/placeholder/obelisk /home/$USERNAME/obelisk
fi

# Execute the passed command
exec "$@"
