#!/bin/bash

set -e

cd /home/user/workspace

# If no command is provided, set bash to start interactive shell
if [ -z "$1" ]; then
    set - "/bin/bash" -l
fi

# Set color
export PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

# Run the provided command using user
exec "$@"
