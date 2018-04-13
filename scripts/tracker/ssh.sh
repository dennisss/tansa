#!/bin/bash
# Helper script for SSHing into a tracker node using mDNS via the hostname given its numerical id

HOSTNAME="tracker-$1.local"
SSH_KEY="~/.ssh/id_tracker"

ssh -o IdentitiesOnly=yes -F /dev/null -i "$SSH_KEY" pi@$HOSTNAME" "${@:2}"
