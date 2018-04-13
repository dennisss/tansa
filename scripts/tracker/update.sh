#!/bin/bash
# Updates a remote tracker's code from github
# Run as:
#     ./scripts/tracker/update.sh ID


# Update code from
# TODO: Force pull
$SSH cd tansa; git pull origin mocap

# Rebuild
# TODO

# Restart the service
# TODO
