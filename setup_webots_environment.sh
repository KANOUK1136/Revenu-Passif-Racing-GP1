#!/bin/bash

# Avoir la complétion VS Code
grep -q WEBOTS_HOME ~/.bashrc
if [ "$?" != "0" ] ; then
    echo 'export WEBOTS_HOME=/usr/local/webots' >> ~/.bashrc
    echo 'export WEBOTS_CONTROLLER=${WEBOTS_HOME}/lib/controller/' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=${WEBOTS_CONTROLLER}' >> ~/.bashrc
    echo 'export PYTHONPATH=${WEBOTS_CONTROLLER}/python' >> ~/.bashrc

    echo 'DONE !'
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]] ; then
        # script not sourced
        echo "[IMPORTANT] apply modification with following command :"
        echo "  source ~/.bashrc"
    else
        # script sourced
        source ~/.bashrc
    fi
else
    echo 'Webots variables are already setup'
fi
