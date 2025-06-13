#!/bin/bash

echo "=========================="
echo "Starting App visual_side_radar for {APP_PRETTY_NAME}"


systemctl start visual
systemctl start rosnodeChecker
