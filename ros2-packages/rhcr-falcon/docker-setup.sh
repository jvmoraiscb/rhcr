#!/bin/bash

sudo docker build -t rhcr-falcon .
cp docker-run.sh ~/
cd
sudo chmod +x docker-run.sh