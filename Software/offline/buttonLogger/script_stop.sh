#!/bin/bash
sudo kill $(ps aux | grep "ros" | awk '{print $2}')

