#!/bin/bash
sudo kill -int $(ps aux | grep "ros" | awk '{print $2}')

