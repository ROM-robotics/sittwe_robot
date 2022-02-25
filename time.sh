#!/bin/bash

timedatectl set-timezone Asia/Yangon
timedatectl set-ntp false
timedatectl set-time $(ssh mr_robot@192.168.0.101 "date +%Y-%m-%d") && timedatectl set-time $(ssh mr_robot@192.168.0.101 "date +%H:%M:%S")
timedatectl set-ntp true