#!/usr/bin/env bash

current_date_time=`date "+%Y%m%d%H%M%S"`;
echo $current_date_time;
arecord /home/pal/logs/${current_date_time}.wav