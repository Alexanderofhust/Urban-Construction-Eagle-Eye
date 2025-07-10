#!/bin/bash

./begin_livox.sh &
sleep 1
{
./begin_camera.sh
}&
sleep 1
{
./begin_convert.sh
}&
sleep 1
{
./begin_rknn.sh
}&
sleep 3
{
./begin_mapping.sh
}&
sleep 1
{
./begin_compress.sh
}&
sleep 1
{
   bash /home/elf/slam/record.sh
}