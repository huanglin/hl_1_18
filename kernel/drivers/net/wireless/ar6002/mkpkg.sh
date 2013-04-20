#!/bin/sh

echo "Begin to make AR6002 wifi driver package ..."

mkdir -p ar6002
rm -f ar6002/*

cp -f Makefile.ar6002 ar6002/Makefile
cp -f ../wifi_power/wifi_power.h ar6002/
cp -f ../wifi_power/wifi_power.c ar6002/
uuencode ar6002.o ar6002.o > ar6002.uu
mv ar6002.uu ar6002/

echo "Done"

