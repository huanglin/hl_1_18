#!/bin/sh

find . -name '*.o' -exec rm -f {} \;
find . -name '.*.cmd' -exec rm -f {} \;
find . -name modules.order -exec rm -f {} \;
find . -name '*.uu' -exec rm -f {} \;

