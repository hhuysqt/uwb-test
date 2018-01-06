#!/bin/bash
JLinkExe -device stm32f072cb -if swd -speed 4000 -commanderscript jlink.script 
