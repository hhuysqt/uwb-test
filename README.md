UWB module
===

A simple example of UWB system using DWM1000 and STM32F072CB.

modes
---
> **Anchor mode**: basic TWR-TOA<br>
> **Tag mode**: basic TWR-TOA<br>
> **TDOA anchor mode**: anchors mesure themselves automatically<br>
> **TDOA tag mode**: sniff anchors' messages and calculate its coordinate<br>
> **Router mode**: a radio receiver<br>

**note: TDOA is buggy**

making
---
1. Install arm-none-eabi-gcc<br>
2. make

flashing
---
JLink and JLinkExe
