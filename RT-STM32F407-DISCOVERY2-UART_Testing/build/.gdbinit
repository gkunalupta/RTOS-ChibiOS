# copy this file to .gdbinit in your Firmware tree

# this sets up gdb to use openocd. You must start openocd first
target extended-remote :3333

set remotetimeout 20
monitor reset init
monitor sleep 50

b main.c:42