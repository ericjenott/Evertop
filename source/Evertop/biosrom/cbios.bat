nasm bios.asm
xxd -i -c 256 bios | sed '1d;$d' | sed '$d' >biosrom.h
del bios
