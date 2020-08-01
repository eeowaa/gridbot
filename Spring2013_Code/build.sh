#!/bin/bash
gcc -ansi -pedantic GridBot.c -o VirtualBot 2> errlog
if [ -s errlog ]
then
    cat errlog
else
    sed 's|^#define VIRTUAL_BOT|/\* #define VIRTUAL_BOT \*/|g' < GridBot.c > avr.c
    gcc -ansi -pedantic avr.c -o AVRBot 2> errlog
    rm avr.c
    if [ -s errlog ]
    then
        cat errlog
    else
        sh size.sh
    fi
fi
rm errlog
