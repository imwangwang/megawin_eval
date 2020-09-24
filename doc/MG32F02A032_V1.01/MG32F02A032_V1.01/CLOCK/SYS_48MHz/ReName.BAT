set Tag=MG32F02A032_SYS_48MHz

REN *.uvoptx  %Tag%.uvoptx
REN *.uvprojx  %Tag%.uvprojx

CD Source.si4project
REN **  %Tag%.*

PAUSE
