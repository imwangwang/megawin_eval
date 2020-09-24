set Tag=MG32F02A032_EXIC_STOP_Wakeup

REN *.uvoptx  %Tag%.uvoptx
REN *.uvprojx  %Tag%.uvprojx

CD Source.si4project
REN **  %Tag%.*

PAUSE
