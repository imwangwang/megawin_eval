set Tag=MG32F02A032_TM36_PWM_DeadTime

REN *.uvoptx  %Tag%.uvoptx
REN *.uvprojx  %Tag%.uvprojx

CD Source.si4project
REN **  %Tag%.*

PAUSE
