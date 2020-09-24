set Tag=MG32F02A032_TM36_PWM_TRG_AD_DMA

REN *.uvoptx  %Tag%.uvoptx
REN *.uvprojx  %Tag%.uvprojx

CD Source.si4project
REN **  %Tag%.*

PAUSE
