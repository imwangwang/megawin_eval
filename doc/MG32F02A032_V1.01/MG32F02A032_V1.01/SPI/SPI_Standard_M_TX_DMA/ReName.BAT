set Tag=MG32F02A032_SPI_Standard_M_TX_DMA

REN *.uvoptx  %Tag%.uvoptx
REN *.uvprojx  %Tag%.uvprojx

CD Source.si4project
REN **  %Tag%.*

PAUSE
