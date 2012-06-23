/*
Default linker script for STM32F051R8T6 64k 8k
Copyright RAISONANCE S.A.S. 2007

06-19-2012 rick@kimballsoftware.com - modified for stm32f0 discovery board stripped to bare minimum

stm32f0.ld
*/

/* include the memory spaces definitions sub-script */
INCLUDE "stm32f0discovery_def.ld"

/* include the sections management sub-script for FLASH mode */
INCLUDE "sections_flash.ld"

