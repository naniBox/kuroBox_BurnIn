@set PATH=%PATH%;"C:\Program Files (x86)\SEGGER\JLinkARM_V470a\"
c:
cd "c:\ChibiStudio\workspace\kuroBox_BurnIn\"
jlink -CommanderScript FLASH_kuroBox_BurnIn.jlink
pause
