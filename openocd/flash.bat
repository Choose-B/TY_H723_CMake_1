@echo off
@REM 使用openocd中的daplink.cfg中的内容,对这个elf文件进行确认烧录结束 然后reset 再退出openocd

openocd -f openocd/daplink.cfg -c "program build/debug/TY_H723_CMake_1.elf verify reset exit"