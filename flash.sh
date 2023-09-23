rm build/STM32.*
cd build
ninja clean
cmake ..
ninja
cd ..
openocd -f openocd.cfg -c init -c 'reset halt' -c wait_halt -c 'flash write_image erase build/STM32.bin 0x8000000' -c reset -c shutdown
