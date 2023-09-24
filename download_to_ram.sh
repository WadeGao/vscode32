rm build/STM32.*
cd build
ninja clean
cmake ..
ninja
cd ..
openocd -f openocd.cfg -c init -c 'reset halt' -c wait_halt -c 'load_image build/STM32.bin 0x20000000' -c 'reg pc 0x20000000' -c reset -c shutdown
