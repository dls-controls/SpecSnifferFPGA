function clocks(FREQ)


testClkPeriod = 1/100;
tmp = 8192 * testClkPeriod;
tmp = tmp / (FREQ + 1);
clock = 1/tmp
