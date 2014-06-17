Updates from the last version:

This script automatically creates new data files at an interval determined by comparing the fileInterval to the countlog (thanks adafruit!) There is also a nice #ifdef …#endif trick to control echoing to serial by simply commenting out the ECHO_TO_SERIAL def at the beginning of the code. And finally an error subroutine, which hangs the system in a while(1); if something goes wrong. I use somewhat cryptic error codes, as every character passed to the error routine eats into my precious SRAM budget.

The current data output looks like this on the SD card:

The sample interval for this series is: 1 minute
MM/DD/YY HH:MM:SS Cycle# = Toffset ,Vcc(mV), X = Y = Z = ,BMATemp, RTCtemp
2/27/2014 13:53 time offset: 0   2972   10  -5 228   28    25
2/27/2014 13:53 time offset: 1   2964    9  -5 227   27.5  25
2/27/2014 13:53 time offset: 2   2964    9  -4 227   27.5  24
2/27/2014 13:53 time offset: 3   2964    8  -3 227   26.5  24
2/27/2014 13:53 time offset: 4   2964    9  -4 227   26    24
2/27/2014 13:53 time offset: 5   2964    9  -4 228   26    24
2/27/2014 13:53 time offset: 6   2964   10  -3 228   26.5  24
2/27/2014 13:53 time offset: 7   2964   10  -4 228   25.5  24
2/27/2014 13:53 time offset: 8   2964   10  -4 228   25.5  24
2/27/2014 13:53 time offset: 9   2964    9  -4 228   25.5  24
2/27/2014 13:53 time offset: 10  2964   10  -4 227   25.5  24
2/27/2014 13:53 time offset: 11  2964   10  -4 228   25.5  24
2/27/2014 14:05 time offset: 0   2964    8  -3 228   25.5  24
…etc
(Note: Excel switched to d/m/y here! I always use Y/M/D!)

Only one time stamp is recorded per 12 cycles, as that’s just too many characters to buffer in the limited SRAM (no unix time yet!). So I will have to re-constitute the full time stamp in post. If you use this code, keep a very close eye on the freemem, as you sensors will generate different data, and every single byte/character you are buffering to SRAM matters. My system gets wobbly whenever the freemem goes down near 550…