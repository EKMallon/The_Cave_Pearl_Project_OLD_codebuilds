Spent some time making that rough code more efficient, so that I can buffer more sensor read cycles to the Arduino’s very limited SRAM before having to burn away battery power writing data to the SD cards.

implemented here:

Eliminate unnecessary cruft:

I stopped dumping all the data into a text string, which got rid of the commas, spaces, etc that I had been buffering for nothing. I guess this is a pretty common newbie mistake.

And instead of buffering the entire date & time stamp, I simply record it once at the beginning of a sample cycle and then use a one byte record/offset number, as I can re-calculate the actual time for each sample from this offset number. This saved a huge amount of free ram! (Thanks Mike!)

Use the smallest variable types possible.

I converted the floating point temperature readings into sets of two integers (the part before the decimal place, and the part after the decimal place) and any integers that were storing values less than 256, are now bytes, or uint8_t, which cut them down by half. The single time stamp per cycle is now stored in a well defined “char” variable. One person suggested that with some fancy bit management I could probably cut those variables in half again, but I have not quite wrapped my head around bit shifting & two’s complement yet. (Thanks Ken!)

Write better code:

The crude code duplication of the earlier versions has been replaced with array variables and for loops, which you control via the SampleInterval and SamplesPerCycle defines at the beginning of the script. These cascade through to the array definitions. I should mention here that I tried this approach before when I had the data packed into strings and it simply did not work. The freeRam routine showed me that every cycle bled away another byte or two till the system crashed. But AFTER I got rid of all the strings in the script, the loops are stable. I suspect there is a bug in the way the AVR compiler handles string data.

The net result of these changes is that each record went from about 60 characters down to 12 bytes of data, so this script can easily buffer 10 cycles or more of data and still leave 680 bytes of free ram. A nice safe margin away from the 512 bytes that I have to leave available for the SD.h buffer.