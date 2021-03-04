# Pi Pico Pi
I'm trying to calculate 1 million digits of π using a Raspberry Pi Pico. I plan to use the [spigot algorithm](http://stanleyrabinowitz.com/bibliography/spigot.pdf) to calculate the digits one at a time and display them one per second on a seven segment display.

Originally I wanted to do 32 million digits since that's one for every second of the year. It quickly became apparent that this was a much bigger task than the Pico could reasonably handle.

The spigot algorithm needs quite a bit of ram for 1 million digits, so I got four [hyperram](https://1bitsquared.com/products/pmod-hyperram) modules to use on the Pico. I'm using PIO to communicate with the hyperram and will use it to store the entire array for the algorithm while using the Pico's ram as a sort of sliding cache window.

Current code status: Just throwing it up here without much polish for now. Will get more polished as I go on.

I hope I can get this finished in time for pi day.