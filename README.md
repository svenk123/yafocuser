# YaFocuser - Yet Another Focuser
YaFocuser is a cheap MoonLite-compatible motorized focuser for an astronomical telescope.

# Features
* Supports [MoonLite protocol](http://www.indilib.org/media/kunena/attachments/1/HighResSteppermotor107.pdf)
* Controlled via USB
* USB bus-powered
* Several motor speeds
* Current temperature readout supported (with optional parts)
* Releases idle current to the motor windings after 5 seconds when motion is finished to reduce power consumption
* Additional #HELP command to print out all available MoonLite commands

## Hardware Parts
The focuser hardware is based on the following parts.

* 1 [Teensy 2.0 board](https://www.pjrc.com/store/teensy.html)
* 1 ULN2003 Motor driver board
* 1 DS18B20 temperature sensor ([datasheet](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf))
* 1 4,7 kOhm Resistor

## Supported Stepper Motors
* 5V 4-phase Stepper Motor 28BYJ-48

## Images

You can find some images in the images directory how to solder the stepper motor driver and teensy board.

There are also some images of a 3D printed focuser for a Celestron C8.



## Installation
Compile and transfer the program to the Teensy board.

```bash
make
make install
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
    
## Support
This project was developed for the public domain and as such it is unsupported. I will listen to problems and suggestions for improvement. I wish you my best and hope that everything works out.
