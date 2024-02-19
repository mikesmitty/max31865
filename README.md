mikesmitty/max31865
====================

This is a library for using the [MAX31865](https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf) PT100/PT1000 RTD temperature sensor amplifier in Go using [Periph](https://periph.io). Originally based on the [vemo-france/max31865](https://github.com/vemo-france/max31865) port of [Adafruit's MAX31865 library](https://github.com/adafruit/Adafruit_MAX31865), though it has been fairly substantially rewritten. Intended for use with Adafruit's MAX31865 breakout boards [PT100](https://www.adafruit.com/product/3328)/[PT1000](https://www.adafruit.com/product/3648) on any device supported by [Periph](https://periph.io/device/).


Installation
------------

````
go get github.com/mikesmitty/max31865
````

Usage
----

````Go
package main

import (
	"fmt"

	"github.com/mikesmitty/max31865"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

func main() {
	sb, _ := spireg.Open(yourSpiBus)
	dev, _ := max31865.New(sb, max31865.AdafruitPT100())

	// Perform a one-off reading with Sense()
	var e physic.Env
	_ := dev.Sense(&e)
	fmt.Printf("Temperature: %0.2f\n", e.Temperature.Celsius())

	// Alternatively, SenseContinuous() returns a channel for continuous updates
	c, _ := dev.SenseContinuous(updateInterval)
	for {
		e := <-c
		fmt.Printf("Temperature: %0.2f\n", e.Temperature.Celsius())
	}
}
````

TODO
----
[ ] Improve documentation
[ ] Add tests
[ ] Surface hardware fault error codes rather than just clearing them like the Adafruit library does