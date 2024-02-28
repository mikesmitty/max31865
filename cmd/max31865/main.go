package main

import (
	"flag"
	"log"
	"time"

	"github.com/mikesmitty/max31865"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

func main() {
	bus := flag.String("bus", "", "Name of the bus")
	flag.Parse()

	_, err := host.Init()
	if err != nil {
		log.Fatal(err)
	}

	sb, err := spireg.Open(*bus)
	if err != nil {
		log.Fatal(err)
	}

	dev, err := max31865.New(sb, max31865.AdafruitPT100())
	if err != nil {
		log.Fatal(err)
	}

	ticker := time.NewTicker(1 * time.Second)

	for {
		var e physic.Env
		err = dev.Sense(&e)
		if err != nil {
			log.Print(err)
		}
		log.Printf("Temperature: %f", e.Temperature.Celsius())

		<-ticker.C
	}
}
