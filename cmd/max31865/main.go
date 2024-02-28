package main

import (
	"flag"
	"log"
	"strings"
	"time"

	"github.com/mikesmitty/max31865"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

func main() {
	bus := flag.String("bus", "", "Name of the bus")
	devFlag := flag.String("type", "", "Sensor type (PT100 or PT1000)")
	flag.Parse()

	_, err := host.Init()
	if err != nil {
		log.Fatal(err)
	}

	sb, err := spireg.Open(*bus)
	if err != nil {
		log.Fatal(err)
	}

	devType := strings.ToLower(*devFlag)
	var opts *max31865.Opts
	switch devType {
	case "pt100":
		opts = max31865.AdafruitPT100()
	case "pt1000":
		opts = max31865.AdafruitPT1000()
	default:
		log.Fatal("Invalid sensor type")
	}

	dev, err := max31865.New(sb, opts)
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
