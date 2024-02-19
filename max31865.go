package max31865

import (
	"errors"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
	"time"

	"periph.io/x/conn/v3"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
)

// Opts holds various configuration options for the sensor
type Opts struct {
	Port        string
	RefResistor float64
	RTDType     RTDType
	WireCount   WireCount
}

func DefaultOptions() *Opts {
	return AdafruitPT100()
}

func AdafruitPT100() *Opts {
	return &Opts{
		RefResistor: 430.0,
		RTDType:     RTDPT100,
		WireCount:   WireCount3,
	}
}

func AdafruitPT1000() *Opts {
	return &Opts{
		RefResistor: 4300.0,
		RTDType:     RTDPT1000,
		WireCount:   WireCount3,
	}
}

func New(p spi.Port, opts *Opts) (*Dev, error) {
	c, err := p.Connect(5*physic.MegaHertz, spi.Mode3, 8)
	if err != nil {
		return nil, fmt.Errorf("max31865: %v", err)
	}

	d := &Dev{
		d:         c,
		opts:      *opts,
		name:      p.String(),
		measDelay: 65 * time.Millisecond,
	}

	switch opts.RTDType {
	case RTDPT100:
		d.rtdNominal = 100.0
	case RTDPT1000:
		d.rtdNominal = 1000.0
	default:
		return nil, fmt.Errorf("max31865: invalid RTD type: %v", opts.RTDType)
	}

	// Set wire count (either 3 or 2/4)
	err = d.writeConfig(config3Wire, opts.WireCount != WireCount3)
	if err != nil {
		return nil, fmt.Errorf("max31865: %v", err)
	}

	// Disable bias when not in use to reduce self-heating
	d.enableBias(false)
	d.clearFault()

	return d, nil
}

type Dev struct {
	d          conn.Conn
	opts       Opts
	measDelay  time.Duration
	name       string
	rtdNominal float64

	mu   sync.Mutex
	stop chan struct{}
	wg   sync.WaitGroup
}

func (d *Dev) String() string {
	// d.dev.Conn
	return fmt.Sprintf("%s{%s}", d.name, d.d)
}

func (d *Dev) Sense(e *physic.Env) error {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.stop != nil {
		return d.wrap(errors.New("already sensing continuously"))
	}

	return d.sense(e)
}

// SenseContinuous returns measurements as °C on a continuous basis.
//
// The application must call Halt() to stop the sensing when done to stop the
// sensor and close the channel.
//
// It's the responsibility of the caller to retrieve the values from the
// channel as fast as possible, otherwise the interval may not be respected.
func (d *Dev) SenseContinuous(interval time.Duration) (<-chan physic.Env, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.stop != nil {
		// Don't send the stop command to the device.
		close(d.stop)
		d.stop = nil
		d.wg.Wait()
	}

	sensing := make(chan physic.Env)
	d.stop = make(chan struct{})
	d.wg.Add(1)
	go func() {
		defer d.wg.Done()
		defer close(sensing)
		d.sensingContinuous(interval, sensing, d.stop)
	}()
	return sensing, nil
}

// 15-Bit ADC Resolution; Nominal temperature resolution varies due to RTD non-linearity
func (d *Dev) Precision(e *physic.Env) {
	e.Temperature = physic.Kelvin / 32
}

// Halt stops the MAX31865 from acquiring measurements as initiated by
// SenseContinuous().
//
// It is recommended to call this function before terminating the process to
// reduce idle power usage and a goroutine leak.
func (d *Dev) Halt() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.stop == nil {
		return nil
	}
	close(d.stop)
	d.stop = nil
	d.wg.Wait()

	return nil
}

func (d *Dev) sense(e *physic.Env) error {
	d.clearFault()
	d.enableBias(true)
	// Need 10ms for bias voltage startup.
	time.Sleep(10 * time.Millisecond)

	if err := d.writeConfig(config1Shot); err != nil {
		return d.wrap(err)
	}
	time.Sleep(d.measDelay)

	var result [2]byte
	if err := d.readReg(rtdMsbReg, result[:]); err != nil {
		return d.wrap(err)
	}

	// Disable bias current again to reduce selfheating.
	if err := d.enableBias(false); err != nil {
		return d.wrap(err)
	}

	temp, err := d.parseTemperature(result[:])
	if err != nil {
		return d.wrap(err)
	}

	e.Temperature = physic.Temperature(temp*1000)*physic.MilliCelsius + physic.ZeroCelsius

	return nil
}

func (d *Dev) parseTemperature(data []byte) (float64, error) {
	rtd := (uint16(data[0]) << 8) | uint16(data[1])

	// Clear fault flag
	rtd >>= 1

	// Convert to resistance
	Rt := float64(rtd) * d.opts.RefResistor / 32768

	// Convert to temperature
	var temp float64

	Z1 := -rtdA
	Z2 := rtdA*rtdA - (4 * rtdB)
	Z3 := (4 * rtdB) / d.rtdNominal
	Z4 := 2 * rtdB

	temp = Z2 + (Z3 * Rt)
	temp = (math.Sqrt(temp) + Z1) / Z4

	if temp >= 0 {
		return temp, nil
	}

	// Normalize to 100 ohm
	Rt = Rt * 100 / d.rtdNominal

	rpoly := Rt

	temp = -242.02
	temp = temp + (2.2228 * rpoly)
	rpoly = rpoly * Rt // Resistance squared
	temp = temp + (2.5859e-3 * rpoly)
	rpoly = rpoly * Rt // ^3
	temp = temp - (4.8260e-6 * rpoly)
	rpoly = rpoly * Rt // ^4
	temp = temp - (2.8183e-8 * rpoly)
	rpoly = rpoly * Rt // ^5
	temp = temp + (1.5243e-10 * rpoly)

	return temp, nil
}

func (d *Dev) sensingContinuous(interval time.Duration, sensing chan<- physic.Env, stop <-chan struct{}) {
	t := time.NewTicker(interval)
	defer t.Stop()

	var err error
	for {
		// Do one initial sensing right away.
		e := physic.Env{}
		d.mu.Lock()
		err = d.sense(&e)
		d.mu.Unlock()
		if err != nil {
			log.Printf("%s: failed to sense: %v", d, err)
			return
		}
		select {
		case sensing <- e:
		case <-stop:
			return
		}
		select {
		case <-stop:
			return
		case <-t.C:
		}
	}
}

func (d *Dev) clearFault() error {
	var b [1]byte
	if err := d.readReg(configReg, b[:]); err != nil {
		return d.wrap(err)
	}
	b[0] &= ^uint8(0x2C)
	b[0] |= configFaultStat

	return d.writeCommands(b[:])
}

func (d *Dev) enableBias(b bool) error {
	var err error
	if b {
		d.writeConfig(configBias)
	} else {
		d.writeConfig(configBias, true)
	}

	return err
}

func (d *Dev) readFault() uint8 {
	var b [1]byte
	d.readReg(faultStatReg, b[:])
	return b[0]
}

func (d *Dev) readReg(reg uint8, b []byte) error {
	// MSB is 0 for write and 1 for read.
	read := make([]byte, len(b)+1)
	write := make([]byte, len(read))

	// Rest of the write buffer is ignored.
	write[0] = reg
	if err := d.d.Tx(write, read); err != nil {
		return d.wrap(err)
	}
	copy(b, read[1:])

	return nil
}

// writeConfig writes a config setting to the device.
//
// Warning: b may be modified!
func (d *Dev) writeConfig(b byte, invert ...bool) error {
	var config [1]byte
	if err := d.readReg(configReg, config[:]); err != nil {
		return d.wrap(err)
	}

	if len(invert) > 0 && invert[0] {
		config[0] &= ^b
	} else {
		config[0] |= b
	}
	if err := d.writeCommands(config[:]); err != nil {
		return d.wrap(err)
	}

	return nil
}

// writeCommands writes a command to the device.
//
// Warning: b may be modified!
func (d *Dev) writeCommands(b []byte) error {
	for i := 0; i < len(b); i += 2 {
		b[i] |= 0x80
	}

	if err := d.d.Tx(b, nil); err != nil {
		return d.wrap(err)
	}

	return nil
}

func (d *Dev) wrap(err error) error {
	return fmt.Errorf("%s: %v", strings.ToLower(d.name), err)
}

var _ conn.Resource = &Dev{}
var _ physic.SenseEnv = &Dev{}
