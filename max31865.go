package max31865

import (
	"errors"
	"fmt"
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
	// ContinuousMode removes delays by leaving the bias voltage enabled between
	// readings. This slightly increases power consumption and self-heating, but
	// is useful when taking measurements as quickly as possible (< 55/66ms)
	ContinuousMode bool
	// 60Hz noise is filtered by default, enable to filter 50Hz noise instead.
	Filter50Hz  bool
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

	if opts == nil {
		opts = DefaultOptions()
	}

	d := &Dev{
		d:    c,
		opts: *opts,
		name: p.String(),
	}

	switch {
	case opts.Filter50Hz && opts.ContinuousMode:
		d.measDelay = 21 * time.Millisecond
	case opts.ContinuousMode:
		d.measDelay = 18 * time.Millisecond
	case opts.Filter50Hz:
		d.measDelay = 66 * time.Millisecond
	default:
		d.measDelay = 55 * time.Millisecond
	}

	switch opts.RTDType {
	case RTDPT100:
		d.rtdNominal = 100.0
	case RTDPT1000:
		d.rtdNominal = 1000.0
	default:
		return nil, fmt.Errorf("max31865: invalid RTD type: %v", opts.RTDType)
	}

	// Clear any existing fault flags
	if err := d.clearFault(); err != nil {
		return nil, d.wrap(err)
	}

	// Disable bias voltage when not in use to slightly reduce self-heating
	if err := d.setConfigFlag(configFlagBiasVoltage, d.opts.ContinuousMode); err != nil {
		return nil, d.wrap(err)
	}

	// Enable/disable hardware continuous mode
	if err := d.setConfigFlag(configFlagContinuousMode, d.opts.ContinuousMode); err != nil {
		return nil, d.wrap(err)
	}

	// Filter 50Hz/60Hz noise (defaults to 60Hz)
	if err := d.setConfigFlag(configFlagFilter50Hz, d.opts.Filter50Hz); err != nil {
		return nil, d.wrap(err)
	}

	// Set wire count (either 3 or 2/4)
	if err := d.setConfigFlag(configFlagWireCount, opts.WireCount == WireCount3); err != nil {
		return nil, d.wrap(err)
	}

	// Set resistance error thresholds
	if err := d.SetThreshold(0x0000, 0xFFFF); err != nil {
		return nil, d.wrap(err)
	}

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

	return d.sense(e, d.opts.ContinuousMode)
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

func (d *Dev) sense(e *physic.Env, continuousMode bool) error {
	d.clearFault()

	if !continuousMode {
		// Enable bias voltage and wait 10ms to stabilize
		if err := d.setConfigFlag(configFlagBiasVoltage, true); err != nil {
			return d.wrap(err)
		}
		time.Sleep(10 * time.Millisecond)

		// Trigger one-shot reading
		if err := d.setConfigFlag(configFlagOneShot, true); err != nil {
			return d.wrap(err)
		}

		time.Sleep(d.measDelay)
	}

	var result [2]byte
	if err := d.readReg(rtdMsbReg, result[:]); err != nil {
		return d.wrap(err)
	}

	if !continuousMode {
		// Disable bias current again to reduce self-heating.
		if err := d.setConfigFlag(configFlagBiasVoltage, false); err != nil {
			return d.wrap(err)
		}
	}

	temp := d.parseTemperature(result[:])
	e.Temperature = physic.Temperature(temp*1000)*physic.MilliCelsius + physic.ZeroCelsius

	if err := d.CheckError(); err != nil {
		return d.wrap(err)
	}

	return nil
}

func (d *Dev) parseTemperature(data []byte) float64 {
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
		return temp
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

	return temp
}

func (d *Dev) sensingContinuous(interval time.Duration, sensing chan<- physic.Env, stop <-chan struct{}) {
	// Ensure the interval is at least the minimum measurement delay.
	if interval < d.measDelay {
		interval = d.measDelay
	}
	t := time.NewTicker(interval)
	defer t.Stop()

	var err error
	for {
		// Do one initial sensing right away.
		e := physic.Env{}
		d.mu.Lock()
		err = d.sense(&e, d.opts.ContinuousMode)
		d.mu.Unlock()
		if err != nil {
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

func (d *Dev) CheckError() error {
	fault, err := d.readFault()
	if err != nil {
		return err
	}
	switch {
	case d.getFlagBit(fault, 7):
		return fmt.Errorf("fault detected (%#0x): rtd resistance value > high threshold register", fault)
	case d.getFlagBit(fault, 6):
		return fmt.Errorf("fault detected (%#0x): rtd resistance value < low threshold register", fault)
	}
	return nil
}

func (d *Dev) clearFault() error {
	var b [2]byte
	if err := d.readReg(configReg, b[1:]); err != nil {
		return d.wrap(err)
	}

	// Clear bits 5, 3, 2 (one-shot plus fault status bits) and set bit 1 to clear fault
	config := b[1] | byte(0x2C)
	config |= (1 << configFlagFaultClear)
	b[1] = config

	return d.writeCommands(b[:])
}

func (d *Dev) readFault() (uint8, error) {
	var b [1]byte
	err := d.readReg(faultStatReg, b[:])
	if err != nil {
		return b[0], d.wrap(err)
	}
	return b[0], nil
}

func (d *Dev) readReg(reg uint8, b []byte) error {
	read := make([]byte, len(b)+1)
	write := make([]byte, len(read))

	write[0] = reg & 0x7F
	if err := d.d.Tx(write, read); err != nil {
		return d.wrap(err)
	}
	copy(b, read[1:])

	return nil
}

func (d *Dev) SetThreshold(lower, upper uint16) error {
	if err := d.writeCommands([]byte{lFaultLsbReg, byte(lower & 0xFF)}); err != nil {
		return err
	}
	if err := d.writeCommands([]byte{lFaultMsbReg, byte(lower >> 8)}); err != nil {
		return err
	}
	if err := d.writeCommands([]byte{hFaultLsbReg, byte(upper & 0xFF)}); err != nil {
		return err
	}
	return d.writeCommands([]byte{hFaultMsbReg, byte(upper >> 8)})
}

// Gets a config flag bit.
func (d *Dev) getFlagBit(b uint8, flag uint8) bool {
	val := b & (1 << flag)
	return (val > 0)
}

// Sets a config flag bit to on or off.
func (d *Dev) setConfigFlag(flag uint8, on bool) error {
	var result [1]byte
	if err := d.readReg(configReg, result[:]); err != nil {
		return d.wrap(err)
	}

	newConfig := result[0]
	if on {
		newConfig |= (1 << flag)
	} else {
		newConfig &= ^(1 << flag)
	}
	var config [2]byte
	config[0] = configReg
	config[1] = newConfig
	return d.writeCommands(config[:])
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
