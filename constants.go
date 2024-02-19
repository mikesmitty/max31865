package max31865

type RTDType int

const (
	RTDPT100 RTDType = iota
	RTDPT1000
)

type WireCount int

const (
	WireCount2 WireCount = iota
	WireCount3
	WireCount4
)

// Magic numbers for computing temperature
const (
	rtdA float64 = 3.9083e-3
	rtdB float64 = -5.775e-7
)

const (
	configReg uint8 = iota
	rtdMsbReg
	rtdLsbReg
	hFaultMsbReg
	hFaultLsbReg
	lFaultMsbReg
	lFaultLsbReg
	faultStatReg
)

const (
	faultHighThresh uint8 = 0x80
	faultLowThresh  uint8 = 0x40
	faultRefInLow   uint8 = 0x20
	faultRefInHigh  uint8 = 0x10
	faultRtdInLow   uint8 = 0x08
	faultOvUv       uint8 = 0x04
)

const (
	configBias      uint8 = 0x80
	configModeAuto  uint8 = 0x40
	configModeOff   uint8 = 0x00
	config1Shot     uint8 = 0x20
	config24Wire    uint8 = 0x00
	config3Wire     uint8 = 0x10
	configFaultStat uint8 = 0x02
	configFilt50Hz  uint8 = 0x01
	configFilt60Hz  uint8 = 0x00
)
