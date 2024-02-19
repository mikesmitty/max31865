package max31865

type RTDType int

const (
	RTDPT100 RTDType = iota
	RTDPT1000
)

type WireCount int

const (
	WireCount2 WireCount = iota + 2
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
	configFlagFilter50Hz        uint8 = iota // D0 in the datasheet
	configFlagFaultClear                     // D1
	configFlagFaultDetectCycleB              // D2
	configFlagFaultDetectCycleA              // D3
	configFlagWireCount                      // D4
	configFlagOneShot                        // D5
	configFlagContinuousMode                 // D6
	configFlagBiasVoltage                    // D7
)
