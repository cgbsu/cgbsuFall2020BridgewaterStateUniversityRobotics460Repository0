package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
	// "math"
)

/*
	* : Robot will start within a foot of the box, approximatley perpendicular.
	* : Robot will try to maintain at least an inch's distance from the box
	* : First goal, measure a side of the box.
	* : Second goal, measure full box.
*/

const StartDistanceConstant = 20//29.4
// const OneFootInCentimetersRoundUpConstant = 30

func UniformMove( gopigo3 *g.Driver, dps int ) {
	gopigo3.SetMotorDps( g.MOTOR_LEFT, dps )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, dps )
}

func Move( gopigo3 *g.Driver, leftDps int, rightDps int ) {
	gopigo3.SetMotorDps( g.MOTOR_LEFT, leftDps )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, rightDps )
}

func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {

	value, err :=  gopigo3.GetBatteryVoltage()
	fmt.Println( "Voltage: ", value )
	if err {
		fmt.Println( "error: ", err )
	}

	err := lidarSensor.Start()
	//sideLength := 0.0
	initialized := false
	goalDistance  := 0
	const Initialspeed = -180
	const InitialMeasuringspeed = -10
	//In Centimeters//
	const RobotWidthConstant = 13
	const RobotWheelRadiusConstant = 6.5
	//leftMeasuringSpeed := InitialMeasuringspeed
	//rightMeasuringSpeed := InitialMeasuringspeed
	//measuredInitialSpeed := false
	previousSpeed := 0
	previousDistance := 0
	foundBox := false
	initialSample := 0
	const MaxInitializationSamplesConstant = 10

	const OutOfBoundsDistanceConstant = 30
	const MaxOutOfBoundSamplesConstant = 5
	outOfBoundSamples := 0
	outOfBoundsAverage := 0

	wentOutOfBounds := false

	if err != nil {
		fmt.Println("error starting lidarSensor")
	}
	gobot.Every( time.Millisecond, func() { //loop forever
		lidarReading, err := lidarSensor.Distance()
		if err != nil {
			fmt.Println("Error reading lidar sensor %+v", err)
		}
		if lidarReading < StartDistanceConstant {
			foundBox = true
		}
		if foundBox == false {
			UniformMove( gopigo3, Initialspeed )
		} else if foundBox == true {
			if initialized == false {
				goalDistance += lidarReading
				UniformMove( gopigo3, InitialMeasuringspeed )
				previousSpeed = -180
				previousDistance = lidarReading
				initialSample += 1
				if initialSample >= MaxInitializationSamplesConstant {
					goalDistance /= initialSample
					initialized = true
				}
			} else if wentOutOfBounds == false {
				if lidarReading > goalDistance {
					fmt.Println( "Greater lr: ", lidarReading, " gd: ", goalDistance )
					outOfBoundSamples += 1
					outOfBoundsAverage += lidarReading
					Move( gopigo3, -10, -5 )
				} else if outOfBoundSamples < MaxOutOfBoundSamplesConstant {
					outOfBoundSamples = 0
					outOfBoundsAverage = 0
					if lidarReading < goalDistance {
						fmt.Println( "Less lr: ", lidarReading, " gd: ", goalDistance )
						Move( gopigo3, -5, -10 )
					} else {
						UniformMove( gopigo3, -10 )
					}
				} else if ( outOfBoundsAverage / outOfBoundSamples ) >= OutOfBoundsDistanceConstant {
					wentOutOfBounds = true
					fmt.Println( "OUT OF BOUNDS 1" )
				}
			} else {
				fmt.Println( "OUT OF BOUNDS 2 ", ( outOfBoundsAverage / outOfBoundSamples ) )
			}
		}
	} )
}

func main() {
	raspberryPi := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspberryPi)
	lidarSensor := i2c.NewLIDARLiteDriver(raspberryPi)
	lightSensor := aio.NewGroveLightSensorDriver(gopigo3, "AD_2_1")
	workerThread := func() {
		RobotMainLoop(raspberryPi, gopigo3, lidarSensor)
	}
	robot := gobot.NewRobot("Gopigo Pi4 Bot",
		[]gobot.Connection{raspberryPi},
		[]gobot.Device{gopigo3, lidarSensor, lightSensor},
		workerThread)

	robot.Start()
}
