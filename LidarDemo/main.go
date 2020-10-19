package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
	"math"
)

/*
	* : Robot will start within a foot of the box, approximatley perpendicular.
	* : Robot will try to maintain at least an inch's distance from the box
	* : First goal, measure a side of the box.
	* : Second goal, measure full box.
*/

const OneFootInCentimetersConstant = 29.4
const OneFootInCentimetersRoundUpConstant = 30

func UniformMove( gopigo3 *g.Driver, dps int ) {
	gopigo3.SetMotorDps( g.MOTOR_LEFT, dps )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, dps )
}

func Move( gopigo3 *g.Driver, leftDps int, rightDps int ) {
	gopigo3.SetMotorDps( g.MOTOR_LEFT, leftDps )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, rightDps )
}

func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {
	err := lidarSensor.Start()
	sideLength := 0.0
	initialized := false
	goalDistance  := 0
	const Initialspeed = -180
	const InitialMeasuringspeed = -10
	//In Centimeters//
	const RobotWidthConstant = 13
	const RobotWheelRadiusConstant = 6.5
	leftMeasuringSpeed := InitialMeasuringspeed
	rightMeasuringSpeed := InitialMeasuringspeed
	measuredInitialSpeed := false
	previousSpeed := 0
	previousDistance := 0
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}

	gobot.Every( time.Millisecond, func() { //loop forever
		lidarReading, err := lidarSensor.Distance()
		if err != nil {
			fmt.Println("Error reading lidar sensor %+v", err)
		}
		fmt.Printf("Lidar Reading: %d\n", lidarReading)/*
		fmt.Println(lidarReading)
		fmt.Println(message)
		time.Sleep(time.Second * 3)*/

		if lidarReading < OneFootInCentimetersRoundUpConstant {
			if initialized == false {
				goalDistance = lidarReading
				UniformMove( gopigo3, InitialMeasuringspeed )
				previousSpeed = InitialMeasuringspeed
				previousDistance = lidarReading
				initialized = true
			} else {
				//Calculate where we want to go//
				deltaY := float64( previousDistance - lidarReading )
				deltaX := math.Sqrt( math.Pow( float64( previousSpeed ), 2.0 ) - math.Pow( deltaY, 2.0 ) )
				theta := math.Arctan( deltaY / deltaX )
				speed := math.Sqrt( math.Pow( deltaX, 2.0 ) + math.Pow( deltaY, 2.0 ) )
				//Calculate arc radius//
				leftWheelLength := speed * math.Sin( theta )
				rightWheelLength := speed * math.Cos( theta )
				/*const DeltaLengthConstant = leftWheelLength - rightWheelLength
				greaterLength := leftWheelLength 
				sideScaler := -1
				if leftWheelLength <= rightWheelLength {
					greaterLength = rightWheelLength
					sideScaler = 1
				}
				const Sidetheta = math.Arctan( DeltaLengthConstant / RobotWidthConstant )
				const SideOffsetMagnitudeConstant = math.Sqrt( - math.Pow( greaterLength, 2 ) / ( math.Pow( math.Cos( Sidetheta ), 2 ) - 1.0 ) )
				const RadiusConstant = math.Cos( Sidetheta ) * sideScaler * SideOffsetMagnitudeConstant */

				//Calculate Dps for each wheel//
				const LeftWheelDpsConstant = int( leftWheelLength / RobotWheelRadiusConstant )
				const RightWheelDpsConstant = int( rightWheelLength / RobotWheelRadiusConstant )
				Move( gopigo3, LeftWheelDpsConstant, RightWheelDpsConstant )
			}
		} else if initialized == false {
			UniformMove( gopigo3, Initialspeed )
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
