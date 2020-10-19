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
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}

	turnActive := false
	turnCount := 0
	const MaxTurnCount = 9

	gobot.Every( time.Millisecond, func() { //loop forever
		lidarReading, err := lidarSensor.Distance()
		if err != nil {
			fmt.Println("Error reading lidar sensor %+v", err)
		}
		//fmt.Printf("Lidar Reading: %d\n", lidarReading)
		/*
		fmt.Println(lidarReading)
		fmt.Println(message)
		time.Sleep(time.Second * 3)*/

		if lidarReading < OneFootInCentimetersRoundUpConstant {
			if initialized == false {
				//fmt.Println( "A" )
				goalDistance = lidarReading
				UniformMove( gopigo3, InitialMeasuringspeed )
				previousSpeed = InitialMeasuringspeed
				previousDistance = lidarReading
				initialized = true
			} else {
				// fmt.Println( "B" )
				UniformMove( gopigo3, InitialMeasuringspeed )				
			}
		} else if initialized == false {
			// fmt.Println( "C" )
			UniformMove( gopigo3, Initialspeed )
		} else if turnCount < MaxTurnCount {
			// fmt.Println( "D" )
			Move( 10, -10 )
			turnCount += 1
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