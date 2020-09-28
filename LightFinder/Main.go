package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
	"math"
)

func CircleLeft( gopigo3 *g.Driver, howFast int ) {
	howFast = int( math.Abs( float64( howFast ) ) )
	gopigo3.SetMotorDps( g.MOTOR_LEFT, howFast )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, -howFast )
}

func CircleRight( gopigo3 *g.Driver, howFast int ) {
	howFast = int( math.Abs( float64( howFast ) ) )
	gopigo3.SetMotorDps( g.MOTOR_LEFT, -howFast )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, howFast )
}


func robotRunLoop(gopigo3 *g.Driver, lightSensors [ 2 ]*aio.GroveLightSensorDriver, reachedDestinationCount *int) {
	gobot.Every( time.Millisecond, func() {
		const WaitCountConstant = 10
		if ( *reachedDestinationCount >= WaitCountConstant ) {
			TolerenceConstant := 3
			DestinationDataRageConstant := 3050
			gopigo3.SetLED( g.LED_EYE_RIGHT, 0, 0, 0 )
			gopigo3.SetLED( g.LED_EYE_LEFT, 0, 0, 0 )
			sensor0Data, error0 := lightSensors[ 0 ].Read()
			sensor1Data, error1 := lightSensors[ 1 ].Read()
			if error0 != nil {
				fmt.Errorf( "Error reading sensor0 %+v", error0 )
			}
			if error1 != nil {
				fmt.Errorf( "Error reading sensor1 %+v", error1 )
			}
				sensorDifference := int( math.Abs( float64( sensor0Data - sensor1Data ) ) )
			if ( sensor0Data < sensor1Data && sensorDifference > TolerenceConstant ) || ( sensor0Data < 1000 && sensor1Data < 1000 ) {
				CircleLeft( gopigo3, 10 )
			} else if ( sensor0Data > sensor1Data && sensorDifference > TolerenceConstant ) {
				CircleRight( gopigo3, 10 )
			} else {
				gopigo3.SetMotorDps( g.MOTOR_LEFT, 180 )
				gopigo3.SetMotorDps( g.MOTOR_RIGHT, 180 )
			}
			if ( sensor0Data >= DestinationDataRageConstant && 
					sensor1Data >= DestinationDataRageConstant ) {
				fmt.Println( "Meep" )
				*reachedDestinationCount += 1
			} else {
				*reachedDestinationCount = 0
			}
			//fmt.Println( "Sensors: ", sensor0Data, sensor1Data )
		} else {
			gopigo3.SetLED( g.LED_EYE_RIGHT, 0, uint8 ( *reachedDestinationCount - WaitCountConstant ), 0 )
			gopigo3.SetLED( g.LED_EYE_LEFT, 0, uint8 ( *reachedDestinationCount - WaitCountConstant ), 0 )
			if ( *reachedDestinationCount > WaitCountConstant ) {
				*reachedDestinationCount = WaitCountConstant
			} else {
				*reachedDestinationCount = WaitCountConstant + 1
			}
		}
	} )
}

func Stop( gopigo3 *g.Driver ) {
	gopigo3.SetMotorDps( g.MOTOR_LEFT, 0 )
	gopigo3.SetMotorDps( g.MOTOR_RIGHT, 0 )
}

func main() {
	raspiAdaptor := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdaptor)
	reachedDestinationCount := 0
	//AnalogDigital Port 1 is "AD_1_1" this is port 2
	lightSensors := [ 2 ]*aio.GroveLightSensorDriver{ aio.NewGroveLightSensorDriver( gopigo3, "AD_1_1" ), 
			aio.NewGroveLightSensorDriver( gopigo3, "AD_2_1" ) }
	
	mainRobotFunc := func() {
		robotRunLoop( gopigo3, lightSensors, &reachedDestinationCount )
	}


	//this is the crux of the gobot framework. The factory function to create a new robot
	//struct (go uses structs and not objects) It takes four parameters
	robot := gobot.NewRobot("gopigo3sensorChecker", //first a name
		[]gobot.Connection{raspiAdaptor}, //next a slice of connections to one or more robot controllers
		[]gobot.Device{gopigo3, lightSensors[ 0 ], lightSensors[ 1 ] }, //next a slice of one or more sensors and actuators for the robots
		mainRobotFunc, //the variable holding the function to run in a new thread as the main function
	)

	robot.Start() //actually run the function
	robot.Stop()
}