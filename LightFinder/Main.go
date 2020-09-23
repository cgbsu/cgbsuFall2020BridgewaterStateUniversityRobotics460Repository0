package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	//"time"
)

func robotRunLoop(gopigo3 *g.Driver, lightSensors [ 2 ]*aio.GroveLightSensorDriver) {
	for {
		sensor0Data, error0 := lightSensors[ 0 ].Read()
		sensor1Data, error1 := lightSensors[ 1 ].Read()
		if error0 != nil {
			fmt.Errorf( "Error reading sensor0 %+v", error0 )
		}
		if error1 != nil {
			fmt.Errorf( "Error reading sensor1 %+v", error1 )
		}
		if ( sensor0Data - sensor1Data ) < 10 || ( sensor0Data < 1000 && sensor1Data < 1000 ) {
			gopigo3.SetMotorDps( g.MOTOR_LEFT, 30 )
			gopigo3.SetMotorDps( g.MOTOR_RIGHT, -30 )
		} else if ( sensor1Data - sensor0Data ) < 10 {
			gopigo3.SetMotorDps( g.MOTOR_LEFT, -30 )
			gopigo3.SetMotorDps( g.MOTOR_RIGHT, 30 )
		} else {
			gopigo3.SetMotorDps( g.MOTOR_LEFT, 180 )
			gopigo3.SetMotorDps( g.MOTOR_RIGHT, 180 )
		}
		fmt.Println( sensor0Data )
		//fmt.Println( sensor1Data )
	}
}

func main() {
	raspiAdaptor := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdaptor)
	//AnalogDigital Port 1 is "AD_1_1" this is port 2
	lightSensors := [ 2 ]*aio.GroveLightSensorDriver{ aio.NewGroveLightSensorDriver( gopigo3, "AD_1_1" ), 
			aio.NewGroveLightSensorDriver( gopigo3, "AD_2_1" ) }
	
	mainRobotFunc := func() {
		robotRunLoop( gopigo3, lightSensors )
	}


	//this is the crux of the gobot framework. The factory function to create a new robot
	//struct (go uses structs and not objects) It takes four parameters
	robot := gobot.NewRobot("gopigo3sensorChecker", //first a name
		[]gobot.Connection{raspiAdaptor}, //next a slice of connections to one or more robot controllers
		[]gobot.Device{gopigo3, lightSensors[ 0 ], lightSensors[ 1 ] }, //next a slice of one or more sensors and actuators for the robots
		mainRobotFunc, //the variable holding the function to run in a new thread as the main function
	)

	robot.Start() //actually run the function
}