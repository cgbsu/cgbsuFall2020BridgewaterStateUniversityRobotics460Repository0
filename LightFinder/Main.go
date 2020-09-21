package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
)

func robotRunLoop(gopigo3 *g.Driver, lightSensor *aio.GroveLightSensorDriver) {
	for {
		sensorVal, err := lightSensor.Read()
		if err != nil {
			fmt.Errorf("Error reading sensor %+v", err)
		} else {
			if sensorVal >= 1000 {
				gopigo3.SetMotorDps( g.MOTOR_LEFT, 180 )
				gopigo3.SetMotorDps( g.MOTOR_RIGHT, -180 )
			} else {
				gopigo3.SetMotorDps( g.MOTOR_LEFT, 0 )
				gopigo3.SetMotorDps( g.MOTOR_RIGHT, 0 )
			}
		}
		time.Sleep(time.Second)
	}
}

func main() {
	raspiAdaptor := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdaptor)
	lightSensor:= aio.NewGroveLightSensorDriver(gopigo3, "AD_2_1") //AnalogDigital Port 1 is "AD_1_1" this is port 2

	mainRobotFunc := func() {
		robotRunLoop(gopigo3, lightSensor)
	}


	//this is the crux of the gobot framework. The factory function to create a new robot
	//struct (go uses structs and not objects) It takes four parameters
	robot := gobot.NewRobot("gopigo3sensorChecker", //first a name
		[]gobot.Connection{raspiAdaptor}, //next a slice of connections to one or more robot controllers
		[]gobot.Device{gopigo3, lightSensor}, //next a slice of one or more sensors and actuators for the robots
		mainRobotFunc, //the variable holding the function to run in a new thread as the main function
	)

	robot.Start() //actually run the function
}