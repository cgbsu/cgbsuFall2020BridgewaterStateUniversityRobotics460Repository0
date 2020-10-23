package main

import (
	"gobot.io/x/gobot"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	// "math"
)

func main() {
	raspberryPi := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspberryPi)
	workerThread := func() {
		gopigo3.SetMotorDps( g.MOTOR_LEFT, 0 )
		gopigo3.SetMotorDps( g.MOTOR_RIGHT, 0 )	
	}
	robot := gobot.NewRobot("Gopigo Pi4 Bot",
		[]gobot.Connection{raspberryPi},
		[]gobot.Device{gopigo3},
		workerThread)
	robot.Start()
	robot.Stop()
}