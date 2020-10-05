package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
)

func robotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver,

/*lcd *i2c.GroveLcdDriver*/) {
	//err := lcd.Start()
	//if err != nil {
	//	fmt.Println("error starting lcd")
	//}
	err := lidarSensor.Start()
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}
	//err = lcd.SetRGB(100, 0, 0)
	//if err != nil {
	//	fmt.Println("Error setting lcd color")
	//}
	for { //loop forever
		lidarReading, err := lidarSensor.Distance()
		if err != nil {
			fmt.Println("Error reading lidar sensor %+v", err)
		}
		message := fmt.Sprintf("Lidar Reading: %d", lidarReading)
		//err = lcd.Clear()
		//if err != nil {
		//	fmt.Println("error clearing lcd")
		//}
		//err = lcd.Home()
		//if err != nil {
		//	fmt.Println("error homeing lcd")
		//}
		//err = lcd.Write(message)
		//if err != nil {
		//	fmt.Println("error writing to LCD")
		//}
		fmt.Println(lidarReading)
		fmt.Println(message)
		time.Sleep(time.Second * 3)
	}
}

func main() {
	raspberryPi := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspberryPi)
	lidarSensor := i2c.NewLIDARLiteDriver(raspberryPi)
	//lcd := i2c.NewGroveLcdDriver(raspberryPi)
	lightSensor := aio.NewGroveLightSensorDriver(gopigo3, "AD_2_1")
	workerThread := func() {
		robotMainLoop(raspberryPi, gopigo3, lidarSensor /*lcd*/)
	}
	robot := gobot.NewRobot("Gopigo Pi4 Bot",
		[]gobot.Connection{raspberryPi},
		[]gobot.Device{gopigo3, lidarSensor /*lcd,*/, lightSensor},
		workerThread)

	robot.Start()

}
