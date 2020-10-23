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

//In Centimeters//
const RobotWidthConstant = 13
const RobotWheelRadiusConstant = 6.5

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

//Calculates a robot wheel moving at a given dps will travel.//
func DpsToDistance( dps int ) float64 {
	return float64( dps ) * ( 2.0 * math.Pi * 
			RobotWheelRadiusConstant / 360.0 )
}


//Calculates the arclength the robot will travel given 2 dps's//
func CalculateArcData( leftDps, rightDps int ) ( float64, float64 ) {
	leftDistance := float64( DpsToDistance( leftDps ) )
	rightDistance := float64( DpsToDistance( rightDps ) )
	sidePolarity := 0.0
	if sidePolarity = -1.0; leftDistance > rightDistance {
		sidePolarity = 1.0
	}
	maxDistance := math.Max( leftDistance, rightDistance )
	theta := math.Atan( ( leftDistance - rightDistance ) / RobotWidthConstant )
	magnitude := math.Sqrt( - math.Pow( maxDistance, 2.0 ) / ( math.Pow( math.Cos( theta ), 2.0 ) - 1 ) )
	radius := math.Cos( theta ) * sidePolarity * magnitude
	// fmt.Println( "leftDistance ", leftDistance, " rightDistance ", rightDistance, " theta ", theta, " magnitude ", magnitude, " radius ", radius )
	return radius, theta
}

type Average struct {
	 buffer, samples, desiredSampleCount int 
}

func ( self *Average ) InitializeAverage( desiredSampleCount int ) {
	self.desiredSampleCount = desiredSampleCount
}

func ( self *Average ) CalculateAverage() int {
	if self.samples == 0 {
		return 0
	}
	return self.buffer / self.samples
}

func ( self *Average ) AddSample( sample int ) {
	self.buffer += sample
	self.samples += 1
}

func ( self *Average ) Clear() {
	self.buffer = 0
	self.samples = 0
}

func ( self *Average ) AtDesiredSampleCount() bool {
	if self.desiredSampleCount == 0 {
		return true
	}
	return self.samples >= self.desiredSampleCount
}

const MaxInitializationSamplesConstant = 10
const OutOfBoundsDistanceConstant = 50
const MaxOutOfBoundSamplesConstant = 5
const CornerTurnAngleConstant = math.Pi / 2.0//90.0

type Side struct { 
	foundBox, goalDistanceFound, measuredSide bool
	goalDistanceCalculator, outOfBoundsDistance Average
	goalDistance, initialSpeed, initialMeasuringSpeed int
	cornerTurnAngle float64
}

func ( self *Side ) InitializeSide( initialSpeed, initialMeasuringSpeed int ) {
	self.goalDistanceCalculator.InitializeAverage( MaxInitializationSamplesConstant )
	self.outOfBoundsDistance.InitializeAverage( MaxOutOfBoundSamplesConstant )
	self.initialSpeed = initialSpeed
	self.initialMeasuringSpeed = initialMeasuringSpeed
}

func ( self *Side ) MeasureInitialDistance( gopigo3 *g.Driver, lidarReading int ) bool {
	if self.foundBox == true {
		if self.goalDistanceFound == false {
			self.goalDistanceCalculator.AddSample( lidarReading )
			if self.goalDistanceCalculator.AtDesiredSampleCount() {
				self.goalDistance = self.goalDistanceCalculator.CalculateAverage()
				self.goalDistanceFound = true
			}
			UniformMove( gopigo3, self.initialMeasuringSpeed )
		}
	} else if lidarReading < StartDistanceConstant {
		self.foundBox = true
	} else {
		UniformMove( gopigo3, self.initialSpeed )
	}
	return self.goalDistanceFound
}

func ( self *Side ) UpdateCornerTurnAngle( leftDps, rightDps int, loopRuntimeInSeconds float64 ) bool {
	_, angle := CalculateArcData( leftDps, rightDps )
	// fmt.Println( "Updating corner angle with angle ", angle, " loopRuntimeInSeconds", loopRuntimeInSeconds, " self.cornerTurnAngle", self.cornerTurnAngle )
	self.cornerTurnAngle += ( angle / loopRuntimeInSeconds )
	return self.TurnedCorner()
}

func ( self *Side ) ClearCornerTurnAngle() {
	self.cornerTurnAngle = 0.0
}

func ( self *Side ) TurnedCorner() bool {
	return math.Abs( self.cornerTurnAngle ) >= CornerTurnAngleConstant
}

func ( self* Side ) Creep( lidarReading int, gopigo3 *g.Driver, loopRuntimeInSeconds float64 ) {
	if lidarReading > self.goalDistance {
		fmt.Println( "Greater lr: ", lidarReading, " gd: ", self.goalDistance )
		Move( gopigo3, -100, -50 )
		self.UpdateCornerTurnAngle( -100, -50, loopRuntimeInSeconds )
	} else {
		self.ClearCornerTurnAngle()
		if lidarReading < self.goalDistance {
			fmt.Println( "Less lr: ", lidarReading, " gd: ", self.goalDistance )
			Move( gopigo3, -50, -100 )
		} else {
			UniformMove( gopigo3, -100 )
		}
	}
}

func ( self *Side ) MeasureSide( gopigo3 *g.Driver, lidarReading int, loopRuntimeInSeconds float64 ) bool {
	fmt.Println( "Turned Corner: ", self.TurnedCorner(), " Corner Turn Angle: ", self.cornerTurnAngle )
	if self.outOfBoundsDistance.AtDesiredSampleCount() == false && self.TurnedCorner() == false {
		if lidarReading >= OutOfBoundsDistanceConstant {
			self.outOfBoundsDistance.AddSample( lidarReading )
		} else {
			self.outOfBoundsDistance.Clear()
			self.Creep( lidarReading, gopigo3, loopRuntimeInSeconds )
		}
	} else if self.outOfBoundsDistance.CalculateAverage() >= OutOfBoundsDistanceConstant || self.TurnedCorner() == true {
		self.measuredSide = true
	} else {
		fmt.Println( "WHAT DO I DO!?" )
		self.outOfBoundsDistance.Clear()
		UniformMove( gopigo3, -360 )
	}
	return self.measuredSide
}

func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {
	const InitialSpeed = -180
	const InitialMeasuringSpeed = -10
	const LoopRuntimeConstant = time.Millisecond
	//const LoopTimeInSecondsConstant = 1.0

	var currentSide Side
	var previousTime time.Time
	// var work *time.Ticker

	firstLoop := false
	voltage, voltageErr := gopigo3.GetBatteryVoltage()

	currentSide.InitializeSide( InitialSpeed, InitialMeasuringSpeed )
	fmt.Println( "Voltage: ", voltage )
	if voltageErr != nil {
		fmt.Println( "RobotMainLoop::Error::Failure reading Voltage: ", voltageErr )
	}
	err := lidarSensor.Start()
	if err != nil {
		fmt.Println( "RobotMainLoop::Error::Failure starting Lidar Sensor: ", err )
	}



	gobot.Every( time.Millisecond, func() {
		lidarReading, err := lidarSensor.Distance()
		if err != nil {
			fmt.Println( "RobotMainLoop::Error::Failure reading Lidar Sensor: ", err )
		}
		if currentSide.goalDistanceFound == false {
			currentSide.MeasureInitialDistance( gopigo3, lidarReading )
		} else if currentSide.measuredSide == false {
			if firstLoop {
				previousTime = time.Now()
				firstLoop = true
			}
			deltaTime := time.Since( previousTime ).Seconds()
			fmt.Println( "Delta Time ", deltaTime )
			currentSide.MeasureSide( gopigo3, lidarReading, deltaTime ) //, LoopTimeInSecondsConstant )
			previousTime = time.Now()
		} else {
			fmt.Println( "Success!" )
			gopigo3.Halt()
		}
	} )
	defer func() {
		//work.Stop()
		gopigo3.Halt()
	}()
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
	defer func() { 
		UniformMove( gopigo3, 0 )
		robot.Stop()
	}()
}
