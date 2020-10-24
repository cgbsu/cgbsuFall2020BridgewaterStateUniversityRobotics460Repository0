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


type QuantativeDirection int

const( 
	Left QuantativeDirection = 0
	Right = 1
	Forward = 2
)

type Robot struct {
	gopigo3 *g.Driver
	lidarSensor *i2c.LIDARLiteDriver
	lidarReading, leftDps, rightDps int
	lastDirection, currentDirection QuantativeDirection
	startTimeTravelingWithDps time.Time
}

func NewRobot( gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) *Robot {
	robot := new( Robot )
	robot.gopigo3 = gopigo3
	robot.lidarSensor = lidarSensor
	err := robot.lidarSensor.Start()
	if err != nil {
		fmt.Println( "NewRobot::Error::Failure starting Lidar Sensor: ", err )
	}
	return robot
}

//In Centimeters//
const RobotWidthConstant = 13
const RobotWheelRadiusConstant = 6.5

const StartDistanceConstant = 20//29.4
// const OneFootInCentimetersRoundUpConstant = 30

func ( self *Robot ) UniformMove( dps int ) bool {
	self.gopigo3.SetMotorDps( g.MOTOR_LEFT, dps )
	self.gopigo3.SetMotorDps( g.MOTOR_RIGHT, dps )
	self.leftDps = dps
	self.rightDps = dps
	self.startTimeTravelingWithDps = time.Now()
	return self.ChangeDirection( Forward )
}

func Abs( number int ) int {
	if number < 0 {
		return -number
	}
	return number
}

func ( self *Robot ) Move( leftDps, rightDps int ) bool {
	self.gopigo3.SetMotorDps( g.MOTOR_LEFT, leftDps )
	self.gopigo3.SetMotorDps( g.MOTOR_RIGHT, rightDps )
	changedDirection := false
	if Abs( leftDps ) > Abs( rightDps ) {
		changedDirection = self.ChangeDirection( Left )
	} else if Abs( leftDps ) == Abs( rightDps ) {
		if leftDps == -rightDps {
			if leftDps < 0 {
				changedDirection = self.ChangeDirection( Left )
			} else {
				changedDirection = self.ChangeDirection( Right )
			}
		} else {
			changedDirection = self.ChangeDirection( Forward )
		}
	} else {
		changedDirection = self.ChangeDirection( Right )
	}
	self.leftDps = leftDps
	self.rightDps = rightDps
	self.startTimeTravelingWithDps = time.Now()
	return changedDirection
}

func ( self *Robot ) ChangeDirection( to QuantativeDirection ) bool {
	if self.currentDirection != to {
		self.lastDirection = self.currentDirection
		self.currentDirection = to
		return true
	}
	return false
}

func ( self *Robot ) TimeTraveledWithDps() float64 {
	return time.Since( self.startTimeTravelingWithDps ).Seconds()
}

func ( self *Robot ) ReadLidar() int {
	lidarReading, err := self.lidarSensor.Distance()
	if err != nil {
		fmt.Println( "Robot::ReadLidar::Error::Failure reading Lidar Sensor: ", err )
	}
	self.lidarReading = lidarReading
	return self.lidarReading
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

func CalculateTraveledArcBoxDistance( endingLidarReading int, robot *Robot ) float64 {
	radius, theta := CalculateArcData( robot.leftDps, robot.rightDps )
	beginSide := radius + float64( robot.lidarReading )
	endSide := radius + float64( endingLidarReading )
	fmt.Println( "beginSide ", beginSide, " endSide ", endSide, " theta ", theta, " radius ", radius )
	//Law of cosines//
	result := math.Sqrt( math.Pow( beginSide, 2.0 ) + math.Pow( endSide + radius, 2.0 ) - ( 2.0 * beginSide * endSide * math.Cos( theta ) ) )
	fmt.Println( "Result: ", result )
	return result
}

func CalculateTraveledLineBoxDistance( endingLidarReading int, robot *Robot ) float64 {
	//Pythagorean theorem, delta distance from box
	return math.Sqrt( Math.Pow( float64( endingLidarReading - robot.lidarReading ), 2.0 ) + math.Pow( robot.TimeTraveledWithDps() * DpsToDistance( robot.leftDps ), 2.0 ) )
}

func CalculateTraveledBoxDistance( endingLidarReading int, robot *Robot ) float64 {
	result := 0.0
	if robot.leftDps == robot.rightDps {
		result = CalculateTraveledLineBoxDistance( endingLidarReading, robot )
		fmt.Println( "Line calc ", result )
	} else {
		result = CalculateTraveledArcBoxDistance( endingLidarReading, robot )
	}
	return result
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
	previousLidarReading, goalDistance, initialSpeed, initialMeasuringSpeed int
	totalDistance, cornerTurnAngle float64
	lastDirection, currentDirection QuantativeDirection
}

func ( self *Side ) InitializeSide( initialSpeed, initialMeasuringSpeed int ) {
	self.goalDistanceCalculator.InitializeAverage( MaxInitializationSamplesConstant )
	self.outOfBoundsDistance.InitializeAverage( MaxOutOfBoundSamplesConstant )
	self.initialSpeed = initialSpeed
	self.initialMeasuringSpeed = initialMeasuringSpeed
	self.lastDirection = Forward
	self.currentDirection = Forward
}

func ( self *Side ) MeasureInitialDistance( robot *Robot ) bool {
	if self.foundBox == true {
		if self.goalDistanceFound == false {
			self.goalDistanceCalculator.AddSample( robot.lidarReading )
			if self.goalDistanceCalculator.AtDesiredSampleCount() {
				self.goalDistance = self.goalDistanceCalculator.CalculateAverage()
				self.goalDistanceFound = true
				self.previousLidarReading = self.goalDistance
			}
			robot.UniformMove( self.initialMeasuringSpeed )
		}
	} else if robot.lidarReading < StartDistanceConstant {
		self.foundBox = true
	} else {
		robot.UniformMove( self.initialSpeed )
	}
	return self.goalDistanceFound
}

func ( self *Side ) UpdateCornerTurnAngle( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	_, angle := CalculateArcData( robot.leftDps, robot.rightDps )
	// fmt.Println( "Updating corner angle with angle ", angle, " loopRuntimeInSeconds", loopRuntimeInSeconds, " self.cornerTurnAngle", self.cornerTurnAngle )
	self.cornerTurnAngle += ( angle * loopRuntimeInSeconds )
	return self.TurnedCorner()
}

func ( self *Side ) ClearCornerTurnAngle() {
	self.cornerTurnAngle = 0.0
}

func ( self *Side ) TurnedCorner() bool {
	return math.Abs( self.cornerTurnAngle ) >= CornerTurnAngleConstant
}

func ( self* Side ) AddToTotalDistance( robot *Robot ) {
	self.totalDistance += CalculateTraveledBoxDistance( self.previousLidarReading, robot )
	self.previousLidarReading = robot.lidarReading
}

func ( self* Side ) Creep( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	changedDirection := false
	if robot.lidarReading > self.goalDistance {
		//fmt.Println( "Greater lr: ", robot.lidarReading, " gd: ", self.goalDistance )
		changedDirection = robot.Move( -100, -50 )
		self.UpdateCornerTurnAngle( robot, loopRuntimeInSeconds )
	} else {
		self.ClearCornerTurnAngle()
		if robot.lidarReading < self.goalDistance {
			//fmt.Println( "Less lr: ", robot.lidarReading, " gd: ", self.goalDistance )
			changedDirection = robot.Move( -50, -100 )
		} else {
			changedDirection = robot.UniformMove( -100 )
			//fmt.Println( "Equal lr: ", robot.lidarReading, " gd: ", self.goalDistance )
		}
	}
	return changedDirection
}

func ( self *Side ) MeasureSide( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	//fmt.Println( "Turned Corner: ", self.TurnedCorner(), " Corner Turn Angle: ", self.cornerTurnAngle )
	if self.outOfBoundsDistance.AtDesiredSampleCount() == false && self.TurnedCorner() == false {
		if robot.lidarReading >= OutOfBoundsDistanceConstant {
			self.outOfBoundsDistance.AddSample( robot.lidarReading )
		} else {
			self.outOfBoundsDistance.Clear()
			if self.Creep( robot, loopRuntimeInSeconds ) == true {
				self.AddToTotalDistance( robot )
			}
		}
	} else if self.outOfBoundsDistance.CalculateAverage() >= OutOfBoundsDistanceConstant || self.TurnedCorner() == true {
		self.measuredSide = true
	} else {
		//fmt.Println( "WHAT DO I DO!?" )
		self.outOfBoundsDistance.Clear()
		robot.UniformMove( -360 )
	}
	return self.measuredSide
}

func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {
	const InitialSpeed = -180
	const InitialMeasuringSpeed = -10
	const LoopRuntimeConstant = time.Millisecond
	robot := NewRobot( gopigo3, lidarSensor )
	var currentSide Side
	var previousTime time.Time
	firstLoop := false
	voltage, voltageErr := gopigo3.GetBatteryVoltage()
	currentSide.InitializeSide( InitialSpeed, InitialMeasuringSpeed )
	fmt.Println( "Voltage: ", voltage )
	if voltageErr != nil {
		fmt.Println( "RobotMainLoop::Error::Failure reading Voltage: ", voltageErr )
	}
	gobot.Every( time.Millisecond, func() {
		robot.ReadLidar()
		if currentSide.goalDistanceFound == false {
			currentSide.MeasureInitialDistance( robot )
		} else if currentSide.measuredSide == false {
			deltaTime := 0.0
			if firstLoop == false {
				previousTime = time.Now()
				firstLoop = true
			} else {
				deltaTime = time.Since( previousTime ).Seconds()
			}
			//fmt.Println( "Delta Time ", deltaTime )
			currentSide.MeasureSide( robot, deltaTime ) //, LoopTimeInSecondsConstant )
			previousTime = time.Now()
		} else {
			fmt.Println( "Success! ", currentSide.totalDistance )
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
		gopigo3.SetMotorDps( g.MOTOR_LEFT, 0 )
		gopigo3.SetMotorDps( g.MOTOR_RIGHT, 0 )
		robot.Stop()
	}()
}
