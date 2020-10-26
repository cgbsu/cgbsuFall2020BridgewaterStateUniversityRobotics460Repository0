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

const StartDistanceConstant = 40//29.4
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

func ( self *Robot ) ContinueMoving() {
	self.gopigo3.SetMotorDps( g.MOTOR_LEFT, self.leftDps )
	self.gopigo3.SetMotorDps( g.MOTOR_RIGHT, self.rightDps )
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
	if leftDistance == rightDistance {
		// fmt.Println( "ARC DATA Sent me the wrong thing! Sending back this distance" )
		return leftDistance, 0.0
	}
	sidePolarity := 0.0
	if sidePolarity = -1.0; leftDistance > rightDistance {
		sidePolarity = 1.0
	}
	maxDistance := math.Max( leftDistance, rightDistance )
	theta := math.Atan( ( leftDistance - rightDistance ) / RobotWidthConstant )
	magnitude := math.Sqrt( - math.Pow( maxDistance, 2.0 ) / ( math.Pow( math.Cos( theta ), 2.0 ) - 1 ) )
	radius := math.Cos( theta ) * sidePolarity * magnitude
	// fmt.Println( "ARC DATA leftDistance ", leftDistance, " rightDistance ", rightDistance, " theta ", theta, " magnitude ", magnitude, " radius ", radius )
	return radius, theta
}

func CalculateTraveledArcBoxDistance( beginingLidarReading int, robot *Robot ) float64 {
	radius, theta := CalculateArcData( robot.leftDps, robot.rightDps )
	if theta == 0.0 {
		// fmt.Println( "ORIGINAL Returning result" )
		return radius
	}
	beginSide := radius + float64( robot.lidarReading )
	endSide := radius + float64( beginingLidarReading )
	//Law of cosines//
	result := math.Sqrt( math.Pow( beginSide, 2.0 ) + math.Pow( endSide + radius, 2.0 ) - ( 2.0 * beginSide * endSide * math.Cos( theta ) ) )
	// fmt.Println( "ORIGINAL beginSide ", beginSide, " endSide ", endSide, " theta ", theta, " radius ", radius, " result ", result )
	return result
}

func CalculateTraveledLineBoxDistance( beginingLidarReading int, robot *Robot ) float64 {
	//Pythagorean theorem, delta distance from box
	fmt.Println( "LINE CALC distance ", robot.TimeTraveledWithDps() * DpsToDistance( robot.leftDps ), " Distance ", DpsToDistance( robot.leftDps ), " Time ", robot.TimeTraveledWithDps() )
	return math.Sqrt( math.Pow( float64( robot.lidarReading - beginingLidarReading ), 2.0 ) + math.Pow( robot.TimeTraveledWithDps() * DpsToDistance( robot.leftDps ), 2.0 ) )
}

func CalculateTraveledInvertedArcBoxDistance( beginingLidarReading int, robot *Robot ) float64 {
	// fmt.Println( "INVERTED" )
	return math.Sqrt( 2.0 * math.Pow( float64( robot.lidarReading ), 2.0 ) ) - math.Sqrt( 2.0 * math.Pow( float64( beginingLidarReading ), 2.0 ) )
}

func LastDesprateAttempt( beginingLidarReading int, robot *Robot ) float64 {
	radius, theta = CalculateArcData( robot.leftDps, robot.rightDps )
	return 2.0 * raidus * math.Sin( theta / 2.0 ) * robot.TimeTraveledWithDps()
}

func CalculateTraveledBoxDistance( beginingLidarReading int, robot *Robot, direction QuantativeDirection ) float64 {
	result := 0.0
	// _, theta := CalculateArcData( robot.leftDps, robot.rightDps )
	if direction == Forward {
		result = CalculateTraveledLineBoxDistance( beginingLidarReading, robot )
		fmt.Println( "Line calc ", result )
	} else if direction == Left {
		result = LastDesprateAttempt( beginingLidarReading, robot )//CalculateTraveledArcBoxDistance( beginingLidarReading, robot )
		fmt.Println( "Inverted calc ", result )
	} else {
		result = LastDesprateAttempt( beginingLidarReading, robot )//CalculateTraveledArcBoxDistance( beginingLidarReading, robot )
//		result = CalculateTraveledArcBoxDistance( beginingLidarReading, robot )
		fmt.Println( "Arc calc ", result )
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


const TurnTolerenceConstant = 3
const MaxInitializationSamplesConstant = 10
const OutOfBoundsDistanceConstant = 50
const MaxOutOfBoundSamplesConstant = 5
const CornerTurnAngleConstant = math.Pi / 10.0 //.24//math.Pi / 2.0//90.0
const TurnSamplesConstant = 10
const MaxLeftTurnsConstant = 25
const ResetCountMaximumConstant = 3
const MaxTurnRestConstant = 300 //200

type Side struct { 
	needsToTurn, foundBox, goalDistanceFound, measuredSide bool
	readyToTurnSamples, goalDistanceCalculator, outOfBoundsDistance Average
	turnResetCount, resetCount, turnLeftCount, previousLidarReading, goalDistance, initialSpeed, initialMeasuringSpeed int
	totalDistance, cornerTurnAngle float64
	lastDirection, currentDirection QuantativeDirection
}

func ( self *Side ) InitializeSide( initialSpeed, initialMeasuringSpeed int ) {
	self.goalDistanceCalculator.InitializeAverage( MaxInitializationSamplesConstant )
	self.outOfBoundsDistance.InitializeAverage( MaxOutOfBoundSamplesConstant )
	self.readyToTurnSamples.InitializeAverage( TurnSamplesConstant )
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
	// fmt.Println( "CornerTurnAngle ", self.cornerTurnAngle )
	return self.TurnedCorner()
}

func ( self *Side ) ClearCornerTurnAngle() {
	self.cornerTurnAngle = 0.0
}

func ( self *Side ) TurnedCorner() bool {
	return math.Abs( self.cornerTurnAngle ) >= CornerTurnAngleConstant
}

func ( self* Side ) AddToTotalDistance( robot *Robot, robotDirection QuantativeDirection ) {
	self.totalDistance += math.Abs( CalculateTraveledBoxDistance( self.previousLidarReading, robot, robotDirection ) )
	self.previousLidarReading = robot.lidarReading
}

func ( self* Side ) Creep( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	changedDirection := false
	self.readyToTurnSamples.AddSample( robot.lidarReading )
	if self.readyToTurnSamples.AtDesiredSampleCount() == true {
		averageSample := self.readyToTurnSamples.CalculateAverage()
		deltaSample := averageSample - self.goalDistance
		if deltaSample > TurnTolerenceConstant { // averageSample > self.goalDistance {
			//fmt.Println( "Greater lr: ", robot.lidarReading, " gd: ", self.goalDistance )
			changedDirection = robot.Move( -100, -50 )
			self.UpdateCornerTurnAngle( robot, loopRuntimeInSeconds )
			self.turnLeftCount += 1
		} else {
			// self.ClearCornerTurnAngle()
			if deltaSample < -TurnTolerenceConstant {// averageSample < self.goalDistance {
				self.UpdateCornerTurnAngle( robot, loopRuntimeInSeconds )
				//fmt.Println( "Less lr: ", robot.lidarReading, " gd: ", self.goalDistance )
				changedDirection = robot.Move( -50, -100 )
				self.turnLeftCount -= 1
			} else {
				changedDirection = robot.UniformMove( -100 )
				//fmt.Println( "Equal lr: ", robot.lidarReading, " gd: ", self.goalDistance )
			}
		}
		self.readyToTurnSamples.Clear()
	} else {
		robot.ContinueMoving()
	}
	return changedDirection
}

func ( self *Side ) MeasureSide( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	//fmt.Println( "Turned Corner: ", self.TurnedCorner(), " Corner Turn Angle: ", self.cornerTurnAngle )
	//fmt.Println( "Turn left count, ", self.turnLeftCount )
	if self.outOfBoundsDistance.AtDesiredSampleCount() == false && self.TurnedCorner() == false && self.turnLeftCount < MaxLeftTurnsConstant {
		if robot.lidarReading >= OutOfBoundsDistanceConstant {
			self.outOfBoundsDistance.AddSample( robot.lidarReading )
		} else {
			self.outOfBoundsDistance.Clear()
			previousDirection := robot.currentDirection
			if self.Creep( robot, loopRuntimeInSeconds ) == true {
				fmt.Println( "Change direction" )
				self.AddToTotalDistance( robot, previousDirection )
			}
		}
	} else if self.outOfBoundsDistance.CalculateAverage() >= OutOfBoundsDistanceConstant {
		self.needsToTurn = true
		self.measuredSide = true
	} else if self.TurnedCorner() == true || self.turnLeftCount >= MaxLeftTurnsConstant {
		self.measuredSide = true
	} else {
		//fmt.Println( "WHAT DO I DO!?" )
		self.outOfBoundsDistance.Clear()
		robot.UniformMove( -360 )
	}
	return self.measuredSide
}

func ( self *Side ) Reset( robot *Robot ) bool {
	if self.needsToTurn == true {
		if self.turnResetCount < MaxTurnRestConstant {
			robot.Move( -100, -50 )
			self.turnResetCount += 1
		} else {
			self.needsToTurn = false
		}
	} else {
		robot.UniformMove( 100 )
		self.resetCount += 1
		if self.resetCount >= ResetCountMaximumConstant {
			return true
		}
	}
	return false
}

const ErrorConstant = 1.0 / ( 2.43 )


func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {
	const InitialSpeed = -180
	const InitialMeasuringSpeed = -10
	const LoopRuntimeConstant = time.Millisecond
	robot := NewRobot( gopigo3, lidarSensor )
	const MaxSideConstant = 4
	sides := new( [ MaxSideConstant ]Side )
	for index, _ := range sides {
		sides[ index ].InitializeSide( InitialSpeed, InitialMeasuringSpeed )
	}
	// var ticker *time.Ticker
	currentSideIndex := 0
	currentSide := &sides[ currentSideIndex ]
	var previousTime time.Time
	firstLoop := false
	voltage, voltageErr := gopigo3.GetBatteryVoltage()
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
		} else if currentSide.Reset( robot ) == false {
			fmt.Println( "Side distance ", currentSide.totalDistance * ErrorConstant )
		} else if ( currentSideIndex + 1 ) < MaxSideConstant {
			if currentSide.totalDistance == 0 {
				currentSide.AddToTotalDistance( robot, Forward )
			}
			fmt.Println( "NEXT SIDE" )
			currentSideIndex += 1
			currentSide = &sides[ currentSideIndex ]
		} else {
			for _, side := range sides {
				fmt.Println( "Side Distance ", side.totalDistance * ErrorConstant )
			}
			fmt.Println( "DONE!" )
			gopigo3.Halt()
			// ticker.Stop()
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
