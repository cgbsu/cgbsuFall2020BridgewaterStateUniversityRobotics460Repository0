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


type QualativeDirection int

const( 
	Left QualativeDirection = 0
	Right = 1
	Forward = 2
)

type Robot struct {
	gopigo3 *g.Driver
	lidarSensor *i2c.LIDARLiteDriver
	lidarReading, leftDps, rightDps int
	lastDirection, currentDirection QualativeDirection
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

const StartDistanceConstant = 40

func ( self *Robot ) UniformMove( dps int ) bool {
	self.gopigo3.SetMotorDps( g.MOTOR_LEFT, dps )
	self.gopigo3.SetMotorDps( g.MOTOR_RIGHT, dps )
	self.leftDps = dps
	self.rightDps = dps
	//So the amount of time traveling in this direction can be known.//
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
	if Abs( leftDps ) > Abs( rightDps ) { //Arcing left
		changedDirection = self.ChangeDirection( Left )
	} else if Abs( leftDps ) == Abs( rightDps ) { //Turning straight or spinning around
		if leftDps == -rightDps { //Spinning around
			if leftDps < 0 { //Spinning around left
				changedDirection = self.ChangeDirection( Left )
			} else { //Spinning around right
				changedDirection = self.ChangeDirection( Right )
			}
		} else { //Going straight
			changedDirection = self.ChangeDirection( Forward )
		}
	} else { //All thats left is arcing right
		changedDirection = self.ChangeDirection( Right )
	}
	self.leftDps = leftDps
	self.rightDps = rightDps
	//So the amount of time traveling in this direction can be known.//
	self.startTimeTravelingWithDps = time.Now()
	return changedDirection
}


//Change the state of currentDirection//
func ( self *Robot ) ChangeDirection( to QualativeDirection ) bool {
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
		return leftDistance, 0.0
	}
	sidePolarity := 0.0
	if sidePolarity = -1.0; leftDistance > rightDistance {
		sidePolarity = 1.0
	}
	maxDistance := math.Max( leftDistance, rightDistance )
	//Angle of turn
	theta := math.Atan( ( leftDistance - rightDistance ) / RobotWidthConstant )
	magnitude := math.Sqrt( - math.Pow( maxDistance, 2.0 ) / ( math.Pow( math.Cos( theta ), 2.0 ) - 1 ) )
	//Turn radius
	radius := math.Cos( theta ) * sidePolarity * magnitude
	return radius, theta
}

func CalculateTraveledArcBoxDistance( beginingLidarReading int, robot *Robot ) float64 {
	radius, theta := CalculateArcData( robot.leftDps, robot.rightDps )
	//We are traveling in a stragiht line if this is the case.//
	if theta == 0.0 {
		return radius
	}
	beginSide := radius + float64( robot.lidarReading )
	endSide := radius + float64( beginingLidarReading )
	//Law of cosines//
	result := math.Sqrt( math.Pow( beginSide, 2.0 ) + math.Pow( endSide + radius, 2.0 ) - ( 2.0 * beginSide * endSide * math.Cos( theta ) ) )
	return result
}

func CalculateTraveledLineBoxDistance( beginingLidarReading int, robot *Robot ) float64 {
	//Pythagorean theorem, delta distance from box, and predicted travel time on a straight line
	time := robot.TimeTraveledWithDps()
	return math.Sqrt( math.Pow( -float64( robot.lidarReading - beginingLidarReading ), 2.0 ) + math.Pow( time * DpsToDistance( robot.leftDps ), 2.0 ) )
}

func CalculateTraveledInvertedArcBoxDistance( beginingLidarReading int, robot *Robot ) float64 {
	return math.Sqrt( 2.0 * math.Pow( float64( robot.lidarReading ), 2.0 ) ) - math.Sqrt( 2.0 * math.Pow( float64( beginingLidarReading ), 2.0 ) )
}

//Calculates the "chord" of the robot's arc.
func CalculateTraveledChord( beginingLidarReading int, robot *Robot ) float64 {
	radius, theta := CalculateArcData( robot.leftDps, robot.rightDps )
	return 2.0 * radius * math.Cos( theta / 2.0 ) * robot.TimeTraveledWithDps()
}

//Error constants.//
const ArcMultiplierConstant = 0.0//1000.0
const LineMultiplierConstant = 1.0//3.8

func CalculateTraveledBoxDistance( beginingLidarReading int, robot *Robot, direction QualativeDirection ) float64 {
	result := 0.0
	if direction == Forward {
		result = math.Abs( LineMultiplierConstant * CalculateTraveledLineBoxDistance( beginingLidarReading, robot ) )
	} else if direction == Left {
		result = ArcMultiplierConstant * CalculateTraveledChord( beginingLidarReading, robot )
	} else {
		result = ArcMultiplierConstant * CalculateTraveledChord( beginingLidarReading, robot )
	}
	return result
}


/*For averaging measurments, it would be nice if 
Go had generics so the sample type could be a generic*/
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
const CornerTurnAngleConstant = math.Pi / 10.0
const TurnSamplesConstant = 10
const MaxLeftTurnsConstant = 25
const ResetCountMaximumConstant = 30
const MaxTurnRestConstant = 300


//All information needed to measure a side of the box.//
type Side struct { 
	needsToTurn, foundBox, goalDistanceFound, measuredSide bool
	readyToTurnSamples, goalDistanceCalculator, outOfBoundsDistance Average
	turnResetCount, resetCount, turnLeftCount, previousLidarReading, goalDistance, initialSpeed, initialMeasuringSpeed int
	totalDistance, cornerTurnAngle float64
	lastDirection, currentDirection QualativeDirection
}

/*I have no need of a NewSide function, but if I did it would call this AND allocate memory, 
seperation of concerns.*/
func ( self *Side ) InitializeSide( initialSpeed, initialMeasuringSpeed int ) {
	self.goalDistanceCalculator.InitializeAverage( MaxInitializationSamplesConstant )
	self.outOfBoundsDistance.InitializeAverage( MaxOutOfBoundSamplesConstant )
	self.readyToTurnSamples.InitializeAverage( TurnSamplesConstant )
	self.initialSpeed = initialSpeed
	self.initialMeasuringSpeed = initialMeasuringSpeed
	self.lastDirection = Forward
	self.currentDirection = Forward
}


//Determine the goal distance to keep from the box. Returns weather or not the measuremnt has been made//
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

//The "turn angle" is a 90 degree left hand turn. If we have done that, we are on another side of the box.//
func ( self *Side ) UpdateCornerTurnAngle( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	_, angle := CalculateArcData( robot.leftDps, robot.rightDps )
	self.cornerTurnAngle += ( angle * loopRuntimeInSeconds )
	return self.TurnedCorner()
}

func ( self *Side ) ClearCornerTurnAngle() {
	self.cornerTurnAngle = 0.0
}

func ( self *Side ) TurnedCorner() bool {
	return math.Abs( self.cornerTurnAngle ) >= CornerTurnAngleConstant
}

//When direction is changed, use this to update the side length measurement.//
func ( self* Side ) AddToTotalDistance( robot *Robot, robotDirection QualativeDirection ) {
	/////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////////////
	/////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////////////
	/////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////////////
	//self.totalDistance += math.Abs( CalculateTraveledBoxDistance( self.previousLidarReading, robot, robotDirection ) )
	self.totalDistance += ( CalculateTraveledBoxDistance( self.previousLidarReading, robot, robotDirection ) )
	self.previousLidarReading = robot.lidarReading
}

func ( self* Side ) Creep( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	changedDirection := false
	self.readyToTurnSamples.AddSample( robot.lidarReading )
	/*Do we have enough samples to average and figure out if we are ready for a change in 
	direction? Okay, use that average to make that determination. Otherwise keep moving in the same direction.*/
	if self.readyToTurnSamples.AtDesiredSampleCount() == true {
		averageSample := self.readyToTurnSamples.CalculateAverage()
		deltaSample := averageSample - self.goalDistance
		/*Considered that this case as well as the next 2 in the else block should set cahgned direciton to true to lock
		the time the robot keeps track of that it traveled to new calls to these functions.*/
		if deltaSample > TurnTolerenceConstant || self.turnLeftCount < ( -MaxLeftTurnsConstant + 10 ) {
			self.AddToTotalDistance( robot, robot.currentDirection )
			changedDirection = robot.Move( -100, -50 )
			//The angle self.TurnedCorner() uses to determine if we have made about a 90 degree turn.//
			self.UpdateCornerTurnAngle( robot, loopRuntimeInSeconds )
			self.turnLeftCount += 1
		} else {
			if deltaSample < -TurnTolerenceConstant {
				self.AddToTotalDistance( robot, robot.currentDirection )
				self.UpdateCornerTurnAngle( robot, loopRuntimeInSeconds )
				robot.Move( -50, -100 )
				self.turnLeftCount -= 1
			} else {
				self.AddToTotalDistance( robot, robot.currentDirection )
				robot.UniformMove( -100 )
			}
		}
		self.readyToTurnSamples.Clear()
	} else {
		robot.ContinueMoving()
	}
	return changedDirection
}

func ( self *Side ) MeasureSide( robot *Robot, loopRuntimeInSeconds float64 ) bool {
	/*If we have not gotten enough samples to establish we are out of bounds 
	the angle we have turned is insufficiant to say its a new side or 
	the amount of left turns is too small to say we have turned onto a new side 
	then move on to continue measuring/moving
	
	Otherwise what kind of position are we to measure the rest of the box? Encode that state.
	*/ 
	if self.outOfBoundsDistance.AtDesiredSampleCount() == false && self.TurnedCorner() == false && self.turnLeftCount < MaxLeftTurnsConstant {
		if robot.lidarReading >= OutOfBoundsDistanceConstant {
			self.outOfBoundsDistance.AddSample( robot.lidarReading )
		} else {
			self.outOfBoundsDistance.Clear()
			//previousDirection := robot.currentDirection
			if self.Creep( robot, loopRuntimeInSeconds ) == true {
				fmt.Println( "Change direction" )
				// self.AddToTotalDistance( robot, previousDirection )
			}
		}
	} else if self.outOfBoundsDistance.CalculateAverage() >= OutOfBoundsDistanceConstant {
		self.needsToTurn = true
		self.measuredSide = true
	} else if self.TurnedCorner() == true || self.turnLeftCount >= MaxLeftTurnsConstant {
		self.measuredSide = true
	} else {
		self.outOfBoundsDistance.Clear()
		robot.UniformMove( -360 )
	}
	return self.measuredSide
}


/*If we have made a discovery of enough "out of bounds sampels" (we are looking at the other 
side of the room), then we need to turn, make a 90 degree turn. In any case, back up a little.*/
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

const ErrorConstant = .5


func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {
	//Constants, and new variables.//
	const InitialSpeed = -180
	const InitialMeasuringSpeed = -10
	const LoopRuntimeConstant = time.Millisecond
	/*Make the robot instance, makes it easir to pass around information rather than 
	having a bunch of parameters, even though there is just one instance.*/
	robot := NewRobot( gopigo3, lidarSensor )
	const MaxSideConstant = 4
	sides := new( [ MaxSideConstant ]Side )
	//Initialize every side using constants.//
	for index, _ := range sides {
		sides[ index ].InitializeSide( InitialSpeed, InitialMeasuringSpeed )
	}
	currentSideIndex := 0
	currentSide := &sides[ currentSideIndex ]
	var previousTime time.Time
	firstLoop := false
	//Diagnostic//
	voltage, voltageErr := gopigo3.GetBatteryVoltage()
	fmt.Println( "Voltage: ", voltage )
	if voltageErr != nil {
		fmt.Println( "RobotMainLoop::Error::Failure reading Voltage: ", voltageErr )
	}

	var robotEndDirection QualativeDirection = Forward

	gobot.Every( time.Millisecond, func() {
		robot.ReadLidar()
		if currentSide.goalDistanceFound == false { //Have we measured the initial goal distance? No, keep measuring.
			currentSide.MeasureInitialDistance( robot )
		} else if currentSide.measuredSide == false { //Measure the side//
			//Calculate the delta time for the robot to make calculations.//
			deltaTime := 0.0
			if firstLoop == false {
				previousTime = time.Now()
				firstLoop = true
			} else {
				deltaTime = time.Since( previousTime ).Seconds()
			}
			robotEndDirection = QualativeDirection( robot.currentDirection )
			/*Updates the side's state, no info out needed yet, does send back currentSide.measuredSide however.
			Really in truth I should have done away with the delta time here and just used the time spent going 
			in a particular direction, which looking back on it, I think had a bug in that it was not locked to 
			measuring when the start time for a movement would change.*/
			currentSide.MeasureSide( robot, deltaTime )
			previousTime = time.Now()
		} else if currentSide.Reset( robot ) == false { //Wait for the robot to reset, in the mean time print out the current measuriment.
			fmt.Println( "Side distance ", currentSide.totalDistance * ErrorConstant )
		} else if ( currentSideIndex + 1 ) < MaxSideConstant {
			fmt.Println( "NEXT SIDE" )
			currentSideIndex += 1
			/*Address problem where the last measurment is not counted, this may be for the best that it is not counted in some cases 
			as it can include part of the turn to the next side.*/
			//currentSide.AddToTotalDistance( robot, robotEndDirection )
			currentSide = &sides[ currentSideIndex ]
		} else {
			//I found I got better readings when I averaged what I got for the two sides, at least proporational readings.//
			pair0 := -15.0 + ( ( sides[ 0 ].totalDistance * ErrorConstant ) + ( sides[ 2 ].totalDistance * ErrorConstant ) ) / 2.0
			pair1 := -15.0 + ( ( sides[ 1 ].totalDistance * ErrorConstant ) + ( sides[ 3 ].totalDistance * ErrorConstant ) ) / 2.0
			fmt.Println( "Sides first and third side length ", pair0 )
			fmt.Println( "Sides second and fourth side length ", pair1 )
			fmt.Println( "DONE!" )
			/*Would call ticker.Stop (the loop is running in a time.Ticker), but I didnt hve time to make 
			this work properly and it introduced bugs when I did try.*/
			gopigo3.Halt()
		}
	} )
	//Supposidly the robot will stop when Ctrl+C is pressed.//
	defer func() {
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
	//Hopefully stop when Ctrl+C is pressed.//
	defer func() { 
		gopigo3.SetMotorDps( g.MOTOR_LEFT, 0 )
		gopigo3.SetMotorDps( g.MOTOR_RIGHT, 0 )
		robot.Stop()
		gopigo3.Halt()
	}()
}
