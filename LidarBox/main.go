package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"time"
	// "math"
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
func DpsToDistance( dps int ) {
	return float32( dps ) * ( 2.0 * math.Pi * 
			RobotWheelRadiusConstant / 360.0 )
}


//Calculates the arclength the robot will travel given 2 dps's//
func CalculateArcData( leftDps, rightDps int ) {
	sidePolarity := 0
	leftDistance := float32( DpsToDistance( leftDps ) )
	rightDistance := float32( DpsToDistance( rightDps ) )
	if sidePolarity = -1; leftDistance > rightDistance {
		sidePolarity = 1
	}
	const MaxDistanceConstant = math.Max( leftDistance, rightDistance )
	const DeltaDistanceConstant = leftDistance - rightDistance
	theta := math.Atan( DeltaDistanceConstant / RobotWidthConstant )
	magnitude := math.Sqrt( - math.Pow( MaxDistanceConstant, 2.0 ) / ( mathPow( math.Cos( theta ), 2.0 ) - 1 ) )
	radius := math.cos( theta ) * sidePolarity * magnitude
	return radius, theta
}

func Creep( ) {

}

type Average struct {
	 buffer, samples, desiredSampleCount int 
}

func ( self *Average ) InitializeAverage( desiredSampleCount int ) {
	self.desiredSampleCount = desiredSampleCount
}

func ( self *Average ) CalculateAverage() {
	if samples == 0 {
		return 0
	}
	return buffer / samples
}

func ( self *Average ) AddSample( int sample ) {
	self.buffer += sample
	self.samples += 1
}

func ( self *Average ) Clear() {
	self.buffer = 0
	self.samples = 0
}

func ( self *Average ) AtDesiredSampleCount() {
	if self.desiredSampleCount == 0 {
		return true
	}
	return self.samples >= self.desiredSampleCount
}


const MaxInitializationSamplesConstant = 10
const OutOfBoundsDistanceConstant = 50
const MaxOutOfBoundSamplesConstant = 5
const GreaterThanMovementMax = 90.0

type Side, struct { 
	foundBox, goalDistanceFound, measuredSide bool
	goalDistanceCalculator, outOfBoundDistance Average( int )
	goalDistance, initialSpeed, initialMeasuringSpeed int
}

func ( self *Side ) InitializeSide( initialSpeed, initialMeasuringSpeed ) {
	self.goalDistanceCalculator.InitializeAverage( MaxInitializationSamplesConstant )
	self.outOfBoundDistance.InitializeAverage( MaxOutOfBoundSamplesConstant )
	self.initialSpeed = initialSpeed
	self.initialMeasuringSpeed = initialMeasuringSpeed
}

func ( self *Side ) MeasureInitialDistance( gopigo3 *g.Driver, lidarReading ) {
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

func MeasureSide( gopigo3 *g.Driver, side *Side, lidarReading, secondsToLoopRunTime )
{
	if side.outOfBoundsDistance.AtDesiredSampleCount() == false {
		if lidarReading >= OutOfBoundsDistanceConstant {
			side.outOfBoundsDistance.AddSample( lidarReading )
		} else {
			side.outOfBoundsDistance.Clear()
			if lidarReading > side.goalDistance {
				fmt.Println( "Greater lr: ", lidarReading, " gd: ", side.goalDistance )
				Move( gopigo3, -100, -50 )
			} else if lidarReading < side.goalDistance {
				fmt.Println( "Less lr: ", lidarReading, " gd: ", side.goalDistance )
				Move( gopigo3, -50, -100 )
			} else {
				UniformMove( gopigo3, -100 )
			}
		}
	} else if outOfBoundsDistance.CalculateAverage() >= OutOfBoundsDistanceConstant {
		return self.measuredSide = true
	} else {
		fmt.Println( "WHAT DO I DO!?" )
		side.outOfBoundsDistance.Clear()
		UniformMove( gopigo3, -360 )
	}
	return false
}

func RobotMainLoop(piProcessor *raspi.Adaptor, gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver ) {
	const InitialSpeed = -180
	const InitialMeasuringSpeed = -10
	const LoopRuntimeConstant = time.Millisecond
	const SecondsToLoopRunTime = float64( time.Second / LoopRuntimeConstant )

	Side currentSide
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

	work := gobot.Every( time.Millisecond, func() {
		lidarReading, err := lidarSensor.Distance()
		if err != nil {
			fmt.Println( "RobotMainLoop::Error::Failure reading Lidar Sensor: ", err )
		}
		if currentSide.goalDistanceFound == false {
			currentSide.MeasureInitialDistance( gopigo3, lidarReading )
		} else if currentSide.measuredSide == false {
			currentSide.MeasureSide( gopigo3, lidarReading, SecondsToLoopRunTime ) == true
		} else {
			fmt.PrintLn( "Success!" )
			work.CallCancelFunc()
		}
	} )
	defer func() {
		work.CallCancelFunc()
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
		robot.WorkerRegistry()
		robot.Stop()
	}()
}
