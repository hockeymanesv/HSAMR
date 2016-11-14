package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module setting start parameters
 */
public class ControlRST implements IControl {

	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual and last
	 * request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual and last
	 * request
	 */
	IPerception.EncoderSensor encoderRight = null;

	/**
	 * reference to data class for measurement of the left wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null;
	/**
	 * reference to data class for measurement of the right wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorRight = 0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorLeft = 0;
	/**
	 * line information measured by right light sensor, values from 0 to 100
	 */
	int lineSensorRightV = 0;
	/**
	 * line information measured by left light sensor, values from 0 to 100
	 */
	int lineSensorLeftV = 0;
	/**
	 * sums of the measured sensor data for integral version 1
	 */
	static int sumRightSensor = 0;
	static int sumLeftSensor = 0;

	/**
	 * version 1
	 */
	int deltaRightSensorOld = 0;
	int deltaLeftSensorOld = 0;

	/**
	 * version 2
	 */
	static double sume = 0;
	static double eold = 0;
	static double motorPower = 0;
	double deltaBrightnessOld = 0;

	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;

	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

	// Motor Power
	int leftMotorPower = 0;
	int rightMotorPower = 0;

	// Speed parameters
	double velocity = 0.0;
	double angularVelocity = 0.0;

	// Position parameters
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();

	ControlMode currentCTRLMODE = null;

	// Encoder
	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;

	int lastTime = 0;

	// Distance
	double currentDistance = 0.0;
	double Distance = 0.0;

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * navigation object (to obtain the current position of the car from) and
	 * starts the control thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 * @param navigation
	 *            corresponding main module Navigation class object
	 * @param monitor
	 *            corresponding main module Monitor class object
	 * @param leftMotor
	 *            corresponding NXTMotor object
	 * @param rightMotor
	 *            corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor,
			IMonitor monitor) {
		// Setting other modules
		this.perception = perception;
		this.navigation = navigation;
		this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.currentCTRLMODE = ControlMode.INACTIVE;

		// setting encoders and sensors
		this.encoderLeft = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
		this.lineSensorRightV = perception.getRightLineSensorValue();
		this.lineSensorLeftV = perception.getLeftLineSensorValue();

		// MONITOR (example)
		// what happens here
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");

		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		ctrlThread.start();
	}

	// Inputs

	/**
	 * set velocity
	 * 
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * 
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity; // Winkelgeschwindigkeit w

	}

	/**
	 * set destination
	 * 
	 * @see parkingRobot.IControl#setDestination(double heading, double x,
	 *      double y)
	 */
	public void setDestination(double heading, double x, double y) {
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}

	/**
	 * sets current pose Pose = Position
	 * 
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}

	/**
	 * set start time
	 */
	public void setStartTime(int startTime) {
		this.lastTime = startTime;
	}

	/**
	 * selection of control-mode
	 * 
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO() {

		switch (currentCTRLMODE) {
		case LINE_CTRL:
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			break;
		case VW_CTRL:
			update_VWCTRL_Parameter();
			exec_VWCTRL_ALGO();
			break;
		case SETPOSE:
			update_SETPOSE_Parameter();
			exec_SETPOSE_ALGO();
			break;
		case PARK_CTRL:
			update_PARKCTRL_Parameter();
			exec_PARKCTRL_ALGO();
			break;
		case INACTIVE:
			exec_INACTIVE();
			break;
		}

	}

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		// Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
		this.lineSensorRightV = perception.getRightLineSensorValue();
		this.lineSensorLeftV = perception.getLeftLineSensorValue();

	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		this.drive(this.velocity, this.angularVelocity);
	}

	private void exec_SETPOSE_ALGO() {
		// Aufgabe 3.3
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		// Aufgabe 3.4
	}

	private void exec_INACTIVE() {
		this.stop();
	}

	/**
	 * DRIVING along black line Minimalbeispiel Linienverfolgung fuer gegebene
	 * Werte 0,1,2 white = 0, black = 2, grey = 1 Idee fuer die Aufgabe 3.1
	 * Variante 1: Statt lowPower auf 1 zu setzen, kann hier ein hoeherer Wert
	 * gewaehlt werden, es muss aber darauf geachtet werden, dass die Drehung
	 * nicht zu langsam erfolgt. hier muss die Erweiterung mit den Farbverlaufen
	 * stattfinden (Aufgabe 3.1 Variante 2) Idee: solange sich die Sensoren in
	 * einem definiertem Farbraum befinden, findet nur vorwaertsfahren statt.
	 * Sobald sie sich in einen grauen Bereich bewegen, bekommt ein Motor mehr
	 * Power als der andere, so dass der Roboter sich wieder zurueckdreht, dabei
	 * soll aber der andere Motor nicht stehen bleiben --> Erhoehung der
	 * Geschwindigkeit
	 */

	private void exec_LINECTRL_ALGO() {
		// leftMotor.forward();
		// rightMotor.forward();
		int version = 0; // 0 --> drei farbwerte (zickzack), 1--> PID

		if (version == 0) { // Entscheidung je nach Version, ob zickzack oder
							// PID
			leftMotor.forward();
			rightMotor.forward();
			int lowPower = 3;
			int highPower = 40;

			// MONITOR (example)
			monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
			monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

			if (this.lineSensorLeft == 2 && (this.lineSensorRight == 1)) {

				// when left sensor is on the line, turn left
				leftMotor.setPower(lowPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				monitor.writeControlComment("turn left");

			} else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 1)) {

				// when right sensor is on the line, turn right
				leftMotor.setPower(highPower);
				rightMotor.setPower(lowPower);

				// MONITOR (example)
				monitor.writeControlComment("turn right");
			} else if (this.lineSensorLeft == 2 && (this.lineSensorRight == 0)) {

				// when left sensor is on the line, turn left
				leftMotor.setPower(lowPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				monitor.writeControlComment("turn left");

			} else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 0)) {

				// when right sensor is on the line, turn right
				leftMotor.setPower(highPower);
				rightMotor.setPower(lowPower);

				// MONITOR (example)
				monitor.writeControlComment("turn right");
			} else if (this.lineSensorLeft == 1 && this.lineSensorRight == 0) {

				// when left sensor is on the line, turn left
				leftMotor.setPower(lowPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				monitor.writeControlComment("turn left");

			} else if (this.lineSensorRight == 1 && this.lineSensorLeft == 0) {

				// when right sensor is on the line, turn right
				leftMotor.setPower(highPower);
				rightMotor.setPower(lowPower);

				// MONITOR (example)
				monitor.writeControlComment("turn right");

			} else if (this.lineSensorRight == 0 && this.lineSensorLeft == 0) {
				leftMotor.setPower(highPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				monitor.writeControlComment("straight forward");
			}

		} else if (version == 1) {
			leftMotor.forward();
			rightMotor.forward();

			// desired values from perception
			final int desBlackRight = 29;
			final int desBlackLeft = 33;
			final int desGreyRight = 44;
			final int desGreyLeft = 48;
			final int desWhiteRight = 59;
			final int desWhiteLeft = 63;
			// parameters for PID
			final double kp = 0.1;
			final double ti = 1;
			final double td = 0;
			// sensor values
			int actRightSensor = 0;
			int actLeftSensor = 0;
			int oldRightSensor = 0;
			int oldLeftSensor = 0;
			actRightSensor = this.lineSensorRightV;
			actLeftSensor = this.lineSensorLeftV;

			// differences
			int deltaRightSensor = 0;
			int deltaLeftSensor = 0;
			// int deltaRightSensorOld=0;
			// int deltaLeftSensorOld=0;

			// calculation delta light Sensors
			deltaRightSensor = desWhiteRight - actRightSensor;
			deltaLeftSensor = desWhiteLeft - actLeftSensor;

			// PID
			double powerRight = kp * deltaRightSensor + 1 / ti * sumRightSensor
					+ (deltaRightSensorOld - deltaRightSensor) * td;
			double powerLeft = kp * deltaLeftSensor + 1 / ti * sumLeftSensor
					+ (deltaLeftSensorOld - deltaLeftSensor) * td;
			// int
			// powerRight=kp*actRightSensor+ki/ti*sumRightSensor+kd*(actRightSensor-oldRightSensor)*td;
			// int
			// powerRight=kp*actLeftSensor+ki/ti*sumLeftSensor+kd*(actLeftSensor-oldLeftSensor)*td;

			// set power for motors
			leftMotor.setPower((int) powerLeft);
			rightMotor.setPower((int) powerRight);

			// set new parameters
			deltaRightSensorOld = deltaRightSensor;
			deltaLeftSensorOld = deltaLeftSensor;
			// oldRightSensor=actRightSensor;
			// oldLeftSensor=actLeftSensor;
			if (sumRightSensor < 10) {
				sumRightSensor = sumRightSensor + deltaRightSensor;
			}
			if (sumLeftSensor < 10) {
				sumLeftSensor = sumLeftSensor + deltaLeftSensor;
			}

			// sumRightSensor=sumRightSensor+actRightSensor;
			// sumLeftSensor=sumLeftSensor+actLeftSensor;

			// ab hier Variante 2
			// hier muessen die ranges festgelegt werden
			/*
			 * int upperBound=80; int midBound=40; int lowerBound=10;
			 * 
			 * if(this.lineSensorLeft>=upperBound &&
			 * this.lineSensorRight>=upperBound){ leftMotor.setPower(highPower);
			 * rightMotor.setPower(highPower); } else
			 * if(this.lineSensorLeft>=upperBound &&
			 * (this.lineSensorRight<=upperBound &&
			 * this.lineSensorRight>=midBound)){
			 * 
			 * } else if()
			 *
			 * 
			 * Implementierung PID Regler y=Kp*e+Ki*Ta*esum+Kd(e-ealt)*1/Ta
			 *
			 * int Kp; int Ki; int Kd; int Tn; int Tv; static int esum; static
			 * int ealt; int y;
			 * 
			 * esum = esum + e; y = Kp * e + Ki * 1/Ti * esum + Kd * (e –
			 * ealt)*Td; ealt = e;
			 * 
			 * return y;
			 * 
			 */
		} else if (version == 2) {
			leftMotor.forward();
			rightMotor.forward();

			int actRightSensor = 0;
			int actLeftSensor = 0;
			int oldRightSensor = 0;
			int oldLeftSensor = 0;

			double powerLeft = 0;
			double powerRight = 0;
			actRightSensor = this.lineSensorRightV;
			actLeftSensor = this.lineSensorLeftV;

			// parameters for PID
			final double kp = 0.5;
			final double ti = 30;//45;
			final double td = 1;

			// rechter - linker Sensor
			double deltaBrightness = actRightSensor - actLeftSensor;
			double e = deltaBrightness - deltaBrightnessOld;
			double iandd = sume * 1 / ti + td * (eold - e);
			motorPower = kp * e +sume*1/ti;
			// betrag = abs()
			eold = e;
			sume = sume + e;

			powerLeft = 30 - motorPower;
			powerRight = 30 + motorPower;
			// set power for motors
			deltaBrightnessOld=deltaBrightness;
			leftMotor.setPower((int) powerLeft);
			rightMotor.setPower((int) powerRight);

		}
	}

	private void stop() {
		this.leftMotor.stop();
		this.rightMotor.stop();
	}

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v
	 *            velocity of the robot
	 * @param omega
	 *            angle velocity of the robot
	 */
	private void drive(double v, double omega) {
		// Aufgabe 3.2
	}
}