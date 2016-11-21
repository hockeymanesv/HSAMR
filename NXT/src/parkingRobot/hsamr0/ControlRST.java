package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;
import java.lang.Math;

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
	 * version 3
	 */
	static double integralE = 0;
	static double eold;

	/**
	 * vw Control and wheelControl
	 */
	static double eoldRightMotor = 0;
	static double eoldLeftMotor = 0;
	static double integralERightMotor = 0;
	static double integralELeftMotor = 0;
	static boolean schalterR = false;
	static boolean schalterL = false;
	// Motors
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
	double velocity = 15 * 3.6;// 3;//0 //in cm/s
	double angularVelocity = 400;// 3;//0

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

	// Diameter m
	double wheelDiameter = 0.05;

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
		// monitor.addControlVar("e");
		// monitor.addControlVar("outgoingPID");

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
		int version = 3; // 0 --> drei farbwerte (zickzack), 1--> PID version 1,
							// 2 --> PID version 2, 3 --> PID version 3 best
							// working

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

		} else if (version == 3) {
			double powerLeft = 0;
			double powerRight = 0;
			double powerOffset = 50;// 30
			// if(this.lineSensorLeft==0 && this.lineSensorRight==0) {
			// powerRight=-20;
			// powerLeft=0;
			// integralE=0;
			// eold=0;
			// } else {
			monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
			monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

			// Variables

			int actRightSensor = this.lineSensorRightV;
			int actLeftSensor = this.lineSensorLeftV;
			// parameters for PID
			// Werte fuer schwarz: 0.1 (0.08) ,0.003,0.25
			// Werte fuer weiss: 0.2,0.0015,0.05
			final double kp = 0.08;// 0.2
			final double ki = 0.003;// 0.0015//0.003;
			final double td = 0.25;// 0.05//0.2;

			double deltaBrightness = actRightSensor - actLeftSensor;
			double e = 0 - deltaBrightness;
			// monitor.writeControlVar("E", "" + e);

			double diffE = td * (e - eold);
			if (integralE <= 40) {
				integralE = integralE + e;
			}

			// Motorpower berechnen
			double outgoingPID = kp * e + td * diffE + ki * integralE; // PID-Regler
			// monitor.writeControlVar("outgoingPID", "" + outgoingPID);

			powerLeft = powerOffset + outgoingPID;
			powerRight = powerOffset - outgoingPID;

			// neue Variablenzuweisung
			eold = e;
			// }
			// set power for motors
			leftMotor.forward();
			rightMotor.forward();

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
	 *            velocity of the robot m/s
	 * @param omega
	 *            angle velocity of the robot in
	 */
	private void drive(double v, double omega) {
		// Aufgabe 3.2

		// defining variables
		// double velocityLeft = 0;
		// double velocityRight = 0;
		double wheelDistance = 0.101; // Radabstand in m

		double velocityLeft = (v - wheelDistance * omega / 2);
		double velocityRight = (v + wheelDistance * omega / 2);

		//TEST
		double phiIst = Math.toRadians(this.encoderRight.getEncoderMeasurement().getAngleSum());
		long tIst = this.encoderRight.getEncoderMeasurement().getDeltaT()/1000;
		double vIst = phiIst / tIst * wheelDiameter / 2; // Einheit rad/s*m
		
		monitor.writeControlVar("LeftSensor", "" + tIst);
		monitor.writeControlVar("RightSensor", "" + vIst);

		// set power for motors
		leftMotor.forward();
		rightMotor.forward();

		// Uebergabe an control Funktionen zur Regelung der Motoren
		rightMotor.setPower((int) (velocityRight));
		leftMotor.setPower((int) (velocityLeft));

//		rightMotor.setPower(controlRightMotor(velocityRight));
//		leftMotor.setPower(controlLeftMotor(velocityLeft));

	}

	/**
	 * Methode PID Regler fuer das Anpassen der Geschwindigkeit an
	 * Sollgeschwindigkeit fuer den rechten Motor
	 * 
	 * @param vSoll
	 *            Sollgeschwindigkeit in m/s
	 * @return @return powerCalculated Wert der Geschwindigkeit in power
	 *         ausgedrueckt
	 */
	private int controlRightMotor(double vSoll) {
		if (schalterR == false) {
			rightMotor.setPower(30);
			leftMotor.setPower(30);
			schalterR = true;
		}
		double factorVPower = 0.34; // Umrechnung in Power Wert mit:
									// velocity/factor=power

		double phiIst = Math.toRadians(this.encoderRight.getEncoderMeasurement().getAngleSum()); // Umrechnung
																									// in
																									// rad
		double tIst = this.encoderRight.getEncoderMeasurement().getDeltaT() / 1000; // Umrechnung
																					// in
																					// s
		double vIst = phiIst / tIst * wheelDiameter / 2; // Einheit rad/s*m
		double e = vSoll - vIst;
		integralERightMotor += e;

		// Regler
		double kp = 1;
		double ki = 0;
		double td = 0;
		double outgoingPID = kp * e + ki * integralERightMotor + td * (e - eoldRightMotor);
		eoldRightMotor = e;

		double powerCalculated = (vSoll + outgoingPID) / factorVPower; // Umrechnung
																		// in
																		// Power-Wert
		return (int) (powerCalculated);

	}

	/**
	 * Methode PID Regler fuer das Anpassen der Geschwindigkeit an
	 * Sollgeschwindigkeit fuer den linken Motor
	 * 
	 * @param vSoll
	 *            Sollgeschwindigkeit in m/s
	 * @return powerCalculated Wert der Geschwindigkeit in power ausgedrueckt
	 */
	private int controlLeftMotor(double vSoll) {
		if (schalterL == false) {
			rightMotor.setPower(30);
			leftMotor.setPower(30);
			schalterL = true;
		}
		double factorVPower = 0.34; // Umrechnung in Power Wert mit:
									// velocity/factor=power
		double phiIst = Math.toRadians(this.encoderLeft.getEncoderMeasurement().getAngleSum()); // Umrechung
																								// deg
																								// to
																								// rad
		double tIst = this.encoderLeft.getEncoderMeasurement().getDeltaT() / 1000; // Umrechnung
																					// in
																					// s
		double vIst = phiIst / tIst * wheelDiameter / 2; // Einheit rad/s*m
		double e = vSoll - vIst;
		integralELeftMotor += e;
		// Regler
		double kp = 1;
		double ki = 0;
		double td = 0;
		double outgoingPID = kp * e + ki * integralELeftMotor + td * (e - eoldLeftMotor);
		eoldLeftMotor = e;

		double powerCalculated = (vSoll + outgoingPID) / factorVPower; // Umrechnung
																		// in
																		// Power-Wert
		return (int) (powerCalculated);

	}

}
