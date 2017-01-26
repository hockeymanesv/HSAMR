package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IControl.ControlMode;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import parkingRobot.INavigation;
import java.lang.Math;
import lejos.util.Matrix;

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
	 * for Line Control PID
	 */
	static double integralE = 0; // fuer Integral
	static double eold = 0; // fuer Differentation
	static int blackMarkerRight = 0; // Variablen zum ausloesen der Kurvenfahrt
	static int blackMarkerLeft = 0;
	static int previousStatus = 0; // zeigt an, ob im vorhergehenden Zyklus eine
									// Links (1) oder Rechts (2) Kurve gefahren
									// wurde
	static int marker = 0; // zeigt an, ob in rechtskurve, 1 fuer rechtskurve, 0
							// fuer linkskurve
	static boolean turn = false;

	/**
	 * Line control PID version 1
	 */
	static boolean blackRight = false; // rechter Sensor schwarz
	static boolean blackLeft = false; // linker Sensor schwarz
	static boolean zwischenmarker = false; // Kurve registriert aber weiter
											// geradeaus
	static boolean turningRight = false; // zur Zeit rechts drehen
	static boolean turningLeft = false; // zur Zeit links drehen
	static boolean straightDriving = true; // geradeausfahrt
	static int velocityFactor = 0; // Geschwindigkeit anheben

	/**
	 * vw Control and wheelControl
	 */
	static double eoldRightMotor = 0; // fuer Differentation
	static double eoldLeftMotor = 0;
	double integralERightMotor = 0; // fuer Integration
	double integralELeftMotor = 0;
	static double phiIstR = 0; // Gefahrener Winkel Rechts
	static double phiIstL = 0;
	static double currentAngle = 0; // Insgesamt abgefahrener Winkel
	static double wheelDistance = 0.114; // Radabstand in m
	double outgoingPID = 0;

	/**
	 * setPose
	 */
	static double eOldSetPose = 0;
	static double destinationX = -1.8; // in m
	static double destinationY = -0.6; // in m
	static double destinationPhi = -Math.PI / 2.0;// 11.0/36.0*Math.PI;
	static boolean direction = false;
	static boolean xReached = false; //Endmarker
	static boolean yReached = false;//Endmarker
	static boolean phiReached = false;//Endmarker
	static double outgoingPD = 0;
	static boolean firstAccessDriveToPose = true; // zum berechnen der Winkel und Abstaende
	static double xStart = 0; //Startposition
	static double yStart = 0;
	static double xDist = 0; //Startdistanz
	static double yDist = 0;
	static int which = 2; // version of setPose

	/**
	 * Path Finder
	 */
	static double x1 = -0.0022; // Werte der Polynomkoeffizienten
	static double x2 = -0.1;
	static double x3 = -2.5;
	static double x4 = 0.0;
	static double abtastx = 0.0; // x-Wert an dem abgetastet wird
	static double anfangx = 0.0;
	static double anfangy = 0.0;
	static double neuy = 0; // Einheit m
	static double neux = 0; // Einheit m
	static boolean setStartCoordinates = true; //Startwerte berechnen
	static double destinationPhiOut = 0; // Startwinkel, der an Setpose uebergeben wird
	static int schritt = 0; //zeigt Stelle im Pathfinder
	static double sign = 0; //Vorzeichen x1

	/**
	 * exampleProgramOne
	 */
	static int markerExample = 0; //Schritt im Beipsielprogramm
	static boolean firstExample = false; //Erstes beispielprogramm in betrieb

	/**
	 * exampleProgramTwo
	 */
	static int luecke = 0; //Lueckenzahl
	static boolean statusParked = false; //zeigt an ob eingeparkt
	static boolean secondExample = false; //zweites Beispielprogramm in Betrieb
	static boolean statusParkedOut = false; // zeigt an ob ausgeparkt
	static int markerSecondExample = 0; // Schritt

 
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
	double velocity = 5.0;// 3;//0 //in cm/s
	double angularVelocity = 0.5;// Math.PI/6;// 3;//0 //in rad/s

	// Position parameters
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();

	ControlMode currentCTRLMODE = null;

	// Encoder
	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;

	int lastTime = 0;

	// Distance in m
	// double currentDistance = 0.0;
	static double distance = 0.0;

	// Diameter m
	double wheelDiameter = 0.056;

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

		// MONITOR

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
		this.currentPosition = currentPosition;
	}

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}

	/**
	 * returns current ControlMode
	 * 
	 * @return ControlMode
	 */
	public ControlMode getCtrlMode() {
		return this.currentCTRLMODE;
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
		case EXAMPLE_ONE:
			update_EXAMPLE_Parameter();
			firstExample = true;
			firstExampleProgram();
			break;
		case EXAMPLE_TWO:
			update_EXAMPLE_Parameter();
			secondExample = true;
			secondExampleProgram();
			break;
		}

	}

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter() {
		angleMeasurementRight = encoderRight.getEncoderMeasurement();
		angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		angleMeasurementRight = encoderRight.getEncoderMeasurement();
		angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		// Aufgabe 3.4
		angleMeasurementRight = encoderRight.getEncoderMeasurement();
		angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		setPose(navigation.getPose());
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
	 * update parameters during second example two
	 */
	private void update_EXAMPLE_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
		this.lineSensorRightV = perception.getRightLineSensorValue();
		this.lineSensorLeftV = perception.getLeftLineSensorValue();
		angleMeasurementRight = encoderRight.getEncoderMeasurement();
		angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		setPose(navigation.getPose());
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		// exampleProgram();
		this.drive(this.velocity, this.angularVelocity);
	}

	/**
	 * drive to a definded position
	 */
	private void exec_SETPOSE_ALGO() {
		this.driveToPose(this.destinationX, this.destinationY, this.destinationPhi, which);
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		this.parkControl();
	}

	/**
	 * sets the roboter inactive
	 */
	private void exec_INACTIVE() {
		this.stop();
	}

	/**
	 * DRIVING along black line 
	 * Minimalbeispiel Linienverfolgung fuer gegebene
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
		this.lineControl();
	}

	private void lineControl() {
		int version = 1; // 0 --> drei farbwerte (zickzack), 1--> PID version

		if (version == 0) { // Entscheidung je nach Version, ob zickzack oder
							// PID
			leftMotor.forward();
			rightMotor.forward();
			int lowPower = 3;
			int highPower = 40;

			if (this.lineSensorLeft == 2 && (this.lineSensorRight == 1)) {

				// when left sensor is on the line, turn left
				leftMotor.setPower(lowPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				// monitor.writeControlComment("turn left");

			} else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 1)) {

				// when right sensor is on the line, turn right
				leftMotor.setPower(highPower);
				rightMotor.setPower(lowPower);

				// MONITOR (example)
				// monitor.writeControlComment("turn right");
			} else if (this.lineSensorLeft == 2 && (this.lineSensorRight == 0)) {

				// when left sensor is on the line, turn left
				leftMotor.setPower(lowPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				// monitor.writeControlComment("turn left");

			} else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 0)) {

				// when right sensor is on the line, turn right
				leftMotor.setPower(highPower);
				rightMotor.setPower(lowPower);

				// MONITOR (example)
				// monitor.writeControlComment("turn right");
			} else if (this.lineSensorLeft == 1 && this.lineSensorRight == 0) {

				// when left sensor is on the line, turn left
				leftMotor.setPower(lowPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				// monitor.writeControlComment("turn left");

			} else if (this.lineSensorRight == 1 && this.lineSensorLeft == 0) {

				// when right sensor is on the line, turn right
				leftMotor.setPower(highPower);
				rightMotor.setPower(lowPower);

				// MONITOR (example)
				// monitor.writeControlComment("turn right");

			} else if (this.lineSensorRight == 0 && this.lineSensorLeft == 0) {
				leftMotor.setPower(highPower);
				rightMotor.setPower(highPower);

				// MONITOR (example)
				// monitor.writeControlComment("straight forward");
			}

		} else if (version == 1) { // mit PID
			if (lineSensorLeft == 2 && (lineSensorRight == 0 || lineSensorRight == 1)) { // wenn linker Sensor auf schwarz links drehen
				
				leftTurn();
			} else if ((lineSensorLeft == 0 || lineSensorLeft == 1) && lineSensorRight == 2) { // wenn rechter Sensor auf schwarz rechts drehen
				rightTurn();
			} else if (previousStatus == 1 || (lineSensorLeft == 2 && lineSensorRight == 2 && marker == 0)) { // wenn linker Sensor immer noch auf schwarz links drehen
				leftTurn();
				if (lineSensorRight == 0 && lineSensorLeft == 0) {
					previousStatus = 0; //zurueck zu Geradeausfahrt
				}
			} else if (previousStatus == 2 || (lineSensorLeft == 2 && lineSensorRight == 2)) { // wenn rechter Sensor immer noch auf schwarz rechts drehen
				rightTurn();
				if (lineSensorRight == 0 && lineSensorLeft == 0) {
					previousStatus = 0; //zurueck zu Geradeausfahrt
				}
			} else if (!turn) { // Geradeausfahrt
				previousStatus = 0;
				if (navigation.getCurrentLine() == 3 || navigation.getCurrentLine() == 2
						|| navigation.getCurrentLine() == 6 || navigation.getCurrentLine() == 5) {
					straightForward(35.0); //slower at lines 2,3,5,6
				} else { //increase velocity with time up to full speed
					velocityFactor++;
					if (velocityFactor > 25) {
						velocityFactor = 25;
					}
					straightForward(35.0 + velocityFactor * 0.5);
				}
			}
		}
	}

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v
	 *            velocity of the robot m/s
	 * @param omega
	 *            angle velocity of the robot in rad/s
	 */
	private void drive(double v, double omega) {
		// defining variables
		double wheelDistance = 0.114; // Radabstand in m

		// Berechnung der Geschwindigkeiten fuer den rechten und linken Motor
		double velocityLeft = (v / 100.0 - wheelDistance / 2.0 * omega); // v/100
																			// Umrechnung
																			// cm/s
																			// zu
																			// m/s
		double velocityRight = (v / 100.0 + wheelDistance / 2.0 * omega);

		// set power for motors
		leftMotor.forward();
		rightMotor.forward();

		// Uebergabe an control Funktionen zur Regelung der Motoren
		rightMotor.setPower(controlRightMotor(velocityRight));
		leftMotor.setPower(controlLeftMotor(velocityLeft));

	}

	/**
	 * drives the roboter automatically to a defined position and turns it in a defined angle
	 * 
	 * @param xIn Destination x-Axis
	 * @param yIn Destination y-Axis
	 * @param phiIn angle at the end
	 * @param wich version of SetPose, 1 for ParkControl, 2 for long distance control
	 */
	private void driveToPose(double xIn, double yIn, double phiIn, int wich) {
		int version = wich;
		double phiSoll = 0; //angle from current position to destination
		double x = 0; // current position
		double y = 0;
		double deltaX = 0; // difference from current position to destination
		double deltaY = 0;
		double phiIst = 0; // current angle
		double deltaXV2 = 0; // difference from current position to start
		double deltaYV2 = 0;
		if (version == 2) { // long distance version
			//setting variables
			phiSoll = 0;
			destinationX = xIn; 
			destinationY = yIn;
			destinationPhi = phiIn;
			
			// aktuelle x und y Position
			x = navigation.getPose().getX();
			y = navigation.getPose().getY();

			// Vorberechnungen
			deltaX = destinationX - x;
			deltaY = destinationY - y;
			if (firstAccessDriveToPose) {
				xStart = x;
				yStart = y;
				xDist = destinationX - xStart;
				yDist = destinationY - yStart;
				firstAccessDriveToPose = false;
			}

			deltaXV2 = x - xStart;
			deltaYV2 = y - yStart;

			// calculation of phiSoll
			if (deltaX > 0) {
				phiSoll = Math.atan2(deltaY, deltaX);
			} else if (deltaX < 0 && deltaY >= 0) {
				phiSoll = Math.atan2(deltaY, deltaX) - Math.PI;
			} else if (deltaX < 0 && deltaY < 0) {
				phiSoll = Math.atan2(deltaY, deltaX) + Math.PI;
			} else if (deltaX == 0 && deltaY > 0) {
				phiSoll = Math.PI / 2;
			} else if (deltaX == 0 && deltaY < 0) {
				phiSoll = -Math.PI / 2;
			}
			
			// current angle
			phiIst = navigation.getPose().getHeading();

			// start routine
			// turning to phiSoll
			if (!direction && !xReached && !yReached) {
				if (phiSoll >= 0) {
					drive(0, 0.5);
				} else if (phiSoll >= 0) {
					drive(0, -0.5);
				}
				if (intervalContains(phiSoll - Math.toRadians(1), phiSoll + Math.toRadians(1), phiIst)) { // stop
					direction = true;
				}
			}

			// Regelung auf Abschnitt zwischen Start- und Zielposition
			if (direction && !xReached && !yReached) {
				double e = 1 / Math.sqrt(xDist * xDist + yDist * yDist) * (xDist * (deltaYV2) - yDist * (deltaXV2)); //distance formula
				
				//Werte fuer Regler
				double kp = 55;
				double td = 900;
				
				//Regler
				outgoingPD = kp * e + td * (e - eOldSetPose); 
				
				//to VW-Control
				drive(10, outgoingPD);
				
				//setting old Variables
				eOldSetPose = e;
			}
			
			// Endroutine
			// Regelung abbrechen, wenn DestinationX und DestinationY erreicht ist
			if (intervalContains(Math.abs(destinationX) - 0.05, Math.abs(destinationX) + 0.05, x)
					&& intervalContains(Math.abs(destinationY) - 0.05, Math.abs(destinationY) + 0.05, y)) {
				xReached = true;
				yReached = true;
				phiReached = false;
			}
			// Endroutine fuer erstes Beispielprogramm
			if (intervalContains(Math.abs(destinationX) - 0.05, Math.abs(destinationX) + 0.05, x)
					&& intervalContains(Math.abs(destinationY) - 0.08, Math.abs(destinationY) + 0.08, y)
					&& firstExample) {
				xReached = true;
				yReached = true;
				phiReached = false;
			}

			//phiSoll einstellen, mit VW-Control
			if (xReached && yReached && !phiReached) {
				if (xReached && yReached && phiIst > destinationPhi) { 
					drive(0, -Math.PI / 6.0); //rechtsdrehung
				} else if (xReached && yReached && phiIst < destinationPhi) {
					drive(0, Math.PI / 6.0); //linksdrehung
				}
			}
			
			//wenn phiSoll erreicht abbrechnen der SetPose
			if (xReached && yReached && intervalContains(destinationPhi - Math.toRadians(5),
					destinationPhi + Math.toRadians(5), phiIst)) {
				phiReached = true;
				stop();
			}
		} else if (version == 1) {
			phiSoll = 0;
			destinationX = xIn;
			destinationY = yIn;
			destinationPhi = phiIn;

			// aktuelle x und y Position
			x = navigation.getPose().getX();
			y = navigation.getPose().getY();

			// Vorberechnungen
			deltaX = destinationX - x;
			deltaY = destinationY - y;

			// calculation of phiSoll
			if (deltaX > 0) {
				phiSoll = Math.atan2(deltaY, deltaX);
			} else if (deltaX < 0 && deltaY >= 0) {
				phiSoll = Math.atan2(deltaY, deltaX) + Math.PI;
			} else if (deltaX < 0 && deltaY < 0) {
				phiSoll = Math.atan2(deltaY, deltaX) - Math.PI;
			} else if (deltaX == 0 && deltaY > 0) {
				phiSoll = Math.PI / 2;
			} else if (deltaX == 0 && deltaY < 0) {
				phiSoll = -Math.PI / 2;
			}

			// current Winkel
			phiIst = navigation.getPose().getHeading();

			// Startroutine
			if (!direction && !xReached && !yReached) {
				if (phiSoll >= 0) {
					drive(0, 0.5);
				} else if (phiSoll <= 0) {
					drive(0, -0.5);
				}
				if (intervalContains(phiSoll - Math.toRadians(3), phiSoll + Math.toRadians(3), phiIst)) {
					direction = true;
				}
			}

			// Regelung
			if (version == 1 && direction && !xReached && !yReached) {
				double e = phiSoll - phiIst;
				double td = 0.03;
				double kp = 0.05;
				
				//Regler
				outgoingPD = kp * e + td * (e - eOldSetPose);
				
				//Reglerausgang setzen
				drive(10, outgoingPD);
				
				// set variables
				eOldSetPose = e;
			}
			
			// Endroutine für Zielostion, abbrechen, wenn erreicht
			if (intervalContains(destinationX - 0.005, destinationX + 0.005, x)
					&& intervalContains(destinationY - 0.005, destinationY + 0.005, y)) {
				xReached = true;
				yReached = true;
				phiReached = false;
			}

			//Endroutine fuer Winkel auf phiSoll
			if (xReached && yReached && !phiReached) {
				if (xReached && yReached && phiIst > destinationPhi) {
					drive(0, -Math.PI / 6.0); //Rechtsdrehung
				} else if (xReached && yReached && phiIst < destinationPhi) { 
					drive(0, Math.PI / 6.0); //Linksdrehung
				}
			}
			
			//Abbruch der SetPose, wenn phiSoll erreicht
			if (xReached && yReached && intervalContains(destinationPhi - Math.toRadians(5),
					destinationPhi + Math.toRadians(5), phiIst)) {
				phiReached = true;
			}
		}

	}

	/**
	 * drives the roboter into a parking slot
	 * at first the roboter gets adjusted to an angle, which is the gain of the polynom at P(0/0)
	 * at second the roboter drives along the polynom
	 * at third the roboter gets adjusted parallel to the line
	 * 
	 */
	private void parkControl() {
		double generalx = navigation.getPose().getX(); // current xPosition
		double generaly = navigation.getPose().getY(); // current yPosition

		// Schritt 0: drehen des Roboters auf den Anfangswinkel des Polynoms
		if (schritt == 0) {
			if (setStartCoordinates) {
				//starting point of the roboter
				anfangx = generalx; // Einheit m
				anfangy = generaly; // Einheit m
				
				//for SetPose
				destinationX = anfangx;
				destinationY = anfangy;
				
				//only one time in this if
				setStartCoordinates = false;

				// calculation phiSoll for SetPose for every line and park in and park out
				if (inOrOut() == 1 && navigation.getCurrentLine() == 0) {
					destinationPhi = -Math.PI / 2.0 + Math.atan(x3);
				}
				if (inOrOut() == 1 && navigation.getCurrentLine() == 1) {
					destinationPhi = Math.atan(x3);				
				}
				if (inOrOut() == 1 && navigation.getCurrentLine() == 4) {
					destinationPhi = Math.PI / 2.0 + Math.atan(x3);
				}
				if (inOrOut() == -1 && navigation.getCurrentLine() == 0) {
					destinationPhiOut = Math.PI / 2.0 + Math.atan(x3);
				}
				if (inOrOut() == -1 && navigation.getCurrentLine() == 1) {
					destinationPhiOut = Math.PI + Math.atan(x3);
				}
				if (inOrOut() == -1 && navigation.getCurrentLine() == 4) {
					destinationPhiOut = Math.PI * 3.0 / 2.0 + Math.atan(x3);
				}
			}

			//adjust the angle
			if ((inOrOut() == -1)) { //ausparken
				drive(0, Math.PI / 10.0);
				if (intervalContains(destinationPhiOut - Math.toRadians(1), destinationPhiOut + Math.toRadians(6),
						navigation.getPose().getHeading())) { //in Pfadmodus, wenn Anfangswinkel erreicht
					schritt = 1;
				}
			} else if (inOrOut() == 1) { //einparken
				driveToPose(destinationX, destinationY, destinationPhi, 1);
			}
			if (phiReached) {//in Pfadmodus, wenn Anfangswinkel erreicht
				schritt = 1;
			}
			
		}
		
		// Modus zum abfahren des Pfads
		if (schritt == 1) {
			xReached = false;
			yReached = false;
			phiReached = false;
			setStartCoordinates = true;

			/* Position des Roboters in der Parkluecke
			* neues Koordinatensystem erstellen, fuer jede Linie einzeln und fuer ein und ausparken,
			* da einmal in positive xneu Richtung (einparken) und einmal in 
			* negative xneu Richtung (ausparken) abgefahren wird
			*/
			if (navigation.getCurrentLine() == 0) {
				if ((inOrOut() == -1)) { // ausparken
					neuy = generalx - anfangx;
					neux = -Math.abs((Math.abs(anfangy) - Math.abs(generaly)));
				} else if (inOrOut() == 1) { //einparken
					neuy = generalx - anfangx;
					neux = -generaly - anfangy;
				}
			} else if (navigation.getCurrentLine() == 1) {
				if ((inOrOut() == -1)) {// ausparken
					neuy = generaly - anfangy;
					neux = -Math.abs(Math.abs(anfangx) - Math.abs(generalx));
				} else if (inOrOut() == 1) {//einparken
					neuy = generaly - anfangy;
					neux = generalx - anfangx;
				}
			} else if (navigation.getCurrentLine() == 4) {
				if ((inOrOut() == -1)) {// ausparken
					neuy = -(generalx - anfangx);
					neux = -Math.abs(Math.abs(anfangy) - Math.abs(generaly));
				} else if (inOrOut() == 1) {//einparken
					neuy = -(generalx - anfangx);
					neux = Math.abs(Math.abs(anfangy) - Math.abs(generaly)); 
				}
			}

			// Abtastung neu setzen
			// Punkt an dem Polynom abgetastet wird
			abtastx = neux * 100; // Einheit cm

			double d = 0; // Abtastabstand in cm fuer naechsten Punkt
			if (inOrOut() == 1) {
				d = 0.5;
			} else if (inOrOut() == -1) {
				d = -0.5;
			}
			
			// Polynomberechnung
			double y = x1 * abtastx * abtastx * abtastx + x2 * abtastx * abtastx + x3 * abtastx + x4; //y der jetzigen Position
			double yForward = x1 * (abtastx + d) * (abtastx + d) * (abtastx + d) + x2 * (abtastx + d) * (abtastx + d)
					+ x3 * (abtastx + d) + x4; // y der naechsten Position

			double yDerivation = (3 * x1 * abtastx * abtastx + x2 * 2 * abtastx + x3); // Anstieg
																						// an
																						// der
																						// jetzigen
																						// Position
			double yDerivationForward = (3 * x1 * (abtastx + d) * (abtastx + d) + 2 * x2 * (abtastx + d) + x3); // Anstieg
																												// an
																												// der
																												// naechsten
																												// Position
			
			// Winkel berechnen
			double phi = Math.atan(yDerivation); //Winkel jetzt
			double phiForward = Math.atan(yDerivationForward); //winkel danach
			
			//Winkeldifferenz
			double deltaPhi = (phiForward - phi);

			// Berechung Radius r
			double hyp = Math.sqrt((yForward - y) * (yForward - y) + d * d);
			double k = Math.PI * 2.0 / deltaPhi;
			double r = k * hyp / Math.PI / 2.0;

			// Parameter fuer VW-Control
			double velocity = 7.5; // konstante Geschwindigkeit in cm/s
			double omegaPark = velocity / r; // * 1000 wegen
											// umrechnung zu s
			// in VW-Control
			drive(velocity, omegaPark);

			// Stoppbedingungen
			boolean markerxIn = intervalContains(0.28, 0.3, neux);
			if (markerxIn) {
				schritt = 2; // wenn Pfad abgefahren, in Endausrichtung
			}
		}
		if (schritt == 2) { // Endausrichtung
		
			//aktuelle Position ist Zielposition --> nur Drehung
			destinationX = navigation.getPose().getX();
			destinationY = navigation.getPose().getY();
			
			//Winkel parallel zu Ebene bestimmen
			if (navigation.getCurrentLine() == 0) {
				destinationPhi = 0;
			} else if (navigation.getCurrentLine() == 1) {
				destinationPhi = Math.PI/2.0;
			} else if (navigation.getCurrentLine() == 4) {
				destinationPhi = Math.PI;
			}
 
			//in SetPose
			driveToPose(destinationX, destinationY, destinationPhi, 1);

			//Variablen zuruecksetzen, wenn richtig ausgerichtet
			if (intervalContains(0, 3, navigation.getPose().getHeading() / Math.PI * 180)
					&& navigation.getCurrentLine() == 0) {
				phiReached = false;
				xReached = false;
				yReached = false;
				neux = 0;
				neuy = 0;
				anfangx = 0;
				anfangy = 0;
				abtastx = 0;
				schritt = 0;
				markerSecondExample = 1;
				GuidanceAT.setParkmaneuverFinished();
			} else if (intervalContains(90, 93, navigation.getPose().getHeading() / Math.PI * 180)
					&& navigation.getCurrentLine() == 1) {
				phiReached = false;
				xReached = false;
				yReached = false;
				neux = 0;
				neuy = 0;
				anfangx = 0;
				anfangy = 0;
				abtastx = 0;
				markerSecondExample = 4;
				schritt = 0;
				GuidanceAT.setParkmaneuverFinished();
			} else if (intervalContains(180, 183, navigation.getPose().getHeading() / Math.PI * 180)
					&& navigation.getCurrentLine() == 4) {
				phiReached = false;
				xReached = false;
				yReached = false;
				neux = 0;
				neuy = 0;
				anfangx = 0;
				anfangy = 0;
				abtastx = 0;
				schritt = 0;
				GuidanceAT.setParkmaneuverFinished();
			}

			//Abbruch erstes Beispielprogramm
			if (firstExample) {
				setCtrlMode(ControlMode.INACTIVE);
			}
		}
	}

	/**
	 * zeigt an, ob im ausparken oder einparken
	 * 
	 * @return true wenn ausparken, true wenn einparken
	 */
	public double inOrOut() {
		sign = Math.signum(x1);
		return sign;
	}

	/**
	 * turns the roboter left
	 */
	private void leftTurn() {
		setMotorPowers(-15, 48);
		previousStatus = 1; // Status fuer naechste Drehung setzen

		// Parameter ruecksetzen
		integralE = 0;
		eold = 0;
		velocityFactor = 0;

	}

	/**
	 * turns the roboter right
	 */
	private void rightTurn() {
		setMotorPowers(45, -20);
		previousStatus = 2; // Status fuer naechste Drehung setzen

		// Parameter ruecksetzen
		integralE = 0;
		eold = 0;
		marker = 1;
		velocityFactor = 0;
	}

	/**
	 * PID for straight forward driving
	 * deviation is the difference between right and left light sensor
	 * 
	 *  @param powerOffset ground speed
	 */
	private void straightForward(double powerOffset) {

		// Variables
		marker = 0; //ruecksetzen fuer geradeausfahrt
		int actRightSensor = this.lineSensorRightV;
		int actLeftSensor = this.lineSensorLeftV;

		// parameters for PID
		final double kp = 0.075;
		final double ki = 0.0;
		final double td = 0.065;

		// Calculation parameters PID
		double deltaBrightness = actRightSensor - actLeftSensor;
		double e = - deltaBrightness; // Fuehrungsgroesse = 0
		integralE += e;

		// Regelung
		double outgoingPID = kp * e + td * (e - eold) + ki * integralE; // PID-Regler
		
		//Motorpower pro Motor
		double powerLeft = powerOffset + outgoingPID;
		double powerRight = powerOffset - outgoingPID;

		// setting new variables
		eold = e;

		setMotorPowers(powerLeft, powerRight);
	}

	/**
	 * sets power of motors
	 * 
	 * @param powerLeft
	 *            power of left motor
	 * @param powerRight
	 *            power of right motor
	 */
	private void setMotorPowers(double powerLeft, double powerRight) {
		leftMotor.forward();
		rightMotor.forward();
		
		leftMotor.setPower((int) powerLeft);
		rightMotor.setPower((int) powerRight);
	}

	/**
	 * stops the NXT
	 */
	private void stop() {
		this.leftMotor.stop();
		this.rightMotor.stop();
	}

	/**
	 * Methode PID Regler fuer das Anpassen der Geschwindigkeit an
	 * Sollgeschwindigkeit fuer den rechten Motor
	 * 
	 * @param vSoll
	 *            Sollgeschwindigkeit in m/s
	 * @return powerCalculated Wert der Geschwindigkeit in power
	 *         umgerechnet
	 */
	private int controlRightMotor(double vSoll) {
		double factorVPower = 0.0034; // Umrechnung in Power Wert mit:
										// velocity/factor=power

		//Drehwinkel des Radeuncoders
		phiIstR = Math.toRadians(angleMeasurementRight.getAngleSum()); // Umrechnung
																		// in
																		// rad

		// calculate data
		double tIst = angleMeasurementRight.getDeltaT() / 1000.0; // Umrechnung
																	// in
																	// s
		double vIst = phiIstR / tIst * wheelDiameter / 2.0; // Einheit rad/s*m

		//Regelabweichung
		double e = vSoll - vIst;

		integralERightMotor += e;

		// Regelparameter
		double kp = 0.14;
		double ki = 0.33;
		double td = 0.27;
		
		//Regler
		outgoingPID = kp * e + ki * integralERightMotor + td * (e - eoldRightMotor);

		// set variables
		eoldRightMotor = e;

		// set power
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
	 * @return powerCalculated Wert der Geschwindigkeit in power umgerechnet
	 */
	private int controlLeftMotor(double vSoll) {
		double factorVPower = 0.0034; // Umrechnung in Power Wert mit:
										// velocity/factor=power

		//Drehwinkel des Radencoders
		phiIstL = Math.toRadians(angleMeasurementLeft.getAngleSum()); // Umrechung
																		// deg
																		// to
																		// rad

		// calculate data
		double tIst = angleMeasurementLeft.getDeltaT() / 1000.0; // Umrechnung
																	// in
																	// s
		double vIst = phiIstL / tIst * wheelDiameter / 2.0; // Einheit rad/s*m
		
		//Regelabweichung
		double e = vSoll - vIst;

		// set new variables
		integralELeftMotor += e;

		// Regelparameter
		double kp = 0.14;
		double ki = 0.33;
		double td = 0.27;
		
		//Regler
		outgoingPID = kp * e + ki * integralELeftMotor + td * (e - eoldLeftMotor);

		// set variables
		eoldLeftMotor = e;

		// set power
		double powerCalculated = (vSoll + outgoingPID) / factorVPower; // Umrechnung
																		// in
																		// Power-Wert
		return (int) (powerCalculated);

	}

	/**
	 * Function for checking a number if it is in a defined interval
	 * 
	 * @param low
	 * @param high
	 * @param n
	 * @return true if n is in interval, false if not
	 */
	private boolean intervalContains(double low, double high, double n) {
		return n >= low && n <= high;
	}

	/**
	 * set the destination for SetPose must be called every cycle
	 * 
	 * @param pose pose which is wanted
	 */
	public void setDestinationPose(Pose pose) {
		destinationX = pose.getX();
		destinationY = pose.getY();
		destinationPhi = pose.getHeading();
	}

	/**
	 * Koeffizienten fuer Einparken und Ausparken uebergeben
	 * 
	 * @param matrix
	 *            Matrix mit berechneten Parametern
	 */
	public void setCoefficients(Matrix matrix) {
		x1 = matrix.get(0, 0);
		x2 = matrix.get(1, 0);
		x3 = matrix.get(2, 0);
		x4 = matrix.get(3, 0);
	}

	/**
	 * program for first and second defence
	 */
	private void firstExampleProgram() {
		double sollAngle = Math.PI / 2.0 * wheelDistance / wheelDiameter;
		if (markerExample == 0) { // erste Geradeausfahrt
			drive(10, 0);
			currentAngle = 0;
			if (distance >= 1.5) {
				markerExample = 1;
				stop();
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} else if (markerExample == 1) { // erste Drehung
			drive(0, Math.PI / 12.0);
			distance = 0;
			if (currentAngle >= sollAngle - 3 / 18 * Math.PI) {
				markerExample = 2;
				stop();
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} else if (markerExample == 2) { // zweite Geradeausfahrt
			drive(5, 0);
			currentAngle = 0;
			if (distance >= 0.3) {
				markerExample = 3;
				stop();
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} else if (markerExample == 3) { // zweite Drehung
			drive(0, Math.PI / 6.0);
			distance = 0;
			if (currentAngle >= sollAngle) {
				markerExample = 4;
				stop();
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

		} else if (markerExample == 4) { // Uebergehen in Line-Control
			lineControl();
			if (intervalContains(-10, 0, (navigation.getPose().getHeading() / Math.PI * 180))) {
				markerExample = 5;
			}
		} else if (markerExample == 5) { //SetPose
			driveToPose(-1.8, -0.6, -Math.PI / 2.0, 2);
			if (phiReached) {
				markerExample = 6;
				currentAngle = 0;
				navigation.setPoseFirstExampleProgram(navigation.getPose().getX(), navigation.getPose().getY(),
						3 * Math.PI / 2.0);
			}
		} else if (markerExample == 6) { // Linienverfolgung
			lineControl();
			currentAngle += (Math.abs(phiIstR) + Math.abs(phiIstL)) / 2;
			if (intervalContains(Math.PI / 2.0 - 0.15, Math.PI / 2.0 + 0.15, navigation.getPose().getHeading())) {
				markerExample = 7;
				phiReached = false;
				xReached = false;
				yReached = false;
			}
		} else if (markerExample == 7) { //letzte Drehung
			drive(0, -Math.PI / 6.0);
			if (intervalContains(0, Math.toRadians(3), navigation.getPose().getHeading())) {
				markerExample = 8;
			}
		} else if (markerExample == 8) { //Linienverfolgung bis Luecke
			lineControl();
			if (perception.getFrontSideSensorDistance() > 200 && perception.getBackSideSensorDistance() < 200) {
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				markerExample = 9;
			}
		} else if (markerExample == 9) { //Einparken
			x1 = 0.0022;
			x2 = -0.1;
			x3 = 2.5;
			x4 = 0;
			parkControl();
		}

		// calculate new variables for VW-Control sequences
		distance += (phiIstR + phiIstL) / 4.0 * wheelDiameter;
		currentAngle += (Math.abs(phiIstR) + Math.abs(phiIstL)) / 2;
	}

	/**
	 * example program for second defence
	 */
	private void secondExampleProgram() {
	
		if (luecke == 0) { //Linienverfolgug bsi erste Luecke
			lineControl();
			if (perception.getFrontSideSensorDistance() > 200 && perception.getBackSideSensorDistance() < 200) {
				luecke++;
			}
		} else if (luecke == 1) { //Einparken
			if (!statusParked && !statusParkedOut) {
				x1 = 0.0045;
				x2 = -0.2026;
				x3 = 4.03856;
				x4 = 0;
				parkControl();
			}
			if (markerSecondExample == 1) { //Rueckwaertsfahren
				drive(-10, 0);
				if (perception.getBackSensorDistance() <= 50) {
					markerSecondExample = 2;
					resetParkVariables();
				}
			}
			if (markerSecondExample == 2) { //Ausparken
				x1 = -6.7541;
				x2 = -0.04559;
				x3 = -2.0258;
				x4 = 0;
				if ((!(lineSensorRight == 0) && !(lineSensorLeft == 2))
						|| (!(lineSensorRight == 2) && !(lineSensorRight == 0))) {
					parkControl();
				} else {
					markerSecondExample = 3;
				}
			}
			if (markerSecondExample == 3) { //zur letzten Luecke Linienverfolgung
				lineControl();
				if (perception.getFrontSideSensorDistance() > 200 && perception.getBackSideSensorDistance() < 200
						&& navigation.getCurrentLine() == 1) {
					luecke++;
					statusParked = false;
					statusParkedOut = false;
					markerSecondExample = 0;
				}
			}
		} else if (luecke == 2) {
			if (!statusParked && !statusParkedOut) { //einparken
				x1 = 0.00272;
				x2 = -0.12256;
				x3 = 2.83843;
				x4 = 0;
				// monitor.writeControlComment("in if fuer einparken");
				parkControl();
			}
			if (markerSecondExample == 4) { //Rueckwaertsfahren
				drive(-8, 0);
				if (perception.getBackSensorDistance() <= 50) {
					markerSecondExample = 5;
					resetParkVariables();
				}
			}
			if (markerSecondExample == 5) { //ausparken
				x1 = -1.4856;
				x2 = -0.01;
				x3 = -1.2256;
				x4 = 0;
				if ((!(lineSensorLeft == 0) && !(lineSensorRight == 2))
						|| (!(lineSensorLeft == 2) && !(lineSensorRight == 0))) {
					parkControl();
				} else {
					markerSecondExample = 6;
				}
			}
			if (markerSecondExample == 6) { //linienverfolgung
				lineControl();
			}
		}
	}

	/**
	 * reset variables which are relevant for parking
	 */
	public void resetParkVariables() {
		schritt = 0;
		xReached = false;
		yReached = false;
		phiReached = false;
		setStartCoordinates = true;
		sign = 0;
	}
}