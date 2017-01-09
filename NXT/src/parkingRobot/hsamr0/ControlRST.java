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
	static int marker = 0; // zeigt an, ob in rechtskurve
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
	double destinationX = 1.8; // in m
	double destinationY = 0.6; // in m
	double destinationPhi = -Math.PI / 2.0;// 11.0/36.0*Math.PI;
	static boolean direction = false;
	static boolean xReached = false;
	static boolean yReached = false;
	static boolean phiReached = false;
	double outgoingPD = 0;

	/**
	 * Path Finder
	 */
	static double x1 = 0.0022;
	static double x2 = -0.1;
	static double x3 = 2.5;
	static double x4 = 0.0;
	static double d = 0.5; // Abtastabstand in cm
	static double abtastx = 0.0; // x-Wert an dem abgetastet wird
	static double timeOld = 0.0;
	static boolean inOrOut = true; // true wenn in, false wenn out
	static boolean beginningPark = true; // zeigt an, ob es sich um den Anfang
											// des parkens handelt, damit
											// Anfangsposition gesetzt werden
											// kann, reset, wenn parken beendet
	static double anfangx = 0.0;
	static double anfangy = 0.0;
	static boolean endParking = false;

	/**
	 * exampleProgram
	 */
	static int markerExample = 0;
	static boolean firstExample = false;
	static boolean markerOWR = true;
	static boolean markerFEP2 = false; // Abschnitt zwei, wird true ab
										// LineControl falsch herum
	static boolean markerFEP3 = false; // Abschnitt drei, wird true ab
										// LineControl wieder richtig herum zur
										// Parkluecke

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
	double angularVelocity = 0;// Math.PI/6;// 3;//0 //in rad/s

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

		// MONITOR (example)
		// what happens here
		// monitor.addControlVar("Marker");
		// monitor.addControlVar("phiSoll");
		// monitor.addControlVar("deltaX");
		// monitor.addControlVar("deltaY");
		// monitor.addControlVar("x");
		// monitor.addControlVar("y");
		// monitor.addControlVar("poseX");
		// monitor.addControlVar("poseY");
		// monitor.addControlVar("outgoingPD");
		// monitor.addControlVar("phiIst");
		// monitor.addControlVar("phiSoll");
		// monitor.addControlVar("omega");
		// monitor.addControlVar("calculatedy");
		// monitor.addControlVar("abtastx");
		// monitor.addControlVar("neux");
		// monitor.addControlVar("neuy");
		// monitor.addControlVar("generaly");
		// monitor.addControlVar("heading");
		// monitor.addControlVar("radius");
		// monitor.addControlVar("yDerivation");
		// monitor.addControlVar("yDerivationForward");

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
			// firstExampleProgram();
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
		// case EXAMPLE_ONE:
		// exec_example_one();
		// break;
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
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		// exampleProgram();
		this.drive(this.velocity, this.angularVelocity);
	}

	/**
	 * Funktion zum Anfahren einer Zielposition
	 */
	private void exec_SETPOSE_ALGO() {
		this.driveToPose(this.destinationX, this.destinationY, this.destinationPhi);
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		// Aufgabe 3.4
		// Werte der Parkluecke
		int version = 2;
		if (beginningPark) {
			anfangx = navigation.getPose().getX(); // Einheit m
			anfangy = navigation.getPose().getY(); // Einheit m
			destinationX = anfangx;
			destinationY = anfangy;
			destinationPhi = -(Math.PI / 2 - Math.atan(3 * x1 * anfangx * anfangx + x2 * 2 * anfangx + x3));
			driveToPose(destinationX, destinationY, destinationPhi);
			if (phiReached) {
				beginningPark = false;
			}
		} else if (!beginningPark && !endParking && version==1) {

			// Position des Roboters auf der Platte im grossen Koordinatensystem
			double generalx = navigation.getPose().getX(); // Einheit m
			double generaly = navigation.getPose().getY(); // Einheit m
			double heading = navigation.getPose().getHeading(); // Einheit rad
			

			// Position des Roboters in der Parkluecke
			double neuy = generalx - anfangx; // Einheit m
			double neux = -generaly - anfangy; // Einheit m
			
			monitor.writeControlComment("neux=" + neux);
			// Abtastung neu setzen
			abtastx = neux * 100; // Einheit cm

			

			// Zeit auslesen und berechnen
			double time = System.currentTimeMillis(); // Einheit millisekunden
			double deltaTime = time - timeOld; // Einheit millisekunden
			timeOld = time; // fuer naechsten Durchlauf // Einheit millisekunden

			// Polynomberechnung
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

			double phi = Math.atan(yDerivation);
			double phiForward = Math.atan(yDerivationForward);
			double deltaPhi = 0;

			double phiKorrigiert=Math.PI-phi;
			double phiForwardKorrigiert=Math.PI-phiForward;
			// Wendung in der Mitte des Pfads, jeweils fuer einparken und
			// ausparken
			if (abtastx < 15 && inOrOut) {// eiparken
				deltaPhi = (phiForwardKorrigiert - phiKorrigiert);
			} else if (abtastx > 15 && inOrOut) {// einparken
				deltaPhi = (phiForwardKorrigiert - phiKorrigiert);
			} else if (abtastx < 15 && !inOrOut) {// ausparken
				deltaPhi = -(phiForwardKorrigiert - phiKorrigiert);
			} else if (abtastx > 15 && !inOrOut) {// ausparken
				deltaPhi = (phiForwardKorrigiert - phiKorrigiert);
			}

			double omega = deltaPhi / deltaTime * 1000; // * 1000 wegen
			// umrechnung zu s

			// Berechnung von omega mit w=deltaPhi/deltaT
			double velocity = 4.5; // konstante Geschwindigkeit in cm/s
			drive(velocity, omega);

			// Stoppbedingungen
			boolean markerx = intervalContains(0.25, 0.3, neux);
			if (markerx) {
				endParking = true;
			}
			} else if (!beginningPark && !endParking && version==2) {
				

				// Position des Roboters auf der Platte im grossen Koordinatensystem
				double generalx = navigation.getPose().getX(); // Einheit m
				double generaly = navigation.getPose().getY(); // Einheit m
				double heading = navigation.getPose().getHeading(); // Einheit rad
				

				// Position des Roboters in der Parkluecke
				double neuy = generalx - anfangx; // Einheit m
				double neux = -generaly - anfangy; // Einheit m
				
				monitor.writeControlComment("neux=" + neux);
				// Abtastung neu setzen
				abtastx = neux * 100; // Einheit cm

				

				// Zeit auslesen und berechnen
				double time = System.currentTimeMillis(); // Einheit millisekunden
				double deltaTime = time - timeOld; // Einheit millisekunden
				timeOld = time; // fuer naechsten Durchlauf // Einheit millisekunden

				// Polynomberechnung
				double y=x1*abtastx*abtastx*abtastx+x2*abtastx*abtastx+x3*abtastx+x4;
				double yForward=x1*(abtastx + d)*(abtastx + d)*(abtastx + d)+x2*(abtastx + d)*(abtastx + d)+x3*(abtastx + d)+x4;
				
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

				double phi = Math.atan(yDerivation);
				double phiForward = Math.atan(yDerivationForward);
				double deltaPhi = 0;

				// Wendung in der Mitte des Pfads, jeweils fuer einparken und
				// ausparken
				if (abtastx < 15 && inOrOut) {// eiparken
					deltaPhi = (phiForward - phi);
				} else if (abtastx > 15 && inOrOut) {// einparken
					deltaPhi = (phiForward - phi);
//				} else if (abtastx < 15 && !inOrOut) {// ausparken
//					deltaPhi = -(phiForward - phi);
//				} else if (abtastx > 15 && !inOrOut) {// ausparken
//					deltaPhi = (phiForward - phi);
//				}
				}
				
				double hyp=Math.sqrt((yForward-y)*(yForward-y)+d*d);
				double k=Math.PI*2.0/deltaPhi;
				double r=k*hyp/Math.PI/2.0;
				
				double velocity = 4.5; // konstante Geschwindigkeit in cm/s
				double omega = velocity/r; // * 1000 wegen
				// umrechnung zu s

				// Berechnung von omega mit w=deltaPhi/deltaT
				 
				drive(velocity, omega);

				// Stoppbedingungen
				boolean markerx = intervalContains(0.29, 0.3, neux);
				if (markerx) {
					endParking = true;
				}
		} else if (endParking) { // Endausrichtung
			destinationX = navigation.getPose().getX();
			destinationY = navigation.getPose().getY();
			destinationPhi = 0;
			driveToPose(destinationX, destinationY, destinationPhi);

			if (intervalContains(0, 2, (navigation.getPose().getHeading() / Math.PI * 180))) {
				beginningPark = true;
				endParking = false;
				GuidanceAT.setParkmaneuverFinished();
			}
		}
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
		int version = 1; // 0 --> drei farbwerte (zickzack), 1--> PID version

		if (version == 0) { // Entscheidung je nach Version, ob zickzack oder
							// PID
			leftMotor.forward();
			rightMotor.forward();
			int lowPower = 3;
			int highPower = 40;

			// MONITOR (example)
			// monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
			// monitor.writeControlVar("RightSensor", "" +
			// this.lineSensorRight);

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
			// checkForTurn();
			if (this.lineSensorLeft == 2 && this.lineSensorRight == 0) { // wenn
																			// linker
																			// sensor
																			// auf
																			// schwarz
																			// kommt
				leftTurn();
			} else if (this.lineSensorLeft == 0 && this.lineSensorRight == 2) { // wenn
																				// rechter
																				// sensor
																				// auf
																				// schwarz
																				// kommt
				rightTurn();
			} else if (previousStatus == 1 || (this.lineSensorLeft == 2 && this.lineSensorRight == 2 && marker == 0)) { // wenn
				// links
				// immer
				// noch
				// auf
				// schwarz

				leftTurn();
				if (this.lineSensorRight == 0 && this.lineSensorLeft == 0) {
					previousStatus = 0;
				}
			} else if (previousStatus == 2 || (this.lineSensorLeft == 2 && this.lineSensorRight == 2)) { // wenn
																											// rechts
																											// immer
																											// noch
																											// auf
																											// schwarz
				rightTurn();
				if (this.lineSensorRight == 0 && this.lineSensorLeft == 0) {
					previousStatus = 0;
				}
			} else if (!turn) {
				previousStatus = 0;
				if (navigation.getCurrentLine()==3){
					straightForward(35.0);
				} else {
				straightForward(40.0);
				}
			} else if (turn) {
				previousStatus = 0;
				straightForward(20.0);
				// stop();
			}

			// Abbruchbedingung fuer Beispielprogramm, muss sonst auskommentiert
			// werden
			if (intervalContains(-10, 0, (navigation.getPose().getHeading() / Math.PI * 180)) && firstExample
					&& !markerFEP2 && !markerFEP3) {
				stop();
				try {
					Thread.sleep(3000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				destinationX = 1.8;
				destinationY = 0.6;
				destinationPhi = -Math.PI / 2.0;
				setCtrlMode(ControlMode.SETPOSE);
			}
			if (intervalContains(-10, 10, (navigation.getPose().getHeading() / Math.PI * 180))
					&& intervalContains(0, 0.05, navigation.getPose().getX()) && markerFEP2) {
				destinationX = navigation.getPose().getX();
				destinationY = navigation.getPose().getY();
				destinationPhi = 0;
				phiReached = false;
				xReached = false;
				yReached = false;
				setCtrlMode(ControlMode.SETPOSE);
			}
			if (markerFEP3 && perception.getFrontSideSensorDistance() > 200
					&& perception.getBackSideSensorDistance() < 200) {
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				setCtrlMode(ControlMode.PARK_CTRL);
			}
		}
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
		// monitor.writeControlComment("Rechte Power:" + powerRight + "linke
		// Power:" + powerLeft);
		leftMotor.setPower((int) powerLeft);
		rightMotor.setPower((int) powerRight);
	}

	/**
	 * turns the roboter left
	 */
	private void leftTurn() {
		setMotorPowers(-15, 48); // gut funktionierende Werte sind -15, 45
		previousStatus = 1; // Status fuer naechste Drehung setzen

		// Parameter ruecksetzen
		integralE = 0;
		eold = 0;

		// Funktion um Linienzahl zu erhoehen
		// GuidanceAT.incrementCurrentLine();

	}

	/**
	 * turns the roboter right
	 */
	private void rightTurn() {
		setMotorPowers(45, -20);// gut funktionierende Werte sind 45, -15
		previousStatus = 2; // Status fuer naechste Drehung setzen

		// Parameter ruecksetzen
		integralE = 0;
		eold = 0;
		marker = 1;

		// Funktion um Linienzahl zu erhoehen
		// GuidanceAT.incrementCurrentLine();
	}

	/**
	 * PID for straight forward driving
	 */
	private void straightForward(double powerOffset) {

		// Variables
		marker = 0;
		// double powerOffset = 50;// 45;// 30
		int actRightSensor = this.lineSensorRightV;
		int actLeftSensor = this.lineSensorLeftV;

		// parameters for PID
		// gut funktionierende Werte fuer vollen Akku sind: 0.067, 0.0074, 0.067
		final double kp = 0.067;
		final double ki = 0.0;// 0.0078;
		final double td = 0.06;// 0.063;

		// Calculation parameters PID
		double deltaBrightness = actRightSensor - actLeftSensor;
		double e = 0 - deltaBrightness; // Fuehrungsgroesse = 0
		double diffE = (e - eold);
		integralE += e;

		// Motorpower berechnen
		double outgoingPID = kp * e + td * diffE + ki * integralE; // PID-Regler
		double powerLeft = powerOffset + outgoingPID;
		double powerRight = powerOffset - outgoingPID;

		// setting new variables
		eold = e;

		setMotorPowers(powerLeft, powerRight);
	}

	private void checkForTurn() {
		// double poseX = navigation.getPose().getX();
		// double poseY = navigation.getPose().getY();
		// monitor.writeControlVar("poseX", "" + poseX);
		// monitor.writeControlVar("poseY", "" + poseY);
		double frontTriang = perception.getFrontSensorDistance() / 1000.0;
		if (frontTriang <= 0.25) {
			turn = true;
		} else {
			turn = false;
		}
	}

	/**
	 * stops the NXT
	 */
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
	 *            angle velocity of the robot in rad/s
	 */
	private void drive(double v, double omega) {
		// defining variables
		monitor.writeControlComment("in drive mit parametern v=" + v + "und w=" + omega);
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
	 * Methode PID Regler fuer das Anpassen der Geschwindigkeit an
	 * Sollgeschwindigkeit fuer den rechten Motor
	 * 
	 * @param vSoll
	 *            Sollgeschwindigkeit in m/s
	 * @return @return powerCalculated Wert der Geschwindigkeit in power
	 *         ausgedrueckt
	 */
	private int controlRightMotor(double vSoll) {
		double factorVPower = 0.0034; // Umrechnung in Power Wert mit:
										// velocity/factor=power

		// read data
		// angleMeasurementRight = encoderRight.getEncoderMeasurement();
		phiIstR = Math.toRadians(angleMeasurementRight.getAngleSum()); // Umrechnung
																		// in
																		// rad

		// calculate data
		double tIst = angleMeasurementRight.getDeltaT() / 1000.0; // Umrechnung
																	// in
																	// s
		double vIst = phiIstR / tIst * wheelDiameter / 2.0; // Einheit rad/s*m
		monitor.writeControlComment("phiIstR" + phiIstR);
		double e = vSoll - vIst;

		integralERightMotor += e;

		// control
		double kp = 0.14;
		double ki = 0.33;
		double td = 0.27;
		outgoingPID = kp * e + ki * integralERightMotor + td * (e - eoldRightMotor);
		// monitor.writeControlComment("integralERightMotor" +
		// integralERightMotor);
		// monitor.writeControlComment("eoldRightMotor" + eoldRightMotor);
		monitor.writeControlComment("outPID" + outgoingPID);
		// set new variables
		eoldRightMotor = e;

		// set power
		double powerCalculated = (vSoll + outgoingPID) / factorVPower; // Umrechnung
																		// in
																		// Power-Wert
		monitor.writeControlComment("powerCalculated" + powerCalculated);
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
		double factorVPower = 0.0034; // Umrechnung in Power Wert mit:
										// velocity/factor=power

		// read data
		// angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		phiIstL = Math.toRadians(angleMeasurementLeft.getAngleSum()); // Umrechung
																		// deg
																		// to
																		// rad

		// calculate data
		double tIst = angleMeasurementLeft.getDeltaT() / 1000.0; // Umrechnung
																	// in
																	// s
		double vIst = phiIstL / tIst * wheelDiameter / 2.0; // Einheit rad/s*m
		double e = vSoll - vIst;

		// set new variables
		integralELeftMotor += e;

		// control
		// Werte: 0.13, 0.255, 0.24
		double kp = 0.14;// 0.45;// 0.8;
		double ki = 0.33;// 0.3;// 0.3;
		double td = 0.27;// 0.25// 0.1;// 0.1;
		outgoingPID = kp * e + ki * integralELeftMotor + td * (e - eoldLeftMotor);

		// set new variables
		eoldLeftMotor = e;

		// set power
		double powerCalculated = (vSoll + outgoingPID) / factorVPower; // Umrechnung
																		// in
																		// Power-Wert
		return (int) (powerCalculated);

	}

	/**
	 * program for first defence
	 */
	private void firstExampleProgram() {
		double sollAngle = Math.PI / 2.0 * wheelDistance / wheelDiameter;
		// monitor.writeControlComment("Sollwinkel" + sollAngle);
		firstExample = true;
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
			setCtrlMode(ControlMode.LINE_CTRL);
		}

		// calculate new variables
		distance += (phiIstR + phiIstL) / 4.0 * wheelDiameter;
		currentAngle += (Math.abs(phiIstR) + Math.abs(phiIstL)) / 2;
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
	 * @param pose
	 */
	public void setDestinationPose(Pose pose) {
		destinationX = pose.getX();
		destinationY = pose.getY();
		destinationPhi = pose.getHeading();
	}

	/**
	 * Koeffizienten fuer Einparken und Ausparken uebergeben
	 * 
	 * @param Matrix
	 *            Matrix mit berechneten den Parametern
	 */
	public void setCoefficients(Matrix matrix) {
		x1 = matrix.get(0, 0);
		x2 = matrix.get(1, 0);
		x3 = matrix.get(2, 0);
		x4 = matrix.get(3, 0);
	}

	/**
	 * setzt, ob beim Einparken oder Ausparken
	 * 
	 * @param status
	 */
	public void setInOrOut(boolean status) {
		inOrOut = status;
	}

	private void driveToPose(double xIn, double yIn, double phiIn) {
		double phiSoll = 0;
		destinationX = xIn;
		destinationY = yIn;
		destinationPhi = phiIn;

		// aktuelle x und y Position
		double x = navigation.getPose().getX();
		double y = navigation.getPose().getY();
		// monitor.writeControlVar("x", "" + x);
		// monitor.writeControlVar("y", "" + y);

		// Vorberechnungen
		double deltaX = destinationX - x;
		double deltaY = destinationY - y;
		// monitor.writeControlVar("deltaX", "" + deltaX);
		// monitor.writeControlVar("deltaY", "" + deltaY);

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

		// monitor.writeControlVar("phiSoll", "" + phiSoll);
		// aktueller Winkel
		double phiIst = navigation.getPose().getHeading();

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

		// Regler
		// monitor.writeControlComment("direction=" + direction);
		if (direction && !xReached && !yReached) {
			// monitor.writeControlComment("in if");
			double e = phiSoll - phiIst; // phiSoll-phiIst
			double td = 0.03;
			double kp = 0.05;
			outgoingPD = kp * e + td * (e - eOldSetPose); // muss auf VW
															// Control
															// wirken

			// monitor.writeControlVar("outgoingPD", "" + outgoingPD);
			// monitor.writeControlVar("phiIst", "" + phiIst);
			// monitor.writeControlVar("phiSoll", "" + phiSoll);
			drive(10, outgoingPD);
			// set variables
			eOldSetPose = e;
		}
		// Endroutine //normalerweise Toleranzwert 0.005
		if (intervalContains(destinationX - 0.005, destinationX + 0.005, x)
				&& intervalContains(destinationY - 0.005, destinationY + 0.005, y)) {
			xReached = true;
			yReached = true;
			phiReached = false;
			// monitor.writeControlComment("STOP");
		}
		if (intervalContains(destinationX - 0.05, destinationX + 0.05, x)
				&& intervalContains(destinationY - 0.1, destinationY + 0.1, y) && firstExample) {
			xReached = true;
			yReached = true;
			phiReached = false;
			// monitor.writeControlComment("STOP");
		}

		if (xReached && yReached && !phiReached) {
			if (xReached && yReached && phiIst > destinationPhi) { // Beziehen
																	// auf
																	// Winkel
																	// des
																	// Roboters
																	// =
																	// Sollwinkel

				drive(0, -Math.PI / 6.0);
				// monitor.writeControlComment("Rechtsdrehung");
			} else if (xReached && yReached && phiIst < destinationPhi) { // Beziehen
																			// auf
																			// Winkel
																			// des
																			// Roboters
																			// =
																			// Sollwinkel

				drive(0, Math.PI / 6.0);
				// monitor.writeControlComment("Linksdrehung");
			}
		}
		if (xReached && yReached
				&& intervalContains(destinationPhi - Math.toRadians(5), destinationPhi + Math.toRadians(5), phiIst)) {
			phiReached = true;
			// monitor.writeControlComment("Stop weil Zielposition erreicht");
			// setCtrlMode(ControlMode.INACTIVE);
		}
		if (firstExample && phiReached && !markerFEP2) {
			if (markerOWR) {
				distance = 0;
			}
			markerOWR = false;
			drive(5, 0);
			distance += (phiIstR + phiIstL) / 4.0 * wheelDiameter;
			if (distance >= 0.05) {
				setCtrlMode(ControlMode.LINE_CTRL);
				markerFEP2 = true;
			}
		}
		if (markerFEP2 && intervalContains(0, 10, (navigation.getPose().getHeading() / Math.PI * 180))) {
			distance = 0;
			markerFEP2 = false;
			markerFEP3 = true;
			setCtrlMode(ControlMode.LINE_CTRL);
		}
	}
}
