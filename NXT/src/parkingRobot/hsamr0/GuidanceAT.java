package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;
import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.hsamr0.ControlRST;
import parkingRobot.hsamr0.HmiPLT;
import parkingRobot.hsamr0.NavigationAT;
import parkingRobot.hsamr0.PerceptionPMP;

import Jama.Matrix;

/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students
 * of electrical engineering with specialization 'automation, measurement and
 * control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego
 * NXT system witch can perform parking maneuvers on an predefined course. To
 * fulfill the interdisciplinary aspect of this project the software structure
 * is divided in 5 parts: human machine interface, guidance, control, perception
 * and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be
 * controlled by one or more finite state machines (FSM). It may be advantageous
 * to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be
 * realized in one main module class. Every class (except guidance) has
 * additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module
 * class thread is only handled in a synchronized context to avoid inconsistent
 * or corrupt data!
 */
public class GuidanceAT {

	/**
	 * states for the main finite state machine. This main states are
	 * requirements because they invoke different display modes in the human
	 * machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and detecting parking
		 * slots
		 */
		SCOUT,
		/**
		 * indicates that robot is following the line to a special parking slot
		 * (ID) and park into this slot
		 */
		PARK_THIS,
		/**
		 * indicates that robot is following the line to the next parking slot
		 * and park into this slot
		 */
		PARK_NOW,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}

	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus = CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering
	 * the actual state
	 */
	protected static CurrentStatus lastStatus = CurrentStatus.INACTIVE;

	/**
	 * one line of the map of the robot course. The course consists of a closed
	 * chain of straight lines. Thus every next line starts where the last line
	 * ends and the last line ends where the first line starts. This
	 * documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(0, 0, 180, 0);
	static Line line1 = new Line(180, 0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30, 30, 30);
	static Line line5 = new Line(30, 30, 30, 60);
	static Line line6 = new Line(30, 60, 0, 60);
	static Line line7 = new Line(0, 60, 0, 0);
	/**
	 * map of the robot course. The course consists of a closed chain of
	 * straight lines. Thus every next line starts where the last line ends and
	 * the last line ends where the first line starts. All above defined lines
	 * are bundled in this array and to form the course map.
	 */
	static Line[] map = { line0, line1, line2, line3, line4, line5, line6, line7 };

	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args
	 *            standard string arguments for main method
	 * @throws Exception
	 *             exception for thread management
	 */
	public static void main(String[] args) throws Exception {
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus = CurrentStatus.EXIT;

		// Generate objects

		NXTMotor leftMotor = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi hmi = new HmiPLT(perception, navigation, control, monitor);

		monitor.startLogging();

		while (true) {
			showData(navigation, perception);

			switch (currentStatus) {
			case SCOUT:
				// MONITOR (example)
				// monitor.writeGuidanceComment("Guidance_Driving");

				// Into action
				if (lastStatus != CurrentStatus.SCOUT) {
					control.setCtrlMode(ControlMode.VW_CTRL);
					navigation.setDetectionState(true);
				}

			// While action
			{
				// nothing to do here
			}

				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
					currentStatus = CurrentStatus.INACTIVE;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.INACTIVE;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}

				// Leave action
				if (currentStatus != CurrentStatus.SCOUT) {
					navigation.setDetectionState(false);
				}
				break;
			case PARK_THIS:
				// Into action
				if (lastStatus != CurrentStatus.PARK_THIS) {
					// aktion die auszuführen ist
				}

				// While action

				// State transition check

				// Leave action

				break;
			case PARK_NOW:
				// Into action
				if (lastStatus != CurrentStatus.PARK_NOW) {
					// aktion die auszuführen ist
				}

				// While action

				// State transition check

				// Leave action

				break;
			case INACTIVE:
				// Into action
				if (lastStatus != CurrentStatus.INACTIVE) {
					control.setCtrlMode(ControlMode.INACTIVE);
				}

			// While action
			{
				// nothing to do here
			}

				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {
					currentStatus = CurrentStatus.SCOUT;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.SCOUT;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}

				// Leave action
				if (currentStatus != CurrentStatus.INACTIVE) {
					// nothing to do here
				}
				break;
			case EXIT:
				hmi.disconnect();
				/**
				 * NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
				 * // monitor.sendOfflineLog();
				 */
				monitor.stopLogging();
				System.exit(0);
				break;
			default:
				break;
			}

			Thread.sleep(100);
		}
	}

	/**
	 * returns the actual state of the main finite state machine as defined by
	 * the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus() {
		return GuidanceAT.currentStatus;
	}

	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation
	 *            reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception) {
		LCD.clear();

		LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);

		// perception.showSensorData();

		// if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
		// LCD.drawString("HMI Mode SCOUT", 0, 3);
		// }else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
		// LCD.drawString("HMI Mode PAUSE", 0, 3);
		// }else{
		// LCD.drawString("HMI Mode UNKNOWN", 0, 3);
		// }
	}

	// Polynom für Pfadgenerator berechnen

	/*
	 * Finden der Polynomkoeffizienten mittels lösen eines LGS (Ax = b)
	 * 
	 * 1.Bedingung: f(0)=0 2.Bedingung: f(Lückentiefe)=Lückenbreite 3.Bedingung:
	 * f(Lückentiefe/2)=Lückenbreite/2 4.Bedingung: f'(Lückentiefe/2)=1 <--
	 * Steigung des Polynoms zur halben Breite und halb abgefahrenen Strecke
	 * gleich 45°
	 * 
	 */
	/*
	private double[][] koeffizientenberechnung(double lueckenbreite, double lueckentiefe) {
	
//	double lueckenbreite = 180;
//	double lueckentiefe = 30;
	double steigung = 1;

	double[][] vals = { { 0                              , 0                         , 0                 , 1 }, 
						{ Math.pow(lueckentiefe, 3)      , Math.pow(lueckentiefe,2)  , lueckentiefe      , 1 },
						{ Math.pow(lueckentiefe/2, 3)    , Math.pow(lueckentiefe/2,2), (lueckentiefe / 2), 1 },
						{ 3 * Math.pow(lueckentiefe/2, 2), 2 * (lueckentiefe / 2)    , 1                 , 0 }};

	double[][] solvals = { {0              }, 
						   {lueckenbreite  }, 
						   {lueckenbreite/2},
						   {steigung       }};
	
	Matrix b = new Matrix(solvals);
	Matrix A = new Matrix(vals);

	Matrix x = A.solve(b);

	return x.getArray();
	}*/
}