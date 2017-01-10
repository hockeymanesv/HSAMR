package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;
import lejos.util.Matrix;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;
import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;

import parkingRobot.hsamr0.ControlRST;
import parkingRobot.hsamr0.HmiPLT;
import parkingRobot.hsamr0.NavigationAT;
import parkingRobot.hsamr0.PerceptionPMP;


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
		 * indicates that robot is parking in a parking slot
		 */
		PARKED,
		/**
		 * indicates that robot is parking out of the parking slot
		 */
		PARK_OUT,
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
	
	static Line currentLine = line0;
	static Line lastLine = line7;


	
	
	static boolean dummy3 = false;
	static double dummy4 = 0;
	
	
	
	
	
	
	
	
	
	/**
	 * states for the sub state machine(park_this-modus)
	 */
	private enum SM_park_this {
		/**
		 * indicates that robot is following the line until he is on the line with the correct ID
		 */
		DRIVE_TO_BEGINNING_OF_SLOTLINE,
		/**
		 * indicates that robot is following the line to the beginning of the parking slot
		 */
		DRIVE_TO_SLOT_BEGINNING,
		/**
		 * calculate the polynomial of path
		 */
		PATH_GENERATOR,
		/**
		 * indicates that robot is parking into the slot
		 */
		PARKING_MANEUVER,
		/**
		 * indicates that robot is correcting the parking maneuver
		 */
		CORRECT_PARKING_MANEUVER,
		/**
		 * indicates that robot is correcting the parking_pose
		 */
		CORRECT_PARKING_POSE
	}
	/**
	 * state in which the sub state machine(park_this-modus) is running at the moment
	 */
	protected static SM_park_this sm_park_this_currentStatus = SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE;  
	/**
	 * state in which the sub state machine(park_this-modus) was running before entering
	 * the actual state
	 */
	protected static SM_park_this sm_park_this_lastStatus = SM_park_this.CORRECT_PARKING_POSE;
	
	/**
	 * 
	 */
	static ParkingSlot selected_Parking_Slot = null;			/////////////7 noch ordentlich beschriften
	//static ParkingSlot selected_Parking_Slot = new ParkingSlot(ParkingSlots.size(), new Point((float)backBoundaryPositionX,(float)backBoundaryPositionY), new Point((float)frontBoundaryPositionX,(float)frontBoundaryPositionY), ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING, 0);			/////////////7 noch ordentlich beschriften
	
	
	static int selected_Parking_Slot_Num = 0;
	static Line selected_Parking_Slot_Line = line1;
	static double selected_Parkingslot_Slotrange = 0;
	static Matrix coefficienten = null;
	static boolean parkManeuver_finished = false;
	
	static boolean marker1 = false;
	static double range_slotbeginning = 0;
	static boolean marker2 = false;
	
	static int dummy = 0;
	static int dummy2 = 0;
	static int kacke = 0; 
	
	static boolean robo_in_parkingmovement = false;

	
	/**
	 * states for the sub state machine(park_now-modus)
	 */
	private enum SM_park_now {
		/**
		 * indicates that robot is following the line until he is on the line with the correct ID
		 */
		LOOKING_FOR_SLOTS,
		/**
		 * indicates that robot is checking the known Parking slots
		 */
		IS_THE_SLOT_KNOWN,
		/**
		 * indicates that robot is measuring the slot
		 */
		MEASURE_SLOT,
		/**
		 * indicates that robot is following the line to the beginning of the parking slot
		 */
		DRIVE_TO_SLOT_BEGINNING,
		/**
		 * calculate the polynomial of path
		 */
		PATH_GENERATOR,
		/**
		 * indicates that robot is parking into the slot
		 */
		PARKING_MANEUVER,
		/**
		 * indicates that robot is correcting the parking maneuver
		 */
		CORRECT_PARKING_MANEUVER,
		/**
		 * indicates that robot is correcting the parking_pose
		 */
		CORRECT_PARKING_POSE
	}
	/**
	 * state in which the sub state machine(park_now-modus) is running at the moment
	 */
	protected static SM_park_now sm_park_now_currentStatus = SM_park_now.LOOKING_FOR_SLOTS;
	/**
	 * state in which the sub state machine(park_now-modus) was running before entering
	 * the actual state
	 */
	protected static SM_park_now sm_park_now_lastStatus = SM_park_now.CORRECT_PARKING_POSE;
	
	
	
	static float parkNow_backBoundery = 0;
	static float parkNow_frontBoundery = 0;
	
	
	static double parkNow_slot_width = 0;

	
	
	/**
	 * states for the sub state machine(park_out-modus)
	 */
	private enum SM_park_out {
		/**
		 * calculate the polynomial of path
		 */
		PATH_GENERATOR,
		/**
		 * indicates that robot is parking into the slot
		 */
		OUTPARKING_MANEUVER,
		/**
		 * indicates that robot is correcting the parking maneuver
		 */
		CORRECT_OUTPARKING_MANEUVER,
	}
	
	/**
	 * state in which the sub state machine(park_now-modus) is running at the moment
	 */
	protected static SM_park_out sm_park_out_currentStatus = SM_park_out.PATH_GENERATOR;
	/**
	 * state in which the sub state machine(park_now-modus) was running before entering
	 * the actual state
	 */
	protected static SM_park_out sm_park_out_lastStatus = SM_park_out.CORRECT_OUTPARKING_MANEUVER;
	
	
	
	static double park_out_slotrange = 0;
	
	
	
	
	
	
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args
	 *            standard string arguments for main method
	 * @throws Exception
	 *             exception for thread management
	 */
	public static void main(String[] args) throws Exception {
		currentStatus = CurrentStatus.INACTIVE;						//////////////////////////////////////////////7
		lastStatus = CurrentStatus.EXIT;

		// Generate objects

		NXTMotor leftMotor = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
//		perception.calibrateLineSensors();

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
					control.setCtrlMode(ControlMode.LINE_CTRL);
					navigation.setDetectionState(true);	
				}

				// While action
				{
					// nothing to do here
				}

				// State transition check
				lastStatus = currentStatus;
				
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {				//SCOUT_MODUS
					currentStatus = CurrentStatus.SCOUT;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.SCOUT;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
					
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS) {	//PARK_THIS_MODUS
					currentStatus = CurrentStatus.PARK_THIS;
				
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW) {	//PARK_NOW_MODUS
					currentStatus = CurrentStatus.PARK_NOW;
					
				} else if (Button.ESCAPE.isDown()) {								//EXIT_PROJECT
					currentStatus = CurrentStatus.EXIT;								
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
					currentStatus = CurrentStatus.INACTIVE;
				}
				
				
				// Leave action
				if (currentStatus != CurrentStatus.SCOUT) {
					navigation.setDetectionState(false);
				}
				break;
			case PARK_THIS:
				// Into action
				if (lastStatus != CurrentStatus.PARK_THIS) {
					sm_park_this_currentStatus = SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE;
//					sm_park_this_currentStatus = SM_park_this.DRIVE_TO_SLOT_BEGINNING;		//Probeversuche
					
					selected_Parking_Slot_Num = hmi.getSelectedParkingSlot();
					selected_Parking_Slot = navigation.getParkingSlots()[selected_Parking_Slot_Num];
					selected_Parking_Slot_Line = map[selected_Parking_Slot.getLine()];							////////////////////////
					range_slotbeginning = (selected_Parking_Slot.getBackBoundaryPosition().getX() - 10);
					
					dummy2 = selected_Parking_Slot.getLine();
				}
				
				// While action
				/**
				 * sub-state machine for park this modus
				 */
				switch (sm_park_this_currentStatus) {
				case DRIVE_TO_BEGINNING_OF_SLOTLINE:
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE) {
						control.setCtrlMode(ControlMode.LINE_CTRL);	
					}
					// While action
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if (selected_Parking_Slot_Line != currentLine){
						marker1 = true;
					}
					if(/*marker1 &&*/ (selected_Parking_Slot_Line == currentLine)){					///////////////7 test... auskommentierung zur rückgängigmachung killen
						sm_park_this_currentStatus = SM_park_this.DRIVE_TO_SLOT_BEGINNING;
					}
					
					// Leave action	
					if (sm_park_this_currentStatus != SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE) {
						control.setCtrlMode(ControlMode.INACTIVE);
						marker1 = false;
					}	
					break;
				case DRIVE_TO_SLOT_BEGINNING:
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.DRIVE_TO_SLOT_BEGINNING) {
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}
					// While action

					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if ((navigation.getPose().getX() * 100) > range_slotbeginning){
						marker2 = true;
					}
					if (marker2 && (perception.getFrontSideSensorDistance() > 200) && (perception.getBackSideSensorDistance() < 200)){
						sm_park_this_currentStatus = SM_park_this.PATH_GENERATOR;					////////////////////////7
					}
					
					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.DRIVE_TO_SLOT_BEGINNING) {
						control.setCtrlMode(ControlMode.INACTIVE);
						marker2 = false;
					}
					break;
				case PATH_GENERATOR:
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.PATH_GENERATOR) {
					
					}

					// While action;
					if(selected_Parking_Slot.getLine() % 2 == 0){
						selected_Parkingslot_Slotrange = Math.abs(selected_Parking_Slot.getBackBoundaryPosition().getX() - selected_Parking_Slot.getFrontBoundaryPosition().getX());
					} else if (selected_Parking_Slot.getLine() % 2 == 1){
						selected_Parkingslot_Slotrange = Math.abs(selected_Parking_Slot.getBackBoundaryPosition().getY() - selected_Parking_Slot.getFrontBoundaryPosition().getY());
					} 
					park_out_slotrange = selected_Parkingslot_Slotrange;
					
					coefficienten = coefficient_calculation(selected_Parkingslot_Slotrange, 30.0, 1);
					control.setCoefficients(coefficienten);
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if(coefficienten != null){
						sm_park_this_currentStatus = SM_park_this.PARKING_MANEUVER;
					}

					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.PATH_GENERATOR) {
						coefficienten = null;
					}
					break;
				case PARKING_MANEUVER:
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.PARKING_MANEUVER) {
						
						robo_in_parkingmovement = true;
						
						kacke = 1;
						control.setCtrlMode(ControlMode.PARK_CTRL);						
					}
					// While action

					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if(parkManeuver_finished){
						
						sm_park_this_currentStatus = SM_park_this.CORRECT_PARKING_POSE;    //////////////////////  verzweigen auf korrektur bei kollision
					}
					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.PARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					break;
				case CORRECT_PARKING_MANEUVER:
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.CORRECT_PARKING_MANEUVER) {
						
					}
					// While action

					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;

					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.CORRECT_PARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					break;
				case CORRECT_PARKING_POSE:
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.CORRECT_PARKING_POSE) {
						Sound.beepSequenceUp();

						control.setAngularVelocity(0.0);		///// rad/s
						control.setVelocity(-5.0);						///// cm/s
						control.setCtrlMode(ControlMode.VW_CTRL);
					}
					// While action

					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;

					if (perception.getBackSensorDistance() < 60){
						sm_park_this_currentStatus = SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE;
					}
					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.CORRECT_PARKING_POSE) {
						control.setCtrlMode(ControlMode.INACTIVE);
						currentStatus = CurrentStatus.PARKED;				///////777 leave PARK_THIS
					}
					break;
				default:
					break;
				}
				//End of sub-state machine for park this modus
				
				// State transition check
				lastStatus = currentStatus;
				if (Button.ESCAPE.isDown()) {								//EXIT_PROJECT
					currentStatus = CurrentStatus.EXIT;								
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					}
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
					currentStatus = CurrentStatus.INACTIVE;
				}
				
				// Leave action
				if (currentStatus != CurrentStatus.PARK_THIS) {
	
				}
				break;
			case PARK_NOW:
				// Into action
				if (lastStatus != CurrentStatus.PARK_NOW) {
					sm_park_now_currentStatus = SM_park_now.LOOKING_FOR_SLOTS;      /////////////////
//					sm_park_now_currentStatus = SM_park_now.DRIVE_TO_SLOT_BEGINNING;
//					sm_park_now_currentStatus = SM_park_now.MEASURE_SLOT;
				}

				// While action
				/**
				 * sub-state machine for park this modus
				 */
				switch (sm_park_now_currentStatus) {
				case LOOKING_FOR_SLOTS:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.LOOKING_FOR_SLOTS) {
						control.setCtrlMode(ControlMode.LINE_CTRL);	
					}
					// While action
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					if ((perception.getFrontSideSensorDistance() > 200) && (perception.getBackSideSensorDistance() < 200)){
						sm_park_now_currentStatus = SM_park_now.IS_THE_SLOT_KNOWN;
					}
					
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.LOOKING_FOR_SLOTS) {       ////////////////////////
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				case IS_THE_SLOT_KNOWN:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.IS_THE_SLOT_KNOWN) {
						Sound.beepSequenceUp();
					}
					// While action
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					ParkingSlot slots[] = navigation.getParkingSlots();
					boolean slotFound = false;
					
					for(int i = 0;i < slots.length; i++){
						if(Math.abs(slots[i].getBackBoundaryPosition().getX() - (navigation.getPose().getX())/100) < 10 && Math.abs(slots[i].getBackBoundaryPosition().getY() - (navigation.getPose().getY()/100)) < 10){
			                slotFound = true;
							break;
			            }
			        }
					
					if(slotFound){
						sm_park_now_currentStatus = SM_park_now.PATH_GENERATOR;
					} else if (!slotFound){
						sm_park_now_currentStatus = SM_park_now.MEASURE_SLOT;
					}
					
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.IS_THE_SLOT_KNOWN) {

					}	
					break;
				case MEASURE_SLOT:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.MEASURE_SLOT) {
						control.setCtrlMode(ControlMode.LINE_CTRL);				/////////////////////
						navigation.setDetectionState(true);
						
						if(dummy % 2 == 0){
							parkNow_backBoundery = navigation.getPose().getX();
						} else if (dummy % 2 == 1){
							parkNow_backBoundery = navigation.getPose().getY();
						}
					}
					// While action
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					if ((perception.getFrontSideSensorDistance() < 200) && (perception.getBackSideSensorDistance() > 200)){
						sm_park_now_currentStatus = SM_park_now.DRIVE_TO_SLOT_BEGINNING;
					}
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.MEASURE_SLOT) {
						control.setCtrlMode(ControlMode.INACTIVE);
						navigation.setDetectionState(false);
					
						if(dummy % 2 == 0){
							parkNow_frontBoundery = navigation.getPose().getY();		////////////7 evtl in anschlussstatus reinschreiben um rechenzeit zu sparen und direkt an der kante anzuhalten
						} else if (dummy % 2 == 1){
							parkNow_frontBoundery = navigation.getPose().getY();
						}
						
						parkNow_slot_width = Math.abs(parkNow_frontBoundery - parkNow_backBoundery);
					}	
					break;
				case DRIVE_TO_SLOT_BEGINNING:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.DRIVE_TO_SLOT_BEGINNING) {
						robo_in_parkingmovement = true;
						control.setAngularVelocity(0.0);		///// rad/s
						control.setVelocity(-5.0);						///// cm/s
						control.setCtrlMode(ControlMode.VW_CTRL);
					}
					// While action
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					if ((perception.getFrontSideSensorDistance() > 200) && (perception.getBackSideSensorDistance() < 200)){
						sm_park_now_currentStatus = SM_park_now.PATH_GENERATOR;
					}
					
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.DRIVE_TO_SLOT_BEGINNING) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				case PATH_GENERATOR:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.PATH_GENERATOR) {
						coefficienten = null;
					}
					// While action;
					park_out_slotrange = parkNow_slot_width;
					
					coefficienten = coefficient_calculation(parkNow_slot_width, 30.0, 1);
					control.setCoefficients(coefficienten);
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					if(coefficienten != null){
						sm_park_now_currentStatus = SM_park_now.PARKING_MANEUVER;
					}
	
					// Leave action
					if (sm_park_now_currentStatus != SM_park_now.PATH_GENERATOR) {
						coefficienten = null;
					}
					break;
				case PARKING_MANEUVER:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.PARKING_MANEUVER) {
						
						robo_in_parkingmovement = true;
						
						kacke = 1;
						control.setCtrlMode(ControlMode.PARK_CTRL);						
					}
					// While action

					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					if(parkManeuver_finished){
						
						sm_park_now_currentStatus = SM_park_now.CORRECT_PARKING_POSE;    //////////////////////  verzweigen auf korrektur bei kollision
					}
					// Leave action
					if (sm_park_now_currentStatus != SM_park_now.PARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					break;
				case CORRECT_PARKING_MANEUVER:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.CORRECT_PARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.LINE_CTRL);	
					}
					// While action
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.CORRECT_PARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				case CORRECT_PARKING_POSE:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.CORRECT_PARKING_POSE) {
						Sound.beepSequenceUp();

						control.setAngularVelocity(0.0);		///// rad/s
						control.setVelocity(-5.0);						///// cm/s
						control.setCtrlMode(ControlMode.VW_CTRL);
					}
					// While action

					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;

					if (perception.getBackSensorDistance() < 60){
						sm_park_now_currentStatus = SM_park_now.LOOKING_FOR_SLOTS;
					}
					// Leave action
					if (sm_park_now_currentStatus != SM_park_now.CORRECT_PARKING_POSE) {
						control.setCtrlMode(ControlMode.INACTIVE);
						currentStatus = CurrentStatus.PARKED;				///////777 leave PARK_THIS
					}
					break;
				default:
					break;
				}
				
				
				// State transition check
				lastStatus = currentStatus;
				if (Button.ESCAPE.isDown()) {								//EXIT_PROJECT
					currentStatus = CurrentStatus.EXIT;								
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					}
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
					currentStatus = CurrentStatus.INACTIVE;
				}
				
				// Leave action

				break;
			case PARKED:
				// Into action
				if (lastStatus != CurrentStatus.PARKED) {
					
				}

				// While action
				
				// State transition check
				lastStatus = currentStatus;
				
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {				//PARK_OUT
					currentStatus = CurrentStatus.PARK_OUT;			//////////////////////////////
				} else if (Button.ENTER.isDown()) {
						currentStatus = CurrentStatus.PARK_OUT;			//////////////////////////////	
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {								//EXIT_PROJECT
					currentStatus = CurrentStatus.EXIT;								
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}
				
				// Leave action
				if (currentStatus != CurrentStatus.PARKED) {
				
				}
				break;
			case PARK_OUT:
				// Into action
				if (lastStatus != CurrentStatus.PARK_OUT) {
					control.setCtrlMode(ControlMode.INACTIVE);
				}

				// While action
				switch (sm_park_out_currentStatus) {
				case PATH_GENERATOR:
					// Into action
					if (sm_park_out_lastStatus != SM_park_out.PATH_GENERATOR) {
					
					}
					
					// While action;
					coefficienten = coefficient_calculation(park_out_slotrange, -30.0, -1);
					control.setCoefficients(coefficienten);
					
					// State transition check
					sm_park_out_lastStatus = sm_park_out_currentStatus;
					
					if(coefficienten != null){
						sm_park_out_currentStatus = SM_park_out.OUTPARKING_MANEUVER;
					}

					// Leave action
					if (sm_park_out_currentStatus != SM_park_out.PATH_GENERATOR) {
						coefficienten = null;
					}
					break;
				case OUTPARKING_MANEUVER:
					// Into action
					if (sm_park_out_lastStatus != SM_park_out.OUTPARKING_MANEUVER) {
						
					}
					// While action
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					// Leave action	
					if (sm_park_out_currentStatus != SM_park_out.OUTPARKING_MANEUVER) {

					}	
					break;
				case CORRECT_OUTPARKING_MANEUVER:
					// Into action
					if (sm_park_out_lastStatus != SM_park_out.CORRECT_OUTPARKING_MANEUVER) {

					}
					// While action
					
					// State transition check
					sm_park_out_lastStatus = sm_park_out_currentStatus;
					
					// Leave action	
					if (sm_park_out_currentStatus != SM_park_out.CORRECT_OUTPARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				default:
					break;
				}

				// State transition check
				lastStatus = currentStatus;


				// Leave action
				if (currentStatus != CurrentStatus.PARK_OUT) {
					robo_in_parkingmovement = false;
				}
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
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {				//SCOUT_MODUS
						currentStatus = CurrentStatus.SCOUT;			//////////////////////////////
				} else if (Button.ENTER.isDown()) {
						currentStatus = CurrentStatus.SCOUT;			//////////////////////////////	
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS) {	//PARK_THIS_MODUS
					currentStatus = CurrentStatus.PARK_THIS;
				
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW) {	//PARK_NOW_MODUS
					currentStatus = CurrentStatus.PARK_NOW;
					
				} else if (Button.ESCAPE.isDown()) {								//EXIT_PROJECT
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

		boolean version = true;
		
		if(version){
		
			LCD.drawString("###PAULS TEST###", 0, 5);
			LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);    /////////////////////////////////
			LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
	//		LCD.drawString("B (in cm): " + (range_slotbeginning), 0, 1);			////////////////////////////
			LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
			LCD.drawString("KACKE: " + (dummy3), 0, 3);
			LCD.drawString("XNEU: " + (dummy4), 0, 4);
		} else if (!version /*&& coefficienten != null*/){
			
//			LCD.drawString("A: " + (coefficienten.get(0, 0)), 0 ,0);    /////////////////////////////////
//			LCD.drawString("B: " + (coefficienten.get(1, 0)), 0 ,1);    /////////////////////////////////
//			LCD.drawString("C: " + (coefficienten.get(2, 0)), 0, 2);    /////////////////////////////////
//			LCD.drawString("D: " + (coefficienten.get(3, 0)), 0, 3);    /////////////////////////////////
//			LCD.drawString("Weite: " + (selected_Parkingslot_Slotrange), 0, 5);    /////////////////////////////////
			
		 perception.showSensorData();
		}
		
		// if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
		// LCD.drawString("HMI Mode SCOUT", 0, 3);
		// }else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
		// LCD.drawString("HMI Mode PAUSE", 0, 3);
		// }else{
		// LCD.drawString("HMI Mode UNKNOWN", 0, 3);
		// }
	}
	
	
	
	public static void setCurrentLine(int line){
		currentLine = map[line];
		dummy = line;					/////////////////////////////////
	}
	
	public static void setParkmaneuverFinished(){
		parkManeuver_finished = true;
	}
	
	public static boolean getParkmovementInfo(){
		return robo_in_parkingmovement;
//		return true;
		
	}
	
	public static void test (boolean test, double test2){
		dummy3 = test;
		dummy4 = test2;
	}
	
	public static boolean park_in_or_out(){
		
		boolean modus = false;
		if(currentStatus == CurrentStatus.PARK_THIS || currentStatus == CurrentStatus.PARK_NOW){
			modus = false;
		} else if(currentStatus == CurrentStatus.PARKED){
			modus = true;
		}
		
		return modus;
	}

	// Polynom für Pfadgenerator berechnen

	/*
	 * Finden der Polynomkoeffizienten mittels lösen eines LGS (Ax = b)
	 * 
	 * 1.Bedingung: f(0)=0 
	 * 2.Bedingung: f(Lückentiefe)=Lückenbreite 
	 * 3.Bedingung: f(Lückentiefe/2)=Lückenbreite/2 
	 * 4.Bedingung: f'(Lückentiefe/2)=1 <-- Steigung des Polynoms zur halben Breite und halb abgefahrenen Strecke gleich 45°
	 * 
	 */
	private static Matrix coefficient_calculation(double lueckenbreite, double lueckentiefe, double steigung) {
	
//	double lueckenbreite = 180;
//	double lueckentiefe = 30.0;
//	double steigung = 1.0;

	double[][] A_Array = { { 0                              , 0                         , 0                 , 1 }, 
						   { Math.pow(lueckentiefe, 3)      , Math.pow(lueckentiefe,2)  , lueckentiefe      , 1 },
						   { Math.pow(lueckentiefe/2, 3)    , Math.pow(lueckentiefe/2,2), (lueckentiefe / 2), 1 },
						   { 3 * Math.pow(lueckentiefe/2, 2), 2 * (lueckentiefe / 2)    , 1                 , 0 }};

	double[][] b_Array = { {0              }, 
					       {lueckenbreite  }, 
					       {lueckenbreite/3},
					       {steigung       }};
	
	Matrix b = new Matrix(b_Array);
	Matrix A = new Matrix(A_Array);

	return A.solve(b);
	}
}