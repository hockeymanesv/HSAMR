package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.util.Matrix;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;
import lejos.geom.Line;
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
	
	/////////////////////////////////////////
	//GENERAL --> DECLARATIONS
	/////////////////////////////////////////
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
	 * represents the actual line of the robot
	 */
	static Line currentLine = line0;
	/**
	 * represents the actual line of the robot (as an int)
	 */
	static int currentLine_int = 0;


	/////////////////////////////////////
	//MAIN-STATE-MACHINE --> DECLARATIONS
	/////////////////////////////////////
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
		
	//////////////////////////////////////////////////
	//PARK_THIS --> SUB-STATE-MACHINE --> DECLARATIONS
	//////////////////////////////////////////////////
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
	 * represents the selected parking slot in PARK_THIS mode
	 */
	static ParkingSlot park_this_selected_Parking_Slot = null;	
	/**
	 * represents the number of selected parking slot in PARK_THIS mode
	 */
	static int park_this_selected_Parking_Slot_int = 0;
	/**
	 * represents the line of selected parking slot in PARK_THIS mode
	 */
	static Line park_this_selected_Parking_Slot_Line = null;
	/**
	 * represents the range of selected parking slot in PARK_THIS mode
	 */	
	static double park_this_selected_Parkingslot_Slotrange = 0;
	/**
	 * represents that he robot is not behind the slot on the choosen line in PARK_THIS mode
	 */		
	static boolean park_this_into_correct_line = false;
	/**
	 * represents the beginning range of selected parking slot in PARK_THIS mode
	 */	
	static double park_this_range_slotbeginning = 0;
	/**
	 * represents that the robot is in beginning range of selected parking slot in PARK_THIS mode
	 */	
	static boolean park_this_robot_near_the_slot = false;	
	/**
	 * represents that the PARK_THIS state machine is run through
	 */
	static boolean SM_park_this_finnished = false;

	//////////////////////////////////////////////////
	//PARK_NOW --> SUB-STATE-MACHINE --> DECLARATIONS
	//////////////////////////////////////////////////	
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
	/**
	 * represents the detected back boundery of an unknown parking slot in PARK_NOW mode
	 */	
	static float park_now_back_boundery = 0;
	/**
	 * represents the detected front boundery of an unknown parking slot in PARK_NOW mode
	 */
	static float park_now_front_boundery = 0;
	/**
	 * represents the detected slot width of an unknown parking slot in PARK_NOW mode
	 */	
	static double park_now_slot_width = 0;
	/**
	 * represents that the PARK_NOW state machine is run through
	 */
	static boolean SM_park_now_finnished = false;
	
	//////////////////////////////////////////////////
	//PARK_OUT --> SUB-STATE-MACHINE --> DECLARATIONS
	//////////////////////////////////////////////////		
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
	/**
	 * represents the detected slot width of an unknown parking slot in PARK_NOW mode
	 */	
	static double park_out_slotrange = 0;
	/**
	 * represents that the PARK_OUT state machine is run through
	 */
	static boolean SM_park_out_finnished = false;
	
	///////////////////////////////////
	//PARKING_MOVEMENT --> DECLARATIONS
	///////////////////////////////////		
	/**
	 * represents the calculated coefficients path to drive into the parking slot
	 */	
	static Matrix coefficients = null;
	/**
	 * represents that the robot is in a parking movement
	 */			
	static boolean robo_in_parking_movement = false;
	/**
	 * represents the signal that the parking maneuver is finished
	 */
	static boolean park_maneuver_finished = false;


	
	static double x1 = 0;					///////////test
	static double x2 = 0;					///////////test
	static double x3 = 0;					///////////test
	static double x4 = 0;					///////////test
	
	
	
	
	/////////////
	//MAIN METHOD
	/////////////	
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
					
					park_this_selected_Parking_Slot_int = hmi.getSelectedParkingSlot();
					park_this_selected_Parking_Slot = navigation.getParkingSlots()[park_this_selected_Parking_Slot_int];
					park_this_selected_Parking_Slot_Line = map[park_this_selected_Parking_Slot.getLine()];
					park_this_range_slotbeginning = (park_this_selected_Parking_Slot.getBackBoundaryPosition().getX() - 10);
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
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if (park_this_selected_Parking_Slot_Line != currentLine){
						park_this_into_correct_line = true;
					}
					if(/*park_this_into_correct_line && */park_this_selected_Parking_Slot_Line == currentLine){	///////////// normal muss das kommentar einkommentiert sein
						sm_park_this_currentStatus = SM_park_this.DRIVE_TO_SLOT_BEGINNING;
					}
					
					// Leave action	
					if (sm_park_this_currentStatus != SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE) {
						control.setCtrlMode(ControlMode.INACTIVE);
						park_this_into_correct_line = false;
					}	
					break;
				case DRIVE_TO_SLOT_BEGINNING:
					
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.DRIVE_TO_SLOT_BEGINNING) {
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if ((navigation.getPose().getX() * 100) > park_this_range_slotbeginning){
						park_this_robot_near_the_slot = true;
					}
					if (park_this_robot_near_the_slot && (perception.getFrontSideSensorDistance() > 200) && (perception.getBackSideSensorDistance() < 200)){
						sm_park_this_currentStatus = SM_park_this.PATH_GENERATOR;
					}
					
					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.DRIVE_TO_SLOT_BEGINNING) {
						control.setCtrlMode(ControlMode.INACTIVE);
						park_this_robot_near_the_slot = false;
					}
					break;
				case PATH_GENERATOR:
					
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.PATH_GENERATOR) {							
						// nothing to do here
					}

					// While action;
					if(park_this_selected_Parking_Slot.getLine() % 2 == 0){
						park_this_selected_Parkingslot_Slotrange = Math.abs(park_this_selected_Parking_Slot.getBackBoundaryPosition().getX() - park_this_selected_Parking_Slot.getFrontBoundaryPosition().getX());
					} else if (park_this_selected_Parking_Slot.getLine() % 2 == 1){
						park_this_selected_Parkingslot_Slotrange = Math.abs(park_this_selected_Parking_Slot.getBackBoundaryPosition().getY() - park_this_selected_Parking_Slot.getFrontBoundaryPosition().getY());
					} 
					
					park_out_slotrange = park_this_selected_Parkingslot_Slotrange;
					
					coefficients = coefficient_calculation(park_this_selected_Parkingslot_Slotrange, 32.0, 1);
					control.setCoefficients(coefficients);
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if(coefficients != null){
						sm_park_this_currentStatus = SM_park_this.PARKING_MANEUVER;
					}

					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.PATH_GENERATOR) {
						x1 = coefficients.get(0, 0);					///////////////////7/test
						x2 = coefficients.get(1, 0);					///////////////////7/test
						x3 = coefficients.get(2, 0);					///////////////////7/test
						x4 = coefficients.get(3, 0);					///////////////////7/test
						
						coefficients = null;
					}
					break;
				case PARKING_MANEUVER:
					
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.PARKING_MANEUVER) {
						robo_in_parking_movement = true;
						control.setCtrlMode(ControlMode.PARK_CTRL);						
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;
					
					if(park_maneuver_finished){
						
						sm_park_this_currentStatus = SM_park_this.CORRECT_PARKING_POSE;    //////////////////////  verzweigen auf korrektur bei kollision
					}
					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.PARKING_MANEUVER) {
						
						park_maneuver_finished = false;
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					break;
				case CORRECT_PARKING_MANEUVER:
					
					// Into action
					if (sm_park_this_lastStatus != SM_park_this.CORRECT_PARKING_MANEUVER) {
						// nothing to do here
					}
					
					// While action
					{
						// nothing to do here
					}
					
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
						control.setAngularVelocity(0.0);
						control.setVelocity(-8.0);						
						control.setCtrlMode(ControlMode.VW_CTRL);
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_this_lastStatus = sm_park_this_currentStatus;

					if (perception.getBackSensorDistance() < 60){
						sm_park_this_currentStatus = SM_park_this.DRIVE_TO_BEGINNING_OF_SLOTLINE;
					}
					
					// Leave action
					if (sm_park_this_currentStatus != SM_park_this.CORRECT_PARKING_POSE) {
						control.setCtrlMode(ControlMode.INACTIVE);
						SM_park_this_finnished = true;
					}
					break;
				default:
					break;
				}//End of sub-state machine for park this mode
				
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
				} else if (SM_park_this_finnished) {
					currentStatus = CurrentStatus.PARKED;
				}
				
				// Leave action
				if (currentStatus != CurrentStatus.PARK_THIS) {
					SM_park_this_finnished = false;
				}
				break;
			case PARK_NOW:
				
				// Into action
				if (lastStatus != CurrentStatus.PARK_NOW) {
					sm_park_now_currentStatus = SM_park_now.LOOKING_FOR_SLOTS;
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
					
					if(currentLine_int == 0 || currentLine_int == 1 || currentLine_int == 4){
						if ((perception.getFrontSideSensorDistance() > 300) && (perception.getBackSideSensorDistance() < 300)){
							sm_park_now_currentStatus = SM_park_now.IS_THE_SLOT_KNOWN;
						}
					}
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.LOOKING_FOR_SLOTS) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				case IS_THE_SLOT_KNOWN:
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.IS_THE_SLOT_KNOWN) {

					}
					
					// While action
					{
						// nothing to do here
					}
					
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
						control.setCtrlMode(ControlMode.LINE_CTRL);
						navigation.setDetectionState(true);
						
						if(currentLine_int % 2 == 0){
							park_now_back_boundery = navigation.getPose().getX() * 100;
						} else if (currentLine_int % 2 == 1){
							park_now_back_boundery = navigation.getPose().getY() * 100;
						}
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					if ((perception.getFrontSideSensorDistance() < 200) && (perception.getBackSideSensorDistance() > 300)){	/////////////////////////////////////////////////
						if(currentLine_int % 2 == 0){
							park_now_front_boundery = navigation.getPose().getX() * 100 + 5;
						} else if (currentLine_int % 2 == 1){
							park_now_front_boundery = navigation.getPose().getY() * 100 + 5;
						}
						park_now_slot_width = Math.abs(park_now_front_boundery - park_now_back_boundery);
						
						if(park_now_slot_width > 40){
							sm_park_now_currentStatus = SM_park_now.DRIVE_TO_SLOT_BEGINNING;
						} else if(park_now_slot_width <= 40){
							sm_park_now_currentStatus = SM_park_now.LOOKING_FOR_SLOTS;
						}
					}
					
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.MEASURE_SLOT) {
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				case DRIVE_TO_SLOT_BEGINNING:
					
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.DRIVE_TO_SLOT_BEGINNING) {
						control.setAngularVelocity(0.0);
						control.setVelocity(-8.0);					
						control.setCtrlMode(ControlMode.VW_CTRL);
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					if ((perception.getFrontSideSensorDistance() < 300) && (perception.getBackSideSensorDistance() < 300) || perception.getBackSensorDistance() < 50){
						sm_park_now_currentStatus = SM_park_now.PATH_GENERATOR;
					}
					
					// Leave action	
					if (sm_park_now_currentStatus != SM_park_now.DRIVE_TO_SLOT_BEGINNING) {
						
						navigation.setDetectionState(false);
						control.setCtrlMode(ControlMode.INACTIVE);
					}	
					break;
				case PATH_GENERATOR:
					
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.PATH_GENERATOR) {
						coefficients = null;
					}
					
					// While action;
					park_out_slotrange = park_now_slot_width;
					
					coefficients = coefficient_calculation(park_now_slot_width, 32.0, 1);
					control.setCoefficients(coefficients);
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					if(coefficients != null){
						sm_park_now_currentStatus = SM_park_now.PARKING_MANEUVER;
					}
	
					// Leave action
					if (sm_park_now_currentStatus != SM_park_now.PATH_GENERATOR) {
						
						x1 = coefficients.get(0, 0);			///////////////////////test
						x2 = coefficients.get(1, 0);			///////////////////////test
						x3 = coefficients.get(2, 0);			///////////////////////test
						x4 = coefficients.get(3, 0);			///////////////////////test
						
						coefficients = null;
					}
					break;
				case PARKING_MANEUVER:
					
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.PARKING_MANEUVER) {
						
						robo_in_parking_movement = true;
						
						control.setCtrlMode(ControlMode.PARK_CTRL);						
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;
					
					if(park_maneuver_finished){
						
						sm_park_now_currentStatus = SM_park_now.CORRECT_PARKING_POSE;    //////////////////////  verzweigen auf korrektur bei kollision
					}
					
					// Leave action
					if (sm_park_now_currentStatus != SM_park_now.PARKING_MANEUVER) {
						
						park_maneuver_finished = false;
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					break;
				case CORRECT_PARKING_MANEUVER:
					
					// Into action
					if (sm_park_now_lastStatus != SM_park_now.CORRECT_PARKING_MANEUVER) {
						control.setCtrlMode(ControlMode.LINE_CTRL);	
					}
					
					// While action
					{
						// nothing to do here
					}
					
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

						control.setAngularVelocity(0.0);		
						control.setVelocity(-5.0);				
						control.setCtrlMode(ControlMode.VW_CTRL);
					}
					
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_now_lastStatus = sm_park_now_currentStatus;

					if (perception.getBackSensorDistance() < 60){
						sm_park_now_currentStatus = SM_park_now.LOOKING_FOR_SLOTS;
					}
					
					// Leave action
					if (sm_park_now_currentStatus != SM_park_now.CORRECT_PARKING_POSE) {
						control.setCtrlMode(ControlMode.INACTIVE);
						SM_park_now_finnished = true;
					}
					break;
				default:
					break;
				}//End of sub-state machine for park now mode
				
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
				} else if (SM_park_now_finnished) {
					currentStatus = CurrentStatus.PARKED;
				}
				
				// Leave action
				if (currentStatus != CurrentStatus.PARK_NOW) {
					SM_park_now_finnished = false;
				}
				break;
			case PARKED:
				
				// Into action
				if (lastStatus != CurrentStatus.PARKED) {
					
				}

				// While action
				{
					// nothing to do here
				}
				
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
					coefficients = coefficient_calculation(park_out_slotrange, -32.0, -1);
					control.setCoefficients(coefficients);
					
					// State transition check
					sm_park_out_lastStatus = sm_park_out_currentStatus;
					
					if(coefficients != null){
						sm_park_out_currentStatus = SM_park_out.OUTPARKING_MANEUVER;
					}

					// Leave action
					if (sm_park_out_currentStatus != SM_park_out.PATH_GENERATOR) {
						
						x1 = coefficients.get(0, 0);				//////////////////test
						x2 = coefficients.get(1, 0);				//////////////////test
						x3 = coefficients.get(2, 0);				//////////////////test
						x4 = coefficients.get(3, 0);				//////////////////test
						
						coefficients = null;
					}
					break;
				case OUTPARKING_MANEUVER:
					
					// Into action
					if (sm_park_out_lastStatus != SM_park_out.OUTPARKING_MANEUVER) {
						
						control.setCtrlMode(ControlMode.PARK_CTRL);						
					}
					// While action
					{
						// nothing to do here
					}
					
					// State transition check
					sm_park_out_lastStatus = sm_park_out_currentStatus;
					
					if(park_maneuver_finished){
						
						sm_park_out_currentStatus = SM_park_out.PATH_GENERATOR;    //////////////////////  verzweigen auf korrektur bei kollision
					}
					
					// Leave action
					if (sm_park_out_currentStatus != SM_park_out.OUTPARKING_MANEUVER) {
						
						control.setCtrlMode(ControlMode.INACTIVE);
						park_maneuver_finished = false;
						SM_park_out_finnished = true;							///////////////7 gucken ob das wirklich das ende ist..... kommt ja noch korrigieren
					}
					break;
				case CORRECT_OUTPARKING_MANEUVER:
					
					// Into action
					if (sm_park_out_lastStatus != SM_park_out.CORRECT_OUTPARKING_MANEUVER) {

					}
					
					// While action
					{
						// nothing to do here
					}
					
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
				
				if (SM_park_out_finnished) {				//SCOUT_MODUS
					currentStatus = CurrentStatus.SCOUT;			//////////////////////////////
				}else if (Button.ESCAPE.isDown()) {								//EXIT_PROJECT
					currentStatus = CurrentStatus.EXIT;								
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					}
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}
				
				// Leave action
				if (currentStatus != CurrentStatus.PARK_OUT) {
					robo_in_parking_movement = false;
					SM_park_out_finnished = false;
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
				
				Sound.buzz();
				
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

	/////////////
	//GET METHODS
	/////////////	

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
	 * returns a information about robot is in parking movement or not
	 * 
	 * @return robot is in parking movement or not
	 */
	public static boolean getParkmovementInfo(){
		return robo_in_parking_movement;
	}
	/**
	 * returns a information about robot is in a park-in or park-out movement 
	 *
	 * @return robot is in a park-in or park-out movement
	 */
	public static boolean park_in_or_out(){
		
		boolean modus = false;
		if(currentStatus == CurrentStatus.PARK_THIS || currentStatus == CurrentStatus.PARK_NOW){
			modus = false;
		} else if(currentStatus == CurrentStatus.PARKED || currentStatus == CurrentStatus.PARK_OUT){
			modus = true;
		}
		
		return modus;
	}
	
	/////////////
	//SET METHODS
	/////////////
	
	/**
	 * set the current line from the navigation modul
	 * 
	 * @param line
	 * 			reference to the calculated line 
	 */
	public static void setCurrentLine(int line){
		currentLine = map[line];
		currentLine_int = line;
	}
	
	/**
	 * set that the parking maneuver is finished from the control modul
	 */
	public static void setParkmaneuverFinished(){
		park_maneuver_finished = true;
	}
	
	////////////////
	//CASUAL METHODS
	////////////////	
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation
	 *            reference to the navigation class for getting pose information
	 * @param perception
	 * 			  reference to the perception class for getting sensor information
	 */
	protected static void showData(INavigation navigation, IPerception perception) {
		LCD.clear();

		boolean version = true;
		
		if(version){
		
			
			LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);    /////////////////////////////////
			LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
			LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
//			LCD.drawString("Status: " + (currentStatus), 0, 3);
//			LCD.drawString("OUT_Status: " + (sm_park_out_currentStatus), 0, 4);
//			LCD.drawString("IN_OUT: " + (park_in_or_out()), 0, 5);
			LCD.drawString("Rigth: " + (perception.getRightLineSensorValue()), 0, 3);
			LCD.drawString("Left: " + (perception.getLeftLineSensorValue()), 0, 4);

		} else if (!version){
			
			LCD.drawString("A: " + (x1), 0 ,0);    /////////////////////////////////
			LCD.drawString("B: " + (x2), 0 ,1);    /////////////////////////////////
			LCD.drawString("C: " + (x3), 0, 2);    /////////////////////////////////
			LCD.drawString("D: " + (x4), 0, 3);    /////////////////////////////////
			LCD.drawString("Status: " + (currentStatus), 0, 4);    /////////////////////////////////
			LCD.drawString("OUT_Weite: " + (park_now_slot_width), 0, 5);    /////////////////////////////////
			LCD.drawString("B_Bound: " + (park_now_back_boundery), 0, 6);    /////////////////////////////////
			LCD.drawString("F_Bound: " + (park_now_front_boundery), 0, 7);    /////////////////////////////////
//			
//			LCD.drawString("Left: " + (perception.getLeftLineSensor()), 0 ,0);    /////////////////////////////////
//			LCD.drawString("Rigth: " + (perception.getRightLineSensor()), 0 ,1);    /////////////////////////////////
//			LCD.drawString("Prev_Status: " + (), 0, 2);    /////////////////////////////////
//			LCD.drawString("D: " + (x4), 0, 3);    /////////////////////////////////
//			LCD.drawString("OUT_Weite: " + (park_now_slot_width), 0, 5);    /////////////////////////////////
//			LCD.drawString("B_Bound: " + (park_now_back_boundery), 0, 6);    /////////////////////////////////
//			LCD.drawString("F_Bound: " + (park_now_front_boundery), 0, 7);    /////////////////////////////////

//		 perception.showSensorData();
		}
		
		// if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
		// LCD.drawString("HMI Mode SCOUT", 0, 3);
		// }else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
		// LCD.drawString("HMI Mode PAUSE", 0, 3);
		// }else{
		// LCD.drawString("HMI Mode UNKNOWN", 0, 3);
		// }
	}
	
/**
 * f(x)= a*x^3 + b*x^2 + c*x + d
 * 
 * find the polynomial coefficients with solving a SLE (Ax = b)
 * 
 * 1.condition: f(0)=0 
 * 2.condition: f(Lückentiefe) = Lückenbreite 
 * 3.condition: f(Lückentiefe/2) = Lückenbreite/2 
 * 4.condition: f'(Lückentiefe/2)=m 
 * @param lueckenbreite
 * 			reference to the width of the slot
 * @param lueckentiefe
 * 			reference to the depth of the slot 
 * @param steigung
 * 			reference to the slop in at the construction point
 * @return coefficients
 * 			calculated coefficients
 */
	private static Matrix coefficient_calculation(double lueckenbreite, double lueckentiefe, double steigung) {

	double[][] A_Array = { { 0                              , 0                         , 0                 , 1 }, 
						   { Math.pow(lueckentiefe, 3)      , Math.pow(lueckentiefe,2)  , lueckentiefe      , 1 },
						   { Math.pow(lueckentiefe/2, 3)    , Math.pow(lueckentiefe/2,2), (lueckentiefe / 2), 1 },
						   { 3 * Math.pow(lueckentiefe/2, 2), 2 * (lueckentiefe / 2)    , 1                 , 0 }};

	double[][] b_Array = { {0              }, 
					       {lueckenbreite  }, 
					       {lueckenbreite/2},
					       {steigung       }};
	
	Matrix b = new Matrix(b_Array);
	Matrix A = new Matrix(A_Array);

	Matrix coefficients = A.solve(b);
	
	return coefficients;
	}
}