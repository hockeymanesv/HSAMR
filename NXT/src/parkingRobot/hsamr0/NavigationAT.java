package parkingRobot.hsamr0;

import java.util.ArrayList;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IMonitor;

import parkingRobot.hsamr0.NavigationThread;

/**
 * A executable basic example implementation of the corresponding interface
 * provided by the Institute of Automation with limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception}
 * implementation are used for periodically calculating the robots position and
 * corresponding heading angle (together called 'pose'). Neither any use of the
 * map information or other perception sensor information is used nor any
 * parking slot detection is performed, although the necessary members are
 * already prepared. Advanced navigation calculation and parking slot detection
 * should be developed and invented by the students.
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation {

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
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual an last
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
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry
	 * sensor to measure the ground displacement between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;

	/**
	 * reference to data class for measurement of the mouse odometry sensor to
	 * measure the ground displacement between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;

	/**
	 * distance from optical sensor pointing in driving direction to obstacle in
	 * mm
	 */
	double frontSensorDistance = 0;
	/**
	 * distance from optical sensor pointing to the right side of robot to
	 * obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance = 0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to
	 * obstacle in mm
	 */
	double backSensorDistance = 0;
	/**
	 * distance from optical sensor pointing to the right side of robot to
	 * obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance = 0;

	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS = 0.02724; // only rough guess, to be
													// measured exactly and
													// maybe refined by
													// experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS = 0.02724; // only rough guess, to be
													// measured exactly and
													// maybe refined by
													// experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE = 0.11; //0.113;//0.116; // only rough guess, to be
												// measured exactly and maybe
												// refined by experiments

	static final double FRONT_SENSOR_OFFSET = 0.86;
	static final double FRONT_SIDE_SENSOR_OFFSET = 0.75;
	static final double BACK_SENSOR_OFFSET = 0;
	static final double BACK_SIDE_SENSOR_OFFSET = 0.75;

	static final double FRONT_SENSOR_OFFAXIS = 0.01;
	static final double FRONT_SIDE_SENSOR_OFFAXIS = 0.42;
	static final double BACK_SIDE_SENSOR_OFFAXIS = -0.92;
	static final double BACK_SENSOR_OFFAXIS = 0;

	static final double MOUSE_SENSOR_OFFSET = 0;
	static final double MOUSE_SENSOR_OFFAXIS = 0;

	boolean possibleParkingSlot = false;
	boolean finishBack = false;
	boolean frontSensorFinished = false;
	float frontBackBoundaryX;
	float frontBackBoundaryY;
	float backBackBoundaryX;
	float backBackBoundaryY;
	float backFrontBoundaryX;
	float backFrontBoundaryY;
	float frontFrontBoundaryX;
	float frontFrontBoundaryY;
	float backBoundaryPositionX;
	float backBoundaryPositionY;
	float frontBoundaryPositionX;
	float frontBoundaryPositionY;
	Point test;
	
	/**
	 * map array of line references, whose corresponding lines form a closed
	 * chain and represent the map of the robot course
	 */
	Line[] map = null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception = null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off
	 * (false)
	 */
	public ArrayList<ParkingSlot> ParkingSlots = new ArrayList<ParkingSlot>();
	
	boolean parkingSlotDetectionIsOn = false;
	/**
	 * pose class containing bundled current X and Y location and corresponding
	 * heading angle phi
	 */
	Pose pose = new Pose();
	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * perception object (to obtain the measurement information from) and starts
	 * the navigation thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 * @param monitor
	 *            corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor) {
		this.perception = perception;
		this.monitor = monitor;
		this.encoderLeft = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();

		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		navThread.start();
	}

	// Inputs

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map) {
		this.map = map;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn) {
		this.parkingSlotDetectionIsOn = isOn;
	}

	// Class control

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation() {
		this.updateSensors();
		this.calculateLocation();
		if (true){
			this.detectParkingSlot(this.pose);
		}
		// MONITOR (example)
		// monitor.writeNavigationComment("Navigation");
	}

	// Outputs

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose() {
		return this.pose;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		//ParkingSlot []dsf = new ParkingSlot[ParkingSlots.size()];
		return ParkingSlots.toArray(new ParkingSlot[ParkingSlots.size()]);
	}
	
	
	
	// Private methods

	/**
	 * calls the perception methods for obtaining actual measurement data and
	 * writes the data into members
	 */
	private void updateSensors() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		this.angleMeasurementLeft = this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight = this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement = this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance = perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance = perception.getBackSensorDistance();
		this.backSideSensorDistance = perception.getBackSideSensorDistance();
	}

	double xFrontBouandaryOffset(){
		double x =0;
		if(Math.abs(this.pose.getHeading()) < Math.PI/4){
			x = FRONT_SIDE_SENSOR_OFFAXIS;
		} else if((Math.abs(this.pose.getHeading()-Math.PI/2)) < Math.PI/4){
			x = 0;
		} else{
			x = -FRONT_SIDE_SENSOR_OFFAXIS;
		}
		return x;
	}
	
	double yFrontBouandaryOffset(){
		double y;
		if(Math.abs(this.pose.getHeading()) < Math.PI/4){
			y = 0;
		} else if(Math.abs(this.pose.getHeading()-Math.PI/2) < Math.PI/4){
			y = FRONT_SIDE_SENSOR_OFFAXIS;
		} else{
			y = 0;
		}
		return y;
	}
	
	double xBackBouandaryOffset(){
		double x;
		if(Math.abs(this.pose.getHeading()) < Math.PI/4){
			x = BACK_SIDE_SENSOR_OFFAXIS;
		} else if(Math.abs(this.pose.getHeading()-Math.PI/2) < Math.PI/4){
			x = 0;
		} else{
			x = -BACK_SIDE_SENSOR_OFFAXIS;
		}
		return x;
	}
	
	double yBackBouandaryOffset(){
		double y;
		if(Math.abs(this.pose.getHeading()) < Math.PI/4){
			y = 0;
		} else if(Math.abs(this.pose.getHeading()-Math.PI/2) < Math.PI/4){
			y = BACK_SIDE_SENSOR_OFFAXIS;
		} else{
			y = 0;
		}
		return y;
	}
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation() {
		double leftAngleSpeed = this.angleMeasurementLeft.getAngleSum()
				/ ((double) this.angleMeasurementLeft.getDeltaT() / 1000); // degree/seconds
		double rightAngleSpeed = this.angleMeasurementRight.getAngleSum()
				/ ((double) this.angleMeasurementRight.getDeltaT() / 1000); // degree/seconds

		double vLeft = (leftAngleSpeed * Math.PI * LEFT_WHEEL_RADIUS) / 180; // velocity
																				// of
																				// left
																				// wheel
																				// in
																				// m/s
		double vRight = (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180; // velocity
																				// of
																				// right
																				// wheel
																				// in
																				// m/s
		double w = (vRight - vLeft) / WHEEL_DISTANCE; // angular velocity of
														// robot in rad/s

		Double R = new Double((WHEEL_DISTANCE / 2) * ((vLeft + vRight) / (vRight - vLeft)));

		double ICCx = 0;
		double ICCy = 0;

		double xResult = 0;
		double yResult = 0;
		double angleResult = 0;

		double deltaT = ((double) this.angleMeasurementLeft.getDeltaT()) / 1000;

		double frontEffective = (this.frontSensorDistance + FRONT_SENSOR_OFFSET)/10;
		double frontSideEffective = (this.frontSideSensorDistance + FRONT_SIDE_SENSOR_OFFSET)/10;
		double backSideEffective = (this.backSideSensorDistance + BACK_SIDE_SENSOR_OFFSET)/10;
		double backEffective = (this.backSensorDistance + BACK_SENSOR_OFFSET)/10;

		double distanceSideSensor = (FRONT_SIDE_SENSOR_OFFAXIS + BACK_SIDE_SENSOR_OFFAXIS)/10;

		double aTriangSS = 0;
		double xRobo = 0;
		double yRoboFront = 0;
		double yRoboBack = 0;
		double xRoboTrue;
		double yRoboTrueFront;
		double yRoboTrueBack;
		
		/*
		 * double aTriangFS = 0; double aTriangBS = 0; double yRoboFrontSide =
		 * 0; double yRoboBackSide = 0;
		 * 
		 * double xResultMouse = 0; double yResultMouse = 0; double
		 * angleResultMouse = 0;
		 */
		if (R.isNaN()) { // robot don't move
			xResult = this.pose.getX();
			yResult = this.pose.getY();
			angleResult = this.pose.getHeading();
		} else if (R.isInfinite()) { // robot moves straight forward/backward,
										// vLeft==vRight
			xResult = this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult = this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult = this.pose.getHeading();
		} else {
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());

			xResult = Math.cos(w * deltaT) * (this.pose.getX() - ICCx)
					- Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult = Math.sin(w * deltaT) * (this.pose.getX() - ICCx)
					+ Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult = this.pose.getHeading() + w * deltaT;
		}
		/*
		 while (this.pose.getHeading() > Math.PI) { 
			 angleResult = angleResult -Math.PI;
				
		 }
		 
		 while (this.pose.getHeading() < -Math.PI) { 
			 angleResult = angleResult + Math.PI;
		 }
		 */
		
		aTriangSS = Math.atan((frontSideEffective - backSideEffective) / distanceSideSensor);
		xRobo = backSideEffective + Math.tan(aTriangSS) * BACK_SIDE_SENSOR_OFFAXIS;
		yRoboFront = frontEffective + Math.tan(Math.PI/2 + aTriangSS) * FRONT_SENSOR_OFFAXIS;
		yRoboBack = backEffective + Math.tan(aTriangSS) * BACK_SENSOR_OFFAXIS;
		xRoboTrue = xRobo * Math.cos(aTriangSS);
		yRoboTrueFront = yRoboFront * Math.cos(aTriangSS);
		yRoboTrueBack = yRoboBack * Math.cos(aTriangSS);
		
		/*
		 * if (frontEffective < 300 && frontSideEffective < 300) { aTriangFS =
		 * 90 + Math.atan((frontEffective-FRONT_SIDE_SENSOR_OFFAXIS) /
		 * (frontSideEffective - FRONT_SENSOR_OFFAXIS)); yRoboFrontSide =
		 * frontEffective + Math.tan(90 + aTriangSS) * FRONT_SENSOR_OFFAXIS; }
		 * 
		 * if (backEffective < 300 && backSideEffective < 300) { aTriangBS =
		 * Math.atan((backSideEffective - BACK_SENSOR_OFFSET) / (backEffective -
		 * BACK_SIDE_SENSOR_OFFSET)); yRoboBackSide = backEffective +
		 * Math.tan(aTriangSS) * BACK_SENSOR_OFFAXIS; }
		 */
		
		
		/*
		if ((this.pose.getHeading()) < Math.PI/12 && (this.pose.getHeading()) > -Math.PI/12 && yResult < 0.15 && xResult > 0.15 && xResult < 0.165) {
			yResult = 0;
		}

		if (angleResult < 105 && angleResult > 75 && xResult > 15 && yResult > 15 && yResult < 45) {
			xResult = 180;
		}

		if (angleResult < 195 && angleResult > 165 && yResult > 15 && xResult > 30 && yResult < 150) {
			yResult = 30;
		}

		if (angleResult < 285 && angleResult > 255 && xResult < 15 && yResult > 15 && yResult < 45) {
			xResult = xRoboTrue - 20;
			yResult = (yRoboTrueFront - 20 + (80 - yRoboTrueBack)) / 2;
			angleResult = aTriangSS + 270;
		}
		*/
		this.pose.setLocation((float) xResult, (float) yResult);
		this.pose.setHeading((float) angleResult);
	}

	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots.
	 */
	
	private void detectParkingSlot(Pose pPose) {
		
		if(this.pose.getY() < 0.10 || this.pose.getX() > 0.170 || (Math.abs(this.pose.getX() - 0.90) < 0.45 && Math.abs(this.pose.getY() - 0.30) < 5)){
			float poseX= this.pose.getX() * 100;
			float poseY= this.pose.getY() * 100;
			if(this.frontSideSensorDistance > 200  && !possibleParkingSlot){
				frontBackBoundaryX = poseX;
				frontBackBoundaryY = poseY;
//				frontBackBoundaryX += xFrontBouandaryOffset();
//				frontBackBoundaryY += yFrontBouandaryOffset();
				//Sound.twoBeeps();
				possibleParkingSlot = true;
			}
			
			if(this.backSideSensorDistance > 200 && possibleParkingSlot && !finishBack){
				
				backBackBoundaryX = poseX;
				backBackBoundaryY = poseY;
//				backBackBoundaryX += xBackBouandaryOffset();
//				backBackBoundaryY += yBackBouandaryOffset();
				finishBack = true;
				backBoundaryPositionX = (frontBackBoundaryX + backBackBoundaryX)/2;
				backBoundaryPositionY = (frontBackBoundaryY + backBackBoundaryY)/2;
				
			}
			
			if(this.frontSideSensorDistance < 150 && possibleParkingSlot){
				frontFrontBoundaryX = poseX;
				frontFrontBoundaryY = poseY;
//				frontFrontBoundaryX += xFrontBouandaryOffset();
//				frontFrontBoundaryY += yFrontBouandaryOffset();
				possibleParkingSlot = false;
				frontSensorFinished = true;
			}
			
			if(this.backSideSensorDistance < 150 && finishBack && frontSensorFinished && !possibleParkingSlot){
				backFrontBoundaryX = poseX;
				backFrontBoundaryY = poseY;
//				backFrontBoundaryX += xBackBouandaryOffset();
//				backFrontBoundaryY += yBackBouandaryOffset();
				finishBack = false;
				frontBoundaryPositionX = (frontFrontBoundaryX + backFrontBoundaryX)/2;
				frontBoundaryPositionY = (frontFrontBoundaryX + backFrontBoundaryY)/2;
				
				boolean newSlot = true;
				for(int i = 0;i < ParkingSlots.size(); i++){
					if(Math.abs(ParkingSlots.get(i).getBackBoundaryPosition().getX() - backBoundaryPositionX) < 5 && Math.abs(ParkingSlots.get(i).getBackBoundaryPosition().getY() - backBoundaryPositionY) < 5){
						ParkingSlots.get(i).setBackBoundaryPosition(new Point((float)backBoundaryPositionX,(float)backBoundaryPositionY));
						ParkingSlots.get(i).setFrontBoundaryPosition(new Point((float)frontBoundaryPositionX,(float)frontBoundaryPositionY));
						newSlot = false;
						Sound.twoBeeps();
						break;
					}
				}
				if(newSlot){
					//ParkingSlots.add( new ParkingSlot(ParkingSlots.size(), new Point((float)backBoundaryPositionX,(float)backBoundaryPositionY), new Point((float)frontBoundaryPositionX,(float)frontBoundaryPositionY), ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING, 0));
					ParkingSlots.add( new ParkingSlot(ParkingSlots.size(), new Point((float)frontBackBoundaryX,(float)frontBackBoundaryX), new Point((float)frontFrontBoundaryX,(float)frontFrontBoundaryY), ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING, 0));
					Sound.beepSequenceUp();
				} 
			}
			
			
		}
		
		 // has to be implemented by students 
	} 
	
}
