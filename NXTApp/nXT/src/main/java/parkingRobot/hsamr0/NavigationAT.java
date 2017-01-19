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
	static final double RIGHT_WHEEL_RADIUS = 0.027315; // only rough guess, to
														// be
														// measured exactly and
														// maybe refined by
														// experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE = 0.115; // only rough guess, to be
												// measured exactly and maybe
												// refined by experiments

	/**
	 * robot specific constant: distance between front sensor and wheel axis
	 */
	static final double FRONT_SENSOR_OFFSET = 0.097;
	/**
	 * robot specific constant: distance between front side sensor and center line
	 */
	static final double FRONT_SIDE_SENSOR_OFFSET = 0.073;
	/**
	 * robot specific constant: distance between back sensor and wheel axis
	 */
	static final double BACK_SENSOR_OFFSET = 0.17;
	/**
	 * robot specific constant: distance between back side sensor and center line
	 */
	static final double BACK_SIDE_SENSOR_OFFSET = 0.074;

	/**
	 * robot specific constant: distance between front sensor and wheel axis
	 */
	static final double FRONT_SENSOR_OFFAXIS = 0.001;
	/**
	 * robot specific constant: distance between front side sensor and center line
	 */
	static final double FRONT_SIDE_SENSOR_OFFAXIS = 0.05;
	/**
	 * robot specific constant: distance between back side sensor and wheel axis
	 */
	static final double BACK_SIDE_SENSOR_OFFAXIS = -0.074;
	/**
	 * robot specific constant: distance between back sensor and center line
	 */
	static final double BACK_SENSOR_OFFAXIS = 0;

	/**
	 * robot specific constant: distance between mouse sensor and wheel axis
	 */
	static final double MOUSE_SENSOR_OFFSET = 0;
	/**
	 * robot specific constant: distance between mouse sensor and center line
	 */
	static final double MOUSE_SENSOR_OFFAXIS = 0;

	/**
	 * robot specific constant: mouse radius
	 */
	static final double MOUSE_RADIUS = 2 * Math.PI * MOUSE_SENSOR_OFFSET;

	/**
	 * robot specific constant: relative angle error
	 */
	static final double ENCODER_RELATIVE_ANGLE_ERROR = 0.06;
	/**
	 * robot specific constant: relative pose error
	 */
	static final double ENCODER_RELATIVE_POSE_ERROR = 0.20;

	static boolean onLine = true;

	double errorX = 0;
	double errorY = 0;
	double errorAngle = 0;

	boolean possibleParkingSlot = false;
	boolean finishBack = false;
	boolean frontSensorFinished = false;
	boolean angleOverwrite = false;
	boolean angleSetback = false;
	boolean newSlot = false;
	boolean newSlotBack = false;
	boolean newSlotFront = false;
	boolean newSlotOne = false;
	boolean newSlotTwoBack = false;
	boolean newSlotTwoFront = false;
	boolean boundayrsSet = false;
	boolean obstruction = true;
	int obstructionNumber = 0;
	boolean existingParkingSlot = false;
	double aMax = 0;
	double aMin = 0;
	float frontBackBoundaryX;
	float frontBackBoundaryY;
	float backBackBoundaryX;
	float backBackBoundaryY;
	float backFrontBoundaryX;
	float backFrontBoundaryY;
	float frontFrontBoundaryX;
	float frontFrontBoundaryY;
	float backBoundaryPositionX;
	float backBoundaryPositionXBack;
	float backBoundaryPositionXFront;
	float backBoundaryPositionY;
	float frontBoundaryPositionX;
	float frontBoundaryPositionXBack;
	float frontBoundaryPositionXFront;
	float frontBoundaryPositionY;
	float backBoundaryPosition1;
	float backBoundaryPosition2;
	float backBoundaryPosition3;
	float frontBoundaryPosition1;
	float frontBoundaryPosition2;
	float frontBoundaryPosition3;
	int line = 0;
	int i = 0;
	int j = 0;
	Point test;
	double backBackError = 0;
	double frontBackError = 0;
	double backFrontError = 0;
	double frontFrontError = 0;
	double ParkingSlotError = 0;
	static int measurementQuality = 0;
	ParkingSlotStatus ParkingSlotStatus;
	boolean testParking = false;
	boolean firstStart = true;

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

		navThread.setPriority(Thread.MAX_PRIORITY - 4);
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
		if (this.parkingSlotDetectionIsOn && onLine) {
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
		// ParkingSlot []dsf = new ParkingSlot[ParkingSlots.size()];
		return ParkingSlots.toArray(new ParkingSlot[ParkingSlots.size()]);
	}

	public int getCurrentLine() {
		return line;
	}

	public void setPoseFirstExampleProgram(double x, double y, double heading){
		this.pose.setHeading((float) heading);
		this.pose.setLocation((float) x, (float) y);
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

	double xFrontBouandaryOffset() {
		double x = 0;
		if (Math.abs(this.pose.getHeading()) < Math.PI / 4) {
			x = FRONT_SIDE_SENSOR_OFFAXIS;
		} else if ((Math.abs(this.pose.getHeading() - Math.PI / 2)) < Math.PI / 4) {
			x = 0;
		} else {
			x = -FRONT_SIDE_SENSOR_OFFAXIS;
		}
		return x;
	}

	double yFrontBouandaryOffset() {
		double y;
		if (Math.abs(this.pose.getHeading()) < Math.PI / 4) {
			y = 0;
		} else if (Math.abs(this.pose.getHeading() - Math.PI / 2) < Math.PI / 4) {
			y = FRONT_SIDE_SENSOR_OFFAXIS;
		} else {
			y = 0;
		}
		return y;
	}

	double xBackBouandaryOffset() {
		double x;
		if (Math.abs(this.pose.getHeading()) < Math.PI / 4) {
			x = BACK_SIDE_SENSOR_OFFAXIS;
		} else if (Math.abs(this.pose.getHeading() - Math.PI / 2) < Math.PI / 4) {
			x = 0;
		} else {
			x = -BACK_SIDE_SENSOR_OFFAXIS;
		}
		return x;
	}

	double yBackBouandaryOffset() {
		double y;
		if (Math.abs(this.pose.getHeading()) < Math.PI / 4) {
			y = 0;
		} else if (Math.abs(this.pose.getHeading() - Math.PI / 2) < Math.PI / 4) {
			y = BACK_SIDE_SENSOR_OFFAXIS;
		} else {
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
		double angleTest = 0;
		
		double xMouse = 0;
		double yMouse = 0;
		double angleMouse = 0;

		double deltaT = ((double) this.angleMeasurementLeft.getDeltaT()) / 1000;

		double frontEffective = this.frontSensorDistance / 1000 + FRONT_SENSOR_OFFSET;
		double frontSideEffective = this.frontSideSensorDistance / 1000 + FRONT_SIDE_SENSOR_OFFSET;
		double backSideEffective = this.backSideSensorDistance / 1000 + BACK_SIDE_SENSOR_OFFSET;
		double backEffective = this.backSensorDistance / 1000 + BACK_SENSOR_OFFSET;

		double distanceSideSensor = (FRONT_SIDE_SENSOR_OFFAXIS + BACK_SIDE_SENSOR_OFFAXIS) / 10;

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

		errorAngle = errorAngle + Math.abs(w * deltaT * ENCODER_RELATIVE_ANGLE_ERROR);
		errorX = errorX + Math.abs(this.pose.getX() - xResult) * ENCODER_RELATIVE_POSE_ERROR;
		errorY = errorY + Math.abs(this.pose.getY() - yResult) * ENCODER_RELATIVE_POSE_ERROR;

		if (angleResult > Math.PI * 7 / 4) {
			angleResult = angleResult - Math.PI * 2;
		}

		angleMouse = this.mouseOdoMeasurement.getUSum() / MOUSE_SENSOR_OFFSET; // Einheiten
																				// überlegen
		xMouse = Math.cos(this.mouseOdoMeasurement.getVSum()) / 1000;
		yMouse = Math.sin(this.mouseOdoMeasurement.getVSum()) / 1000;

//		if (false) { fuer Beispielprogramm Christoph und if Zeiel darunter auskommentieren
		if (!GuidanceAT.getParkmovementInfo()) {
			if (Math.abs(xResult - 1.3) < 0.15 && Math.abs(yResult) < 0.15) {
				if (!angleOverwrite) {
					aMax = angleResult;
					aMin = angleResult;
				}
				if (angleResult > aMax) {
					aMax = angleResult;
				} else if (angleResult < aMin) {
					aMin = angleResult;
				}

				angleOverwrite = true;
				// Sound.beep();
			}
			if (Math.abs(xResult - 0.25) < 0.15 && Math.abs(yResult) < 0.15) {
				if (!angleOverwrite) {
					aMax = angleResult;
					aMin = angleResult;
				}
				if (angleResult > aMax) {
					aMax = angleResult;
				} else if (angleResult < aMin) {
					aMin = angleResult;
				}
				angleOverwrite = true;
				// Sound.beep();
			}
			if (Math.abs(xResult - 1.8) < 0.1 && Math.abs(yResult - 0.25) < 0.15) {
				if (!angleOverwrite) {
					aMax = angleResult;
					aMin = angleResult;
				}
				if (angleResult > aMax) {
					aMax = angleResult;
				} else if (angleResult < aMin) {
					aMin = angleResult;
				}
				angleOverwrite = true;
				// Sound.beep();
			}
			if (Math.abs(xResult) < 0.1 && Math.abs(yResult - 0.35) < 0.15) {
				if (!angleOverwrite) {
					aMax = angleResult;
					aMin = angleResult;
				}
				if (angleResult > aMax) {
					aMax = angleResult;
				} else if (angleResult < aMin) {
					aMin = angleResult;
				}
				angleOverwrite = true;
				// Sound.beep();
			}
			if (Math.abs(xResult - 1.2) < 0.15 && Math.abs(yResult - 0.3) < 0.1) {
				if (!angleOverwrite) {
					aMax = angleResult;
					aMin = angleResult;
				}
				if (angleResult > aMax) {
					aMax = angleResult;
				} else if (angleResult < aMin) {
					aMin = angleResult;
				}
				angleOverwrite = true;
			}

			if (Math.abs(xResult - 0.65) < 0.15 && Math.abs(yResult - 0.3) < 0.1) {
				if (!angleOverwrite) {
					aMax = angleResult;
					aMin = angleResult;
				}
				if (angleResult > aMax) {
					aMax = angleResult;
				} else if (angleResult < aMin) {
					aMin = angleResult;
				}
				angleOverwrite = true;
			}

			
			
			
			if (Math.abs(xResult - 1.55) < 0.1 && Math.abs(yResult) < 0.1 && angleOverwrite) {
				// Sound.beepSequenceUp();
				angleTest = angleResult - (aMax + aMin) / 2;
				if(Math.abs(angleTest) < 0.17){
					angleResult = angleTest;
					angleSetback = true;
				}
//				angleResult = angleResult - (aMax + aMin) / 2;
				angleOverwrite = false;
				aMax = 0;
				aMin = 0;

			}

			if (Math.abs(xResult - 0.5) < 0.1 && Math.abs(yResult) < 0.1 && angleOverwrite) {
				// Sound.beepSequenceUp();
				angleTest = angleResult - (aMax + aMin) / 2;
				if(Math.abs(angleTest) < 0.17){
					angleResult = angleTest;
					angleSetback = true;
				}
//				angleResult = angleResult - (aMax + aMin) / 2;
				angleOverwrite = false;
				aMax = 0;
				aMin = 0;

			}

			if (Math.abs(xResult - 1.8) < 0.1 && Math.abs(yResult - 0.6) < 0.1 && angleOverwrite) {
				// Sound.beepSequenceUp();
				angleTest = angleResult - (aMax + aMin) / 2 + Math.PI / 2;
				if(Math.abs(angleTest - Math.PI / 2) < 0.17){
					angleResult = angleTest;
					angleSetback = true;
				}
//				angleResult = angleResult - (aMax + aMin) / 2 + Math.PI / 2;
				angleOverwrite = false;
				aMax = 0;
				aMin = 0;

			}
			if (Math.abs(xResult) < 0.1 && Math.abs(yResult - 0.1) < 0.1 && angleOverwrite) {
				// Sound.beepSequenceUp();
				angleTest = angleResult - (aMax + aMin) / 2 + Math.PI * 3 / 2;
				if(Math.abs(angleTest - Math.PI * 3/2) < 0.17){
					angleResult = angleTest;
					angleSetback = true;
				}
//				angleResult = angleResult - (aMax + aMin) / 2 + Math.PI * 3 / 2;
				angleOverwrite = false;
				aMax = 0;
				aMin = 0;

			}

			if (Math.abs(xResult - 0.975) < 0.025 && Math.abs(yResult - 0.3) < 0.15 && angleOverwrite) {
				// Sound.beepSequenceUp();
				angleTest = angleResult - (aMax + aMin) / 2 + Math.PI;
				if(Math.abs(angleTest - Math.PI) < 0.17){
					angleResult = angleTest;
					angleSetback = true;
				}
//				angleResult = angleResult - (aMax + aMin) / 2 + Math.PI;
				angleOverwrite = false;
				aMax = 0;
				aMin = 0;

			}

			if (Math.abs(xResult - 0.475) < 0.025 && Math.abs(yResult - 0.3) < 0.15 && angleOverwrite) {
				// Sound.beepSequenceUp();
				angleTest = angleResult - (aMax + aMin) / 2 + Math.PI;
				if(Math.abs(angleTest - Math.PI) < 0.17){
					angleResult = angleTest;
					angleSetback = true;
				}
//				angleResult = angleResult - (aMax + aMin) / 2 + Math.PI;
				angleOverwrite = false;
				aMax = 0;
				aMin = 0;

			}

			if (angleSetback) {
				errorAngle = 0;
				angleSetback = false;
			}
			
			if (Math.abs(xResult - 1.71) < 0.03 && Math.abs(angleResult) < 0.15) {
				xResult = 2 - frontEffective;
				yResult = frontSideEffective - 0.2;
				errorX = 0;
				errorY = 0;
				errorAngle = 0;
			}

			if (Math.abs(xResult - 1.65) < 0.05 && Math.abs(angleResult - Math.PI) < 0.15) {
				xResult = 1.3 + frontEffective;
				yResult = -frontSideEffective + 0.8;
				errorX = 0;
				errorY = 0;
				errorAngle = 0;
			}

			if (Math.abs(xResult - 0.15) < 0.05 && Math.abs(angleResult - Math.PI) < 0.15) {
				xResult = -0.2 + frontEffective;
				yResult = -frontSideEffective + 0.8;
				errorX = 0;
				errorY = 0;
				errorAngle = 0;
			}

			if (Math.abs(yResult - 0.15) < 0.05 && Math.abs(xResult) < 0.05) {
				xResult = -0.2 + frontSideEffective;
				yResult = -0.2 + frontEffective;
				errorX = 0;
				errorY = 0;
				errorAngle = 0;
			}

			switch (((line + 1) % 8)) {
			case 0:
				if (Math.abs(angleResult) < Math.PI / 12) {
					line = 0;
				}
				break;
			case 1:
				if (Math.abs(angleResult - Math.PI / 2) < Math.PI / 12) {
					line = 1;
				}
				break;
			case 2:
				if (Math.abs(angleResult - Math.PI) < Math.PI / 12) {
					line = 2;
				}
				break;
			case 3:
				if (Math.abs(angleResult - 3 * Math.PI / 2) < Math.PI / 12) {
					line = 3;
				}
				break;
			case 4:
				if (Math.abs(angleResult - Math.PI) < Math.PI / 12) {
					line = 4;
				}
				break;
			case 5:
				if (Math.abs(angleResult - Math.PI / 2) < Math.PI / 12) {
					line = 5;
				}
				break;
			case 6:
				if (Math.abs(angleResult - Math.PI) < Math.PI / 12) {
					line = 6;
				}
				break;
			case 7:
				if (Math.abs(angleResult - 3 * Math.PI / 2) < Math.PI / 12) {
					line = 7;
				}
				break;
			default:
				break;
			}

			GuidanceAT.setCurrentLine(line);

		} else {
			angleOverwrite = false;
			aMax = 0;
			aMin = 0;
		}
		
		if(firstStart){
			xResult = 0;
			yResult = 0;
			angleResult = 0;
			firstStart = false;
		}
		this.pose.setLocation((float) xResult, (float) yResult);
		this.pose.setHeading((float) angleResult);
	}

	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots.
	 */

	private void detectParkingSlot(Pose pPose) {
		if (testParking) {
			ParkingSlots.add(new ParkingSlot(0, new Point(30, 0), new Point(74, 0),
					ParkingSlotStatus.SUITABLE_FOR_PARKING, 0, 0));
			ParkingSlots.add(new ParkingSlot(1, new Point(105, 0), new Point(165, 0),
					ParkingSlotStatus.SUITABLE_FOR_PARKING, 0, 0));
			ParkingSlots.add(new ParkingSlot(2, new Point(180, 15), new Point(180, 60),
					ParkingSlotStatus.SUITABLE_FOR_PARKING, 0, 1));
			testParking = false;
			existingParkingSlot = true;
		}

		if ((this.pose.getY() < 0.10 || this.pose.getX() > 1.70
				|| (0.45 < this.pose.getX() && this.pose.getX() < 1.3 + BACK_SIDE_SENSOR_OFFAXIS))
				&& (Math.abs(this.pose.getHeading()) < 0.2 || Math.abs(this.pose.getHeading() - Math.PI / 2) < 0.2
						|| Math.abs(this.pose.getHeading() - Math.PI) < 0.2)) {
			float poseX = this.pose.getX() * 100;
			float poseY = this.pose.getY() * 100;
			
			if (this.pose.getX() < 0.5 + FRONT_SIDE_SENSOR_OFFAXIS && line == 4) {
				this.frontSideSensorDistance = 150;
			}
			
			if (this.frontSideSensorDistance > 300 && !possibleParkingSlot) {
				// Sound.twoBeeps();
				possibleParkingSlot = true;
			}
			
			if (this.backSideSensorDistance > 300 && possibleParkingSlot && !finishBack) {

				backBackBoundaryX = poseX;
				backBackBoundaryY = poseY;
				backBackBoundaryX += xBackBouandaryOffset();
				backBackBoundaryY += yBackBouandaryOffset();
				finishBack = true;
				backBoundaryPositionX = backBackBoundaryX;
				backBoundaryPositionY = backBackBoundaryY;
				backBackError = errorX + errorY + errorAngle / 3;
			}
			
			if (this.frontSideSensorDistance < 200 && possibleParkingSlot) {
				frontFrontBoundaryX = poseX;
				frontFrontBoundaryY = poseY;
				frontFrontBoundaryX += xFrontBouandaryOffset();
				frontFrontBoundaryY += yFrontBouandaryOffset();
				possibleParkingSlot = false;
				frontSensorFinished = true;
				frontFrontError = errorX + errorY + errorAngle / 3;

				finishBack = false;
				frontBoundaryPositionX = frontFrontBoundaryX;
				frontBoundaryPositionY = frontFrontBoundaryY;
				ParkingSlotError = (frontFrontError + backBackError) / 2;

				if (ParkingSlotError < 0.05) {
					measurementQuality = 1;
				} else if (ParkingSlotError < 0.10) {
					measurementQuality = 2;
				} else if (ParkingSlotError < 0.15) {
					measurementQuality = 3;
				} else if (ParkingSlotError < 0.20) {
					measurementQuality = 4;
				} else if (ParkingSlotError < 0.25) {
					measurementQuality = 5;
				} else if (ParkingSlotError < 0.30) {
					measurementQuality = 6;
				} else if (ParkingSlotError < 0.35) {
					measurementQuality = 7;
				} else if (ParkingSlotError < 0.40) {
					measurementQuality = 8;
				} else if (ParkingSlotError < 0.45) {
					measurementQuality = 9;
				} else {
					measurementQuality = 10;
				}

				if(line == 0){
					frontBoundaryPositionY = -25;
					backBoundaryPositionY = -25;
				}else if(line == 1){
					frontBoundaryPositionX = 205;
					backBoundaryPositionX = 205;
				}else{
					frontBoundaryPositionY = 55;
					backBoundaryPositionY = 55;
				}
				if ((Math.abs(frontBoundaryPositionX - backBoundaryPositionX) > 40
						|| Math.abs(frontBoundaryPositionY - backBoundaryPositionY) > 40) && measurementQuality < 7) {
					ParkingSlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
				} else {
					ParkingSlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
				}
				newSlot = true;
				i = 0;

			}

			if (newSlot) {
				if (existingParkingSlot) {

					if ((Math.abs(ParkingSlots.get(i).getBackBoundaryPosition().getX() - backBoundaryPositionX) < 10
							&& Math.abs(ParkingSlots.get(i).getBackBoundaryPosition().getY()
									- backBoundaryPositionY) < 10)) {
						if (ParkingSlots.get(i).getMeasurementQuality() >= measurementQuality) {
							newSlot = false;
						} else {
//							ParkingSlots.get(i).setBackBoundaryPosition(
//									new Point((float) backBoundaryPositionX, (float) backBoundaryPositionY));
//							ParkingSlots.get(i).setFrontBoundaryPosition(
//									new Point((float) frontBoundaryPositionX, (float) frontBoundaryPositionY));
//							ParkingSlots.get(i).setMeasurementQuality(measurementQuality);
//							ParkingSlots.get(i).setStatus(ParkingSlotStatus);
							ParkingSlots.add(new ParkingSlot(i,
									new Point(backBoundaryPositionX, backBoundaryPositionY),
									new Point(frontBoundaryPositionX, frontBoundaryPositionY), ParkingSlotStatus,
									measurementQuality, line));
							newSlot = false;
//							Sound.beepSequenceUp();
						}
					}

					if (newSlot && i == (ParkingSlots.size() - 1)) {
						ParkingSlots.add(new ParkingSlot(ParkingSlots.size(),
								new Point(backBoundaryPositionX, backBoundaryPositionY),
								new Point(frontBoundaryPositionX, frontBoundaryPositionY), ParkingSlotStatus,
								measurementQuality, line));
						// Sound.beepSequenceUp();
						newSlot = false;
						existingParkingSlot = true;
					}
				}

				if (!existingParkingSlot) {
					ParkingSlots.add(new ParkingSlot(0, new Point(backBoundaryPositionX, backBoundaryPositionY),
							new Point(frontBoundaryPositionX, frontBoundaryPositionY), ParkingSlotStatus,
							measurementQuality, line));
					// Sound.beepSequenceUp();
					newSlot = false;
					existingParkingSlot = true;
				}
				i = i + 1;
			}

		}

		// has to be implemented by students
	}

}
