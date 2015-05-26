/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package amp.sim;

import java.awt.Color;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;

import amp.TrajectoryCells.CellsTrajectoryLists;
import amp.config.Debug;
import amp.config.DebugPoint;
import amp.driver.AutoDriver;
import amp.driver.DriverSimView;
import amp.driver.ProxyDriver;
import amp.driver.coordinator.MaxAccelReservationCheck;
import amp.intersection.IntersectionDetails;
import amp.map.BasicMap;
import amp.map.DataCollectionLine;
import amp.map.Road;
import amp.map.SpawnPoint;
import amp.map.SpawnPoint.SpawnSpec;
import amp.map.lane.Lane;
import amp.msg.v2v.V2VMessage;
import amp.policy.policy;
import amp.util.GeomMath;
import amp.util.TiledArea;
import amp.util.Util;
import amp.util.TiledArea.Tile;
import amp.vehicle.AutoVehicleSimView;
import amp.vehicle.BasicAutoVehicle;
import amp.vehicle.ProxyVehicleSimView;
import amp.vehicle.VehicleSimView;
import amp.vehicle.VehicleSpec;
import amp.vehicle.VehicleUtil;
import amp.vehicle.VinRegistry;

//

/**
 * The autonomous drivers only simulator.
 */
public class AutoDriverOnlySimulator implements Simulator {

	// ///////////////////////////////
	// NESTED CLASSES
	// ///////////////////////////////
	/**
	 * The result of a simulation step.
	 */
	public static class AutoDriverOnlySimStepResult implements SimStepResult {

		/** The VIN of the completed vehicles in this time step */
		List<Integer> completedVINs;
		/**
		 * Create a result of a simulation step
		 * 
		 * @param completedVINs
		 *            the VINs of completed vehicles.
		 */
		public AutoDriverOnlySimStepResult(List<Integer> completedVINs) {
			this.completedVINs = completedVINs;
		}

		/**
		 * Get the list of VINs of completed vehicles.
		 *  
		 * @return the list of VINs of completed vehicles.
		 */
		public List<Integer> getCompletedVINs() {
			return completedVINs;
		}
	}

	// ///////////////////////////////
	// PRIVATE FIELDS
	// ///////////////////////////////

	/** The map */
	private final BasicMap basicMap;
	/** All active vehicles, in form of a map from VINs to vehicle objects. */
	private final Map<Integer, VehicleSimView> vinToVehicles;
	/** The current time */
	private double currentTime;
	/** The number of completed vehicles */
	private int numOfCompletedVehicles;
	/** The total number of bits transmitted by the completed vehicles */
	private int totalBitsTransmittedByCompletedVehicles;
	/** The total number of bits received by the completed vehicles */
	private int totalBitsReceivedByCompletedVehicles;
	/** Intersection area */
	private Area area;
	 /**
	   * The minimum distance to maintain between the Vehicle controlled now and the one in front of it.
	   *  {@value} meters.
	   */	
	private final double MINIMUM_FOLLOWING_DISTANCE = 0.5  ;//1.0;//2.0;//0.5 
	/**
	 *  Distance between vehicle and intersection at which the vehicle is considered to have arrived at the intersection
	 */
	private final double thresholdDistance= 0.0; //0.0 MINIMUM_FOLLOWING_DISTANCE;
	/**
	 *  A vehicle stopping before the intersection will stop this meters apart from the intersection boundaries 
	 */
	private final double stopbeforeintersectionthreshold = 0.5;// 1.0 //2.0 //3.0
	/**
	 *  Start turning when center of vehicle is this distance apart from center of cell (here center of cell is not the actual center but the vertical distance between the vehicle's heading and the vertical bisector line of the cell)
	 *  {@value} meters
	 */
	private final double thresholddistancetoturn = 1.3;//1.3 // we try to adjust it so that vehicles turn without touching any nearby cell
	/**
	 *  Stop completely when rear of vehicle is this distance apart from center of cell (here center of cell is not the actual center but the vertical distance between the vehicle's heading and the vertical bisector line of the cell)
	 *  {@value} meters
	 */
	private final double thresholddistancetostop = 2.0;//2.0 // we try to adjust it so that vehicles dont stop unless they are entirely inside cell
	/**
	 * Number of cells in each side of the Intersection after dividing the intersection into cells
	 */
	private double noofcellsperside=4.0;
	/**
	 * Distance to Intersection at which vehicles start sending Enter messages 
	 *  {@value} meters
	 */
	private final double Denter =80.0;//20.0 meters 

	//SafetyTimeInterval is calculated as Math.ceil(Math.sqrt((2 * cellwidth)/(MaxAcceleration))); 
	private double SafetyTimeInterval;
	
	private double MaxAcceleration;
	
	private int nooflanes=2; // TODO: get it from simulator configuration
	
	// ///////////////////////////////
	// CLASS CONSTRUCTORS
	// ///////////////////////////////

	/**
	 * Create an instance of the simulator.
	 * 
	 * @param basicMap
	 *            the map of the simulation
	 */
	public AutoDriverOnlySimulator(BasicMap basicMap) {
		this.basicMap = basicMap;
		this.vinToVehicles = new HashMap<Integer, VehicleSimView>();

		currentTime = 0.0;
		numOfCompletedVehicles = 0;
		totalBitsTransmittedByCompletedVehicles = 0;
		totalBitsReceivedByCompletedVehicles = 0;
	}

	// ///////////////////////////////
	// PUBLIC METHODS
	// ///////////////////////////////

	// the main loop

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized AutoDriverOnlySimStepResult step(double timeStep) {
		
		
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err.printf("--------------------------------------\n");
			System.err.printf("------SIM:spawnVehicles---------------\n");
		}
		spawnVehicles(timeStep);
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err.printf("------SIM:provideSensorInput---------------\n");
		}
		provideSensorInput();
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err.printf("------SIM:v2vCommunication---------------\n");
		}
		v2vCommunication();
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err.printf("------SIM:controller---------------\n");
		}
		controller();
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err.printf("------SIM:moveVehicles---------------\n");
		}
		moveVehicles(timeStep);
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err
					.printf("------SIM:testing---------------\n");
		}
		testing();
		if (Debug.PRINT_SIMULATOR_STAGE) {
			System.err
					.printf("------SIM:cleanUpCompletedVehicles---------------\n");
		}
		List<Integer> completedVINs = cleanUpCompletedVehicles();
		currentTime += timeStep;

		return new AutoDriverOnlySimStepResult(completedVINs);
	}

	// ///////////////////////////////
	// PUBLIC METHODS
	// ///////////////////////////////

	// information retrieval

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized BasicMap getMap() {
		return basicMap;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized double getSimulationTime() {
		return currentTime;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized int getNumCompletedVehicles() {
		return numOfCompletedVehicles;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized double getAvgBitsTransmittedByCompletedVehicles() {
		if (numOfCompletedVehicles > 0) {
			return ((double) totalBitsTransmittedByCompletedVehicles)
					/ numOfCompletedVehicles;
		} else {
			return 0.0;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized double getAvgBitsReceivedByCompletedVehicles() {
		if (numOfCompletedVehicles > 0) {
			return ((double) totalBitsReceivedByCompletedVehicles)
					/ numOfCompletedVehicles;
		} else {
			return 0.0;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized Set<VehicleSimView> getActiveVehicles() {
		return new HashSet<VehicleSimView>(vinToVehicles.values());
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public VehicleSimView getActiveVehicle(int vin) {
		return vinToVehicles.get(vin);
	}

	// ///////////////////////////////
	// PUBLIC METHODS
	// ///////////////////////////////

	/**
	 * {@inheritDoc}
	 */
	@Override
	public synchronized void addProxyVehicle(ProxyVehicleSimView vehicle) {
		Point2D pos = vehicle.getPosition();
		Lane minLane = null;
		double minDistance = -1.0;

		for (Road road : basicMap.getRoads()) {
			for (Lane lane : road.getLanes()) {
				double d = lane.nearestDistance(pos);
				if (minLane == null || d < minDistance) {
					minLane = lane;
					minDistance = d;
				}
			}
		}
		assert minLane != null;

		ProxyDriver driver = vehicle.getDriver();
		if (driver != null) {
			driver.setCurrentLane(minLane);
			driver.setSpawnPoint(null);
			driver.setDestination(null);
		}

		vinToVehicles.put(vehicle.getVIN(), vehicle);
	}

	// ///////////////////////////////
	// PRIVATE METHODS
	// ///////////////////////////////

	// ///////////////////////////////
	// STEP 1
	// ///////////////////////////////

	/**
	 * Spawn vehicles.
	 * 
	 * @param timeStep
	 *            the time step
	 */
	private void spawnVehicles(double timeStep) {
		for (SpawnPoint spawnPoint : basicMap.getSpawnPoints()) {
			List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
			if (!spawnSpecs.isEmpty()) {
				if (canSpawnVehicle(spawnPoint)) {
					for (SpawnSpec spawnSpec : spawnSpecs) {
						VehicleSimView vehicle = makeVehicle(spawnPoint,
								spawnSpec);
						VinRegistry.registerVehicle(vehicle); // Get vehicle a
																// VIN number
						vinToVehicles.put(vehicle.getVIN(), vehicle);
		System.out.println("#vehicle "+vehicle.getVIN()+" created at current time "+ currentTime);

						break; // only handle the first spawn vehicle
								// TODO: need to fix this
					}
				} // else ignore the spawnSpecs and do nothing
			}
		}
	}

	/**
	 * Whether a spawn point can spawn any vehicle
	 * 
	 * @param spawnPoint
	 *            the spawn point
	 * @return Whether the spawn point can spawn any vehicle
	 */
	private boolean canSpawnVehicle(SpawnPoint spawnPoint) {
		// TODO: can be made much faster.
		Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
		for (VehicleSimView vehicle : vinToVehicles.values()) {
			if (vehicle.getShape().intersects(noVehicleZone)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Create a vehicle at a spawn point.
	 * 
	 * @param spawnPoint
	 *            the spawn point
	 * @param spawnSpec
	 *            the spawn specification
	 * @return the vehicle
	 */
	private VehicleSimView makeVehicle(SpawnPoint spawnPoint,
			SpawnSpec spawnSpec) {
		VehicleSpec spec = spawnSpec.getVehicleSpec();
		Lane lane = spawnPoint.getLane();
		// Now just take the minimum of the max velocity of the vehicle, and
		// the speed limit in the lane
		double initVelocity = Math.min(spec.getMaxVelocity(),
				lane.getSpeedLimit());
		// Obtain a Vehicle
		AutoVehicleSimView vehicle = new BasicAutoVehicle(spec,
				spawnPoint.getPosition(), spawnPoint.getHeading(),
				spawnPoint.getSteeringAngle(), initVelocity, // velocity
				initVelocity, // target velocity
				spawnPoint.getAcceleration(), spawnSpec.getSpawnTime());
		// Set the driver
		AutoDriver driver = new AutoDriver(vehicle, basicMap);
		driver.setCurrentLane(lane);
		driver.setSpawnPoint(spawnPoint);
		driver.setDestination(spawnSpec.getDestinationRoad());
		vehicle.setDriver(driver);
//		vehicle.getDriver().setDestination(Debug.currentMap.getRoad(vehicle.getDriver().getCurrentLane()));
		return vehicle;
	}

	// ///////////////////////////////
	// STEP 2
	// ///////////////////////////////

	/**
	 * Compute the lists of vehicles of all lanes.
	 * 
	 * @return a mapping from lanes to lists of vehicles sorted by their
	 *         distance on their lanes
	 */
	private Map<Lane, SortedMap<Double, VehicleSimView>> computeVehicleLists() {
		// Set up the structure that will hold all the Vehicles as they are
		// currently ordered in the Lanes
		Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists = new HashMap<Lane, SortedMap<Double, VehicleSimView>>();
		for (Road road : basicMap.getRoads()) {
			for (Lane lane : road.getLanes()) {
				vehicleLists.put(lane, new TreeMap<Double, VehicleSimView>());
			}
		}
		// Now add each of the Vehicles, but make sure to exclude those that are
		// already inside (partially or entirely) the intersection
		for (VehicleSimView vehicle : vinToVehicles.values()) {
			// Find out what lanes it is in.
			Set<Lane> lanes = vehicle.getDriver().getCurrentlyOccupiedLanes();
			for (Lane lane : lanes) {
				// Find out what IntersectionDetails is coming up for this
				// vehicle
				IntersectionDetails im = lane.getLaneIM()
						.nextIntersectionManager(vehicle.getPosition());
				// Only include this Vehicle if it is not in the intersection.
//				if (lane.getLaneIM().distanceToNextIntersection(
//						vehicle.getPosition()) > 0
//						|| im == null 
//						|| !im.intersects(vehicle.getShape().getBounds2D())) {
				if (lane.getLaneIM().distanceToNextIntersection(
						vehicle.getPointAtRear()) > 0
						|| im == null) {
				
				// Now find how far along the lane it is.
					double dst = lane.distanceAlongLane(vehicle.getPosition());
					// Now add it to the map.
					vehicleLists.get(lane).put(dst, vehicle);
				}
			}
		}
		// Now consolidate the lists based on lanes
		for (Road road : basicMap.getRoads()) {
			for (Lane lane : road.getLanes()) {
				// We may have already removed this Lane from the map
				if (vehicleLists.containsKey(lane)) {
					Lane currLane = lane;
					// Now run through the lanes
					while (currLane.hasNextLane()) {
						currLane = currLane.getNextLane();
						// Put everything from the next lane into the original
						// lane
						// and remove the mapping for the next lane
						vehicleLists.get(lane).putAll(
								vehicleLists.remove(currLane));
					}
				}
			}
		}

		return vehicleLists;
	}

	/**
	 * Compute the next vehicles of all vehicles.
	 * 
	 * @param vehicleLists
	 *            a mapping from lanes to lists of vehicles sorted by their
	 *            distance on their lanes
	 * @return a mapping from vehicles to next vehicles
	 */
	private Map<VehicleSimView, VehicleSimView> computeNextVehicle(
			Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
		// At this point we should only have mappings for start Lanes, and they
		// should include all the Lanes they run into. Now we need to turn this
		// into a hash map that maps Vehicles to the next vehicle in the Lane
		// or any Lane the Lane runs into
		Map<VehicleSimView, VehicleSimView> nextVehicle = new HashMap<VehicleSimView, VehicleSimView>();
		// For each of the ordered lists of vehicles
		for (SortedMap<Double, VehicleSimView> vehicleList : vehicleLists
				.values()) {
			VehicleSimView lastVehicle = null;
			// Go through the Vehicles in order of their position in the Lane
			for (VehicleSimView currVehicle : vehicleList.values()) {
				if (lastVehicle != null) {
					// Create the mapping from the previous Vehicle to the
					// current one
					nextVehicle.put(lastVehicle, currVehicle);
				}
				lastVehicle = currVehicle;
			}
		}

		return nextVehicle;
	}

	/**
	 * Provide each vehicle with sensor information to allow it to make
	 * decisions. This works first by making an ordered list for each Lane of
	 * all the vehicles in that Lane, in order from the start of the Lane to the
	 * end of the Lane. We must make sure to leave out all vehicles that are in
	 * the intersection. We must also concatenate the lists for lanes that feed
	 * into one another. Then, for each vehicle, depending on the state of its
	 * sensors, we provide it with the appropriate sensor input.
	 */
	private void provideSensorInput() {
		Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists = computeVehicleLists();
		Map<VehicleSimView, VehicleSimView> nextVehicle = computeNextVehicle(vehicleLists);

		provideIntervalInfo(nextVehicle);
		provideVehicleTrackingInfo(vehicleLists);
		provideVehiclePositionWithinCell();
	}

	/**
	 * Provide tracking information to vehicles.
	 * 
	 * @param vehicleLists
	 *            a mapping from lanes to lists of vehicles sorted by their
	 *            distance on their lanes
	 */
	private void provideVehicleTrackingInfo(
			Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
		// Vehicle Tracking
		for (VehicleSimView vehicle : vinToVehicles.values()) {
			// If the vehicle is autonomous
			if (vehicle instanceof AutoVehicleSimView) {
				AutoVehicleSimView autoVehicle = (AutoVehicleSimView) vehicle;
	
				if (autoVehicle.isVehicleTracking()) {
					DriverSimView driver = autoVehicle.getDriver();
					Lane targetLane = autoVehicle
							.getTargetLaneForVehicleTracking();
					Point2D pos = autoVehicle.getPosition();
					double dst = targetLane.distanceAlongLane(pos);
	
					// initialize the distances to infinity
					double frontDst = Double.MAX_VALUE;
					double rearDst = Double.MAX_VALUE;
					VehicleSimView frontVehicle = null;
					VehicleSimView rearVehicle = null;
	
					// only consider the vehicles on the target lane
					SortedMap<Double, VehicleSimView> vehiclesOnTargetLane = vehicleLists
							.get(targetLane);
	
					// compute the distances and the corresponding vehicles
					try {
						double d = vehiclesOnTargetLane.tailMap(dst).firstKey();
						frontVehicle = vehiclesOnTargetLane.get(d);
						frontDst = (d - dst)
								- frontVehicle.getSpec().getLength();
					} catch (NoSuchElementException e) {
						frontDst = Double.MAX_VALUE;
						frontVehicle = null;
					}
					try {
						double d = vehiclesOnTargetLane.headMap(dst).lastKey();
						rearVehicle = vehiclesOnTargetLane.get(d);
						rearDst = dst - d;
					} catch (NoSuchElementException e) {
						rearDst = Double.MAX_VALUE;
						rearVehicle = null;
					}
	
					// assign the sensor readings
	
					autoVehicle.getFrontVehicleDistanceSensor()
							.record(frontDst);
					autoVehicle.getRearVehicleDistanceSensor().record(rearDst);
	
					// assign the vehicles' velocities
	
					if (frontVehicle != null) {
						autoVehicle.getFrontVehicleSpeedSensor().record(
								frontVehicle.getVelocity());
					} else {
						autoVehicle.getFrontVehicleSpeedSensor().record(
								Double.MAX_VALUE);
					}
					if (rearVehicle != null) {
						autoVehicle.getRearVehicleSpeedSensor().record(
								rearVehicle.getVelocity());
					} else {
						autoVehicle.getRearVehicleSpeedSensor().record(
								Double.MAX_VALUE);
					}
	
					// show the section on the viewer
					if (Debug.isTargetVIN(driver.getVehicle().getVIN())) {
						Point2D p1 = targetLane
								.getPointAtNormalizedDistance(Math.max(
										(dst - rearDst)
												/ targetLane.getLength(), 0.0));
						Point2D p2 = targetLane
								.getPointAtNormalizedDistance(Math.min(
										(frontDst + dst)
												/ targetLane.getLength(), 1.0));
						Debug.addLongTermDebugPoint(new DebugPoint(p2, p1,
								"cl", Color.RED.brighter()));
					}
				}
			}
		}
	
	}

	/**
	 * Provide sensing information to the intervalometers of all vehicles.
	 * 
	 * @param nextVehicle
	 *            a mapping from vehicles to next vehicles
	 */
	private void provideIntervalInfo(
			Map<VehicleSimView, VehicleSimView> nextVehicle) {

		// Now that we have this list set up, let's provide input to all the
		// Vehicles.
		for (VehicleSimView vehicle : vinToVehicles.values()) {
			// If the vehicle is autonomous
			if (vehicle instanceof AutoVehicleSimView) {
				AutoVehicleSimView autoVehicle = (AutoVehicleSimView) vehicle;

				switch (autoVehicle.getLRFMode()) {
				case DISABLED:
					// Find the interval to the next vehicle
					double interval;
					// If there is a next vehicle, then calculate it
					if (nextVehicle.containsKey(autoVehicle)) {
						// It's the distance from the front of this Vehicle to
						// the point
						// at the rear of the Vehicle in front of it
						interval = calcInterval(autoVehicle,
								nextVehicle.get(autoVehicle));
					} else { // Otherwise, just set it to the maximum possible
								// value
						interval = Double.MAX_VALUE;
					}
					// Now actually record it in the vehicle
					autoVehicle.getIntervalometer().record(interval);
					autoVehicle.setLRFSensing(false); // Vehicle is not using
														// the LRF sensor
					break;
				case LIMITED:
					// FIXME
					autoVehicle.setLRFSensing(true); // Vehicle is using the LRF
														// sensor
					break;
				case ENABLED:
					// FIXME
					autoVehicle.setLRFSensing(true); // Vehicle is using the LRF
														// sensor
					break;
				default:
					throw new RuntimeException("Unknown LRF Mode: "
							+ autoVehicle.getLRFMode().toString());
				}
			}
		}
	}

	/**
	 * Calculate the distance between vehicle and the next vehicle on a lane.
	 * 
	 * @param vehicle
	 *            the vehicle
	 * @param nextVehicle
	 *            the next vehicle
	 * @return the distance between vehicle and the next vehicle on a lane
	 */
	private double calcInterval(VehicleSimView vehicle,
			VehicleSimView nextVehicle) {
		// From Chiu: Kurt, if you think this function is not okay, probably
		// we should talk to see what to do.
		Point2D pos = vehicle.getPosition();
		if (nextVehicle.getShape().contains(pos)) {
			return 0.0;
		} else {
			// TODO: make it more efficient
			double interval = Double.MAX_VALUE;
			for (Line2D edge : nextVehicle.getEdges()) {
				double dst = edge.ptSegDist(pos);
				if (dst < interval) {
					interval = dst;
				}
			}
			return interval;
		}
	}

	/**
	 * Provide vehicle's position within a cell, to check:
	 * 1. Weather the vehicle has partially/completely entered or intersects the cell
	 * 2. Turning points of a cell have been reached => signaling that the vehicle should start turning
	 */
	private void provideVehiclePositionWithinCell() {
			
		for (IntersectionDetails im : basicMap.getIntersectionManagers()) {
			
		// a tile is another name for a cell that is used originally by AIM	
				area= im.getIntersection().getArea();
				if (noofcellsperside != 4.0 && noofcellsperside !=6.0)
				{	
				noofcellsperside=4.0;	
				System.out.println("This no of cells per side is not supported so no of cells will be initialized automatically to 4");
				}	
				TiledArea tiledarea=new TiledArea(area,area.getBounds2D().getWidth()/noofcellsperside); //TODO: size of tile is hardcoded , change that		
				
						
			for (VehicleSimView sendvehicle : vinToVehicles.values()) {
				if (sendvehicle instanceof AutoVehicleSimView) {
				    AutoVehicleSimView sender = (AutoVehicleSimView) sendvehicle;
				   
				    MaxAcceleration= sender.getSpec().getMaxAcceleration();
				    
				    int deptlane= sender.getDriver().getCurrentLane().getId();
					String deptroad= Debug.currentMap.getRoad(sender.getDriver().getCurrentLane()).getName();
					String arrivroad= sender.getDriver().getDestination().getName();
				    
				    for(int i=0;i<tiledarea.getXNum()-1;i++){
						for(int j=0;j<tiledarea.getYNum()-1;j++){
					int tileid=tiledarea.getTile(i,j).getId();
					Tile tile = tiledarea.getTileById(tileid);
					
	    if(tile.getRectangle().contains(sender.getPosition()))
	    {
	    	 System.out.println("#vehicle " + sender.getVIN() + " inside tileid: "+ tileid +" at time "+ currentTime);
//	 	    System.out.println("#vehicle "+ sender.getVIN() + " distance between its front and center of tile "+ tileid + " is "+ Math.abs(Point2D.distance(sender.getCenterPoint().getX(), sender.getCenterPoint().getY(), tile.getRectangle().getCenterX(), tile.getRectangle().getCenterY()))+  " with tile having center point of  " + tile.getRectangle().getCenterX() + " "+ tile.getRectangle().getCenterY() +" and front vehicle point of "+ sender.getPosition());
	    	 sender.setcurrentcell(tileid); 
	    }
	   
	//	if(tile.getRectangle().contains(sender.getPointAtRear())) // after the whole vehicle enters the cell
	//	{
		int turncellId =new  CellsTrajectoryLists().getturningcell(deptroad, arrivroad, deptlane, noofcellsperside); 
	    if(turncellId == tileid){
		Point2D centerpointofcell = new Point2D.Double(tile.getRectangle().getCenterX(),tile.getRectangle().getCenterY());
		
		double angle = GeomMath.angleToPoint(centerpointofcell,sender.gaugePointBetweenFrontWheels());
		// Need to recenter this value to [-pi, pi]
		double Anglebetweenvehicleandcenterofcell = Util.recenter(angle - sender.getHeading(),
				-1.0 * Math.PI, Math.PI);
		double distancetocenterofcell = Math.abs(Point2D.distance(tile.getRectangle().getCenterX(), tile.getRectangle().getCenterY(), sender.gaugePointBetweenFrontWheels().getX(), sender.gaugePointBetweenFrontWheels().getY()));
		double verticaldistancetocenterofcell = distancetocenterofcell * Math.cos(Anglebetweenvehicleandcenterofcell) ;//with respect to vehicle heading
		
		if(verticaldistancetocenterofcell <thresholddistancetoturn){ 
	//	int turncellId =new  CellsTrajectoryLists().getturningcell(deptroad, arrivroad, deptlane, noofcellsperside); 
	//    	if(turncellId == tileid){
	    		    	sender.setturning(); // we set turning variable when the vehicle is completely inside the turning cell and the distance between the cell's center is under a certain threshold
	    		    } 
	    	
	    System.out.println("#vehicle "+ sender.getVIN() + " distance between its front and center of tile "+ tileid + " is "+ verticaldistancetocenterofcell+ " with tile having center point of  " + tile.getRectangle().getCenterX()+ " "+ tile.getRectangle().getCenterY() +" and rear vehicle point of "+ sender.getPointAtRear());
		
		}
//		}
		// For testing,
		// check if the vehicle intersects the cell
	    if(tile.getRectangle().intersects(sender.getShape().getBounds2D()))
	    {	
	    boolean cellisinlist = false;
	    	
	    for(int e=0;e<sender.getfirsttrajectorylist().size();e++)
	    {
	    	if(sender.getfirsttrajectorylist().get(e) == tileid)
	    		cellisinlist=true;
	    }
	    
	    if (cellisinlist==false){
	    	//system.err.println("#vehicle "+ sender.getVIN() + " intersects cell "+ tileid +" not in traj cell list");
	    	System.out.println("#vehicle "+ sender.getVIN() + " intersects cell "+ tileid +" not in traj cell list");
	    }else
	    	System.out.println("#vehicle "+ sender.getVIN() + " intersects cell "+ tileid + " which is in traj list");
	    
	    }
			}
			}
			}
			}	
			}
		
		AutoVehicleSimView vehicle;
		double cellwidth=area.getBounds2D().getWidth()/noofcellsperside;
		SafetyTimeInterval= Math.ceil(Math.sqrt((2 * cellwidth)/(MaxAcceleration))); 
		
			}
	/////////////////////////////////
	// STEP 3
	/////////////////////////////////

	/**
	 * Send and receive messages between vehicles. 
	 * Each vehicle processes the received message by calling the AMP policy  
	 */
	private void v2vCommunication() {
	// loop through each vehicle and broadcast its message	
	for (VehicleSimView sendvehicle : vinToVehicles.values()) {
	if (sendvehicle instanceof AutoVehicleSimView) {
	    AutoVehicleSimView sender = (AutoVehicleSimView) sendvehicle;
	    
	// depending on the position of the vehicle and its distance to the intersection
	// return the type of message required to be sent      
	// 1 => ENTER , 2 => CROSS, 3 => EXIT  
	int messagetype=typeofmessagetosend(sender);

//if the distance to the next vehicle is smaller than a threshold (following distance) vehicle will stop and it wont know how long will it stop for so it stops sending and receiving messages 
//double stoppingDistance = distIfStopNextTimeStep(vehicle);

	//change-Oct-07-2014 - commenting out these lines///////
double stoppingDistance =
  VehicleUtil.calcDistanceToStop(sender.gaugeVelocity(),sender.getSpec().getMaxDeceleration());
double followingDistance = stoppingDistance + MINIMUM_FOLLOWING_DISTANCE;

//if(VehicleUtil.distanceToCarInFront(sender) >= followingDistance){
//////////
if(sender.getState() > 0){ // start sending and receiving messages from Enter state
// if message is one of the 3 types then create and send the message
	if( messagetype == 1 || messagetype == 2 || messagetype == 3 )
	{
	V2VMessage	message = createmessage(sender,messagetype);
	//  send message by adding message to vehicle's outbox
	sendmessage(sender,message);
	
	}
	//loop for each vehicle and receive broadcasted message and process it	
	for (VehicleSimView receivevehicle : vinToVehicles.values()) {
		if (receivevehicle instanceof AutoVehicleSimView) {
		    AutoVehicleSimView receiver = (AutoVehicleSimView) receivevehicle;    
		// sender vehicle cannot be receiver vehicle    
		    
	    if(sender.getVIN() != receiver.getVIN())
		{
	    	// recieve message by reading from sender's outbox
	    	receivemessage(receiver,sender);
			// process message by running the AMP policy
			processmessage(receiver); //get vehicle dep and arrival lane to convert proposal to tcl through table		
		}
		}
		}
	}
	}
	}
	// decide from all the processed messages which cell to stop at and for how long
	takefinaldecision();
	
	}
	
	/**
	 * Creating message 
	 * 
	 * @param sender
	 * 			Vehicle creating the message
	 * @param messagetype
	 * 			Message type to send (Enter,Cross or Exit) 
	 * @return V2VMessage
	 * 			The created message
	 *<br>
	 * V2VMessage contains:
	 * 		 Vehicle ID
	 *       Current Road Segment
	 *       Current Lane
	 *       Next Road Segment
	 *       Next Vertex
	 *       Arrival-Time
	 *       ExitTime
	 *       TrajectoryCellsList
	 *       CellsArrivalTimeList
	 *       MessageSequenceNumber
	 *       MessageType
	 */
	private V2VMessage createmessage(AutoVehicleSimView sender,int messagetype)
	{
		V2VMessage v2vmessage;
		
		if(messagetype==1 || messagetype == 2)
		{
		int deptlane= sender.getDriver().getCurrentLane().getId();
		String deptroad= Debug.currentMap.getRoad(sender.getDriver().getCurrentLane()).getName();
		String arrivroad= sender.getDriver().getDestination().getName();
		int arrivlane= getArrivLane(deptlane,deptroad,arrivroad);
		
		double arriveTime; // Time at which last Enter message is sent, before entering the intersection
		String msgtype = null;				
	
		//remove cells in the list which the vehicle already crossed		
		int currenttileid =-1;
		for (IntersectionDetails im : basicMap.getIntersectionManagers()) {
				
				// a tile is another name for a cell that is used originally by AIM	
		TiledArea tiledarea=new TiledArea(im.getIntersection().getArea(),area.getBounds2D().getWidth()/noofcellsperside); //TODO: size of tile is hardcoded , change that
	
		OUTERLOOP:		
		for(int k=0;k<tiledarea.getXNum()-1;k++){
				for(int j=0;j<tiledarea.getYNum()-1;j++){
		    int tileid=tiledarea.getTile(k,j).getId();
			Tile tile = tiledarea.getTileById(tileid);
			
			if(tile.getRectangle().contains(sender.getPosition())){
				currenttileid =tileid; 
				break OUTERLOOP;
			}
				}
		}
		}	
		
		CellsTrajectoryLists tctl= new CellsTrajectoryLists(); 
		// TrajectoryCellsListandCellsArrivalTimeListWithExitTime returns two lists:
		//1. Trajectory cells list 
		//2. Trajectory Cells arrival Time list with Exit time: which includes Trajectory Cells arrival Time list + an extra element at the end of the list which is the trajected exit time
		List<Double>[] cellsandtimelist=tctl.TrajectoryCellsListandCellsArrivalTimeListWithExitTime(sender, currentTime,basicMap, noofcellsperside,MINIMUM_FOLLOWING_DISTANCE,thresholddistancetoturn,thresholddistancetostop,stopbeforeintersectionthreshold);
		
		List <Integer> cellslist = new ArrayList<Integer>();
		
		for(int i=0;i<cellsandtimelist[0].size();i++)
		cellslist.add(new Double(cellsandtimelist[0].get(i)).intValue()); // first array is the trajectory cells list 
		
		// After creating the tcl now save it.
		sender.setTrajectoryCellsList(cellslist);
		
		List<Double> CellsArrivalTimeList=cellsandtimelist[1]; // second array is the trajectory cells arrival time list
		double exitTime= CellsArrivalTimeList.get(CellsArrivalTimeList.size()-1); // last element in the list is the exit time
		sender.setExitTime(exitTime);//setting the exit time
		CellsArrivalTimeList.remove(CellsArrivalTimeList.size()-1);// removing the exit time from the cells arrival list
		
		//as if cellarrivaltimelist has only one element it should remain until vehicle exits the intersection 			
		if(CellsArrivalTimeList.size()==0 && cellslist.size()==1)
		 CellsArrivalTimeList=sender.getTrajectoryCellsTimeList();	
		
		sender.setTrajectoryCellsTimeList(CellsArrivalTimeList);
		
		// setting the arrival time of the vehicle
		double distancetonextint= sender.getDriver().getCurrentLane().getLaneIM().distanceToNextIntersection(sender.getPosition()) - thresholdDistance ;
		// We recalculate arrival time until a threshold distance from the intersection when we leave the arrival time as the previous calculated value 
		
		// we calculate arrival time once and keep it for the whole time as it will be used for assigning priorities
		if (messagetype==1) {//&& (distancetonextint > thresholdDistance) ){ 
			double ArrivalTime;
			   Lane departurelane = sender.getDriver().getCurrentLane();
			    // the speed the vehicle tries to preserve
				double initVelocity = Math.min(sender.getSpec().getMaxVelocity(),
						departurelane.getSpeedLimit()); 

			ArrivalTime= distancetonextint/initVelocity + currentTime; // constant so that even if vehicle stops it has priority
			
		if(sender.getAcceleration() == 0.0){ // time= distance/velocity + currentTime
			if(sender.getVelocity() != 0.0){ 
			ArrivalTime= distancetonextint/sender.getVelocity() + currentTime; 
			if(ArrivalTime == 0.0){
				System.err.println("ArrivalTime == 0.0");
			}
			}else{ // if the vehicle is in a complete stop before entering the intersection
				ArrivalTime=Double.POSITIVE_INFINITY;
				if(ArrivalTime == 0.0){
					System.err.println("ArrivalTime == 0.0");
				}
			} // added on 8th-oct-2014 
//			}else if(sender.getAcceleration() < 0.0){ // -ve acceleration 
//				ArrivalTime=Double.POSITIVE_INFINITY;	
			}else {  // +ve acceleration
				// we assume uniform acceleration until reaching the intersection
							
			double Vcurrent= sender.getVelocity();//current velocity
			double d= distancetonextint;//distance to intersection
			double a= sender.getAcceleration();//acceleration
			double ArrivalTimeNotRounded = (-Vcurrent + Math.sqrt(Math.pow(Vcurrent,2) + 2*a*d))/(a) + currentTime; 
		
			ArrivalTime = Math.round(ArrivalTimeNotRounded * 50)/ 50.0 ; //round to the nearest Time_Step
		}
	
//		/// added on 8th-oct-2014
//		if(sender.getTrajectoryCellsTimeList().size() !=0 )
//		ArrivalTime	= sender.getTrajectoryCellsTimeList().get(0); // we changed arrive time to be the time at which the vehicle enters the first cell
//		////
	
		if(ArrivalTime == 0.0)
			ArrivalTime=Double.POSITIVE_INFINITY;
		
		sender.setArrivalTime(ArrivalTime);
		
		///////////////////////////////////////////
		if(sender.getcelltostop()==-5)
			sender.setArrivalTime(Double.POSITIVE_INFINITY);
		
		}
	//	if(sender.getcelltostop()==-5)
	//		sender.setArrivalTime(Double.POSITIVE_INFINITY);
		
		arriveTime = sender.getArrivalTime(); 
		
		//incrementing message sequence number
		sender.setmsgseqno(sender.getmsgsqnno()+1);
		
		//Check if cells arrival time list is in ascending order
				List<Double> CellsArrivalTimeListtest=new ArrayList<Double>();
				for(int k=0;k<CellsArrivalTimeList.size();k++)
				CellsArrivalTimeListtest.add(CellsArrivalTimeList.get(k));
				Collections.sort(CellsArrivalTimeListtest);
				//CellsArrivalTimeListtest.sort(null);
					//System.err.println("CATL sorted "+ CellsArrivalTimeListtest);
					//System.err.println("CATL not sorted " + CellsArrivalTimeList);	
	
		//-		if(!(CellsArrivalTimeListtest.equals(CellsArrivalTimeList)))
		//-		System.err.println("#vehicle " + sender.getVIN() + " has CellsArrivalTimeList in a non ascending order!" + CellsArrivalTimeList );
		
				//Check if all values in cells arrival time list are larger than the arrival time or not 
//				for(int r=0;r<CellsArrivalTimeList.size();r++){
//					if(CellsArrivalTimeList.get(r) < arriveTime)
//			System.err.println("#vehicle " + sender.getVIN() + " one of the values of CellsArrivalTimeList has lower value than arrival time " + CellsArrivalTimeList );
//				}
		
			if (messagetype == 1)
			msgtype = "ENTER";
			else if (messagetype == 2)
			msgtype = "CROSS";
		
			//Oct-14
			//in case we decided to stop
			List <Integer> finalcellslist = new ArrayList<Integer>();
			List <Double> finalCellsArrivalTimeList = new ArrayList<Double>();
			//1.before intersecton
//			if(sender.getcelltostop()!=-1){
//			if(sender.getcelltostop()==-5){
//			finalcellslist.clear();
//			finalCellsArrivalTimeList.clear();
//			arriveTime=Double.POSITIVE_INFINITY;
//			exitTime=Double.POSITIVE_INFINITY;
//			}else {//2.after intersection
//				finalcellslist=cellslist;
//				finalCellsArrivalTimeList=CellsArrivalTimeList;	
//				//Oct 14 //
//				if(cellslist.size()!=0){
//					for(int i=0;i<cellslist.size();i++) {
//					if(cellslist.get(i)==sender.getcelltostop()){
//						for(int j=i+1;j<finalCellsArrivalTimeList.size();j++)
//						finalCellsArrivalTimeList.set(j, -2.0);
//					}
//					}
//				}
//		///////	
//				
//			}	
//			}else{ // no stoping
				finalcellslist=cellslist;
				finalCellsArrivalTimeList=CellsArrivalTimeList;	
	//		}
		//creating a new message with the calculated fields	
		v2vmessage = new V2VMessage(sender.getVIN(),deptroad, deptlane, arrivroad, arrivlane, arriveTime, exitTime, finalcellslist, finalCellsArrivalTimeList,sender.getmsgsqnno(),msgtype);
		
	System.out.println("#vehicle " + sender.getVIN() + " created a message with content: "+ sender.getVIN() +" "+
   deptroad + " " + deptlane + " " + arrivroad + " " + arrivlane + " "+ arriveTime +" " + exitTime +" " + 
   cellslist+" "+CellsArrivalTimeList+" "+sender.getmsgsqnno()+" "+messagetype+" at time "+currentTime);
		
   	 //For testing, to check if the vehicle will intersect with any cell apart from the ones in the trajectory list
   	 if(messagetype ==1 && sender.getmsgsqnno()==1)
   	 {
   		 sender.setfirsttrajectorylist(cellslist);
   	 }
   	 
		
		}else // messageType =3 (Exit)
		{
			// In Exit we have only the following fields in a message: VIN, message sequence number and message type (Exit)
			v2vmessage = new V2VMessage(sender.getVIN(), null, -1, null, -1, -1, -1, null, null,sender.getmsgsqnno(),"EXIT");
			sender.setmsgseqno(sender.getmsgsqnno()+1);
			System.out.println("#vehicle " + sender.getVIN() + " created a message with content: " + sender.getmsgsqnno()+ " EXIT"); 
		}

		return v2vmessage;

	}	
	
	/**
	 * Deciding on which message type to send depending on the vehicle position relative to the intersection
	 * 
	 * @param sender
	 * 			Vehicle creating a message
	 * @return 
	 * 			type of message to send (Enter, Cross or Exit)
	 */
	private int typeofmessagetosend(AutoVehicleSimView sender){
			
			double distancetoint; 			
			
			distancetoint=sender.getDriver().getCurrentLane().getLaneIM().distanceToNextIntersection(sender.getPosition()); //gauge position 			
				
			Area area = sender.getDriver().nextIntersectionManager().getIntersection().getArea();			
			
			//Enter State (Intersection-Approach state)	
			if ((distancetoint <= Denter) && (sender.getState() == 0))    
				sender.incrementState();// Enter message type
				
				// Crossing state (Intersection-Enter state)
				else if((distancetoint == 0.0) && (sender.getState() == 1)) 
				{
					sender.incrementState();// Cross message type
					sender.setmsgseqno(0); // resetting message sequence number as its a new message type 
					System.out.println("#vehicle "+sender.getVIN()+" entered the intersection at current time "+ currentTime);
				}
					
			       //Intersection-Exit state
				else if((VehicleUtil.intersects(sender, area)==false) && (sender.getState() == 2))
				{
					sender.incrementState();// Exit message type
					sender.setmsgseqno(0);// resetting message sequence number as its a new message type 
					System.out.println("#vehicle "+sender.getVIN()+" exited the intersection at current time "+ currentTime);
				}
				
			return sender.getState();	
			
		}

	
/**
 * 	sending the message to all vehicles by adding it to its own Outbox
 * 
 * @param sender
 * 			Vehicle sending the message
 * @param message
 * 			Created message
 */
private	void sendmessage(AutoVehicleSimView sender,V2VMessage message){
		sender.setV2VOutbox(message);		
	}
	
/**
 * 	Each vehicle receives the message by adding the sent message to its own Inbox
 * 
 * @param receiver
 * 			Vehicle receiving the sent message
 * @param sender
 * 			Vehicle that sent the message
 */
private void receivemessage(AutoVehicleSimView receiver,AutoVehicleSimView sender){
	
receiver.setV2VInbox(sender.getV2VOutbox());	
};



/**
 *  Processing the received message by calling the AMP policy
 *  
 * @param receiver
 * 			 Vehicle that has received the message
 */
private void processmessage(AutoVehicleSimView receiver)
{
	V2VMessage message = receiver.getV2VInbox();
	// to include Trajectory Intersecting collision cell (TIC) and VIN of conflicting vehicle. Note: we won't use the VIN in the protocol but we include it for testing purposes
//	int TIC; 
//	double[] TIC_VIN = {0.0,0.0};
	
	if (message == null || receiver.getState()>2) // vehicle received no message or vehicle is in exit state
		return;
	
	policy AMP=new policy();
	AMP.runpolicy(receiver,message);
	
//	if(TIC ==-1)
//		return;
//
//	TIC_VIN[0]=TIC;
//	TIC_VIN[1]=message.VIN();
//	
//	receiver.emptyTIC_VIN_List_1();
//	receiver.addElementtoTIC_VIN_List_1(TIC_VIN); // for keeping track of vehicle we are stopping for so we would know when the vehicle exits the TIC 
//	
//	takedecision(receiver, TIC,message.VIN()); // vehicle has to stop before TIC 
	
}

/**
 * decide from all the processed messages which cell to stop at and for how long
 */
private void takefinaldecision(){
	
	for (VehicleSimView theVehicle : vinToVehicles.values()) {
		if (theVehicle instanceof AutoVehicleSimView) {
		    AutoVehicleSimView vehicle = (AutoVehicleSimView) theVehicle;	

boolean nodecision = true;		    
int currentcell=-1;		    

if (!((vehicle.getTrajectoryCellsList().size() == 0) || (vehicle.getTIC_VIN_List_1().size() == 0)))
{
OUTERLOOP:
for(int i=0; i<vehicle.getTrajectoryCellsList().size();i++){
	for(int j=0;j<vehicle.getTIC_VIN_List_1().size();j++){
		if (vehicle.getTrajectoryCellsList().get(i) == (int)vehicle.getTIC_VIN_List_1().get(j)[0]){
		if(i>0){
			for (IntersectionDetails im : basicMap.getIntersectionManagers()) {
				
				TiledArea tiledarea=new TiledArea(im.getIntersection().getArea(),area.getBounds2D().getWidth()/noofcellsperside); 				
				for(int l=0;l<tiledarea.getXNum()-1;l++){
					for(int m=0;m<tiledarea.getYNum()-1;m++){
				int tileid=tiledarea.getTile(l,m).getId();
				Tile tile = tiledarea.getTileById(tileid);
    if(tile.getRectangle().contains(vehicle.getPosition()))				
    currentcell=tileid;		
	}
	}
	}				
	//-		if((i==1) && (vehicle.getState()>1) && (vehicle.getcurrentcell()== vehicle.getTrajectoryCellsList().get(1))){ //in cross or exit , receiver has a conflicting cell which he is currently occupying
	//-			System.err.println("#vehicle "+vehicle.getVIN() + " has currentcell (second cell in TCL) " +vehicle.getTrajectoryCellsList().get(1) +" as conflicting cell with vehicle "+ vehicle.getTIC_VIN_List_1().get(j)[1]+ " at current time "+ currentTime+ " this is an error!");
	//-		}
			
		vehicle.setcelltostop(vehicle.getTrajectoryCellsList().get(i-1)); // this will be the only collision cell to consider as its the earliest conflicting cell 		
		nodecision = false;
		//	vehicle.settimetogotonextcell(vehicle.getTIC_VIN_List_1().get(j)[2]);// arrival time of conflicting vehicle to the cell after the conflicting cell, at this time our vehicle can start moving as next cell is presumably empty now 
		}		
		else if (i==0){ //if conflicting cell is the first cell (element) in the vehicle's Trajectory cells list
			
			if(!(VehicleUtil.intersects(vehicle, area))){ // before entering the intersection
		vehicle.setcelltostop(-5); // special number to mark stopping before entering intersection // TODO: change	
		nodecision = false;
		//	vehicle.settimetogotonextcell(vehicle.getTIC_VIN_List_1().get(j)[2]);// arrival time of conflicting vehicle to the cell after the conflicting cell, at this time our vehicle can start moving as next cell is presumably empty now 
			}else{ // vehicle's current cell is a conflicting cell, if the protocol allows this then the protocol design is flawed  
	//-			System.err.println("#vehicle " + vehicle.getVIN() + " has current cell (first cell in TCL) " + vehicle.getTrajectoryCellsList().get(0) +" as conflicting cell with vehicle "+vehicle.getTIC_VIN_List_1().get(j)[1] +" at current time "+ currentTime+" this is an error!" );
				vehicle.setcelltostop(-1);
				nodecision = false;
//				vehicle.settimetogotonextcell(-1);				
			}
		}else // i<0
			System.err.println("index of Trajectory list cannot be -ve!");
		
		System.out.println("#vehicle " + vehicle.getVIN()+ " takes decision to stop because of Vehicle "+ vehicle.getTIC_VIN_List_1().get(j)[1]+ " at cell "+ vehicle.getcelltostop() + " at current time " + currentTime);
		break OUTERLOOP;
		}
		}
		}
	
if(nodecision == true)
	vehicle.setcelltostop(-1);

}else{ 
	vehicle.setcelltostop(-1);
//	vehicle.settimetogotonextcell(-1);
}
System.out.println("#vehicle " + vehicle.getVIN()+ " cell to stop is "+ vehicle.getcelltostop() + " at current time " + currentTime);

//System.out.println("#vehicle " + vehicle.getVIN()+ " has tic_vin_list of : "+ vehicle.getTIC_VIN_List_1() + " at current time " + currentTime);
		}
	}
}
	
	// ///////////////////////////////
	// STEP 4
	// ///////////////////////////////

	/**
	 * Control each vehicle's movement by setting its movement parameters (velocity,acceleration and heading)    
	 */
	
	private void controller(){
	
	for (VehicleSimView sendvehicle : vinToVehicles.values()) {
		if (sendvehicle instanceof AutoVehicleSimView) {
		    AutoVehicleSimView sender = (AutoVehicleSimView) sendvehicle;			

		    Lane deptlane = sender.getDriver().getCurrentLane();
		    // the speed the vehicle tries to preserve
			double initVelocity = Math.min(sender.getSpec().getMaxVelocity(),
					deptlane.getSpeedLimit()); 
		
			String arrivroad= sender.getDriver().getDestination().getName();
			double arrivheading=getHeadingFromArrivalRoadname(arrivroad);
			DecimalFormat df = new DecimalFormat("####0.00");
			
			if(Math.abs(Double.parseDouble(df.format(arrivheading - sender.getHeading())))<0.1)
				sender.setHeading(arrivheading);
				
				sender.setSteeringAngle(0); // initialize it with zero before turning
					
		if(sender.getturning())   // Turing is set by provideVehiclePositionWithinCell function earlier 
		    changeHeading(sender);
		
		// Based on vehicle's TIC apply slow to stop and change the trajectorycellstimelist 
		if(sender.getcelltostop() != -1){ // TODO: set it to null after using it
		
			// stop before entering intersection
			if(sender.getcelltostop() == -5){
				
			stopbeforeintersection(sender);
			
			}else {
								 	
		stopatcell(sender);
			}
		}else { // then we don't need to stop and we try to reach and keep a steady speed of initVelocity
			double stoppingDistance =
				      VehicleUtil.calcDistanceToStop(sender.gaugeVelocity(),
				                                     sender.getSpec().getMaxDeceleration());
				    double followingDistance = stoppingDistance + MINIMUM_FOLLOWING_DISTANCE;
				    if (!((VehicleUtil.distanceToCarInFront(sender) < followingDistance) && !(VehicleUtil.intersects(sender, area)))){				    	
							if(sender.getVelocity() <= initVelocity) 
				//			{
				//				if(sender.getturning()==false)
								sender.setAccelWithMaxTargetVelocity(sender.getSpec().getMaxAcceleration()); 
								 else
							    sender.setAccelWithMaxTargetVelocity(0); // don't accelerate while turning 							}
				//			}
				//				else
				//				sender.setAccelWithMaxTargetVelocity(0);
				    }
		}
		
		// stop for vehicles infront of you outside of the intersection, but inside the intersection the protocol should take care of that 
	//if(!(VehicleUtil.intersects(sender, area))) 
		dontHitVehicleInFront(sender);
		
		System.out.println("#vehicle "+ sender.getVIN()+ " has velocity : "+ sender.getVelocity()+ " and acceleration : "+ sender.getAcceleration() +" at time "+ currentTime);
				
		}
		}
	}
	
	/**
	 * Change heading of the vehicle by changing the steering angle depending on the arrival road the vehicle has chosen
	 * 
	 * @param vehicle
	 * 			Vehicle changing its heading
	 */
	private void changeHeading(AutoVehicleSimView vehicle){
	
	int n=3; // Vehicle will change its steering angle n times , each time the change is equal to the total angle required to change heading over n
	String deptroad= Debug.currentMap.getRoad(vehicle.getDriver().getCurrentLane()).getName();
	String arrivroad= vehicle.getDriver().getDestination().getName();
	double deptheading=getHeadingFromArrivalRoadname(deptroad); // from departure road we calculate the departure heading
	double arrivheading=getHeadingFromArrivalRoadname(arrivroad); // from arrival road we calculate the arrival heading
	
//	////
//	double higharrivheading=arrivheading+0.15;
//	double lowarrivheading=arrivheading-0.15;
//	////
	DecimalFormat df = new DecimalFormat("####0.00");
	
		//keep changing steering angle until the vehicle's heading is equal to the vehicle's arrival heading
    if(Double.parseDouble(df.format(vehicle.getHeading())) != Double.parseDouble(df.format(arrivheading))){
	//if( (Double.parseDouble(df.format(vehicle.getHeading())) > Double.parseDouble(df.format(higharrivheading))) || (Double.parseDouble(df.format(vehicle.getHeading())) < Double.parseDouble(df.format(lowarrivheading))) ){	
	//going from East direction to North direction
		if(Double.parseDouble(df.format(arrivheading - deptheading)) == 4.71)
		vehicle.setSteeringAngle(- 1.5708/n);
		//going from North direction to East direction
		else if(Double.parseDouble(df.format(arrivheading - deptheading)) == -4.71)
			vehicle.setSteeringAngle(1.5708/n);
		//going from a lower direction (having lower angle) to a higher direction (having higher angle)
		// for example going from East direction (0.00 heading) to South direction (1.57 heading) 
		else if(arrivheading > deptheading)
			vehicle.setSteeringAngle(1.5708/n);
		//going from a higher direction (having higher angle) to a lower direction (having lower angle)
		// for example going from South direction to East direction 
		else if( deptheading > arrivheading)
			vehicle.setSteeringAngle(-1.5708/n);
		}
			if (Double.parseDouble(df.format(vehicle.getHeading())) >= Double.parseDouble(df.format(6.283))) // one full 360 degrees, reset to 0
			vehicle.setHeading(vehicle.getHeading() - 6.283);
	// After reaching the arrival heading reset the steering angle to zero
	if(Double.parseDouble(df.format(vehicle.getHeading())) == Double.parseDouble(df.format(arrivheading))){
	//if( (Double.parseDouble(df.format(vehicle.getHeading())) < Double.parseDouble(df.format(higharrivheading))) && (Double.parseDouble(df.format(vehicle.getHeading())) > Double.parseDouble(df.format(lowarrivheading))) ){	
			vehicle.setSteeringAngle(0);
		//	vehicle.clearturning();
	}
	
	}
	
	
	/**
	 *  Get arrival lane depending on departure road, departure lane and arrival road 
	 *  
	 * @param deptlane
	 * 			Lane which the vehicle uses until it reaches the intersection
	 * @param deptroad
	 * 			Road which the vehicle uses until it reaches the intersection
	 * @param arrivroad
	 * 			Road which the vehicle uses after crossing the intersection 			
	 * @return Arrivallane
	 * 			Lane which the vehicle uses after crossing the intersection 			 			
	 */
	private int getArrivLane(int deptlane,String deptroad,String arrivroad){	
		
		int a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p;
		
		if(nooflanes==3){
		a=0;b=1;c=2;d=3;e=4;f=5;g=6;h=7;i=8;j=9;k=10;l=11;	
		}
		else //nooflanes=2
		{
		a=0;b=0;c=1;d=2;e=2;f=3;g=4;h=4;i=5;j=6;k=6;l=7;			
		}
		
		if (deptroad.equals(arrivroad)) // same road
			return deptlane; // arrival lane = departure lane
		
		else if(arrivroad.equals("1st Avenue S"))
		{
			if(deptroad.equals("1st Street E"))
			{ if(deptlane==i)
				return f;
			else
				return d;	
			}else if(deptroad.equals("1st Street W"))
				return e;
		}		
		else if(arrivroad.equals("1st Avenue N"))
		{
			if(deptroad.equals("1st Street E"))
				return b;
			else if(deptroad.equals("1st Street W"))
			{ if(deptlane==l)
				return c;
			else
				return a;
			}
			}
		else if(arrivroad.equals("1st Street E"))
		{
			if(deptroad.equals("1st Avenue N"))
			{ if(deptlane==c)
				return j;
			else
				return h;
			}
				else if(deptroad.equals("1st Avenue S"))
				return i;
		}		
		else if(arrivroad.equals("1st Street W"))
		{
			if(deptroad.equals("1st Avenue N"))
				return k;
			else if(deptroad.equals("1st Avenue S"))
			{ if(deptlane==f)
				return l;
			else
				return j;
			}
			}else System.err.print("No valid arrival road \n");		
		
		return -1; // outside range 
		
	}

	/**
	 * Get the heading from the arrival road. 
	 * 
	 * Road to Heading conversion:
	 * East  -> 0.00
	 * South -> 1.57
	 * West  -> 3.14
	 * North -> 4.71
	 * 
	 * @param arrivalroad
	 * 			The vehicle's arrival road after crossing the intersection			
	 * @return Heading
	 * 			 Angle in radians starting from the east direction and going downwards in a clockwise direction  
	 */
	private double getHeadingFromArrivalRoadname(String arrivalroad){
	
	if (arrivalroad.equalsIgnoreCase("1st Avenue S"))
		return 1.57;
	else if (arrivalroad.equalsIgnoreCase("1st Avenue N"))
		return 4.71;
	else if (arrivalroad.equalsIgnoreCase("1st Street W"))
		return 3.14;
	else if (arrivalroad.equalsIgnoreCase("1st Street E"))
		return 0.00;
	else System.err.printf("arrival road doesn't exit \n");
	return -1; // error
		
	}
	
	  /**
	   * Stop before hitting the car in front.
	   *
	   * @param vehicle  
	   * 			The vehicle
	   */
	  private void dontHitVehicleInFront(AutoVehicleSimView  vehicle) {
		  
		  double stoppingDistance =
		      VehicleUtil.calcDistanceToStop(vehicle.gaugeVelocity(),
		                                     vehicle.getSpec().getMaxDeceleration());
		    double followingDistance = stoppingDistance + MINIMUM_FOLLOWING_DISTANCE;
		    
		    if(!(VehicleUtil.intersects(vehicle, area))){ // on the road 
		    if (VehicleUtil.distanceToCarInFront(vehicle) < followingDistance) 
		      vehicle.slowToStop(); 
		    }else // on the intersection
		      {
		    	if (calculateDistancetocarInFrontOnIntersection(vehicle) < followingDistance) 
		  	      vehicle.slowToStop();    
		      }
		    }
	  /**
	   *  Change the vehicle's acceleration so that it stops at the right cell 
	   * @param vehicle
	   * 			The Vehicle stopping
	   */
	  private void stopatcell(AutoVehicleSimView  vehicle) {
		  
	  double distancetocell;
	  int deptlane= vehicle.getDriver().getCurrentLane().getId();
	  String deptroad= Debug.currentMap.getRoad(vehicle.getDriver().getCurrentLane()).getName();
	  String arrivroad= vehicle.getDriver().getDestination().getName();
	  Tile turningcell=null;
	  double distancebetweenturningcellandstopcell=0;
	  double distancetoturningcell=1000; // high number;
	  Lane departurelane = vehicle.getDriver().getCurrentLane();
	  
	  double initVelocity = Math.min(vehicle.getSpec().getMaxVelocity(),departurelane.getSpeedLimit());
	  
	  double stoppingDistance =
	  VehicleUtil.calcDistanceToStop(vehicle.gaugeVelocity(),
		                                     vehicle.getSpec().getMaxDeceleration());
	  double followingDistance = stoppingDistance + thresholddistancetostop;
	  // if the vehicle will stop at a cell before turning then calculating distance to cell is straight forward
	  
	  TiledArea tiledarea=new TiledArea(area,area.getBounds2D().getWidth()/noofcellsperside); 
	
	  Tile stopcell =tiledarea.getTileById(vehicle.getcelltostop());
	  int turncellId =new  CellsTrajectoryLists().getturningcell(deptroad, arrivroad, deptlane,noofcellsperside);
	  if(!(turncellId == -1))
	  turningcell = tiledarea.getTileById(turncellId);
	  
	  double distancetostopcell= Math.abs(Point2D.distance(vehicle.getCenterPoint().getX(), vehicle.getCenterPoint().getY(), stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY()));	  
	  if(!(turncellId == -1)){
	  distancebetweenturningcellandstopcell= Math.abs(Point2D.distance(turningcell.getRectangle().getCenterX(), turningcell.getRectangle().getCenterY(), stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY()));
	  distancetoturningcell = Math.abs(Point2D.distance(vehicle.getCenterPoint().getX(), vehicle.getCenterPoint().getY(), turningcell.getRectangle().getCenterX(), turningcell.getRectangle().getCenterY()));
	  }
	  
	  if((distancetostopcell <= distancetoturningcell) || (turncellId==-1)){ // stop cell comes before turning cell or they are both the same cell or vehicle doesn't turn
		  
		  Point2D centerpointofstopcell = new Point2D.Double(stopcell.getRectangle().getCenterX(),stopcell.getRectangle().getCenterY());
			
			double angle = GeomMath.angleToPoint(centerpointofstopcell,vehicle.gaugePointBetweenFrontWheels());
			// Need to recenter this value to [-pi, pi]
			double Anglebetweenvehicleandcenterofcell = Util.recenter(angle - vehicle.getHeading(),
					-1.0 * Math.PI, Math.PI);
		//change-Oct-07-2014
			//	double distancetocenterofstopcell = Math.abs(Point2D.distance(stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY(), vehicle.gaugePointBetweenFrontWheels().getX(), vehicle.gaugePointBetweenFrontWheels().getY()));
		    double distancetocenterofstopcell = Math.abs(Point2D.distance(stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY(), vehicle.getPointAtRear().getX(), vehicle.getPointAtRear().getY()));		
			double verticaldistancetocenterofstopcell = distancetocenterofstopcell * Math.cos(Anglebetweenvehicleandcenterofcell) ;//with respect to vehicle heading
		  
		  distancetocell = verticaldistancetocenterofstopcell;
	  }
	  // but if vehicle will stop at a cell after turning then calculation is more complicated
	  else if (distancetostopcell>distancetoturningcell){
		  
		  Point2D centerpointofturningcell = new Point2D.Double(turningcell.getRectangle().getCenterX(),turningcell.getRectangle().getCenterY());
			
			double angle = GeomMath.angleToPoint(centerpointofturningcell,vehicle.gaugePointBetweenFrontWheels());
			// Need to recenter this value to [-pi, pi]
			double Anglebetweenvehicleandcenterofcell = Util.recenter(angle - vehicle.getHeading(),
					-1.0 * Math.PI, Math.PI);
			//change-Oct-07-2014
			//	double distancetocenterofturningcell = Math.abs(Point2D.distance(stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY(), vehicle.gaugePointBetweenFrontWheels().getX(), vehicle.gaugePointBetweenFrontWheels().getY()));
		    double distancetocenterofturningcell = Math.abs(Point2D.distance(stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY(), vehicle.getPointAtRear().getX(), vehicle.getPointAtRear().getY()));		
			double verticaldistancetocenterofturningcell = distancetocenterofturningcell * Math.cos(Anglebetweenvehicleandcenterofcell) ;//with respect to vehicle heading
		  		  
		  distancetocell = verticaldistancetocenterofturningcell + distancebetweenturningcellandstopcell;
	  }
		  else // we shouldn't come here
		  {
			  System.err.println("ERROR: In calculation of distances to stop in function stopatcell()!");
			  distancetocell=-1;
		  }

	  System.out.println("#vehicle "+ vehicle.getVIN() + " distance to cell is : " + distancetocell + "and distance to decelerate to zero is: " + stoppingDistance);

	  // vehicle starts to decelerate only after reaching a distance to cell that will allow it to decelerate with a maximum value along the way until stopping before cell 
	  if (distancetocell<= followingDistance) { 
	  vehicle.slowToStop();
	  System.out.println("#vehicle "+ vehicle.getVIN() + " is slowing down, its speed is "+ vehicle.getVelocity()+ " and acceleration is: "+ vehicle.getAcceleration() + "distance to cell is : " + distancetocell + "and distance to decelerate to zero is: " + stoppingDistance);
	   }else if(vehicle.getVelocity() <= initVelocity) 
			//			{
			//				if(vehicle.getturning()==false)
							vehicle.setAccelWithMaxTargetVelocity(vehicle.getSpec().getMaxAcceleration()); 
							 else
						    vehicle.setAccelWithMaxTargetVelocity(0); // don't accelerate while turning 							}
			//			}
			//				else
			//				sender.setAccelWithMaxTargetVelocity(0);
	 }  
	
	  /**
	   *  Change the vehicle's acceleration so that it stops before entering the intersection
	   *  
	   * @param vehicle
	   * 			The vehicle stopping
	   */
	  private void stopbeforeintersection(AutoVehicleSimView  vehicle) {
		
		  Lane departurelane = vehicle.getDriver().getCurrentLane();
		  
		  double initVelocity = Math.min(vehicle.getSpec().getMaxVelocity(),departurelane.getSpeedLimit());
		  
		  double stoppingDistance =
		  VehicleUtil.calcDistanceToStop(vehicle.gaugeVelocity(),
			                                     vehicle.getSpec().getMaxDeceleration());
		  double followingDistance = stoppingDistance +  stopbeforeintersectionthreshold;
		  // if the vehicle will stop at a cell before turning then calculating distance to cell is straight forward
		  
		  double distancetointersection= vehicle.getDriver().getCurrentLane().getLaneIM().distanceToNextIntersection(vehicle.getPosition());

		  // vehicle starts to decelerate only after reaching a distance to intersection that will allow it to decelerate with a maximum value along the way until stopping before intersection
		  if (distancetointersection <= followingDistance) {
		  vehicle.slowToStop();
		   }else if(vehicle.getVelocity() <= initVelocity) 
				//			{
				//				if(vehicle.getturning()==false)
								vehicle.setAccelWithMaxTargetVelocity(vehicle.getSpec().getMaxAcceleration()); 
								 else
							    vehicle.setAccelWithMaxTargetVelocity(0); // don't accelerate while turning 							}
				//			}
				//				else
				//				sender.setAccelWithMaxTargetVelocity(0);
		 }  
	
	  
	  private double calculateDistancetocarInFrontOnIntersection(VehicleSimView vehicle){
			Point2D pos = vehicle.getPosition(); 
			
			if(vehicle.getNextVehicle()==null)
				return Double.MAX_VALUE;
			
			if (vehicle.getNextVehicle().getShape().contains(pos)) {
				return 0.0;
			} else {
				// TODO: make it more efficient
				double interval = Double.MAX_VALUE;
				for (Line2D edge :vehicle.getNextVehicle().getEdges()) {
					double dst = edge.ptSegDist(pos);
					if (dst < interval) {
						interval = dst;
					}
				}
				return interval;
			}
		}	

	  
	// ///////////////////////////////
	// STEP 5
	// ///////////////////////////////

	/**
	 * Move all the vehicles.
	 * 
	 * @param timeStep
	 *            the time step
	 */
	private void moveVehicles(double timeStep) {
		for (VehicleSimView vehicle : vinToVehicles.values()) {
			Point2D p1 = vehicle.getPosition();
			vehicle.move(timeStep);
			Point2D p2 = vehicle.getPosition();
			
			vehicle.addDistanceMoved(p1.distance(p2));			
			for (DataCollectionLine line : basicMap.getDataCollectionLines()) {
				line.intersect(vehicle, currentTime, p1, p2);
			}
			if (Debug.isPrintVehicleStateOfVIN(vehicle.getVIN())) {
				vehicle.printState();
			}
		}
	}

	/**
	 * Testing for collisions in the intersection
	 */
	private void testing(){	
	for (VehicleSimView vehicle1 : vinToVehicles.values()){	
		//System.err.println("Intersection size is: " + area.getBounds2D().getWidth() + " " + area.getBounds2D().getHeight());
		for (VehicleSimView vehicle2 : vinToVehicles.values()){
			if(vehicle1.getVIN()==vehicle2.getVIN())
				continue; // choose the next vehicle
			else{
		//we are only interested in collisions inside the intersection 
//		if((area.intersects(vehicle1.getShape().getBounds2D())) && (area.intersects(vehicle2.getShape().getBounds2D()))){	
		if(vehicle1.getShape().getBounds2D().intersects(vehicle2.getShape().getBounds2D())){

			// to report collisions only once (one of the vehicles will report it)
			if(vehicle1.getcollisionlist().size()>0)
				if(vehicle1.getcollisionlist().contains(vehicle2.getVIN()))
					continue;
			
			if(vehicle2.getcollisionlist().size()>0)
				if(vehicle2.getcollisionlist().contains(vehicle1.getVIN()))
					continue;
			
			System.err.println("#vehicle "+ vehicle1.getVIN() +" collided with vehicle "+ vehicle2.getVIN() + " at current time " +currentTime); // this way we report collisions twice(to log each vehicle's collision) so take this into consideration when calculating number of collisions
			System.err.println(vehicle1.getVIN() + " acc: " + vehicle1.getAcceleration() +" "+vehicle1.getVIN() + " vel: " + vehicle1.getVelocity() + " " +vehicle2.getVIN() + " acc: " + vehicle2.getAcceleration() +vehicle2.getVIN()+ " vel: "+ vehicle2.getVelocity());
	//		try {
	//			System.in.read();
	//		} catch (IOException e) {
				// TODO Auto-generated catch block
	//			e.printStackTrace();
	//		}
			System.out.println("#vehicle "+ vehicle1.getVIN() +" collided with vehicle "+ vehicle2.getVIN() + " at current time " +currentTime); // this way we report collisions twice(to log each vehicle's collision) so take this into consideration when calculating number of collisions
			if( vehicle1.getState()> 2 && vehicle2.getState()> 2)
			System.err.println("both vehicles are in exit state");
			else if(vehicle1.getState()> 2 || vehicle2.getState()> 2)
			System.err.println("One of the vehicles is in exit state");	
			else{
				int n=0;	
				for (int i=0;i<vehicle1.getfirsttrajectorylist().size();i++){
					for (int j=0;j<vehicle2.getfirsttrajectorylist().size();j++)
					if (vehicle1.getfirsttrajectorylist().get(i) == vehicle2.getfirsttrajectorylist().get(j))
						n++;		
				}
			if(n>1)
				System.err.println("vehicles intersect in more than one cell");		
			} 
			
//			
//		boolean collisionduetoprotocol = false;
//		// check for several issues to make sure it is a protocol problem that led to the collision:
//		//1.both vehicles have the collision cell as the first element in their cell list
//		
//		if (vehicle1.getTrajectoryCellsList().size() > 0 && vehicle2.getTrajectoryCellsList().size() > 0 && vehicle1.getcurrentcell()==vehicle1.getTrajectoryCellsList().get(0) && vehicle2.getcurrentcell()==vehicle2.getTrajectoryCellsList().get(0)){
//		//2.lower priority vehicle arrived earlier
//			if(vehicle1.getArrivalTime()> vehicle2.getArrivalTime()){ //vehicle 1 is the lower priority
//				if(vehicle1.getTrajectoryCellsTimeList().get(0)< vehicle2.getTrajectoryCellsTimeList().get(0)){
//					if(vehicle1.getTrajectoryCellsList().size()>1){
//						if(vehicle1.getTrajectoryCellsTimeList().get(1)> vehicle2.getTrajectoryCellsTimeList().get(0))
//							collisionduetoprotocol =true;
//					}else{
//						if(vehicle1.getExitTime()> vehicle2.getTrajectoryCellsTimeList().get(0))
//							collisionduetoprotocol =true;
//					}
//							
//				}
//			}else if(vehicle1.getArrivalTime()< vehicle2.getArrivalTime()){
//				if(vehicle1.getTrajectoryCellsTimeList().get(0)> vehicle2.getTrajectoryCellsTimeList().get(0)){
//		
//					if(vehicle2.getTrajectoryCellsList().size()>1){
//						if(vehicle2.getTrajectoryCellsTimeList().get(1)> vehicle1.getTrajectoryCellsTimeList().get(0))
//							collisionduetoprotocol =true;
//					}else{
//						if(vehicle2.getExitTime()> vehicle1.getTrajectoryCellsTimeList().get(0))
//							collisionduetoprotocol =true;
//					}
//							
//				}
//		
//			}else // both have same arrival time , so we check higher priority due to lower Vehicle ID
//			{
//				if(vehicle1.getVIN()> vehicle2.getVIN()){ 
//					if(vehicle1.getTrajectoryCellsTimeList().get(0)< vehicle2.getTrajectoryCellsTimeList().get(0)){
//				
//						if(vehicle1.getTrajectoryCellsList().size()>1){
//							if(vehicle1.getTrajectoryCellsTimeList().get(1)> vehicle2.getTrajectoryCellsTimeList().get(0))
//								collisionduetoprotocol =true;
//						}else{
//							if(vehicle1.getExitTime()> vehicle2.getTrajectoryCellsTimeList().get(0))
//								collisionduetoprotocol =true;
//						}
//							
//					}
//				}else if(vehicle1.getVIN()< vehicle2.getVIN()){
//					if(vehicle1.getTrajectoryCellsTimeList().get(0)> vehicle2.getTrajectoryCellsTimeList().get(0)){
//						if(vehicle2.getTrajectoryCellsList().size()>1){
//							if(vehicle2.getTrajectoryCellsTimeList().get(1)> vehicle1.getTrajectoryCellsTimeList().get(0))
//								collisionduetoprotocol =true;
//						}else{
//							if(vehicle2.getExitTime()> vehicle1.getTrajectoryCellsTimeList().get(0))
//								collisionduetoprotocol =true;
//						}
//						}
//			}
//			}
//			
//		if(collisionduetoprotocol == true )
//		{	System.err.println("#vehicle "+ vehicle1.getVIN() +" collided with vehicle "+ vehicle2.getVIN() + " at current time " +currentTime + " due to a protocol issue"); // this way we report collisions twice(to log each vehicle's collision) so take this into consideration when calculating number of collisions
//			System.out.println("#vehicle "+ vehicle1.getVIN() +" collided with vehicle "+ vehicle2.getVIN() + " at current time " +currentTime + " due to a protocol issue"); // this way we report collisions twice(to log each vehicle's collision) so take this into consideration when calculating number of collisions
//		}else{	
//			System.err.println("#vehicle "+ vehicle1.getVIN() +" collided with vehicle "+ vehicle2.getVIN() + " at current time " +currentTime + " due to an issue that needs more review"); // this way we report collisions twice(to log each vehicle's collision) so take this into consideration when calculating number of collisions
//			System.out.println("#vehicle "+ vehicle1.getVIN() +" collided with vehicle "+ vehicle2.getVIN() + " at current time " +currentTime + " due to an issue that needs more review"); // this way we report collisions twice(to log each vehicle's collision) so take this into consideration when calculating number of collisions
//		}
//		}
		//report them so that in the next time steps their collisions wont be reported
		vehicle1.addElementtocollisionlist(vehicle2.getVIN());
		vehicle2.addElementtocollisionlist(vehicle1.getVIN());
		
		}
		
//			}
		}	
	}
	}
	}
	
	// ///////////////////////////////
	// STEP 6
	// ///////////////////////////////

	/**
	 * Remove all completed vehicles.
	 * 
	 * @return the VINs of the completed vehicles
	 */
	private List<Integer> cleanUpCompletedVehicles() {
		List<Integer> completedVINs = new LinkedList<Integer>();

		Rectangle2D mapBoundary = basicMap.getDimensions();

		List<Integer> removedVINs = new ArrayList<Integer>(vinToVehicles.size());
		for (int vin : vinToVehicles.keySet()) {
			VehicleSimView v = vinToVehicles.get(vin);
			// If the vehicle is no longer in the layout
			// TODO: this should be replaced with destination zone.
			if (!v.getShape().intersects(mapBoundary)) {
				// Process all the things we need to from this vehicle
				if (v instanceof AutoVehicleSimView) {
					AutoVehicleSimView v2 = (AutoVehicleSimView) v;
					totalBitsTransmittedByCompletedVehicles += v2
							.getBitsTransmitted();
					totalBitsReceivedByCompletedVehicles += v2
							.getBitsReceived();
				}
				
				// calculate vehicle journey time if we assumed there were no other vehicles on the map(vehicle keeps same speed throughout the journey, no delay in waiting in intersection)  
				
				double TimeIfVehicleNeverStops=v.getDistanceMoved()/(v.getDriver().getCurrentLane().getSpeedLimit() + 0.09); // 0.09 is due to vehicles moving with a speed over 0.09 than the speed limit.
						System.out.println("#vehicle "+vin+" Journey time if vehicle never stops is "+ TimeIfVehicleNeverStops);
				
				removedVINs.add(vin);
			}
		}
		// Remove the marked vehicles
		for (int vin : removedVINs) {
			System.out.println("#vehicle "+vin+" removed at current time "+ currentTime);
			vinToVehicles.remove(vin);
			completedVINs.add(vin);
			numOfCompletedVehicles++;
		}

		return completedVINs;
	}


}
