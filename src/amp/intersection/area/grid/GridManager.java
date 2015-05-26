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
package amp.intersection.area.grid;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import amp.config.Constants;
import amp.config.Debug;
import amp.driver.CrashTestDummy;
import amp.driver.Driver;
import amp.intersection.Intersection;
import amp.intersection.area.grid.Grid.TimeTile;
import amp.map.lane.Lane;
//import amp.msg.v2i.Request;
//import amp.msg.v2i.Request.VehicleSpecForRequestMsg;
import amp.sim.StatCollector;
import amp.util.TiledArea;
import amp.util.TiledArea.Tile;
import amp.vehicle.BasicAutoVehicle;
import amp.vehicle.VehicleSpec;
import amp.vehicle.VehicleUtil;

/**
 * Grid manager.
 */
public class GridManager
		implements
		Manager<GridManager.Query, GridManager.Plan, Integer> {

	// ///////////////////////////////
	// NESTED CLASSES
	// ///////////////////////////////

	/**
	 * The configuration of the reservation grid manager.
	 */
	public static class Config {
		/**
		 * The simulation time step.
		 */
		private final double timeStep;
		/**
		 * The length of a discrete time step in the grid
		 */
		private final double gridTimeStep;
		/**
		 * The size of the static buffer, in meters, used by this policy.
		 */
		private final double staticBufferSize;
		/**
		 * The size of the time buffer, in seconds, used for internal tiles.
		 */
		private final double internalTileTimeBufferSize;
		/**
		 * The size of the time buffer, in seconds, used for edge tiles.
		 */
		private final double edgeTileTimeBufferSize;
		/**
		 * Whether or not the edge tile time buffer is enabled.
		 */
		private final boolean isEdgeTileTimeBufferEnabled;
		/**
		 * The granularity.
		 */
		private final double granularity;

		/**
		 * Create a configuration object.
		 * 
		 * @param timeStep
		 * @param gridTimeStep
		 * @param staticBufferSize
		 * @param internalTileTimeBufferSize
		 * @param edgeTileTimeBufferSize
		 * @param isEdgeTileTimeBufferEnabled
		 * @param granularity
		 */
		public Config(double timeStep, double gridTimeStep,
				double staticBufferSize, double internalTileTimeBufferSize,
				double edgeTileTimeBufferSize,
				boolean isEdgeTileTimeBufferEnabled, double granularity) {
			this.timeStep = timeStep;
			this.gridTimeStep = gridTimeStep;
			this.staticBufferSize = staticBufferSize;
			this.internalTileTimeBufferSize = internalTileTimeBufferSize;
			this.edgeTileTimeBufferSize = edgeTileTimeBufferSize;
			this.isEdgeTileTimeBufferEnabled = isEdgeTileTimeBufferEnabled;
			this.granularity = granularity;
		}

		/**
		 * Get the time step.
		 * 
		 * @return the time step
		 */
		public double getTimeStep() {
			return timeStep;
		}

		/**
		 * Get the grid time step.
		 * 
		 * @return the grid time step
		 */
		public double getGridTimeStep() {
			return gridTimeStep;
		}

		/**
		 * Get the static buffer size.
		 * 
		 * @return the static buffer size
		 */
		public double getStaticBufferSize() {
			return staticBufferSize;
		}

		/**
		 * Get the internal time buffer size.
		 * 
		 * @return the internal tile time buffer size
		 */
		public double getInternalTileTimeBufferSize() {
			return internalTileTimeBufferSize;
		}

		/**
		 * Get the edge buffer size.
		 * 
		 * @return the edge tile time buffer size
		 */
		public double getEdgeTileTimeBufferSize() {
			return edgeTileTimeBufferSize;
		}

		/**
		 * Get whether the edge buffer is enabled.
		 * 
		 * @return whether the edge buffer is enabled
		 */
		public boolean getIsEdgeTileTimeBufferEnabled() {
			return isEdgeTileTimeBufferEnabled;
		}

		/**
		 * Get the granularity of the tile.
		 * 
		 * @return the granularity of the tile.
		 */
		public double getGranularity() {
			return granularity;
		}
	}

	/**
	 * The reservation grid manager.
	 */
	public static class Query {

//		/** The VIN of the vehicle */
//		private final int vin;
//		/** The arrival time */
//		private final double arrivalTime;
//		/** The arrival velocity */
//		private final double arrivalVelocity;
//		/** The ID of the arrival lane */
//		private final int arrivalLineId;
//		/** The ID of the departure lane */
//		private final int departureLaneId;
//		/** The vehicle specification for request message */
//	//	private final VehicleSpecForRequestMsg spec;
//		/** The maximum turn velocity */
//		private final double maxTurnVelocity;
//		/** Whether the acceleration is allowed */
//		private final boolean accelerating;

		/**
		 * Create a query.
		 * 
		 * @param vin
		 *            the VIN of the vehicle
		 * @param arrivalTime
		 *            the arrival time
		 * @param arrivalVelocity
		 *            the arrival velocity
		 * @param arrivalLineId
		 *            the arrival lane ID
		 * @param departureLaneId
		 *            the departure lane ID
		 * @param spec
		 *            the vehicle specification
		 * @param maxTurnVelocity
		 *            the maximum turn velocity
		 * @param accelerating
		 *            Whether the acceleration is allowed
		 */
//		public Query(int vin, double arrivalTime, double arrivalVelocity,
//				int arrivalLineId, int departureLaneId,
//				VehicleSpecForRequestMsg spec, double maxTurnVelocity,
//				boolean accelerating) {
//			this.vin = vin;
//			this.arrivalTime = arrivalTime;
//			this.arrivalVelocity = arrivalVelocity;
//			this.arrivalLineId = arrivalLineId;
//			this.departureLaneId = departureLaneId;
//	//		this.spec = spec;
//			this.maxTurnVelocity = maxTurnVelocity;
//			this.accelerating = accelerating;
//		}

		/**
		 * Get the VIN of a vehicle.
		 * 
		 * @return the VIN of a vehicle
		 */
//		public int getVin() {
//			return vin;
//		}
//
//		/**
//		 * Get the arrival time.
//		 * 
//		 * @return the arrival time
//		 */
//		public double getArrivalTime() {
//			return arrivalTime;
//		}
//
//		/**
//		 * Get the arrival velocity.
//		 * 
//		 * @return the arrival velocity
//		 */
//		public double getArrivalVelocity() {
//			return arrivalVelocity;
//		}
//
//		/**
//		 * Get the arrival lane ID.
//		 * 
//		 * @return the arrival lane ID
//		 */
//		public int getArrivalLaneId() {
//			return arrivalLineId;
//		}
//
//		/**
//		 * Get the departure lane ID.
//		 * 
//		 * @return the departure lane ID
//		 */
//		public int getDepartureLaneId() {
//			return departureLaneId;
//		}

		/**
		 * Get the specification of the vehicle for the request message.
		 * 
		 * @return the specification of the vehicle
		 */
//		public VehicleSpecForRequestMsg getSpec() {
//			return spec;
//		}

		/**
		 * Get the maximum turn velocity.
		 * 
		 * @return the maximum turn velocity
		 */
//		public double getMaxTurnVelocity() {
//			return maxTurnVelocity;
//		}
//
//		/**
//		 * Whether the vehicle is allowed to accelerate.
//		 * 
//		 * @return whether the vehicle is allowed to accelerate
//		 */
//		public boolean isAccelerating() {
//			return accelerating;
//		}
	}

	/**
	 * The plan for the reservation.
	 */
	public static class Plan {
		/** The VIN of the vehicle */
		private final int vin;
		/** The exit time */
		private final double exitTime;
		/** The exit velocity */
		private final double exitVelocity;
		/** The list of time tiles reserved */
		private final List<TimeTile> workingList;
		/** The acceleration profile */
		private final Queue<double[]> accelerationProfile;

		/**
		 * Create the plan for the reservation.
		 * 
		 * @param vin
		 *            the VIN of the vehicle
		 * @param exitTime
		 *            the exit time
		 * @param exitVelocity
		 *            the exit velocity
		 * @param workingList
		 *            the list of time tiles reserved
		 * @param accelerationProfile
		 *            the acceleration profile
		 */
		public Plan(int vin, double exitTime, double exitVelocity,
				List<TimeTile> workingList, Queue<double[]> accelerationProfile) {
			this.vin = vin;
			this.exitTime = exitTime;
			this.exitVelocity = exitVelocity;
			this.workingList = workingList;
			this.accelerationProfile = accelerationProfile;
		}

		/**
		 * Get the VIN of the vehicle.
		 * 
		 * @return the VIN of the vehicle
		 */
		public int getVin() {
			return vin;
		}

		/**
		 * Get the exit time.
		 * 
		 * @return the exit time
		 */
		public double getExitTime() {
			return exitTime;
		}

		/**
		 * Get the exit velocity.
		 * 
		 * @return the exit velocity
		 */
		public double getExitVelocity() {
			return exitVelocity;
		}

		/**
		 * Get the list of time tiles reserved.
		 * 
		 * @return the list of time tiles reserved
		 */
		public List<TimeTile> getWorkingList() {
			return workingList;
		}

		/**
		 * Get the acceleration profile.
		 * 
		 * @return the acceleration profile
		 */
		public Queue<double[]> getAccelerationProfile() {
			return accelerationProfile;
		}
	}

	/**
	 * The statistic collector for VIN history.
	 */
	public static class VinHistoryStatCollector implements
			StatCollector<GridManager> {
		/**
		 * A list of the VINs of all reserved tiles at every time step.
		 */
		private final List<Double> vinHistoryTime;

		/**
		 * A mapping from VINs to histories.
		 */
		private final Map<Double, Set<Integer>> vinHistory;

		/**
		 * Create a statistic collector for VIN history.
		 */
		public VinHistoryStatCollector() {
			this.vinHistoryTime = new LinkedList<Double>();
			this.vinHistory = new HashMap<Double, Set<Integer>>();
		}

		/**
		 * {@inheritDoc}
		 */
		@Override
		public void collect(GridManager manager) {
			Set<Integer> s = manager.reservationGrid
					.getVinOfReservedTilesAtTime(manager.currentTime);
			Set<Integer> lasts = null;
			if (vinHistoryTime.size() > 0) {
				lasts = vinHistory
						.get(vinHistoryTime.get(vinHistoryTime.size() - 1));
			}
			if (!s.equals(lasts)) {
				vinHistoryTime.add(manager.currentTime);
				vinHistory.put(manager.currentTime, s);
			}
		}

		/**
		 * {@inheritDoc}
		 */
		@Override
		public void print(PrintStream outfile) {
			for (double time : vinHistoryTime) {
				Set<Integer> vins = vinHistory.get(time);
				outfile.printf("%.2f", time);
				for (int vin : vins) {
					outfile.printf(",%d", vin);
				}
				outfile.println();
			}
		}

	}

	// ///////////////////////////////
	// PRIVATE FIELDS
	// ///////////////////////////////

	// TODO: replace config with other things

	/**
	 * The configuration of this reservation grid manager.
	 */
	public Config config;

	/**
	 * The size of the static buffer, in meters, used by this policy.
	 */
	public double staticBufferSize;
	/**
	 * The size of the time buffer, in time steps, used for internal tiles.
	 */
	private final int internalTileTimeBufferSteps;
	/**
	 * The size of the time buffer, in time steps, used for edge tiles.
	 */
	private final int edgeTileTimeBufferSteps;
	/**
	 * The current time.
	 */
	private double currentTime;
	/**
	 * The intersection
	 */
	private final Intersection intersection;
	/**
	 * The tiled area of the intersection
	 */
	private final TiledArea tiledArea;
	/**
	 * The reservation System
	 */
	private final Grid reservationGrid;
	/**
	 * The statistic collector
	 */
	private final StatCollector<GridManager> statCollector;

	// ///////////////////////////////
	// CONSTRUCTORS
	// ///////////////////////////////

	/**
	 * Create a reservation grid manager.
	 * 
	 * @param config
	 *            the configuration of the grid manager
	 * @param intersection
	 *            the intersection
	 * @param tiledArea
	 *            the tiled area
	 * @param reservationGrid
	 *            the reservation grid
	 */
	public GridManager(Config config, Intersection intersection,
			TiledArea tiledArea, Grid reservationGrid) {
		this(0.0, config, intersection, tiledArea, reservationGrid);
	}

	/**
	 * Create a reservation grid manager.
	 * 
	 * @param currentTime
	 *            the current time
	 * @param config
	 *            the configuration of the grid manager
	 * @param intersection
	 *            the intersection
	 * @param tiledArea
	 *            the tiled area
	 * @param reservationGrid
	 *            the reservation grid
	 */
	public GridManager(double currentTime, Config config,
			Intersection intersection, TiledArea tiledArea,
			Grid reservationGrid) {
		this.currentTime = currentTime;
		this.config = config;
		this.staticBufferSize = config.getStaticBufferSize();
		config
				.getIsEdgeTileTimeBufferEnabled();
		this.internalTileTimeBufferSteps = (int) (config
				.getInternalTileTimeBufferSize() / config.getGridTimeStep());
		this.edgeTileTimeBufferSteps = (int) (config
				.getEdgeTileTimeBufferSize() / config.getGridTimeStep());

		this.intersection = intersection;
		this.tiledArea = tiledArea;
		this.reservationGrid = reservationGrid;
		this.statCollector = new VinHistoryStatCollector();
	}

	// ///////////////////////////////
	// PUBLIC METHODS
	// ///////////////////////////////

	/**
	 * Advance the time step.
	 * 
	 * @param timeStep
	 *            the time step
	 */
	public void act(double timeStep) {
		reservationGrid.cleanUp(currentTime);
		if (statCollector != null)
			statCollector.collect(this);
		currentTime += timeStep;
	}

	/**
	 * Get the configuration.
	 * 
	 * @return the configuration
	 */
	public Config getConfig() {
		return config;
	}

	/**
	 * Get the tiled area.
	 * 
	 * @return the tiled area
	 */
	public TiledArea getTiledArea() {
		return tiledArea;
	}

	/**
	 * Get the statistic collector.
	 * 
	 * @return the statistic collector
	 */
	public StatCollector<GridManager> getStatCollector() {
		return statCollector;
	}

	// ///////////////////////////////
	// PUBLIC METHODS
	// ///////////////////////////////

	/**
	 * Find a set of space-time tile for a particular traversal proposal in a
	 * request message. This attempt can be either with attempting to
	 * setMaxAccelWithMaxTargetVelocity to maximum velocity or with a constant
	 * velocity.
	 * 
	 * @param q
	 *            the query object
	 * 
	 * @return a set of space-time tiles on the trajectory and the exit velocity
	 *         of the vehicle if the reservation is successful; otherwise return
	 *         null.
	 */
	@Override
	public Plan query(Query q) {
		return null;

		// Position the Vehicle to be ready to start the simulation
//		Lane arrivalLane = Debug.currentMap.getLaneRegistry().get(
//				q.getArrivalLaneId());
//		Lane departureLane = Debug.currentMap.getLaneRegistry().get(
//				q.getDepartureLaneId());

		// Create a test vehicle to use in the internal simulation
//		BasicAutoVehicle testVehicle = createTestVehicle(q.getSpec(),
//				q.getArrivalVelocity(), q.getMaxTurnVelocity(), arrivalLane);

		// Create a dummy driver to steer it
//		Driver dummy = new CrashTestDummy(testVehicle, arrivalLane,
//				departureLane);

		// assign the drive to the vehicle
		// testVehicle.setDriver(dummy); // TODO fix this later.

		// Keep track of the TileTimes that will make up this reservation
//		FindTileTimesBySimulationResult fResult = findTileTimesBySimulation(
//				testVehicle, dummy, q.getArrivalTime(), q.isAccelerating());

	//	if (fResult != null) {
	//		List<TimeTile> workingList = fResult.getWorkingList();

//			double exitTime = workingList.get(workingList.size() - 1).getTime();

//			Queue<double[]> accelerationProfile = calcAccelerationProfile(
//					q.getArrivalTime(), q.getArrivalVelocity(),
//					q.getMaxTurnVelocity(), q.getSpec().getMaxAcceleration(),
//					fResult.getExitTime(), q.isAccelerating());

//			return new Plan(q.getVin(), exitTime, testVehicle.gaugeVelocity(),
//					workingList, accelerationProfile);
	//	} else {
	//		return null;
	//	}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Integer accept(Plan plan) {
		boolean b = reservationGrid.reserve(plan.getVin(),
				plan.getWorkingList());
		assert b;
		return plan.getVin();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void cancel(Integer reservationId) {
		reservationGrid.cancel(reservationId); // reservationId == vin
	}

	// ///////////////////////////////
	// PRIVATE FIELDS
	// ///////////////////////////////

	/**
	 * Create a test vehicle to use in the internal simulation.
	 * 
	 * @param spec
	 *            the specification of the test vehicle
	 * @param arrivalVelocity
	 *            the arrival velocity of the vehicle
	 * @param maxVelocity
	 *            the Vehicle's maximum velocity, in meters per second
	 * @param arrivalLane
	 *            the arrival lane of the vehicle in this proposal
	 * 
	 * @return whether or not a reservation could be made
	 */
//	private BasicAutoVehicle createTestVehicle(
//			Request.VehicleSpecForRequestMsg spec, double arrivalVelocity,
//			double maxVelocity, Lane arrivalLane) {
//
//		VehicleSpec newSpec = new VehicleSpec(
//				"TestVehicle",
//				spec.getMaxAcceleration(),
//				spec.getMaxDeceleration(),
//				maxVelocity, // TODO: why not one in
//								// msg.getSpec().getMaxVelocity()
//				spec.getMinVelocity(), spec.getLength(), spec.getWidth(),
//				spec.getFrontAxleDisplacement(),
//				spec.getRearAxleDisplacement(), 0.0, // wheelSpan
//				0.0, // wheelRadius
//				0.0, // wheelWidth
//				spec.getMaxSteeringAngle(), spec.getMaxTurnPerSecond());
//
//		BasicAutoVehicle testVehicle = new BasicAutoVehicle(newSpec,
//				intersection.getEntryPoint(arrivalLane), // Position
//				intersection.getEntryHeading(arrivalLane), // Heading
//				0.0, // Steering angle
//				arrivalVelocity, // velocity
//				0.0, // target velocity
//				0.0, // Acceleration
//				0.0); // the current time // TODO: need to think about the
//						// appropriate
//						// current time
//
//		return testVehicle;
//	}

	

	

	// ///////////////////////////////
	// DEBUG
	// ///////////////////////////////

	/**
	 * Get the rectangles for the ReservationTiles that have a reservation for
	 * the current time.
	 * 
	 * @return the rectangles for the ReservationTiles that have a reservation
	 *         for the current time
	 */
	public List<? extends Shape> getDebugShapes() {
		List<Rectangle2D> reservedRects = new ArrayList<Rectangle2D>();
		for (int i : reservationGrid.getReservedTilesAtTime(currentTime)) {
			reservedRects.add(tiledArea.getTileById(i).getRectangle());
		}
		return reservedRects;
	}

}
