package amp.TrajectoryCells;

import java.awt.geom.Area;
import java.awt.geom.Point2D;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.text.DecimalFormat;

import amp.config.Debug;
import amp.intersection.Intersection;
import amp.intersection.IntersectionDetails;
import amp.map.BasicMap;
import amp.map.lane.Lane;
import amp.util.GeomMath;
import amp.util.TiledArea;
import amp.util.Util;
import amp.util.TiledArea.Tile;
import amp.vehicle.AutoVehicleSimView;
import amp.vehicle.BasicAutoVehicle;
import amp.vehicle.VehicleSpec;
import amp.vehicle.VehicleUtil;

public class CellsTrajectoryLists {	

/** Intersection area */	
private Area area;	
/**
 * The minimum distance to maintain between the Vehicle controlled now and the one in front of it.
 *  {@value} meters.
 */	
private double MINIMUM_FOLLOWING_DISTANCE; 
/**
 * Number of cells in each side of the Intersection after dividing the intersection into cells
 */
private double noofcellsperside;
/**
 *  Start turning when center of vehicle is this distance apart from center of cell (here center of cell is not the actual center but the vertical distance between the vehicle's heading and the vertical bisector line of the cell)
 *  {@value} meters
 */
private double thresholddistancetoturn; 
/**
 *  A vehicle stopping before the intersection will stop this meters apart from the intersection boundaries 
 */
private double stopbeforeintersectionthreshold;
/**
 *  Stop completely when center of vehicle is this distance apart from center of cell (here center of cell is not the actual center but the vertical distance between the vehicle's heading and the vertical bisector line of the cell)
 *  {@value} meters
 */
private double thresholddistancetostop; 

private int nooflanes=2; // TODO: get it from simulator configuration

public int vin;
/**
 * Calculate Trajectory Cells List and Trajectory Cells arrival time list by running an internal simulation
 * of the vehicle's anticipated route   
 * 
 * @param Vehicle
 * 			The vehicle calling this method
 * @param currentTime
 * 			Current Simulation time when the vehicle called this method
 * @param basicmap
 * 	 		The map 
 * @param no_of_cells_per_side
 * 	        Number of cells in each side of the Intersection after dividing the intersection into cells	 
 * @param min_fol_dis
 * 			The minimum distance to maintain between the Vehicle controlled now and the one in front of it
 * @return  Array of Two lists:
 * 	1. Trajectory cells list 
 * 	2. Trajectory Cells arrival Time list with Exit time: which includes Trajectory Cells arrival Time list + an extra element at the end of the list which is the trajected exit time
 * 
 */
public List<Double>[] TrajectoryCellsListandCellsArrivalTimeListWithExitTime(AutoVehicleSimView Vehicle, double currentTime,BasicMap basicmap,double no_of_cells_per_side,double min_fol_dis,double thresh_dist_to_turn,double thresh_dist_to_stop,double stop_before_int_thresh)
{

MINIMUM_FOLLOWING_DISTANCE = min_fol_dis;	
noofcellsperside = no_of_cells_per_side;
thresholddistancetoturn= thresh_dist_to_turn;
thresholddistancetostop= thresh_dist_to_stop;
stopbeforeintersectionthreshold = stop_before_int_thresh;

String deptroad= Debug.currentMap.getRoad(Vehicle.getDriver().getCurrentLane()).getName();
String arrivroad= Vehicle.getDriver().getDestination().getName();
int deptlaneId= Vehicle.getDriver().getCurrentLane().getId(); 

// get the cell id of the cell which the vehicle should turn at 
int turningCellid=getturningcell(deptroad, arrivroad, deptlaneId,noofcellsperside);

return getTrajectorylists(Vehicle,currentTime,basicmap,turningCellid,deptroad,arrivroad);
}


/**
 * Get the cell id of the cell which the vehicle should turn at 
 * 
 * @param deptroad
 *  		Road which the vehicle uses until it reaches the intersection
 * @param arrivroad
 *   		Road which the vehicle uses after crossing the intersection 			
 * @param deptlaneId
 *          Lane which the vehicle uses until it reaches the intersection
 * @param noofcellsperside
 * 	        Number of cells in each side of the Intersection after dividing the intersection into cells	 
 * @return cell id of the cell which the vehicle should turn at
 */
public int getturningcell(String deptroad, String arrivroad,int deptlaneId, double noofcellsperside){

int turningCellId = -1; // out of scope of cell ids	

// we have included only the turning cell of two grid divisions , one when noofcellsperside equals 6 and the other when its equal 4 
// other divisions could be included if desired
if(noofcellsperside ==6.0){

if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street W") && (deptlaneId != 0))
turningCellId=26;
else if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street W") && (deptlaneId == 0))
turningCellId=20;	
else if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street E")&& (deptlaneId != 0))
turningCellId=28;
else if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street E")&& (deptlaneId == 0))
turningCellId=22;
else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street W")&& (deptlaneId != 3))
turningCellId=7;
else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street W")&& (deptlaneId == 3))
turningCellId=13;
else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street E")&& (deptlaneId != 3))
turningCellId=9;
else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street E")&& (deptlaneId == 3))
turningCellId=15;
else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue N")&& (deptlaneId != 6))
turningCellId=22;
else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue N")&& (deptlaneId == 6))
turningCellId=21;
else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue S")&& (deptlaneId != 6))
turningCellId=10;
else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue S")&& (deptlaneId == 6))
turningCellId=9;
else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue N")&& (deptlaneId != 9))
turningCellId=25;
else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue N")&& (deptlaneId == 9))
turningCellId=26;
else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue S")&& (deptlaneId != 9))
turningCellId=13;	
else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue S")&& (deptlaneId == 9))
turningCellId=14;	

}else if(noofcellsperside==4){		
	
	int a,b,c,d;
	
	if(nooflanes==3){
	a=2;b=5;c=8;d=11;	
	}
	else //nooflanes=2
	{
	a=1;b=3;c=5;d=7;		
	}
	
	if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street W") && (deptlaneId != a))
		turningCellId=9;
		else if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street W") && (deptlaneId == a))
		turningCellId=13;	
		else if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street E")&& (deptlaneId != a))
		turningCellId=10;
		else if(deptroad.equals("1st Avenue N") && arrivroad.equals("1st Street E")&& (deptlaneId == a))
		turningCellId=15;
		else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street W")&& (deptlaneId != b))
		turningCellId=5;
		else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street W")&& (deptlaneId == b))
		turningCellId=0;
		else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street E")&& (deptlaneId != b))
		turningCellId=6;
		else if(deptroad.equals("1st Avenue S") && arrivroad.equals("1st Street E")&& (deptlaneId == b))
		turningCellId=2;
		else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue N")&& (deptlaneId != c))
		turningCellId=10;
		else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue N")&& (deptlaneId == c))
		turningCellId=11;
		else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue S")&& (deptlaneId != c))
		turningCellId=6;
		else if(deptroad.equals("1st Street E") && arrivroad.equals("1st Avenue S")&& (deptlaneId == c))
		turningCellId=3;
		else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue N")&& (deptlaneId != d))
		turningCellId=9;
		else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue N")&& (deptlaneId == d))
		turningCellId=12;
		else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue S")&& (deptlaneId != d))
		turningCellId=5;	
		else if(deptroad.equals("1st Street W") && arrivroad.equals("1st Avenue S")&& (deptlaneId == d))
		turningCellId=4;	
	
	
}else {
	System.out.println("This no of cells per side is not supported");
}


return turningCellId;
}

/**
 *  Calculate Trajectory Cells List and Trajectory Cells arrival time list by running an internal simulation
 * of the vehicle's anticipated route   
 * 
 * @param Vehicle
 * 			The Vehicle
 * @param currentTime
 *          Current Simulation time when the vehicle called this method
 * @param basicMap
 * 			The map 
 * @param turningCellId
 * 			cell id of the cell which the vehicle turns at
 * @param deptroad
 * 			Road which the vehicle uses until it reaches the intersection
 * @param arrivroad
 * 			Road which the vehicle uses after crossing the intersection 			
 * @return Array of two lists:
 * 	1. Trajectory cells list 
 * 	2. Trajectory Cells arrival Time list with Exit time: which includes Trajectory Cells arrival Time list + an extra element at the end of the list which is the trajected exit time
 */
 public List<Double>[] getTrajectorylists(AutoVehicleSimView Vehicle,double currentTime,BasicMap basicMap,int turningCellId,String deptroad,String arrivroad){

	 vin = Vehicle.getVIN();
	 DecimalFormat df = new DecimalFormat("####0.00");
	//	System.out.println("#vehicle " +Vehicle.getVIN() + " turning cell is " + turningCellId);

		//get intersection
	IntersectionDetails im = basicMap.getIntersectionManagers().get(0); //TODO: edit to contain the vehicle's intersection manager not only the first IM
	
		// creating test vehicle	
		BasicAutoVehicle testVehicle = createTestVehicle(Vehicle.getSpec(), Vehicle.getVelocity(), Vehicle.getSpec().getMaxVelocity(), Vehicle.getDriver().getCurrentLane(),im.getIntersection(),Vehicle.getPosition(),Vehicle.getAcceleration(), Vehicle.getHeading(),Vehicle.getSteeringAngle());

		area= im.getIntersection().getArea(); // real area used to contain cells
		Area areaplus = im.getIntersection().getAreaPlus(); // used when moving testvehicle in intersection

		// The list of tile-times that will make up this reservation
		List<Double> timelist = new ArrayList<Double>();
		List<Double> cellslist = new ArrayList<Double>();
		
		double TimeStep = 0.02;
		boolean accelerating = false;
		// drive the test vehicle until it leaves the intersection
		
		int currentCell=-2; // any number outside scope of cells and different than initial turningcellid
		int previousCell=-1;
		double previousCellArrivalTime =-1; 
		TiledArea tiledarea=new TiledArea(area,area.getBounds2D().getWidth()/noofcellsperside); 
		Tile previousCelltile =null;
		int tileid;
		Tile tile;
		int state=Vehicle.getState();
		testVehicle.setcelltostop(Vehicle.getcelltostop());
		testVehicle.setAccelWithMaxTargetVelocity(Vehicle.getAcceleration());

		testVehicle.setVelocity(Vehicle.getVelocity());
		testVehicle.setcurrentcell(-1); // initialize currentcell
		testVehicle.setturnthistimestep(Vehicle.getturnthistimestep());
		//Oct-19
		boolean SetArrivalEstimatesToNextCellsToZero=false;
		
		if(testVehicle.getcelltostop()==-5){
			SetArrivalEstimatesToNextCellsToZero=true;
		}
		
		//WHILELOOP:
		while ((VehicleUtil.intersects(testVehicle, areaplus)) || (state==1)) { // as testvehicle is created at boundaries of area so we need an area slightly larger than area 
			
	//		System.out.println("#vehicle " + vin + " has TCL and TCATL: " + cellslist + " " + timelist );
			
			double arrivheading=getHeadingFromArrivalRoadname(arrivroad);
			
			if(Math.abs(Double.parseDouble(df.format(arrivheading - testVehicle.getHeading())))<0.1)
			{
				testVehicle.setHeading(arrivheading);
		//		testVehicle.clearturning();
			}
				
				testVehicle.setSteeringAngle(0); // initialize it with zero before turning
			
			if(VehicleUtil.intersects(testVehicle, areaplus))
				state=2;
			
			// Find out which cells are occupied by the vehicle
			// a tile is another name for a cell that is used originally by AIM								    
			for(int i=0;i<tiledarea.getXNum()-1;i++){
				for(int j=0;j<tiledarea.getYNum()-1;j++){
		    tileid=tiledarea.getTile(i,j).getId();
			tile = tiledarea.getTileById(tileid);

			if(tile.getRectangle().contains(Vehicle.getPosition()))
			{
				if(Vehicle.getTrajectoryCellsList().size()!=0){
				for(int f=0; f<Vehicle.getTrajectoryCellsList().size(); f++){
					if((tileid == Vehicle.getTrajectoryCellsList().get(f)) && (f>0)){
					previousCell=Vehicle
							.getTrajectoryCellsList().get(f-1);	
					previousCellArrivalTime=Vehicle.getTrajectoryCellsTimeList().get(f-1);
					}
				}
				}
				
				if((previousCell != -2) && (previousCellArrivalTime != -1))
				{
					previousCelltile = tiledarea.getTileById(previousCell);
				if(previousCelltile.getRectangle().contains(Vehicle.getPointAtRear())){			
					if(cellslist.size()==0){
						cellslist.add((double)previousCell);
						timelist.add(previousCellArrivalTime);
					}
				}
				}
			}
			
			if(tile.getRectangle().contains(testVehicle.getPosition()))
			{ 
			if (currentCell!=tileid)
			{		
//				// to include the previous cell in the TCL until the vehicle exits that cell completely 
//				if(cellslist.size()==0){
//				if(Vehicle.getTrajectoryCellsList().size()>1){
//				if((Vehicle.getState()==2) && (Vehicle.getTrajectoryCellsList().size() == Vehicle.getTrajectoryCellsTimeList().size()) && (Vehicle.getTrajectoryCellsList().get(1)==tileid) && (tiledarea.getTileById(Vehicle.getTrajectoryCellsList().get(0)).getRectangle().contains(testVehicle.getPointAtRear())))
//				{
//				cellslist.add((double)Vehicle.getTrajectoryCellsList().get(0));	
//				timelist.add(Vehicle.getTrajectoryCellsTimeList().get(0));
//			    }	
//				}	
//				}
			// first cell timing is from previous time list 
			if((Vehicle.getState()==2) && (Vehicle.getTrajectoryCellsList().size()!=0) && (Vehicle.getTrajectoryCellsList().size() == Vehicle.getTrajectoryCellsTimeList().size()) && (Vehicle.getTrajectoryCellsList().get(0)==tileid))//(Vehicle.getTrajectoryCellsList().get(0)==tileid))
				{
				cellslist.add((double)Vehicle.getTrajectoryCellsList().get(0));	
				if(SetArrivalEstimatesToNextCellsToZero==false)
					timelist.add(Vehicle.getTrajectoryCellsTimeList().get(0));
				else
					timelist.add(Double.POSITIVE_INFINITY);

			}
			else if((Vehicle.getState()==2) && (previousCelltile !=null) && (previousCelltile.getRectangle().contains(Vehicle.getPointAtRear())) && (cellslist.size()==1)){
			
				if((Vehicle.getTrajectoryCellsList().size()!=0) && (Vehicle.getTrajectoryCellsList().size() == Vehicle.getTrajectoryCellsTimeList().size()) && (Vehicle.getTrajectoryCellsList().get(1)==tileid))
				{
					cellslist.add((double)tileid);
					if(SetArrivalEstimatesToNextCellsToZero==false)
						timelist.add(Vehicle.getTrajectoryCellsTimeList().get(1));
					else
					timelist.add(Double.POSITIVE_INFINITY);
					
				}
					
			}
			else{
			cellslist.add((double)tileid);
			//in the time when the vehicle is touching two cells the vehicle's arrival time at the 2nd cell should remain unchanged
//			if((Vehicle.getTrajectoryCellsList().size()>1) && (Vehicle.getTrajectoryCellsList().get(1)==tileid) && !(tile.getRectangle().contains(Vehicle.getPointAtRear())) && (Vehicle.getcelltostop()!=-5))
//				timelist.add(Vehicle.getTrajectoryCellsTimeList().get(1));
//			else
			if((Vehicle.getTrajectoryCellsList().size()>1) && (Vehicle.getTrajectoryCellsList().get(1)==tileid) && (tile.getRectangle().contains(Vehicle.getPointAtRear())) &&!(tiledarea.getTileById(Vehicle.getTrajectoryCellsList().get(0)).getRectangle().contains(Vehicle.getPointAtRear())) && (timelist.size()==0) )
			{
				if(SetArrivalEstimatesToNextCellsToZero==false)
					timelist.add(Vehicle.getTrajectoryCellsTimeList().get(1));	
				else
					timelist.add(Double.POSITIVE_INFINITY);
				
			}else{
				if(SetArrivalEstimatesToNextCellsToZero==false)
					timelist.add(currentTime);
				else
					timelist.add(Double.POSITIVE_INFINITY);
			
			}
			}
				currentCell=tileid;
								
					}

			if(testVehicle.getcelltostop()==tileid){
			
			SetArrivalEstimatesToNextCellsToZero=true;	
			//break WHILELOOP;
			}
				}

			//if(tile.getRectangle().contains(testVehicle.getPointAtRear())){					
			//
				if(tileid==turningCellId){ 
//					
				Point2D centerpointofcell = new Point2D.Double(tile.getRectangle().getCenterX(),tile.getRectangle().getCenterY());
			//	
				double angle = GeomMath.angleToPoint(centerpointofcell,testVehicle.gaugePointBetweenFrontWheels());
//				// Need to recenter this value to [-pi, pi]
				double Anglebetweenvehicleandcenterofcell = Util.recenter(angle - testVehicle.getHeading(),
						-1.0 * Math.PI, Math.PI);
				double distancetocenterofcell = Math.abs(Point2D.distance(tile.getRectangle().getCenterX(), tile.getRectangle().getCenterY(), testVehicle.gaugePointBetweenFrontWheels().getX(), testVehicle.gaugePointBetweenFrontWheels().getY()));
				double verticaldistancetocenterofcell = distancetocenterofcell * Math.cos(Anglebetweenvehicleandcenterofcell) ;//with respect to vehicle heading
//				double verticaldistancetocenterofcell=Math.abs(Point2D.distance(tile.getRectangle().getCenterX(), tile.getRectangle().getCenterY(), testVehicle.getPointAtRear().getX(), testVehicle.getPointAtRear().getY()));
			//	
//				 System.out.println("#vehicle " + vin  + " has vertical distance to center of cell from front of vehicle : "+ verticaldistancetocenterofcell);
			//
				//at a distance to center of cell equaling to thresholddistancetoturn the test vehicle should start turning, which is done by setting the vehicle's turning variable 
				if(verticaldistancetocenterofcell <thresholddistancetoturn){
					 testVehicle.setturning(); 
					// System.out.println("#vehicle " + vin  + " has vertical distance to center of cell from front of vehicle : "+ verticaldistancetocenterofcell);
					}
				}
				
			
//if(tile.getRectangle().contains(testVehicle.getPointAtRear())){					
//
//	if(tileid==turningCellId){ 
//		
//	Point2D centerpointofcell = new Point2D.Double(tile.getRectangle().getCenterX(),tile.getRectangle().getCenterY());
//	
//	double angle = GeomMath.angleToPoint(centerpointofcell,testVehicle.gaugePointBetweenFrontWheels());
//	// Need to recenter this value to [-pi, pi]
////	double Anglebetweenvehicleandcenterofcell = Util.recenter(angle - testVehicle.getHeading(),
////			-1.0 * Math.PI, Math.PI);
////	double distancetocenterofcell = Math.abs(Point2D.distance(tile.getRectangle().getCenterX(), tile.getRectangle().getCenterY(), testVehicle.gaugePointBetweenFrontWheels().getX(), testVehicle.gaugePointBetweenFrontWheels().getY()));
////	double verticaldistancetocenterofcell = distancetocenterofcell * Math.cos(Anglebetweenvehicleandcenterofcell) ;//with respect to vehicle heading
//	double verticaldistancetocenterofcell=Math.abs(Point2D.distance(tile.getRectangle().getCenterX(), tile.getRectangle().getCenterY(), testVehicle.getPointAtRear().getX(), testVehicle.getPointAtRear().getY()));
//	
//	 System.out.println("#vehicle " + vin  + " has vertical distance to center of cell from front of vehicle : "+ verticaldistancetocenterofcell);
//
//	//at a distance to center of cell equaling to thresholddistancetoturn the test vehicle should start turning, which is done by setting the vehicle's turning variable 
//	if(verticaldistancetocenterofcell <thresholddistancetoturn){
//		 testVehicle.setturning(); 
//		// System.out.println("#vehicle " + vin  + " has vertical distance to center of cell from front of vehicle : "+ verticaldistancetocenterofcell);
//		}
//	}
//		
//			   }
			}
				}

			if(testVehicle.getturning()==true)
			changeHeading(testVehicle,deptroad,arrivroad);	//start actually changing the vehicle's heading 

				moveTestVehicle(testVehicle,Vehicle, TimeStep,tiledarea, Vehicle.getDriver().getCurrentLane(),Vehicle.gettimetogotonextcell() ,currentTime);
				currentTime=TimeStep+currentTime; // Record that we've moved forward one time step
	
				
		}  
	
		
		//in the period where the vehicle is exiting the last cell in its TCL we need to keep the cell and its arrival time in the cross message 
		if(cellslist.size()==0 && timelist.size()==0 && Vehicle.getState()==2 && Vehicle.getTrajectoryCellsList().size()==1 && Vehicle.getTrajectoryCellsTimeList().size()==1 )
		{
			cellslist.add((double)Vehicle.getTrajectoryCellsList().get(0));
			timelist.add(Vehicle.getTrajectoryCellsTimeList().get(0));
			
		}
			
		//	if (testVehicle.getcelltostop()!=-1)
		if (SetArrivalEstimatesToNextCellsToZero==true)
			timelist.add(Double.POSITIVE_INFINITY); // exit time is not known
		else
		timelist.add(currentTime); // exit time	added at the end of the trajectory cell time list		    
		
		List<Double>[] trajectorylists = (ArrayList<Double>[])new ArrayList[2];
		
		trajectorylists[0]=cellslist;
		trajectorylists[1]=timelist;
		
		return trajectorylists;
	
}

// create test vehicle
// From amp.intersection.area.grid.GridManager
 BasicAutoVehicle createTestVehicle(
		VehicleSpec spec, double arrivalVelocity,
		double maxVelocity, Lane deptLane, Intersection intersection,Point2D position, double acceleration, double heading, double steeringangle) {

	VehicleSpec newSpec = new VehicleSpec(
			"TestVehicle",
			spec.getMaxAcceleration(),
			spec.getMaxDeceleration(),
			maxVelocity, // TODO: why not one in
							// msg.getSpec().getMaxVelocity()
			spec.getMinVelocity(), spec.getLength(), spec.getWidth(),
			spec.getFrontAxleDisplacement(),
			spec.getRearAxleDisplacement(), 0.0, // wheelSpan
			0.0, // wheelRadius
			0.0, // wheelWidth
			spec.getMaxSteeringAngle(), spec.getMaxTurnPerSecond());

	BasicAutoVehicle testVehicle = new BasicAutoVehicle(newSpec,
			//intersection.getEntryPoint(deptLane), // Position
			position,
			heading,//intersection.getEntryHeading(deptLane), // Heading
			steeringangle, //0.0 // Steering angle
			arrivalVelocity, // velocity
			0.0, // target velocity
			acceleration,//0.0, // Acceleration
			0.0); // the current time // TODO: need to think about the
					// appropriate
					// current time

	return testVehicle;
}

/**
 * Advance the test vehicle by a certain duration time
 * 
 * @param testVehicle
 *            The test vehicle
 * @param Vehicle
 *            The original vehicle which called this method
 * @param duration
 * 			  Duration of the move	
 * @param tiledarea
 * 			Tiled Area	  
 * @param deptlane
 * 			Vehicle's departure lane
 * @param timetogotonextcell
 * 			At this time the test vehicle can start moving to the next cell
 * @param currenttime
 * 			Current time of the internal simulation(relative to the test vehicle)
 */
 void moveTestVehicle(BasicAutoVehicle testVehicle, AutoVehicleSimView Vehicle,
		double duration, TiledArea tiledarea, Lane deptlane,double timetogotonextcell, double currenttime) {
	    
	// Now just take the minimum of the max velocity of the vehicle, and
	// the speed limit in the lane to be the velocity that the vehicle wants to reach and keep
	    double initVelocity = Math.min(Vehicle.getSpec().getMaxVelocity(),
	    		Vehicle.getDriver().getCurrentLane().getSpeedLimit());
	
	if(testVehicle.getVelocity() <= initVelocity) {
				//		  if(testVehicle.getturning()==false)
	    testVehicle.setAccelWithMaxTargetVelocity(testVehicle.getSpec().getMaxAcceleration()); 
//					   else
//						   testVehicle.setAccelWithMaxTargetVelocity(0); // don't accelerate while turning 			 
		}else
				testVehicle.setAccelWithMaxTargetVelocity(0);
			
	// Now move the vehicle
	testVehicle.move(duration);
}

 
 	/**
	 * Change heading of the vehicle by changing the steering angle depending on the arrival road the vehicle has chosen
	 * 
	 * @param vehicle
	 * 			Vehicle changing its heading
	 * @param deptroad
	 * 		    Road which the vehicle uses until it reaches the intersection
	 * @param arrivroad
	 * 			Road which the vehicle uses after crossing the intersection
	 */
 void changeHeading(AutoVehicleSimView vehicle,String deptroad,String arrivroad){
	
	int n=3; // Vehicle will change its steering angle n times , each time the change is equal to the total angle required to change heading over n
	double deptheading=getHeadingFromArrivalRoadname(deptroad);// from departure road we calculate the departure heading
	double arrivheading=getHeadingFromArrivalRoadname(arrivroad); // from arrival road we calculate the arrival heading

	DecimalFormat df = new DecimalFormat("####0.00");

//	////
//		double higharrivheading=arrivheading+0.15;
//		double lowarrivheading=arrivheading-0.15;
//		////
		
	//keep changing steering angle until the vehicle's heading is equal to the vehicle's arrival heading
	if(Double.parseDouble(df.format(vehicle.getHeading())) != Double.parseDouble(df.format(arrivheading))){
	//	System.out.println("#vehicle " +vin + " is turning now!!! ");
		
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
		vehicle.setSteeringAngle(- 1.5708/n);	
		}

	if (Double.parseDouble(df.format(vehicle.getHeading())) >= Double.parseDouble(df.format(6.283))) // one full 360 degrees, reset to 0
		vehicle.setHeading(vehicle.getHeading() - 6.283);
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
double getHeadingFromArrivalRoadname(String arrivalroad){
	
	if (arrivalroad.equalsIgnoreCase("1st Avenue S"))
		return 1.57;
	else if (arrivalroad.equalsIgnoreCase("1st Avenue N"))
		return 4.71;
	else if (arrivalroad.equalsIgnoreCase("1st Street W"))
		return 3.14;
	else if (arrivalroad.equalsIgnoreCase("1st Street E"))
		return 0.00;
	else System.err.printf("arrival road doesn't exit \n");
	return 5; // error
		
	}

/**
 *  Change the vehicle's acceleration so that it stops before entering the intersection
 *  
 * @param testVehicle
 * 			test Vehicle stopping
 * @param vehicle
 *			 The original vehicle which called this method 			
 */	
private void stopbeforeintersection(AutoVehicleSimView  testVehicle, AutoVehicleSimView  vehicle) {
		
	 double initVelocity = Math.min(vehicle.getSpec().getMaxVelocity(),
	    		vehicle.getDriver().getCurrentLane().getSpeedLimit());	
	 double stoppingDistance =
	  VehicleUtil.calcDistanceToStop(testVehicle.gaugeVelocity(),
		                                     testVehicle.getSpec().getMaxDeceleration());
	  double followingDistance = stoppingDistance + stopbeforeintersectionthreshold;
	  // if the vehicle will stop at a cell before turning then calculating distance to cell is straight forward
	  
	  double distancetointersection= vehicle.getDriver().getCurrentLane().getLaneIM().distanceToNextIntersection(testVehicle.getPosition());
	  
	  // vehicle starts to decelerate only after reaching a distance to intersection that will allow it to decelerate with a maximum value along the way until stopping before intersection
	  if (distancetointersection <= followingDistance) {
	  testVehicle.slowToStop();
	   }else if(testVehicle.getVelocity() <= initVelocity) 
			//			{
			//				if(testVehicle.getturning()==false)
							testVehicle.setAccelWithMaxTargetVelocity(testVehicle.getSpec().getMaxAcceleration()); 
							 else
						    testVehicle.setAccelWithMaxTargetVelocity(0); // don't accelerate while turning 							}
			//			}
			//				else
			//				sender.setAccelWithMaxTargetVelocity(0);
		   
	 }
	     
/**
 *  Change the vehicle's acceleration so that it stops at the right cell 
 * 
 * @param testVehicle
 * 			test Vehicle stopping
 * @param vehicle 
 * 			The original vehicle which called this method
 */
private void stopatcell(AutoVehicleSimView  testVehicle,AutoVehicleSimView vehicle) {
	  
	  double distancetocell;
	  int deptlane= vehicle.getDriver().getCurrentLane().getId();
	  String deptroad= Debug.currentMap.getRoad(vehicle.getDriver().getCurrentLane()).getName();
	  String arrivroad= vehicle.getDriver().getDestination().getName();
	  Tile turningcell=null;
	  double distancebetweenturningcellandstopcell=0;
	  double distancetoturningcell=1000; // high number;	
	  double initVelocity = Math.min(vehicle.getSpec().getMaxVelocity(),
	    		vehicle.getDriver().getCurrentLane().getSpeedLimit());
	 
	  double stoppingDistance =
	  VehicleUtil.calcDistanceToStop(testVehicle.gaugeVelocity(),
		                                     testVehicle.getSpec().getMaxDeceleration());
	  double followingDistance = stoppingDistance + thresholddistancetostop;
	  // if the testVehicle will stop at a cell before turning then calculating distance to cell is straight forward
	  
	  TiledArea tiledarea=new TiledArea(area,area.getBounds2D().getWidth()/noofcellsperside); //TODO: size of tile is hardcoded , change that
	
	  Tile stopcell =tiledarea.getTileById(testVehicle.getcelltostop());
	  int turncellId =new  CellsTrajectoryLists().getturningcell(deptroad, arrivroad, deptlane, noofcellsperside);
	  if(!(turncellId == -1))
		  turningcell = tiledarea.getTileById(turncellId);
	  
	  double distancetostopcell= Math.abs(Point2D.distance(testVehicle.getPointAtRear().getX(), testVehicle.getPointAtRear().getY(), stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY()));
	  if(!(turncellId == -1)){
	  distancetoturningcell = Math.abs(Point2D.distance(testVehicle.getCenterPoint().getX(), testVehicle.getCenterPoint().getY(), turningcell.getRectangle().getCenterX(), turningcell.getRectangle().getCenterY()));
	  distancebetweenturningcellandstopcell= Math.abs(Point2D.distance(turningcell.getRectangle().getCenterX(), turningcell.getRectangle().getCenterY(), stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY()));
	  }
	  
	  if((distancetostopcell<=distancetoturningcell) || (turncellId==-1)){
		  Point2D centerpointofstopcell = new Point2D.Double(stopcell.getRectangle().getCenterX(),stopcell.getRectangle().getCenterY());
			
			double angle = GeomMath.angleToPoint(centerpointofstopcell,vehicle.gaugePointBetweenFrontWheels());
			// Need to recenter this value to [-pi, pi]
			double Anglebetweenvehicleandcenterofcell = Util.recenter(angle - vehicle.getHeading(),
					-1.0 * Math.PI, Math.PI);
	//change-Oct-07-2014
			//		double distancetocenterofstopcell = Math.abs(Point2D.distance(stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY(), vehicle.gaugePointBetweenFrontWheels().getX(), vehicle.gaugePointBetweenFrontWheels().getY()));
		    double distancetocenterofstopcell = Math.abs(Point2D.distance(stopcell.getRectangle().getCenterX(), stopcell.getRectangle().getCenterY(), vehicle.getPointAtRear().getX(), vehicle.getPointAtRear().getY()));		
			double verticaldistancetocenterofstopcell = distancetocenterofstopcell * Math.cos(Anglebetweenvehicleandcenterofcell) ;//with respect to vehicle heading
		  
		  distancetocell = verticaldistancetocenterofstopcell;
	  }
	  // but if testVehicle will stop at a cell after turning then calculation is more complicated
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
	  
	  // vehicle starts to decelerate only after reaching a distance to cell that will allow it to decelerate with a maximum value along the way until stopping before cell 
	  if (distancetocell<= followingDistance) { 
	  testVehicle.slowToStop();
	 // System.out.println("#vehicle "+ vin + " is stoping due to cell to stop , following distance: " + followingDistance);
	   }else if(testVehicle.getVelocity() <= initVelocity) 
		//			{
		//				if(testVehicle.getturning()==false)
						testVehicle.setAccelWithMaxTargetVelocity(testVehicle.getSpec().getMaxAcceleration()); 
						 else
					    testVehicle.setAccelWithMaxTargetVelocity(0); // don't accelerate while turning 							}
		//			}
		//				else
		//				sender.setAccelWithMaxTargetVelocity(0);
	 }  
}
