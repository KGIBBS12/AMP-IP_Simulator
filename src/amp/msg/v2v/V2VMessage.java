package amp.msg.v2v;

import java.util.List;

/**
 * Main Message class
 */
public class V2VMessage {
	
	/** Vehicle unique Identifier */
	private int VIN;
	 /** The Departure road of the vehicle. */
	private String deptRoad;
	 /** The ID number of the lane in which the vehicle will depart. */
    private int departureLaneID;
	 /** The Arrival road of the vehicle. */
    private String arrivRoad;
    /** The ID number of the lane in which the vehicle will be arriving. */
    private int arrivalLaneID;
    /** When the Vehicle plans to arrive at the intersection. */
    private double arrivalTime;
    /** When the vehicle plans to exit the intersection */
    private double ExitTime;
    /** Ordered list of cell numbers which the vehicle will occupy along its trajectory */
    private List<Integer> TrajectoryCellsList;
    /** Estimated arrival times in each cell found in the Trajectory cells list*/
    private List<Double> CellsArrivalTimeList;
    /** Sequence number of message type*/
    private int MessageSequenceNumber;
    /** Type of message: Enter,Cross,or Exit*/
    private String MessageType;
    
   
    // ///////////////////////////////
    // PUBLIC METHODS
    // ///////////////////////////////
    /** Get Vehicle's unique Identifier
     * 
     * @return Vehicle's unique Identifier
     */
    public int VIN(){
    	return VIN;
    }
    /**
     * Get the lane ID number for the Lane in which the vehicle would like to
     * arrive at the intersection.
     *
     * @return the lane ID number for the Lane in which the vehicle would like
     *         to arrive at the intersection
     */
    public int getArrivalLaneID() {
      return arrivalLaneID;
    }
    /**
     * Get the ID number for the Lane in which the vehicle would like to depart
     * the intersection.
     *
     * @return the ID number for the Lane in which the vehicle would like to
     *         depart the intersection
     */
    public int getDepartureLaneID() {
      return departureLaneID;
    }
    /**
     * Get the arrival road of the vehicle
     * 
     * @return arrival road of the vehicle
     */
    public String getarrivRoad() {
        return arrivRoad;
      }
    /**
     * Get the departure road of the vehicle
     * 
     * @return departure road of the vehicle
     */
    public String getdeptRoad() {
        return deptRoad;
      }
    /**
     * Get the time at which the vehicle wants to arrive at the intersection
     *
     * @return the time at which the vehicle wants to arrive at the intersection
     */
    public double getArrivalTime() {
      return arrivalTime;
    }
    /** Get the time When the vehicle plans to exit the intersection
     * 
     * @return the time When the vehicle plans to exit the intersection
     */
    public double getExitTime() {
        return ExitTime;
      }
    /** Get the ordered list of cell numbers which the vehicle will occupy along its trajectory 
     * 
     * @return the ordered list of cell numbers which the vehicle will occupy along its trajectory 
     */
    public List<Integer> getTrajectoryCellsList() {
        return TrajectoryCellsList;
      }
    /** Get the estimated arrival times in each cell found in the Trajectory cells list 
     * 
     * @return the estimated arrival times in each cell found in the Trajectory cells list
     */
    public List<Double> getCellsArrivalTimeList() {
        return CellsArrivalTimeList;
      }
    /** Get Sequence number of message type
     * 
     * @return Sequence number of message type
     */
    public int MessageSequenceNumber(){
    	return MessageSequenceNumber;
    }
    /** Get message type
     * 
     * @return message type
     */
    public String getMessageType(){
    	return MessageType;
    }
  /////////////////////////////////
  // CONSTRUCTORS
  /////////////////////////////////

  /**
   * Basic class constructor for V2VMessage class
   * 
   * @param VIN
   *  			Vehicle unique Identifier 
   * @param deptRoad
   * 			Departure road of the vehicle.
   * @param departureLaneID
   * 		 	Departure lane of the vehicle.
   * @param arrivRoad
   * 			Arrival road of the vehicle.
   * @param arrivalLaneID
   * 			Arrival lane of the vehicle.
   * @param arrivalTime
   * 			Time at which the vehicle wants to arrive at the intersection
   * @param ExitTime
   * 			Time When the vehicle plans to exit the intersection
   * @param TrajectoryCellsList
   * 			Ordered list of cell numbers which the vehicle will occupy along its trajectory
   * @param CellsArrivalTimeList
   * 			Estimated arrival times in each cell found in the Trajectory cells list
   * @param MessageSequenceNumber
   * 			Sequence number of message type
   * @param MessageType
   * 			Message Type (Enter, Cross , or Exit)
   */
  public V2VMessage(int VIN,String deptRoad,int departureLaneID,String arrivRoad,int arrivalLaneID,double arrivalTime,double ExitTime,List<Integer> TrajectoryCellsList,List<Double> CellsArrivalTimeList,int MessageSequenceNumber,String MessageType){
  
	   this.VIN=VIN;
	   this.deptRoad= deptRoad;
	   this.departureLaneID = departureLaneID;
	   this.arrivRoad = arrivRoad;
	   this.arrivalLaneID=arrivalLaneID;
	   this.arrivalTime=arrivalTime;
	   this.ExitTime=ExitTime;
	   this.TrajectoryCellsList=TrajectoryCellsList;
	   this.CellsArrivalTimeList=CellsArrivalTimeList;
	   this.MessageSequenceNumber = MessageSequenceNumber;
	   this.MessageType = MessageType; // 1=> Enter, 2=> Cross, 3=> Exit

	  }
}    
 