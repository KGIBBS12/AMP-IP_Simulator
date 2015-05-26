//
// TIC V,Y : Trajectory Intersecting Cell between the
// higher-priority vehicle V and lower-priority vehicle Y.
// AT V,c : Arrival Time of vehicle V to cell c.
// ET V,c : Exit Time of vehicle V from cell c.
// Θ: The Safety Time Interval.
// P|V: priority of vehicle V

//Algorithm 3 AMP-IP, Receiver Vehicle
//Input: Safety message received from vehicle A: RM
//Output: Vehicle B’s movement at the intersection
//if RM = ENTER or RM = CROSS then
//Run CDAI to detect trajectory conflicts with vehicle A
//and find TIC A,B
//if (TIC A,B = NULL) then
//Cross the intersection
//else
//Use FCFS priority policy
//if (P|B > P|A ) then
//Cross the intersection
//else
//c = TIC A,B
//if ([AT B,c + Θ] < AT A,c ) then
//Cross the intersection
//else
//Progress and stop before entering TIC A,B
//else if RM = EXIT then
//if TIC A,B is cleared then
//Cross the intersection

// source: http://users.ece.cmu.edu/~sazimi/ICCPS2013.pdf
// Note: there is a typo in the protocol statement 'if ([AT B,c + Θ] < AT B,c )' and
// was corrected to be 'if ([AT B,c + Θ] < AT A,c )'


package amp.policy;

import java.util.ArrayList;
import java.util.List;

import amp.config.Debug;
import amp.msg.v2v.V2VMessage;
import amp.vehicle.AutoVehicleSimView;

public class policy {

int indexofarrivaltimetoconflictingcellofsender;
int indexofarrivaltimetoconflictingcellofreceiver;
int indexofarrivaltimetonextcellafterconflictingcellofreceiver;
double OriginalSafetyTimeInterval= 2;  // As mentioned in the AMP paper
List<Integer> TIClist = new ArrayList<Integer>();

public void runpolicy(AutoVehicleSimView vehicle,V2VMessage message){
		 
	double SafetyTimeInterval=OriginalSafetyTimeInterval;  // We doubled the original safety time interval
	
	if(message.getMessageType().equals("ENTER") || message.getMessageType().equals("CROSS"))
	{
		
		//receiver should not stop for vehicles in case sender or receiver have no more cells in their TCL
		if (vehicle.getTrajectoryCellsList().size()==0 || message.getTrajectoryCellsList().size()==0){
			removeVehicleFromList(vehicle,message,true);
			return; 
		}
	

		if(message.getDepartureLaneID()==vehicle.getDriver().getCurrentLane().getId())
		{
//now		if(message.getExitTime()!=Double.POSITIVE_INFINITY)	
//		{
//			removeVehicleFromList(vehicle,message,true);
//		    return;
//		}
		}	
		
    findTrajectoryIntersectionConflictCell(message.getTrajectoryCellsList(),vehicle.getTrajectoryCellsList());	
	// if TIC is null then don't stop for the sender	
    if (TIClist.size()==0){ // no conflicting cell
			removeVehicleFromList(vehicle,message,true);
			return; 
		}

		//		if((indexofarrivaltimetoconflictingcellofreceiver==0) && (vehicle.getState()>1)){ //in cross or exit , receiver has a conflicting cell which he is currently occupying
//			System.out.println("#vehicle "+vehicle.getVIN() + " has currentcell " +vehicle.getTrajectoryCellsList().get(0) +" as conflicting cell with vehicle "+ message.VIN()+ ", so vehicle will ignore message, this is an issue of the protocol design");
//			return -1;
//		}
		
		
		//check priorities as follows:
		//1. Vehicle which advertises an earlier time for arriving at the intersection is given higher priority
		//2. vehicle with lower VIN (vehicle ID) has higher priority 
		
		if(vehicle.getArrivalTime() < message.getArrivalTime())
		{
		//	receiver has higher priority due to earlier arrival time so it crosses
			removeVehicleFromList(vehicle,message,true);
			return;
		}
		if(vehicle.getArrivalTime() == message.getArrivalTime()){
			
//			if(message.getArrivalTime()!=Double.POSITIVE_INFINITY)
//			System.err.println("arrival time is same: " + vehicle.getArrivalTime());
			
			if(message.getArrivalTime()==Double.POSITIVE_INFINITY)
			{				
				removeVehicleFromList(vehicle,message,true);
				return;
			}
		
			String deptroadsender= message.getdeptRoad();
			String arrivroadsender= message.getarrivRoad();
			String deptroadreceiver= Debug.currentMap.getRoad(vehicle.getDriver().getCurrentLane()).getName();
			String arrivroadreceiver= vehicle.getDriver().getDestination().getName();
			
			if(deptroadreceiver.equals(arrivroadreceiver) && !(deptroadsender.equals(arrivroadsender)))
			{				
				removeVehicleFromList(vehicle,message,true);
				return;
			}
				
			if( ((deptroadreceiver.equals(arrivroadreceiver)) && (deptroadsender.equals(arrivroadsender)) ) || (!(deptroadreceiver.equals(arrivroadreceiver)) && !(deptroadsender.equals(arrivroadsender)) ))
			{
				if(vehicle.getVIN()< message.VIN())
			{
				removeVehicleFromList(vehicle,message,true);
				return;
			}
			}
		}	
		
	
		
		//checking if arrival time of receiver to conflicting cell allows it to pass before sender arrives to that cell  
		// first of all we wont pass if the arrival time of the sender to the conflicting cell is not known (Infinity)
		if(message.getCellsArrivalTimeList().get(indexofarrivaltimetoconflictingcellofsender) != Double.POSITIVE_INFINITY){
		if( vehicle.getTrajectoryCellsTimeList().get(indexofarrivaltimetoconflictingcellofreceiver) + SafetyTimeInterval< message.getCellsArrivalTimeList().get(indexofarrivaltimetoconflictingcellofsender))
		{	
			if (indexofarrivaltimetonextcellafterconflictingcellofreceiver != -1) // as TIC is last cell in TCL so the time to leave the cell is the exit time from the intersection
			{
				return;
			//	if(vehicle.getExitTime() != -1)  
			//	return -1; // receiver can cross the intersection passing by the conflicting cell much earlier than the sender
			}else {
			//	if(vehicle.getTrajectoryCellsTimeList().get(indexofarrivaltimetonextcellafterconflictingcellofreceiver)  != -1.0) 							
				if(vehicle.getExitTime() != Double.POSITIVE_INFINITY)
				return; // receiver can cross the intersection passing by the conflicting cell much earlier than the sender
			}
		}
		}
		//stop for the vehicle otherwise
		// check first if the vehicle that is colliding and its TIC are already included in the list or not, if not add them
		
//		if(vehicle.getState()==1){ // only in enter state will we add vehicles to stop for
//		if(TIClist.size()==1){ // only one conflicting cell
			if(vehicle.getTIC_VIN_List_1().contains(new double[] {TIClist.get(0), message.VIN()}))
				return;
			else
			{
				// remove any old TIC with that vehicle and add the new one
				removeVehicleFromList(vehicle,message,true);
				
				vehicle.addElementtoTIC_VIN_List_1(new double[] {TIClist.get(0), message.VIN()});	
			
			}
				//		}else  // more than one conflicting cell
//		{
//			for(int p=0; p<TIClist.size();p++)
//			{
//				if(!(vehicle.getTIC_VIN_List_1().contains(new double[] {TIClist.get(p), message.VIN()})))
//					vehicle.addElementtoTIC_VIN_List_1(new double[] {TIClist.get(p), message.VIN()});	
//				
//			}
//			
//		}
		
//		}		
		
	}else if(message.getMessageType().equals("EXIT")){
		
		//if TIC A,B is cleared then Cross the intersection

		// This is done automatically when vehicles send Exit messages, 
		// as vehicles receiving those messages find empty cell lists in
		// those messages so they decide that they have no conflicting cells 
		// with those vehicles so they cross the intersection 
		
		removeVehicleFromList(vehicle,message,true);
	}
	
	return;
}	
	

private void findTrajectoryIntersectionConflictCell(List<Integer> otherlist,List<Integer> mylist){
	
	// In the AMP protocoḷ its not mentioned which conflicting cell is TIC , but logicaly we will take it as
	//the first conflicting cell between both lists starting by the receiver's list (the vehicle receiving the message)
	OUTERLOOP:
		for(int j=0 ; j<mylist.size();j++ ) // The earliest cell having collision in the receiver's list is the conflicting cell 
		for(int i=0 ; i<otherlist.size();i++ )
		{
		if (otherlist.get(i)==mylist.get(j)){
			TIClist.add(mylist.get(j));
			indexofarrivaltimetoconflictingcellofsender=i;
			indexofarrivaltimetoconflictingcellofreceiver=j;
			if(j+1<mylist.size())
			indexofarrivaltimetonextcellafterconflictingcellofreceiver=j+1;
			else
			indexofarrivaltimetonextcellafterconflictingcellofreceiver=-1; // signaling exit time
	
			break OUTERLOOP;
		
		}
		}
		
}


//check if this is a vehicle that we decided to stop for, if that is the case check if any of the conflicting cell(s) of that vehicle were cleared
// starting by the first conflicting cell
//We stored all conflicting cells of the sender vehicle in an ascending order before, so now the first element in the list containing the sender 
// vehicle ID will be returned, as if the receiver removed the conflicting cells that it has with the sender it will do it one by one
// we can choose to remove all conflicting cells (in case vehicle exits the intersection) or remove one conflicting cell (the earliest in the list)
private void removeVehicleFromList(AutoVehicleSimView vehicle,V2VMessage message, boolean allconflictingcells) {		
	
	List<Integer> collisioncellslist = new ArrayList<Integer>();
	List<double []> temp = new ArrayList<double []>();
	
	if(vehicle.getTIC_VIN_List_1().size() !=0){
			
		for(int i =0;i<vehicle.getTIC_VIN_List_1().size();i++)	
		{
			if(vehicle.getTIC_VIN_List_1().get(i)[1]==(double)message.VIN()) 
			{
			
			boolean cleared = true;
			
			for(int j=0; j<vehicle.getTrajectoryCellsList().size();j++)
			{
				if(vehicle.getTrajectoryCellsList().get(j)==(int)vehicle.getTIC_VIN_List_1().get(i)[0])
				{
					if(allconflictingcells == false){
						vehicle.removeElementFromTIC_VIN_List_1(i);
						return;
					}
					collisioncellslist.add(i);
				}
				
			}
			
		}
		}
			
		if(collisioncellslist.size() > 0)
		{
			for(int y=0;y<vehicle.getTIC_VIN_List_1().size();y++)
			{
				if(!(collisioncellslist.contains(y)))
				temp.add(vehicle.getTIC_VIN_List_1().get(y));
				
			}
			
			vehicle.emptyTIC_VIN_List_1();
			for(int h=0;h<temp.size();h++)
			vehicle.addElementtoTIC_VIN_List_1(temp.get(h));	
			
		}
			
		}	
}

}
