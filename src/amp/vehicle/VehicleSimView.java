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
package amp.vehicle;

import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.List;

import amp.driver.AutoDriver;
import amp.msg.v2v.V2VMessage;

/**
 * The interface of a vehicle from the viewpoint of a simulator.
 */
public interface VehicleSimView extends VehicleDriverView {
/** incrementing state of the vehicle. State goes from Enter to Cross to Exit */
void incrementState(); 
/** get the state of the vehicle */	  
int getState(); 
/** set velocity of the vehicle to a certain value */
void setVelocity(double velocity); 
/** set Heading of the vehicle to a certain value */	
void setHeading(double heading);
/** set arrival time of the vehicle to the next intersection to a certain value */
void setArrivalTime(double arrivaltime);
/** get the arrival time of the vehicle */	  
double getArrivalTime();
/** set message sequence number of the vehicle, this is reset for each new message type*/	  
public void setmsgseqno(int sn);
/** get message sequence number of the vehicle */	  
public int getmsgsqnno();
/**get turning state of the vehicle to know if the vehicle is/was in the turning state*/
public boolean getturning();
/**set turning state of the vehicle */	
public void setturning();
/**clear turning state of the vehicle */	
public void clearturning();
/** get a message in the Outbox of the vehicle */	  		
public V2VMessage getV2VOutbox();
/** add a message to the Outbox of the vehicle */	  
public void setV2VOutbox(V2VMessage msg);
/** get a message in the Inbox of the vehicle */	  		
public V2VMessage getV2VInbox();
/** add a message to the Inbox of the vehicle */	  
public void setV2VInbox(V2VMessage msg);
/** get trajectory cells list of the vehicle*/	  
public List<Integer> getTrajectoryCellsList();
/** set trajectory cells list of the vehicle*/	  	
public void setTrajectoryCellsList(List<Integer> TCL);
/** get trajectory cells time list of the vehicle*/	  
public List<Double> getTrajectoryCellsTimeList();
/** Set trajectory cells time list of the vehicle*/	  
public void setTrajectoryCellsTimeList(List<Double> TCTL);
/** get a list used to store the TIC and VIN of other vehicles processed messages */	  
public List<double[]> getTIC_VIN_List_1();
/** get a list used to store the TIC and VIN of other vehicles processed messages */	  
public List<double[]> getTIC_VIN_List_2();
/** add an element of TIC and VIN and safetimetocross to TIC_VIN_1 list of the vehicle*/	  
public void addElementtoTIC_VIN_List_1(double[] TIC_VIN);
/** add an element of TIC and VIN and safetimetocross to TIC_VIN_2 list of the vehicle*/	  
public void addElementtoTIC_VIN_List_2(double[] TIC_VIN);
/** Delete TIC_VIN list 1*/
public void emptyTIC_VIN_List_1();
/** Delete TIC_VIN list 2*/
public void emptyTIC_VIN_List_2();
/** get cell that the vehicle should stop at in order not to collide with another vehicle*/
public int getcelltostop();
/** set cell that the vehicle should stop at in order not to collide with another vehicle*/
public void setcelltostop(int cts);
/** Get time the vehicle can start moving to the next cell*/
public double gettimetogotonextcell();
/** Set time the vehicle can start moving to the next cell*/
public void settimetogotonextcell(double time);
/** Get current cell the vehicle occupies */
public int getcurrentcell();
/** Set current cell the vehicle occupies */
public void setcurrentcell(int cc);
/** Get turning state of the vehicle for this time step  */
public boolean getturnthistimestep();
/** Change turning state of the vehicle for this time step */
public void changeturnthistimestep();
/** Set turning state of the vehicle for this time step */
public void setturnthistimestep(boolean ttts);
/** Set steering angle to turn the vehicle */
public void setSteeringAngle(double angle);
/** Get the current steering angle of the vehicle */
public double getSteeringAngle();
/** Get the earliest (first) trajectory cells list of the vehicle */
public List<Integer> getfirsttrajectorylist();
/** Set the earliest (first) trajectory cells list of the vehicle */
public void setfirsttrajectorylist(List<Integer> FTL);
/** Get vehicle's Exit time from intersection */
public double getExitTime();
/** Set vehicle's Exit time from intersection */
public void setExitTime(double exittime);
/** Get vehicle's collision list which includes the VIN of all vehicles that collided with this vehicle */
public List<Integer> getcollisionlist();
/** Add an element (VIN of colliding vehicle) to vehicle's collision list */
public void addElementtocollisionlist(int vin);
/** remove an element (VIN of colliding vehicle) from vehicle's TIC_VIN list */
public void removeElementFromTIC_VIN_List_1(int index);
public VehicleSimView getNextVehicle();
public void setNextVehicle(VehicleSimView nextvehicle);
public void addDistanceMoved(double distance);
public double getDistanceMoved();
  /**
   * Set the VIN number of this Vehicle.
   *
   * @param vin the vehicle's VIN number.
   */
  void setVIN(int vin);

  /**
   * Set this Vehicle's Driver.
   *
   * @param driver  the new driver to control this Vehicle
   */
  void setDriver(AutoDriver driver);

  /**
   * Check whether this vehicle's time is current.
   *
   * @param currentTime  the current time
   */
  void checkCurrentTime(double currentTime);

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // states

  /**
   * Get the position of the vehicle.
   *
   * @return the position of the vehicle
   */
  Point2D getPosition();

  /**
   * Get the heading of the vehicle
   *
   * @return the heading of the vehicle
   */
  double getHeading();

  /**
   * Get the velocity of the vehicle
   *
   * @return the velocity of the vehicle
   */
  double getVelocity();

  /**
   * Get the acceleration of the vehicle
   *
   * @return the acceleration of the vehicle
   */
  double getAcceleration();


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // derived properties

  /**
   * Get a {@link Shape} describing the Vehicle.
   *
   * @return a Shape describing this Vehicle.
   */
  Shape getShape();

  /**
   * Get a {link Shape} describing this Vehicle, if it were larger in each
   * dimension.
   *
   * @param extra the fixed extra amount by which to increase the size of the
   *              Vehicle in each dimension
   * @return      a Shape describing a Vehicle larger in each dimension by the
   *              extra amount.
   */
  Shape getShape(double extra);

  /**
   * Get the edges that represent the boundaries of this Vehicle.
   *
   * @return an array of line segments that represent the edges of the
   *         Vehicle.
   */
  List<Line2D> getEdges();

  /**
   * Get the Shapes of each of the wheels.
   *
   * @return an array of wheel Shapes: front left, front right, rear left,
   *         rear right
   */
  Shape[] getWheelShapes();

  /**
   * Get the point in front of the middle point of the vehicle that is
   * at the distance of delta away from the vehicle.
   *
   * @param delta   the distance of the vehicle and the point
   *
   * @return the projected point
   */
  Point2D getPointAtMiddleFront(double delta);

  /**
   * Get the location of the center of the Vehicle at this point in time.
   *
   * @return the global coordinates of the center of the Vehicle.
   */
  Point2D getCenterPoint();

  /**
   * Get the current global coordinates of the corners of this Vehicle.
   *
   * @return an array of points representing the four corners.
   */
  Point2D[] getCornerPoints();

  /**
   * Get the point at the rear center of the Vehicle.
   *
   * @return the global coordinates of the point at the center of the
   * Vehicle's rear
   */
  Point2D getPointAtRear();


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // messaging

  /**
   * Get the queue of V2I messages waiting to be delivered from this
   * Vehicle.
   *
   * @return the queue of V2I messages to be delivered from this Vehicle
   */
//  Queue<V2IMessage> getV2IOutbox();

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // controls

  /**
   * Move a single Vehicle according to some approximation of the laws
   * of physics.
   *
   * @param timeStep the size of the time step to simulate, in seconds
   */
  void move(double timeStep);

}
