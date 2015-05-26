package amp.intersection.area.Handler;
///*
//Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
//University of Texas at Austin
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//this list of conditions and the following disclaimer in the documentation
//and/or other materials provided with the distribution.
//
//3. Neither the name of the University of Texas at Austin nor the names of its
//contributors may be used to endorse or promote products derived from this
//software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package amp.intersection.area.Handler;
//
//import amp.intersection.area.policy.BasePolicyCallback;
//import amp.msg.i2v.Reject;
//import amp.msg.v2i.Request;
//import amp.sim.StatCollector;
//
///**
// * The all stop request handler
// */
//public class AllStopRequestHandler implements RequestHandler {
//
//  /////////////////////////////////
//  // PRIVATE FIELDS
//  /////////////////////////////////
//
//  /** The base policy */
//  private BasePolicyCallback basePolicy;
//
//
//  /////////////////////////////////
//  // PUBLIC METHODS
//  /////////////////////////////////
//
//  /**
//   * Set the base policy call-back.
//   *
//   * @param basePolicy  the base policy's call-back
//   */
//  @Override
//  public void setBasePolicyCallback(BasePolicyCallback basePolicy) {
//    this.basePolicy = basePolicy;
//  }
//
//  /**
//   * Let the request handler to act for a given time period.
//   *
//   * @param timeStep  the time period
//   */
//  @Override
//  public void act(double timeStep) {
//    // do nothing
//  }
//
//  /**
//   * Process the request message.
//   *
//   * @param msg the request message
//   */
//  @Override
//  public void processRequestMsg(Request msg) {
//    basePolicy.sendRejectMsg(msg.getVin(),
//                             msg.getRequestId(),
//                             Reject.Reason.NO_CLEAR_PATH);
//  }
//
//  /**
//   * Get the statistic collector.
//   *
//   * @return the statistic collector
//   */
//  @Override
//  public StatCollector<?> getStatCollector() {
//    return null;
//  }
//
//}
