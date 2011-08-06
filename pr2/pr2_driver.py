#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: PR2_DRIVER.PY
Date: Monday, June 20 2011
Description: Publishes ds commands to r_cart/command_pose.
"""

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('seds')
import driver

import tf
import rospy
import rospy.rostime as rostime
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

from seds.srv import DSSrv
from seds.srv import DSLoaded
from seds.srv import FloatSrv, IntSrv, StringSrv
from seds.srv import SedsModel
from std_srvs.srv import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy
import getopt
import sys
npa = numpy.array

import threading

class PR2Driver(driver.Driver):

    def init_subscriber(self):
        self.listener = tf.TransformListener()

    def init_publisher(self):
        self.rpub = rospy.Publisher('r_cart/command_pose', PoseStamped)
        self.lpub = rospy.Publisher('l_cart/command_pose', PoseStamped)


    def __init__(self, name, vm, feedback, rate, thresh, source_frameid, target_frameid, msfid, mtfid, waittf):

        driver.Driver.__init__(self,name, vm, feedback, rate)

        self.source_frameid = source_frameid
        self.target_frameid = target_frameid
        self.model_source_frameid = msfid
        self.model_target_frameid = mtfid
        self.adaptive_threshold=thresh
        self.hand="right"

        # wait for the proper /tf transforms
        if waittf:
            self.wait_for_transform(source_frameid, target_frameid)

        self.cmd = PoseStamped()
        self.cmd.header.frame_id = "/" + source_frameid

        # for changing objects
        self.msfidSRV = rospy.Service("/%s/change_object" % name, StringSrv, self.change_object)
        self.handSRV = rospy.Service("/%s/change_hand" % name, Empty, self.change_hand)

    def change_object(self, rec):
        self.runningCV.acquire()
        self.model_source_frameid = str(rec.value)
        self.runningCV.release()
        rospy.loginfo("Changing object to %s" % self.model_source_frameid)
        return []

    def change_hand(self, ignore):
        self.runningCV.acquire()
        if self.hand=="right":
            self.hand="left"
            self.target_frameid=self.ltfid
        else:
            self.hand="right"
            self.target_frameid=self.rtfid
        rospy.loginfo("Changing hand to %s" % self.hand) 
        self.runningCV.release()
        return []

    def wait_for_transform(self,sfid, tfid):
        """
        Blocks until a transform is found.
        """

        rospy.loginfo('Waiting for transform...')
        tfound = False
        #while not tfound:
        try:
            self.listener.waitForTransform(source_frame=sfid, target_frame=tfid,time=self.zerot,timeout=rostime.Duration(10))
            tfound = True # no exception
            rospy.loginfo('Transform found!')
        except tf.Exception, error:
            rospy.logwarn(error)
        return tfound

    def init_start(self):

        self.model = self.dsparams()
        self.endpoint = npa(self.model.model.offset)[:self.model.model.dim/2]
        self.dT = self.model.model.dT
        #if these are empty, assume the same as what were trained with
        if self.model_source_frameid=="":
            self.model_source_frameid = self.model.model.source_fid
        #if self.model_target_frameid=="":
        #    self.model_target_frameid = self.model.model.target_fid
        self.model_target_frameid=self.target_frameid

        cntl_dt = 1.0 / float(self.rateInt)
        self.tscale =  cntl_dt / self.dT

        rospy.loginfo("model dt: %s cntl dt: %s tscale : %s" % (self.dT, cntl_dt, self.tscale))
        rospy.loginfo("Using endpoint %s" % str(self.endpoint))
        rospy.loginfo("Using model sid: %s fid: %s and controller sid: %s fid: %s" % (self.model_source_frameid,
                                                                                      self.model_target_frameid,
                                                                                      self.source_frameid,
                                                                                      self.target_frameid))
        # nothing will work if this is not true!
        #assert self.model_target_frameid == self.target_frameid

        # model source is typically an object, controller source is something like torso_lift_link

        # init some variables (in model frame)
        ret=self.wait_for_transform(self.model_source_frameid, self.model_target_frameid)
        if ret!=True:
            return False
        et = self.listener.lookupTransform(self.model_source_frameid, self.model_target_frameid, self.zerot)
        self.x = et[0][:]+euler_from_quaternion(et[1][:])
        self.newx = self.x
        return True

    def get_current_position(self):
        # feedback for input into seds
        try:
            t = self.listener.getLatestCommonTime(self.model_source_frameid,self.model_target_frameid)
            #if rospy.Time.now()-t < rospy.Duration.from_sec(1):
            et = self.listener.lookupTransformFull(self.model_source_frameid, t, self.model_target_frameid, rostime.Time(0),"torso_lift_link")
            pos = et[0][:] # pos (x,y,z)
            quat = et[1][:]
            eu = euler_from_quaternion(quat)
            return pos + eu
            #else:
            #    #print "Haven't seen anything for 1 sec, using no feedback"
            #    return self.newx

        except tf.Exception, error:
            rospy.logwarn("%s tf exception in gcp: %s!" % (self.name,error))
            
        # if we have an exception just return the previously computed pose!
        #assumes arm moved correctly?
        return self.newx

    def publish(self):
        rospy.logdebug("x : %s dx : %s newx : %s" % (str(self.x), str(self.dx), str(self.newx)))

        # need to transform the new position newx into self.source_frameid reference
        model_command = PoseStamped()
        model_command.header.frame_id = "/" + self.model_source_frameid

        model_command.pose.position.x = self.newx[0]
        model_command.pose.position.y = self.newx[1]
        model_command.pose.position.z = self.newx[2]
        quat=quaternion_from_euler(self.newx[3],self.newx[4],self.newx[5])
        model_command.pose.orientation.x=quat[0]
        model_command.pose.orientation.y=quat[1]
        model_command.pose.orientation.z=quat[2]
        model_command.pose.orientation.w=quat[3]

        # Transfrom the source_frameid from object -> torso_lift_link or some other frame that JTTeleop knows about!
        try:
            #control_command = self.listener.transformPoint(self.source_frameid, model_command)
            control_command=self.listener.transformPose(self.source_frameid,model_command)
       
            rospy.logdebug("model_command %s control_command %s" % (str(model_command), str(control_command)))
            self.cmd.pose.position.x=control_command.pose.position.x
            self.cmd.pose.position.y=control_command.pose.position.y
            self.cmd.pose.position.z=control_command.pose.position.z
            self.cmd.pose.orientation.x=control_command.pose.orientation.x
            self.cmd.pose.orientation.y=control_command.pose.orientation.y
            self.cmd.pose.orientation.z=control_command.pose.orientation.z
            self.cmd.pose.orientation.w=control_command.pose.orientation.w

            #self.cmd.pose.position.x = control_command.point.x
            #self.cmd.pose.position.y = control_command.point.y
            #self.cmd.pose.position.z = control_command.point.z
            

        # just use the last tf pose orientations
         #   ct = self.listener.lookupTransform(self.source_frameid, self.target_frameid,rostime.Time(0))
         #   self.rot = list(ct[1][:])
         #   self.cmd.pose.orientation.x = self.rot[0]
         #   self.cmd.pose.orientation.y = self.rot[1]
         #   self.cmd.pose.orientation.z = self.rot[2]
         #   self.cmd.pose.orientation.w = self.rot[3]

            if self.hand=="right":
                self.rpub.publish(self.cmd)
            else:
                self.lpub.publish(self.cmd)

        except tf.Exception:
            rospy.logdebug("%s tf exception in publish!" % self.name)

def main():

    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'v:f:s:t:a:', ['vm=','feedback=','source=','target=','athresh='])

    rospy.init_node('pr2_driver')

    rsource_frameid = rospy.get_param("/r_cart/root_name","torso_lift_link")
    rtarget_frameid = rospy.get_param("/r_cart/tip_name","r_gripper_tool_frame")
    lsource_frameid = rospy.get_param("/l_cart/root_name","torso_lift_link")
    ltarget_frameid = rospy.get_param("/l_cart/tip_name","l_gripper_tool_frame")
    #both left and right hands must be in the same reference frame
    assert lsource_frameid == rsource_frameid
    
    vm = rospy.get_param("/pr2_driver/velocity_multiplier", 1)
    feedback = rospy.get_param("/pr2_driver/feedback", 'adaptive')
    msfid=""
    mtfid=""
    adaptive_threshold=0.1

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            assert a in ('none','hard','adaptive')
            feedback = a
        elif o in ('-s','--source'):
            msfid = a
        elif o in ('-t','--target'):
            mtfid = a
        elif o in ('-a','--athresh'):
            adaptive_threshold = float(a)

    driver = PR2Driver("pr2_driver", vm, feedback, 100, adaptive_threshold, rsource_frameid, rtarget_frameid, msfid, mtfid,False) # start node
    #driver.adaptive_threshold=adaptive_threshold
    driver.ltfid=ltarget_frameid
    driver.rtfid=rtarget_frameid
    driver.spin()

if __name__ == '__main__':
    main()
