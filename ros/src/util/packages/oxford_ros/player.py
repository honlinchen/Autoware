#!/usr/bin/python

import sdk
import time
import rospy
import cv_bridge
from tf import transformations

from geometry_msgs.msg import PoseStamped as PoseMsg
from sensor_msgs.msg import Image as ImageMsg
from rosgraph_msgs.msg import Clock



class ImagePlayer:
    def __init__ (self, dataset):
        self.publisher = rospy.Publisher ('/oxford/image', ImageMsg, queue_size=1)
        cam_ts = dataset.getStereo()
        self.cameraModel = sdk.CameraModel ('models', sdk.CameraModel.cam_stereo_center)

    def _getEvents (self):
        pass
    
    def _passEvent (self, timestamp, eventId):
        pass


class PosePlayer:
    def __init__ (self, dataset):
        self.poses = dataset.getIns()
        self.publisher = rospy.Publisher ('/oxford/pose', PoseMsg, queue_size=1)
    
    def _getEvents (self):
        eventList = [{'timestamp':self.poses[p,0], 'id':p} for p in range(len(self.poses))]
        return eventList
    
    def _passEvent (self, timestamp, eventId):
        poseRow = self.poses[eventId]
        curPose = PosePlayer.createPoseFromRPY(poseRow[1], poseRow[2], poseRow[3], poseRow[4], poseRow[5], poseRow[6])
        curPose.header.stamp = rospy.Time.from_sec(poseRow[0])
        curPose.header.frame_id = 'world'
        self.publisher.publish(curPose)
        
    @staticmethod
    def createPoseFromRPY (x, y, z, roll, pitch, yaw):
        p = PoseMsg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        qt = transformations.quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation.x = qt[0]
        p.pose.orientation.y = qt[1]
        p.pose.orientation.z = qt[2]
        p.pose.orientation.w = qt[3]
        return p


class Player:
    def __init__ (self, datadir, rate=1.0):
        self.rate = float(rate)
        self.eventList = []
        self.dataset = sdk.Dataset(datadir)
        self.players = []
#         self.clockPub = rospy.Publisher ('/clock', Clock, queue_size=1)
        
    def add_data_player (self, _dataPlayer):
        self.players.append(_dataPlayer)
    
    def run (self):
        self.initRun()
        try:
            for i in range(len(self.eventList)):
                curEvent = self.eventList[i]
                t1 = curEvent['timestamp']
                curEvent['object']._passEvent (curEvent['timestamp'], curEvent['id'])
                if i<len(self.eventList)-1 :
                    t2 = self.eventList[i+1]['timestamp']
                    delay = (t2 - t1) / self.rate
#                     self.publishClock(curEvent['timestamp'])
                    time.sleep(delay)
                if (rospy.is_shutdown()):
                    break
        except KeyboardInterrupt:
            print ("Interrupted")
            return
        
    def publishClock (self, t):
        ct = Clock()
        ct.clock = rospy.Time.from_sec(t)
        self.clockPub.publish(ct)
    
    def initRun (self):
        for player in self.players:
            eventsInThis = player._getEvents ()
            if len(eventsInThis)==0:
                continue
            for evt in eventsInThis:
                e = {'timestamp': evt['timestamp'], 'id':evt['id'], 'object':player}
                self.eventList.append(e)
        self.eventList.sort(key=lambda e: e['timestamp'])
        
        
if __name__ == '__main__' :
    import sys
    rospy.init_node('oxford_player', anonymous=True)
    player = Player (sys.argv[1])
    poses = PosePlayer (player.dataset)
    images = ImagePlayer(player.dataset)
    player.add_data_player(poses)
    player.add_data_player(images)
    player.run()

