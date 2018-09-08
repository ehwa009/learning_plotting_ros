#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import matplotlib.pyplot as plt
import random
import rospkg
import os

from mind_msgs.msg import LearningFigure

class Plot():

    def __init__(self):
        
        self.iteration = []
        self.accuracy = []
        self.loss = []

        rospy.Subscriber('learning', LearningFigure, self.handle_update)
        rospy.loginfo('\033[94m[%s]\033[0m initialized.'%rospy.get_name())

    def handle_update(self, msg):
        if msg.iteration != -1:
            self.iteration.append(msg.iteration)
            self.accuracy.append(msg.accuracy)
            self.loss.append(msg.loss)
        else:
            # draw graph
            plt.plot(self.iteration, self.accuracy,
                    linestyle='-',
                    linewidth=2,
                    color='deepskyblue')
            plt.plot(self.iteration, self.loss,
                    linestyle='--',
                    linewidth=2,
                    color='hotpink')

            plt.title("Learning Rate")
            plt.xlabel("Iteration")
            plt.ylabel("Accuracy/loss")
            plt.grid(True)
            plt.xlim([0, len(self.iteration)])
            plt.ylim([0, self.loss[0]])
            
            path = os.path.join(
                rospkg.RosPack().get_path('learning_plotting_ros'), 'figure')
            
            # save learning figure
            # plt.savefig("%s/learning.png"%path, dpi=350)
            plt.show()

            rospy.signal_shutdown('record completed.')


if __name__ == '__main__':
    rospy.init_node('learning_plotting', anonymous=False)
    p = Plot()
    rospy.spin()
    

    

