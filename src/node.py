#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import matplotlib.pyplot as plt
import random
import rospkg
import os
import numpy as np

from mind_msgs.msg import LearningFigure, ResponseProb
from scipy.interpolate import spline

class Plot():

    def __init__(self):
        
        self.path = os.path.join(
                rospkg.RosPack().get_path('learning_plotting_ros'), 'figure')
        
        # For learning plot
        self.iteration = []
        self.accuracy = []
        self.loss = []

        # For response probailities graph
        self.fig_number = 1    

        rospy.Subscriber('learning', LearningFigure, self.handle_update)
        rospy.Subscriber('probs', ResponseProb, self.handle_probs_update)
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
            
            # save learning figure
            # plt.savefig("%s/learning.png"%self.path, dpi=350)
            plt.show()

            rospy.signal_shutdown('record completed.')
    
    def handle_probs_update(self, msg):
        if msg.resp_prob != []:
            y_probs = msg.resp_prob
            x_name = [i for i, p in enumerate(y_probs)]
            n_groups = len(x_name)

            index = np.arange(n_groups)
            bar_width = 0.35
            opacity = 0.5  

            plt.bar(index, y_probs, bar_width,
                    tick_label=x_name, align='center',
                    alpha=opacity, color='b', label='probs')

            plt.xlabel('Responses on the action template')
            plt.ylabel('Probability')
            plt.title('Response Space Distribution')
            plt.xlim(-1, n_groups)
            plt.ylim(0, 1)

            # save learning figure
            plt.savefig("%s/prob_fig %i.png"%(self.path, self.fig_number), dpi=350)
            self.fig_number += 1

            plt.show()
        
        else:
            rospy.logerr('no data coming from ...')


if __name__ == '__main__':
    rospy.init_node('learning_plotting', anonymous=False)
    p = Plot()
    rospy.spin()
    

    

