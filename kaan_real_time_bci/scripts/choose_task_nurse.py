#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import math
#import copy
import numpy as np

# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import String

from email.message import EmailMessage
import ssl 
import smtplib

class CallNurse(object):
    def __init__(self, hz = 10.0):
        
        rospy.loginfo('Waiting for connection...')

        self.sub_nurse = rospy.Subscriber('/socket_distributer_nurse/data',String, self.send_mail) 
    
    def send_mail(self,msg): # msg name not important based on me    
        email_sender = "expkpolimi@gmail.com"
        email_password = "gmhu wtas fove nirf"

        email_receiver = "kaan1896karas@gmail.com"

        subject="Emergency"
        body="""
        Call Caregiver! 
        """

        em =EmailMessage()
        em['From']=email_sender
        em['To']= email_receiver
        em['subject']= subject
        em.set_content(body)

        context = ssl.create_default_context()
        try:
            with smtplib.SMTP_SSL('smtp.gmail.com',465,context=context) as smtp:
                smtp.login(email_sender, email_password)
                smtp.sendmail(email_sender,email_receiver,em.as_string())
            print("Mail sent")
        except:
            print("Error: unable to send email")

if __name__ == '__main__':
    
    rospy.init_node('socket_server', anonymous = True)
    
    mr = CallNurse()
    try:
        while not rospy.is_shutdown():
            pass
    
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down the node.')
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down the node.')
