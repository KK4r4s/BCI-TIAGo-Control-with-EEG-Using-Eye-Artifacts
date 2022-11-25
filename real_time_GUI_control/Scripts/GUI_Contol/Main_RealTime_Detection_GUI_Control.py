# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 14:35:19 2022
This is the Updated Version for easy access for multiple choses.
        
23/09/22  -> Starts Parallel Processing of Multiple Scripts
          -> "_ImpPlot=True" Plots the Impedance Plot
          -> "_SRT_Plot=True" Plots the Semi Real Time Data
          -> "_save_Event=True" Save the detected events and its time stamps.
          -> " _save_Imp=True" Save the recorded impedance values during the setup
 
@author: Lenovo
"""


if __name__=="__main__":   
    from multiprocessing import Process, Queue
    
    import sys
    sys.path.append("../../")
    from TMSiSDK import kaan_gui
    from TMSiSDK import kaan_GUI_Distributer
    from TMSiSDK import kaan_TMSi_Initialization
    
    queueS1_S2 = Queue()  # s1 to s2 communication # s1.main() writes to queueS1_S2
    queueS2_S1 = Queue()  # S2 to S1 communication # s2.stage2() writes to queueS2_S1
    queueS3_S2 = Queue()    # S3 to S2 communication # s2.stage2() writes to queueS3_S1
    queueS2_S3 = Queue()    # s2 to s3 communication # s1.stage1() writes to queueS1_S3
    
    # # # # Multiprocessing   
    s1= kaan_gui.GUI()
    s1 = Process(target=s1.main, args=(queueS1_S2, queueS2_S1))
    s1.daemon = True
    s1.start()     # Launch the s1 (GUI) process 
    
    s2= kaan_GUI_Distributer.distributer()
    s2 = Process(target=s2.deliver, args=(queueS1_S2, queueS2_S1,queueS2_S3, queueS3_S2))
    s2.daemon = True
    s2.start()     # Launch the s2 (distirbuter) process
    
    # Initialise filter and Measurement Build in and check Impedance Values
    s3 = kaan_TMSi_Initialization.TMSi_Connect(_ImpPlot=True, _SRT_Plot=True, _save_Event=True, _save_Imp=True)  
    s3.Start_Measuring(queueS2_S3, queueS3_S2) # start sending stuff from s1 to s2 
    s1.join() # wait till s1 daemon finishes
    s2.join() # wait till s2 daemon finishes
    