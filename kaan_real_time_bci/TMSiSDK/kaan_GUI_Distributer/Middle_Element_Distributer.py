# -*- coding: utf-8 -*-
"""
Created on Mon Sep 26 15:33:40 2022


This distributer contains a special case condition, if user waits more than 
1.5 seconds after successfully doing a double or triple blink, the algorithm
won't wait for the next event but sends the double blink event. Since 1.5 
seconds is enough for understanding whether the user wants quadblink or not the 
algorithm can establish direct event responds.

@author: Kaan Karas
"""
import time
class distributer:
    def __init__(self):
        super(distributer,self).__init__()
        print("------------- Distributer Initiated ---------------------------")
        
        self.idle_DB=True
        self.count=0
        self.DB_Flag=False
        self.Blinks_Rec=[]
        
        self.events=[]   
        self.Enter_Eye_M= True            
        
        self.page=""
        self.button_N=""
        
        self.loc_event='0'
        self.Move_event="02"
        self.Task_event="10"
        self.Scan_event="21"
        
       
        
        
    def deliver(self,queueS1_S2, queueS2_S1,queueS2_S3, queueS3_S2):
        print("------------- Delivering Begins -------------------------------")

        while True:
            if not queueS1_S2.empty(): # Obtain Page Information
                button=queueS1_S2.get()
                self.page=button[0]
                exit_cond=button[1]
                self.button_N=button[2]
                # print("Line37: ","page: ",self.page,"  button_N: ",self.button_N, "Enter_Eye_M: ",self.Enter_Eye_M)
                if exit_cond=="Exit":
                    queueS2_S3.put("Close")
                    break
                
            if (self.count==1 or self.count==2) and  self.DB_Flag:
                if  time.time() - self.Record_Blink_time > 1.5: 
                    # 1.1 since QB must occur in 1 second after DB
                    self.DB_Flag=False
                    self.count=0
                    print("idle special case DBlink")
                    if self.page=="main":
                        queueS2_S1.put([self.page,self.button_N ,"Pressed"])
                    elif self.page=="RobotControl":
                        queueS2_S1.put([self.page,"00" ,"Pressed"])
                    elif self.page=="TaskSelection":
                        queueS2_S1.put([self.page,self.Task_event ,"Pressed"])
                    elif self.page=="Scan":
                        queueS2_S1.put([self.page,self.Scan_event ,"Pressed"])
                    
            if not queueS3_S2.empty(): # Obtain events From SAGA
                if self.Enter_Eye_M:
                    value=queueS3_S2.get()
                    self.events.append(value[0])
                    event=value[0]
                    self.Blinks_Rec.append(value[1])
                    # time_stream=value[2]
                
                if event=="Left" or event=="Right" or len(self.events)>1:
                    self.Enter_Eye_M=True
                    if event=="Blink":
                        if self.events[-2]=="Blink":
                            if self.Blinks_Rec[-1][0]-self.Blinks_Rec[-2][-1]<0.5:
                                self.count+=1
                                if not self.DB_Flag:
                                    self.DB_Flag=True
                                    self.Record_Blink_time = time.time()
                                if self.count==3:
                                    self.count=0
                                    self.DB_Flag=False
                                    print("Quad Blink")
                                    if self.page=="RobotControl":
                                        queueS2_S1.put([self.page,"0" ,"Back"])
                                    elif self.page=="TaskSelection":
                                        queueS2_S1.put([self.page,"0" ,"Back"])
                                    elif self.page=="Scan":
                                        queueS2_S1.put([self.page,"10" ,"Back"])
                                    elif self.page=="main":
                                        pass
                            elif self.DB_Flag: 
                                # For 2 consequtive Blinks and 1 Seperate Blink
                                self.DB_Flag=False
                                self.count=0
                                print("DBlink")
                                if self.page=="main":
                                    queueS2_S1.put([self.page,self.button_N ,"Pressed"])
                                elif self.page=="RobotControl":
                                    queueS2_S1.put([self.page,"00" ,"Pressed"])
                                elif self.page=="TaskSelection":
                                    queueS2_S1.put([self.page,self.Task_event ,"Pressed"])
                                elif self.page=="Scan":
                                    queueS2_S1.put([self.page,self.Scan_event ,"Pressed"])
                                
                        else: # Single Blink 
                             pass
                    elif event == "Left":
                        if self.DB_Flag:
                            # For Double Blinks before a Look LEFT event
                            if self.page=="main":
                                queueS2_S1.put([self.page,self.button_N ,"Pressed"])
                                print("DBlink",self.page,self.button_N)
                            elif self.page=="RobotControl":
                                queueS2_S1.put([self.page,"00" ,"Pressed"])
                                print("DBlink",self.page,"00")
                            elif self.page=="TaskSelection":
                                queueS2_S1.put([self.page,self.Task_event ,"Pressed"])
                                print("DBlink",self.page,self.Task_event)
                            elif self.page=="Scan":
                                queueS2_S1.put([self.page,"21" ,"Pressed"])
                                print("DBlink",self.page,"21")
                            self.DB_Flag=False
                            self.count=0
                            self.Enter_Eye_M=False
                        
                        if self.Enter_Eye_M:
                            print(event)
                            if self.page=="main":
                                self.loc_event=str(abs(int(self.loc_event)-1)%2)
                                queueS2_S1.put([self.page,self.loc_event,"None"])
                            elif self.page=="RobotControl":
                                self.Move_event="01"
                                queueS2_S1.put([self.page,self.Move_event,"None"])
                            elif self.page=="TaskSelection":
                                self.Task_event="1"+str(abs(int(self.Task_event[1])-1)%4)
                                queueS2_S1.put([self.page,self.Task_event,"None"])
                            elif self.page=="Scan":
                                self.Scan_event="20"
                                queueS2_S1.put([self.page,self.Scan_event,"None"])
                            
                    elif event == "Right":
                        # For Double Blinks before a Look RIGHT event
                        if self.DB_Flag:
                            if self.page=="main":
                                queueS2_S1.put([self.page,self.button_N ,"Pressed"])
                                print("DBlink",self.page,self.button_N)
                            elif self.page=="RobotControl":
                                queueS2_S1.put([self.page,"00" ,"Pressed"])
                                print("DBlink",self.page,"00")
                            elif self.page=="TaskSelection":
                                queueS2_S1.put([self.page,self.Task_event ,"Pressed"])
                                print("DBlink",self.page,self.Task_event)
                            elif self.page=="Scan":
                                queueS2_S1.put([self.page,"21" ,"Pressed"])
                                print("DBlink",self.page,"21")
                            self.DB_Flag=False
                            self.count=0
                            self.Enter_Eye_M=False
                        
                        if self.Enter_Eye_M:
                            print(event)    
                            if self.page=="main":
                                self.loc_event=str(abs(int(self.loc_event)+1)%2)
                                queueS2_S1.put([self.page,self.loc_event,"None"])
                            elif self.page=="RobotControl":
                                self.Move_event="03"
                                queueS2_S1.put([self.page,self.Move_event,"None"])
                            elif self.page=="TaskSelection":
                                self.Task_event="1"+str(abs(int(self.Task_event[1])+1)%4)
                                queueS2_S1.put([self.page,self.Task_event,"None"])
                            elif self.page=="Scan":
                                self.Scan_event="22"
                                queueS2_S1.put([self.page,self.Scan_event,"None"])
                
                if event =="Finished":
                    queueS2_S1.put([self.page,"Nan","Done"])
                    print("------------------------ " + self.events[-1] + " ----------------------------")
                    break
                    
                    
                    
                  
                            
                
                
                
            
                