# -*- coding: utf-8 -*-
"""
@author: Kaan Karas
Date: 16/11/2022

"""

import numpy as np
import pandas as pd
import math
from scipy.signal import find_peaks

import os
os.chdir("C:/Users/Lenovo/Desktop/Thesis_Blink_Kaan")

import matplotlib.pyplot as plt
import seaborn as sns
sns.set_style("darkgrid")

from TMSiSDK.file_readers import Poly5Reader
import sys
sys.path.append("../")
from TMSiSDK import kaan_my_tools 
mytools = kaan_my_tools.tools() # define classes

plt.close("all")
from random import randint,seed
seed(101)


#==============================================================================
# Upload Data From TMSi SAGA Measurements

#Path_N=["C:/Users/Lenovo/Desktop/Thesis_Blink_Kaan/measurements/TMSi SAGA Measurements"]
#Dir_N=["/8-20220615-114145", "/9-20220616-140925", "/10-20220628-121222", "/11-20220629-153314",
#       "/12-20220720-152434", "/14-20220722-150614","/13-20220721-152430"]

Path_N=["C:/Users/Lenovo/Desktop/Thesis_Blink_Kaan/measurements"]
Dir_N=["/Experiments_09_09_2022","/Experiments_09_14_2022","/Experiments_09_23_2022",
       "/Experiments_10_19_2022","/Experiments_10_20_2022"]

f_P5, f_ex= mytools.Retrieve(Path_N,Dir_N[3])
#4-7 LR
#
K_n= 4 # Number of Experiment to retrieve
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

data = Poly5Reader(filename=f_P5[K_n])
excel_data = pd.read_csv(f_ex[K_n])
N_Sig=f_P5[K_n].split("/")[-1].split("-")[0]+"-"+f_P5[K_n].split("_")[-1].split(".")[0]

t_0 = pd.to_datetime(excel_data.iloc[:, 0])
trigger_start = t_0[0].second + (t_0[0].microsecond*10**-6)
trigger_end = (t_0[t_0.shape[0]-1].minute*60) + t_0[t_0.shape[0]-1].second + (t_0[t_0.shape[0]-1].microsecond*10**-6)

sample_rate = data.sample_rate
sampled_data = (data.samples[:, math.floor(trigger_start *
                sample_rate):math.ceil(trigger_end*sample_rate)])
NofS = math.ceil(trigger_end*sample_rate)-math.floor(trigger_start*sample_rate)
NofCh = data.num_channels

Channel_Names = []
for i in range(NofCh):
    Channel_Names.append(data.channels[i]._Channel__name)
# if "i" in globals():
#     del(trigger_end,trigger_start,i)    

#==============================================================================
dCh_N_1=[]
St_1="F7,Fp1,F8".split(",")
dCh_N_2=[]
St_2="AF7,AF8,AF3,Fpz,Fp2,AF4".split(",")     
for i in St_1:
    dCh_N_1.append(np.where(np.isin(Channel_Names,i))[0][0])
for i in St_2:
    dCh_N_2.append(np.where(np.isin(Channel_Names,i))[0][0]) 
# 0:Fp1, 1:Fpz, 2:Fp2, 3:F7, 7:F8, 11:AF7, 12:AF3, 13:AF4, 14:AF8
DCH_N_All=(dCh_N_1+dCh_N_2)
DCH_N_All.sort()
 
    
sig1=mytools.butter_highpass_filtfilt(sampled_data, 1, sample_rate,order=2)
sig2=mytools.butter_lowpass_filtfilt(sig1,13, sample_rate,order=2)
if False: # Plot Channels
    mytools.Multiple_Ch_Plot(data=sig2,freq=sample_rate ,Name_Sig=N_Sig ,Ch_N=Channel_Names,d_Ch=dCh_N_1,SF=False)
    mytools.Multiple_Ch_Plot(data=sig2,freq=sample_rate ,Name_Sig=N_Sig ,Ch_N=Channel_Names,d_Ch=dCh_N_2,SF=False)

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    
Signal=sig2[DCH_N_All] #The desired Channels in Numerical Order Same
Ch_Names=[Channel_Names[i] for i in DCH_N_All] #[0:'Fp1', 1:'Fpz', 2:'Fp2', 3:'F7', 4:'F8', 5:'AF7', 6:'AF3', 7:'AF4', 8:'AF8']
color=mytools.rgb_to_hex(randint(randint(40,100),200),randint(50,randint(140,200)),randint(50,200))


# First_Half= N_Sig.split(str(K_n))[1].split("_")[-2]
# Second_Half= N_Sig.split(str(K_n))[1].split("_")[1] +" "+ N_Sig.split(str(K_n))[1].split("_")[2]
# year= N_Sig.split("-")[-1].split("T")[0][0:4]
# month= N_Sig.split("-")[-1].split("T")[0][4:6]
# day= N_Sig.split("-")[-1].split("T")[0][6:8]
# date="/".join([day,month,year])

# New_Name = "".join(" ".join(N_Sig.split(str(K_n))[1].split("_")).split(First_Half) )+"and"+\
#  "".join(" ".join(N_Sig.split(str(K_n))[1].split("_")).split(Second_Half))+date


m=0
N_Event=[]
WDW=100
Gap=[];Gap_time=[];
Gap_LR=[]; Gap_time_LR=[]
Gap_LL=[]; Gap_time_LL=[]
Gap_Spare=[]; Gap_Spare_Time=[]; m_spare=1000
Chosen_Windows=[]
Chosen_Window_times=[]

B_Flag=False; 
LR_Flag=False; LR_Ind_Cond=False
LL_Flag=False; LL_Ind_Cond=False
Gap_Spare_Cond=True



k=0
Number_of_Blinks=0
Number_of_Look_Left= 0
Number_of_Look_Rights=0


fig,ax= plt.subplots(figsize=(15,9),nrows=3,ncols=1)
# fig.suptitle(New_Name)
ax[0].set_title(Ch_Names[0], fontsize=10)
ax[1].set_title(Ch_Names[3], fontsize=10)
ax[2].set_title(Ch_Names[4], fontsize=10)

while True:
    stream=Signal[:,WDW*m:WDW*(m+1)]
    time= np.arange(stream.shape[1]*(m),stream.shape[1]*(m+1))/sample_rate
    Gap_Spare_Cond=True
    
    ax[0].plot(time,stream[0,:],"b",ls="-" , lw=1)
    ax[1].plot(time,stream[3,:],"b",ls="-" , lw=1)
    ax[2].plot(time,stream[4,:],"b",ls="-" , lw=1)
    
    if (np.any(stream[0,:]>156) or B_Flag):
        # print(m,"Blink Active",B_Flag,time[0])        
        if not B_Flag:
            B_Flag=True
            if np.any(Gap_LR) or np.any(Gap_LL):
                Gap_LR=[]; Gap_time_LR=[]; LR_Flag=False; LR_Cond_Flag=False
                Gap_LL=[]; Gap_time_LL=[]; LL_Flag=False; LL_Cond_Flag=False
            if not np.any(Gap_Spare):
                Gap=stream
                Gap_time=time
            else:
                Gap=np.append(Gap_Spare,stream,axis=1)
                Gap_time=np.append(Gap_Spare_Time,time)
        else:
            Chosen_Windows.append(np.concatenate((Gap,stream),axis=1))
            Chosen_Window_times.append(np.concatenate((Gap_time,time),axis=0))
            
            print("Blink Catched",(Chosen_Window_times[k][0],Chosen_Window_times[k][-1]))
            
            stream=[]; time=[]
            B_Flag=False
            LR_Ind_Cond=False; LL_Ind_Cond=False
            Gap_Spare_Cond=False
            
            
            color=mytools.rgb_to_hex(randint(randint(10,199),200),randint(50,randint(140,200)),randint(50,200))
            ax[0].plot(Chosen_Window_times[k],Chosen_Windows[k][0,:],"k",ls="-" ,lw=1.5)
            ax[1].plot(Chosen_Window_times[k],Chosen_Windows[k][3,:],"k",ls="-" ,lw=1.5)
            k+=1; Number_of_Blinks+=1
    # Right Look --------------------------------------------------------------    
    elif ((np.any(stream[3,:]>20) and np.any(stream[4,:]<-30)) or LR_Flag) and not LL_Flag:
        # print(m,"LR",(time[0],time[-1]))
        if not LR_Flag:
            LR_Flag=True
            if not np.any(Gap_Spare):
                Gap_LR=stream
                Gap_time_LR=time
            else: 
                Gap_LR=np.append(Gap_Spare,stream,axis=1)
                Gap_time_LR=np.append(Gap_Spare_Time,time)
        else:
            Gap_LR=np.append(Gap_LR,stream,axis=1)
            Gap_time_LR=np.append(Gap_time_LR,time)
            
            LR_F7_min_peaks, LR_F7_min_values = find_peaks(-1*Gap_LR[3,:], height=20)
            LR_F7_max_peaks, LR_F7_max_values = find_peaks(Gap_LR[3,:], height=20)
            LR_F8_min_peaks, LR_F8_min_values = find_peaks(-1*Gap_LR[4,:], height=20)
            LR_F8_max_peaks, LR_F8_max_values = find_peaks(Gap_LR[4,:], height=20)
            if (LR_F7_max_peaks.shape[0]>0 and LR_F8_min_peaks.shape[0]>0):
                if np.any(LR_F7_max_peaks[0]>LR_F7_min_peaks):
                    F7_LR_rem_indx=LR_F7_max_peaks[0]<LR_F7_min_peaks
                    LR_F7_min_peaks=LR_F7_min_peaks[F7_LR_rem_indx]
                    LR_F7_min_values['peak_heights']=LR_F7_min_values['peak_heights'][F7_LR_rem_indx]
                    
                if np.any(LR_F8_min_peaks[0]>LR_F8_max_peaks):
                    F8_LR_rem_indx=LR_F8_min_peaks[0]<LR_F8_max_peaks
                    LR_F8_max_peaks=LR_F8_max_peaks[F8_LR_rem_indx]
                    LR_F8_max_values['peak_heights']=LR_F8_max_values['peak_heights'][F8_LR_rem_indx]
                    
            if (LR_F7_max_peaks.shape[0]>=2 and LR_F8_min_peaks.shape[0]>=2 ):#and  LR_F7_min_peaks.shape[0]>=2): 
                # Check Order of the F7(Max,Min,Min,Max) and F8(Min,Max,Max,Min) to detect
                ax[1].plot(Gap_time_LR[LR_F7_max_peaks],LR_F7_max_values["peak_heights"],"k*",ms=8)
                ax[1].plot(Gap_time_LR[LR_F7_min_peaks],-1*LR_F7_min_values["peak_heights"],"m*",ms=8)
                ax[2].plot(Gap_time_LR[LR_F8_max_peaks],LR_F8_max_values["peak_heights"],"k*",ms=8)
                ax[2].plot(Gap_time_LR[LR_F8_min_peaks],-1*LR_F8_min_values["peak_heights"],"m*",ms=8)
                try:   
                    LR_F7_First_Max_Ind=LR_F7_max_peaks[np.where(LR_F7_max_peaks<LR_F7_min_peaks[0])]
                    LR_F7_Second_Max_Ind=LR_F7_max_peaks[np.where(LR_F7_max_peaks>LR_F7_min_peaks[-1])]
        
                    LR_F8_First_Min_Ind=LR_F8_min_peaks[np.where(LR_F8_min_peaks<LR_F8_max_peaks[0])]
                    LR_F8_Second_Min_Ind=LR_F8_min_peaks[np.where(LR_F8_min_peaks>LR_F8_max_peaks[-1])]
                    # print("Cond2",m,LR_F7_First_Max_Ind,LR_F7_Second_Max_Ind,LR_F8_First_Min_Ind,LR_F8_Second_Min_Ind)
                    if LR_Ind_Cond or ((np.any(LR_F7_First_Max_Ind) and np.any(LR_F7_Second_Max_Ind)) and (np.any(LR_F8_First_Min_Ind) and np.any(LR_F8_Second_Min_Ind))):
                        if np.any(abs(Gap_LR[3,-30:])<10): #can be added to correct where it ends 
                            Chosen_Windows.append(Gap_LR)
                            Chosen_Window_times.append(Gap_time_LR)
                            
                            print("LR catched",(Chosen_Window_times[k][0],Chosen_Window_times[k][-1]))
                            
                            Gap_LR=[];Gap_time_LR=[]
                            LR_F7_First_Max_Ind=[]; LR_F7_Second_Max_Ind=[]
                            LR_F8_First_Min_Ind=[]; LR_F8_Second_Min_Ind=[]
                            LR_Flag=False; LR_Ind_Cond=False
                            Gap_Spare_Cond=False
                            
                            ax[1].plot(Chosen_Window_times[k],Chosen_Windows[k][3,:],"g",ls="-" ,lw=1.5)
                            ax[2].plot(Chosen_Window_times[k],Chosen_Windows[k][4,:],"g",ls="-" ,lw=1.5)
                            k+=1 ; Number_of_Look_Rights+=1
                        else:
                            LR_Ind_Cond=True
                except:
                    pass
            else:
                LR_Flag=True
    # Left Look ---------------------------------------------------------------            
    elif ((np.any(stream[3,:]<-20) and np.any(stream[4,:]>30)) or LL_Flag) and not LR_Flag:
        # print(m,"LL",(time[0],time[-1]))
        if not LL_Flag:
            LL_Flag=True
            
            if not np.any(Gap_Spare):
                Gap_LL=stream
                Gap_time_LL=time
            else: 
                Gap_LL=np.append(Gap_Spare,stream,axis=1)
                Gap_time_LL=np.append(Gap_Spare_Time,time)
        else:
            Gap_LL=np.append(Gap_LL,stream,axis=1)
            Gap_time_LL=np.append(Gap_time_LL,time)
            
            LL_F7_min_peaks, LL_F7_min_values = find_peaks(-1*Gap_LL[3,:], height=20) # 19
            LL_F7_max_peaks, LL_F7_max_values = find_peaks(Gap_LL[3,:], height=20) #20
            LL_F8_min_peaks, LL_F8_min_values = find_peaks(-1*Gap_LL[4,:], height=20) #22
            LL_F8_max_peaks, LL_F8_max_values = find_peaks(Gap_LL[4,:], height=20) #22
            if LL_F8_max_peaks.shape[0]>0 and LL_F7_min_peaks.shape[0]>0:    
                if np.any(LL_F8_max_peaks[0]>LL_F8_min_peaks):
                    F8_LL_rem_indx=LL_F8_max_peaks[0]<LL_F8_min_peaks
                    LL_F8_min_peaks=LL_F8_min_peaks[F8_LL_rem_indx]
                    LL_F8_min_values['peak_heights']=LL_F8_min_values['peak_heights'][F8_LL_rem_indx]
                    
                if np.any(LL_F7_min_peaks[0]>LL_F7_max_peaks):
                    F7_LL_rem_indx=LL_F7_min_peaks[0]<LL_F7_max_peaks
                    LL_F7_max_peaks=LL_F7_max_peaks[F7_LL_rem_indx]
                    LL_F7_max_values['peak_heights']=LL_F7_max_values['peak_heights'][F7_LL_rem_indx] 
                    
            if (LL_F7_max_peaks.shape[0]>=2 and LL_F8_min_peaks.shape[0]>=2 and  LL_F7_min_peaks.shape[0]>=2): 
                # Check Order of the F8(Max,Min,Min,Max) and F7(Min,Max,Max,Min) to detect
                
                ax[1].plot(Gap_time_LL[LL_F7_max_peaks],LL_F7_max_values["peak_heights"],"y*",ms=8)
                ax[1].plot(Gap_time_LL[LL_F7_min_peaks],-1*LL_F7_min_values["peak_heights"],"c*",ms=8)
                ax[2].plot(Gap_time_LL[LL_F8_max_peaks],LL_F8_max_values["peak_heights"],"y*",ms=8)
                ax[2].plot(Gap_time_LL[LL_F8_min_peaks],-1*LL_F8_min_values["peak_heights"],"c*",ms=8)
                
                try:    
                    LL_F7_First_Min_Ind=LL_F7_min_peaks[np.where(LL_F7_min_peaks<LL_F7_max_peaks[0])]
                    LL_F7_Second_Min_Ind=LL_F7_min_peaks[np.where(LL_F7_min_peaks>LL_F7_max_peaks[-1])]
        
                    LL_F8_First_Max_Ind=LL_F8_max_peaks[np.where(LL_F8_max_peaks<LL_F8_min_peaks[0])]
                    LL_F8_Second_Max_Ind=LL_F8_max_peaks[np.where(LL_F8_max_peaks>LL_F8_min_peaks[-1])]
                
                
                    # print("Cond2",m,LL_F7_First_Min_Ind,LL_F7_Second_Min_Ind,LL_F8_First_Max_Ind,LL_F8_Second_Max_Ind)
                    if LL_Ind_Cond or ((np.any(LL_F7_First_Min_Ind) and np.any(LL_F7_Second_Min_Ind)) and (np.any(LL_F8_First_Max_Ind) and np.any(LL_F8_Second_Max_Ind))):
                        if np.any(abs(Gap_LL[4,-30:])<20): 
                            Chosen_Windows.append(Gap_LL)
                            Chosen_Window_times.append(Gap_time_LL)
                            
                            print("LL catched",(Chosen_Window_times[k][0],Chosen_Window_times[k][-1]))
                            
                            Gap_LL=[];Gap_time_LL=[]
                            LL_F7_First_Min_Ind=[];LL_F7_Second_Min_Ind=[]
                            LL_F8_First_Max_Ind=[];LL_F8_Second_Max_Ind=[]
                            LL_Flag=False; LL_Ind_Cond=False
                            Gap_Spare_Cond=False
                            
                            ax[1].plot(Chosen_Window_times[k],Chosen_Windows[k][3,:],"r",ls="-" ,lw=1.5)
                            ax[2].plot(Chosen_Window_times[k],Chosen_Windows[k][4,:],"r",ls="-" ,lw=1.5)
                            k+=1; Number_of_Look_Left+=1
                            
                        else:
                            LL_Ind_Cond=True
                except:
                    pass
            else:
                LL_Flag=True
####################################################################################################################################                
    elif np.any(stream[0,:]>80):
        # Activate Blink case for cathing small amplitude blinks, records first one here and other half later
        B_Flag=True
        
        if np.any(Gap_LR) or np.any(Gap_LL):
            Gap_LR=[]; Gap_time_LR=[]; LR_Flag=False; LR_Cond_Flag=False
            Gap_LL=[]; Gap_time_LL=[]; LL_Flag=False; LL_Cond_Flag=False
        if not np.any(Gap_Spare):
            Gap=stream
            Gap_time=time
        else:
            Gap=np.append(Gap_Spare,stream,axis=1)
            Gap_time=np.append(Gap_Spare_Time,time)
    
#####################################################################################################################################            
    if Gap_Spare_Cond:
        Gap_Spare=stream
        Gap_Spare_Time=time
    else:
        Gap_Spare=[]
        Gap_Spare_Time=[]
    
    if m>Signal.shape[1]/WDW:
        break
    m+=1
    
   
print("Number_of_Blinks",Number_of_Blinks)
print("Number_of_Look_Left",Number_of_Look_Left)
print("Number_of_Look_Rights",Number_of_Look_Rights)    
   
    
    
    
    
    
    
    
    
    
    
    
    
    
    