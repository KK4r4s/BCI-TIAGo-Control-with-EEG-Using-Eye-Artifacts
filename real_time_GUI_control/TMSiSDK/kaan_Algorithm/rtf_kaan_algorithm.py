'''
Date: 23/09/22
@author: Kaan Karas & TMSi
Counts the events and at the end save them to a txt document.
TMSiSDK: Real-time filter that can be applied to incoming data
Copyright 2021 Twente Medical Systems international B.V., Oldenzaal The Netherlands

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

#######  #     #   #####   #  ######      #     #
   #     ##   ##  #        #  #     #     #     #
   #     # # # #  #        #  #     #     #     #
   #     #  #  #   #####   #  ######       #   #
   #     #     #        #  #  #     #      #   #
   #     #     #        #  #  #     #       # #
   #     #     #  #####    #  ######   #     #     #
'''
import os
import sys
from .. import sample_data_server
from ..device import ChannelType

import numpy as np
import queue
from copy import deepcopy
import threading
import time
from datetime import datetime

from scipy import signal, fft 
import matplotlib.pyplot as plt

class RTF_Algorithm:
    """ A semi-real time filter that can be used to retrieve and filter the data 
    and send them to queue. Different filters can be used for the different 
    analogue channel types (UNI, BIP. AUX). When no filter is generated/enabled 
    data remains unfiltered.
    Start/Stop also starts/stops sampling of the device
    """
    def __init__(self, device, _save_events):
        """Initialise filter instance
        """
        self.device=device
        self.num_channels = np.size(self.device.channels,0)
        self.sample_rate = self.device.config.get_sample_rate(ChannelType.counter)
        self._save_events=_save_events
        
        _UNI=[]
        _BIP=[]
        _AUX=[]
        
        for idx, ch in enumerate(device.channels):
            if (ch.type == ChannelType.UNI):
                _UNI.append(idx)
            elif (ch.type == ChannelType.BIP):
                _BIP.append(idx)
            elif (ch.type == ChannelType.AUX):
                _AUX.append(idx)
    
        self.channels={'UNI': _UNI,
                      'BIP': _BIP, 
                      'AUX': _AUX}
        self.filter_specs={'UNI': {'Order': None, 'Fc_hp': None, 'Fc_lp': None, 'Enabled': False},
                      'BIP': {'Order': None, 'Fc_hp': None, 'Fc_lp': None,'Enabled': False}, 
                      'AUX': {'Order': None, 'Fc_hp': None, 'Fc_lp': None, 'Enabled': False}}
        self._filter_details={'UNI': {'sos': None, 'z_sos': None},
                              'BIP': {'sos': None, 'z_sos': None}, 
                              'AUX': {'sos': None, 'z_sos': None}}
        
        # Prepare Queues
        _QUEUE_SIZE = 1000
        self.q_sample_sets = queue.Queue(_QUEUE_SIZE)
        
        _MAX_SIZE_FILTER_QUEUE=50
        self.q_filtered_sample_sets=queue.Queue(_MAX_SIZE_FILTER_QUEUE)
        
        _MAX_QUEUE_SIZE=50
        self.q_Kaan_sample_sets=queue.Queue(_MAX_QUEUE_SIZE)
        
        self.Active_Channel_Names=[self.device.channels[i].name for i in range(self.num_channels)]
        
        self.generateFilter(Fc_hp=1, Fc_lp=13)
        
        self.filter_thread = FilterThread(self)
        
        self.filter_thread.start()
    
    def generateFilter(self, order=2, Fc_hp=None, Fc_lp=None, ch_types=None, show=False):
        """ Generate filter with given order and cut-off frequency/frequencies. 
        Generates a high-pass filter when only Fc_hp is specified,a low-pass 
        filter when only Fc_lp is specified or a band-pass when both Fc_hp and 
        Fc_lp are given.
        Filter is applied to the specified channel types or to all analogue 
        channels when no channels types are given.
        Use show to inspect the frequency response of the filter """
        if not ch_types:
            ch_types=list(self.channels.keys())
            
        for ch_type in ch_types:
            self.filter_specs[ch_type]['Order']=order
            self.filter_specs[ch_type]['Fc_hp']=Fc_hp
            self.filter_specs[ch_type]['Fc_lp']=Fc_lp
            
            chan=self.channels[ch_type]
            
            if not (Fc_hp or Fc_lp) or not chan:
                sos=None
                z_sos=None
                self.filter_specs[ch_type]['Enabled']=False
            else:
                if Fc_hp and Fc_lp:
                    sos=signal.butter(order, [Fc_hp, Fc_lp], 'bandpass', fs=self.sample_rate, output='sos')
                elif Fc_hp:
                    sos=signal.butter(order, Fc_hp, 'highpass', fs=self.sample_rate, output='sos')
                elif Fc_lp:
                    sos=signal.butter(order, Fc_lp, 'lowpass', fs=self.sample_rate, output='sos')
                 
                z_sos0 = signal.sosfilt_zi(sos)
                z_sos=np.repeat(z_sos0[:, np.newaxis, :], len(chan), axis=1)
                self.filter_specs[ch_type]['Enabled']=True
            
            self._filter_details[ch_type]['sos']=sos
            self._filter_details[ch_type]['z_sos']=z_sos
        
        if show:
            # Show the frequency response of the filter
            w, h = signal.sosfreqz(sos, worN=fft.next_fast_len(self.sample_rate*10))
            plt.figure()
            plt.subplot(2, 1, 1)
            db = 20*np.log10(np.maximum(np.abs(h), 1e-5))
            plt.plot((self.sample_rate/2)*(w/np.pi), db, label = ch_type)
            plt.ylim(-50, 5)
            plt.grid(True)
            plt.yticks([0, -20, -40])
            plt.ylabel('Gain [dB]')
            plt.title('Frequency Response')
            plt.subplot(2, 1, 2)
            plt.plot((self.sample_rate/2)*(w/np.pi), np.angle(h), label = ch_type)
            plt.grid(True)
            plt.yticks([-np.pi, -0.5*np.pi, 0, 0.5*np.pi, np.pi],
                       [r'$-\pi$', r'$-\pi/2$', '0', r'$\pi/2$', r'$\pi$'])
            plt.ylabel('Phase [rad]')
            plt.xlabel('Frequency [Hz]')
            plt.show()
        
    def disableFilter(self, *ch_types):
        """ Disable the filters for the given channel types. All filters are disabled 
        when no channel types are given"""
        if not ch_types:
            ch_types=list(self.channels.keys())
            
        for ch_type in ch_types:
            self.filter_specs[ch_type]['Enabled']=False
            
    def enableFilter(self, *ch_types):
        """ Enable the filters for the given channel types. All filters are enabled 
        when no channel types are given"""
        if not ch_types:
            ch_types=list(self.channels.keys())
        
        for ch_type in ch_types:
            chan=self.channels[ch_type]
            if chan:
                self.filter_specs[ch_type]['Enabled']=True
                self.reset(ch_type)

    def reset(self, *ch_types):
        """ Reset the filters for the given channel types. All filters are reset 
        when no channel types are given"""
        
        if not ch_types:
            ch_types=list(self.channels.keys())
        
        for ch_type in ch_types:
            chan=self.channels[ch_type]
            if self.filter_specs[ch_type]['Enabled']:
                z_sos0 = signal.sosfilt_zi(self._filter_details[ch_type]['sos'])
                z_sos=np.repeat(z_sos0[:, np.newaxis, :], len(chan), axis=1)
                self._filter_details[ch_type]['z_sos']=z_sos
            
    def start(self):
        """ Start the filter thread and device""" 
        self.filter_thread.start()
        
    def stop(self):
        """ Stop the filter thread and device"""
        self.filter_thread.stop()
        sample_data_server.unregisterConsumer(self.device.id, self.filter_thread.q_sample_sets)


class FilterThread(threading.Thread):
    """A semi-real time filter"""
    
    def __init__(self, main_class):
        """ Setting up the class' properties
        """
        super(FilterThread,self).__init__()
        # TMSi Variables
        self.q_filtered_sample_sets=main_class.q_filtered_sample_sets
        self.q_sample_sets =main_class.q_sample_sets  
        self.channels=main_class.channels
        self._filter_details=main_class._filter_details
        self.filter_specs=main_class.filter_specs
        self.device=main_class.device
        
        # Kaan Karas Algorithm Additional Variables
        self.q_Kaan_sample_sets=main_class.q_Kaan_sample_sets
        self.sample_rate=main_class.sample_rate
        self.Active_Channel_Names=main_class.Active_Channel_Names
        self.numb=100
        self.m=0
        self.Gap=[]; self.Gap_time=[]; self.B_Flag=False; 
        self.Gap_LR=[]; self.Gap_time_LR=[]; self.LR_Flag=False; self.LR_Ind_Cond=False
        self.Gap_LL=[]; self.Gap_time_LL=[]; self.LL_Flag=False; self.LL_Ind_Cond=False
        self.Gap_Spare=[]; self.Gap_Spare_Time=[]; self.Gap_Spare_Cond=True 
        # Recording Detect Event Windows 
        self.Chosen_Windows=[]
        self.Chosen_Window_times=[]
        self.Blinks_Rec=[]
        # To see How Many Times
        self._save_events=main_class._save_events
        self.Number_of_Blinks=0
        self.Number_of_Look_Left= 0
        self.Number_of_Look_Rights=0
        self.Type_Event=[]
        
        # Register the consumer to the sample data server
        sample_data_server.registerConsumer(main_class.device.id, self.q_sample_sets)

####################################################################################################################################
# !!!!   
    def EM_B_Detection_Algorithm(self,stream,time_stream):
        """
        An Algorithm to Detect Eye Blinks and Eye Movements (Right/Left Look).
        Order of the channels in row of the stream can be learned from the
        self.Active_Channel_Names. In this Study, it is
        [0:'CREF', 1:'Fp1', 2:'Fpz', 3:'Fp2', 4:'F7', 5:'F3', 6:'Fz', 7:'F4', 
         8:'F8', 9:'AF7', 10:'AF3', 11:'AF4', 12:'AF8', 13:'F1', 14:'F2', 
         15:'STATUS', 16:'COUNTER'].
        This is usefull since Algorithm does not use all channels and, one 
        need to find the index of used channels.

        Parameters
        ----------
        stream : numpy array
            It is a matrix in size of [num_channels x self.num].
        time_stream : numy array
            It is a self made linearized time array to for the stream.

        Returns
        -------
        None. Since everything related is updated inside of class.
        """
        # print("Entered to Algorithm, size of stream",stream.shape,time_stream[0],time_stream[-1])
        self.Gap_Spare_Cond=True
        Blink_max_peaks, Blink_max_values = signal.find_peaks(stream[1,:], height=100)
        Blink_min_peaks, Blink_min_values = signal.find_peaks(-1*stream[1,:], height=0)  
        if (np.any(Blink_max_values["peak_heights"]>156) or self.B_Flag) and np.any(time_stream):
            if not self.B_Flag:
                self.B_Flag=True
                if np.any(self.Gap_LR) or np.any(self.Gap_LL):
                    self.Gap_LR=[]; self.Gap_time_LR=[]; self.LR_Flag=False
                    self.Gap_LL=[]; self.Gap_time_LL=[]; self.LL_Flag=False
                if not np.any(self.Gap_Spare):
                    self.Gap=stream
                    self.Gap_time=time_stream
                else:
                    self.Gap=np.append(self.Gap_Spare,stream,axis=1)
                    self.Gap_time=np.append(self.Gap_Spare_Time,time_stream)
            else:
                self.Chosen_Windows.append(np.concatenate((self.Gap,stream),axis=1))
                self.Chosen_Window_times.append(np.concatenate((self.Gap_time,time_stream),axis=0))
                self.Blinks_Rec.append([self.Chosen_Window_times[-1][0],self.Chosen_Window_times[-1][-1]])
                
                # print("Blink")
                self.Number_of_Blinks+=1
                self.Type_Event.append("Blink, ")
                
                stream=[]; time_stream=[]
                self.B_Flag=False
                self.LR_Ind_Cond=False; self.LL_Ind_Cond=False
                self.Gap_Spare_Cond=False
                
        # Right Look --------------------------------------------------------------    
        elif ((np.any(stream[4,:]>20) and np.any(stream[8,:]<-30)) or self.LR_Flag) and not self.LL_Flag and np.any(time_stream):
            if not self.LR_Flag:
                self.LR_Flag=True
                if not np.any(self.Gap_Spare):
                    self.Gap_LR=stream
                    self.Gap_time_LR=time_stream
                else: 
                    self.Gap_LR=np.append(self.Gap_Spare,stream,axis=1)
                    self.Gap_time_LR=np.append(self.Gap_Spare_Time,time_stream)
            else:
                self.Gap_LR=np.append(self.Gap_LR,stream,axis=1)
                self.Gap_time_LR=np.append(self.Gap_time_LR,time_stream)
                LR_F7_min_peaks, LR_F7_min_values = signal.find_peaks(-1*self.Gap_LR[4,:], height=20)
                LR_F7_max_peaks, LR_F7_max_values = signal.find_peaks(self.Gap_LR[4,:], height=20)
                LR_F8_min_peaks, LR_F8_min_values = signal.find_peaks(-1*self.Gap_LR[8,:], height=20)
                LR_F8_max_peaks, LR_F8_max_values = signal.find_peaks(self.Gap_LR[8,:], height=20)
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
                    try:   
                        LR_F7_First_Max_Ind=LR_F7_max_peaks[np.where(LR_F7_max_peaks<LR_F7_min_peaks[0])]
                        LR_F7_Second_Max_Ind=LR_F7_max_peaks[np.where(LR_F7_max_peaks>LR_F7_min_peaks[-1])]
                        LR_F8_First_Min_Ind=LR_F8_min_peaks[np.where(LR_F8_min_peaks<LR_F8_max_peaks[0])]
                        LR_F8_Second_Min_Ind=LR_F8_min_peaks[np.where(LR_F8_min_peaks>LR_F8_max_peaks[-1])]
                        if self.LR_Ind_Cond or ((np.any(LR_F7_First_Max_Ind) and np.any(LR_F7_Second_Max_Ind)) and (np.any(LR_F8_First_Min_Ind) and np.any(LR_F8_Second_Min_Ind))):
                            if np.any(abs(self.Gap_LR[3,-30:])<10): #can be added to correct where it ends 
                                self.Chosen_Windows.append(self.Gap_LR)
                                self.Chosen_Window_times.append(self.Gap_time_LR)
                                
                                # print("Look Right")
                                self.Number_of_Look_Rights+=1
                                self.Type_Event.append("Right Look, ")
                                
                                self.Gap_LR=[]; self.Gap_time_LR=[]
                                LR_F7_First_Max_Ind=[]; LR_F7_Second_Max_Ind=[]
                                LR_F8_First_Min_Ind=[]; LR_F8_Second_Min_Ind=[]
                                self.LR_Flag=False; self.LR_Ind_Cond=False
                                self.Gap_Spare_Cond=False
                            else:
                                self.LR_Ind_Cond=True
                    except:
                        pass
                else:
                    self.LR_Flag=True
        # Left Look ---------------------------------------------------------------            
        elif ((np.any(stream[4,:]<-20) and np.any(stream[7,:]>30)) or self.LL_Flag) and not self.LR_Flag and np.any(time_stream):
            if not self.LL_Flag:
                self.LL_Flag=True
                if not np.any(self.Gap_Spare):
                    self.Gap_LL=stream
                    self.Gap_time_LL=time_stream
                else: 
                    self.Gap_LL=np.append(self.Gap_Spare,stream,axis=1)
                    self.Gap_time_LL=np.append(self.Gap_Spare_Time,time_stream)
            else:
                self.Gap_LL=np.append(self.Gap_LL,stream,axis=1)
                self.Gap_time_LL=np.append(self.Gap_time_LL,time_stream)
                LL_F7_min_peaks, LL_F7_min_values = signal.find_peaks(-1*self.Gap_LL[4,:], height=19)
                LL_F7_max_peaks, LL_F7_max_values = signal.find_peaks(self.Gap_LL[4,:], height=20)
                LL_F8_min_peaks, LL_F8_min_values = signal.find_peaks(-1*self.Gap_LL[7,:], height=22)
                LL_F8_max_peaks, LL_F8_max_values = signal.find_peaks(self.Gap_LL[7,:], height=22)
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
                    try:    
                        LL_F7_First_Min_Ind=LL_F7_min_peaks[np.where(LL_F7_min_peaks<LL_F7_max_peaks[0])]
                        LL_F7_Second_Min_Ind=LL_F7_min_peaks[np.where(LL_F7_min_peaks>LL_F7_max_peaks[-1])]
                        LL_F8_First_Max_Ind=LL_F8_max_peaks[np.where(LL_F8_max_peaks<LL_F8_min_peaks[0])]
                        LL_F8_Second_Max_Ind=LL_F8_max_peaks[np.where(LL_F8_max_peaks>LL_F8_min_peaks[-1])]
                        if self.LL_Ind_Cond or ((np.any(LL_F7_First_Min_Ind) and np.any(LL_F7_Second_Min_Ind)) and (np.any(LL_F8_First_Max_Ind) and np.any(LL_F8_Second_Max_Ind))):
                            if np.any(abs(self.Gap_LL[4,-30:])<20): 
                                self.Chosen_Windows.append(self.Gap_LL)
                                self.Chosen_Window_times.append(self.Gap_time_LL)
                                
                                # print("Look Left")
                                self.Number_of_Look_Left+=1
                                self.Type_Event.append("Left Look, ")
                                
                                self.Gap_LL=[];self.Gap_time_LL=[]
                                LL_F7_First_Min_Ind=[];LL_F7_Second_Min_Ind=[]
                                LL_F8_First_Max_Ind=[];LL_F8_Second_Max_Ind=[]
                                self.LL_Flag=False; self.LL_Ind_Cond=False
                                self.Gap_Spare_Cond=False
                            else:
                                self.LL_Ind_Cond=True
                    except:
                        pass
                else:
                    self.LL_Flag=True             
        elif np.any(stream[1,:]>80) and np.any(time_stream):
            # Activate Blink case for cathing small amplitude blinks, records first one here and other half later
            self.B_Flag=True
            if np.any(self.Gap_LR) or np.any(self.Gap_LL):
                self.Gap_LR=[]; self.Gap_time_LR=[]; self.LR_Flag=False
                self.Gap_LL=[]; self.Gap_time_LL=[]; self.LL_Flag=False
            if not np.any(self.Gap_Spare):
                self.Gap=stream
                self.Gap_time=time_stream
            else:
                self.Gap=np.append(self.Gap_Spare,stream,axis=1)
                self.Gap_time=np.append(self.Gap_Spare_Time,time_stream)           
        if self.Gap_Spare_Cond:
            self.Gap_Spare=stream
            self.Gap_Spare_Time=time_stream
        else:
            self.Gap_Spare=[]
            self.Gap_Spare_Time=[]
          
####################################################################################################################################
# !!!!
# !!!!
# !!!!              
    def kaan_function(self):
        """
        Method that retrieves reshaped either filtered or unfiltered signal samples from the queue,
            collects the samples in desired size and put the remaining (if any) back to the queue so that 
            in the next time they will be received and collected. 
        Variable, sample_wind, will be used in the blink detection algorithm. 

        """
        sample_list=[]
        count=0
                
        while not self.q_Kaan_sample_sets.empty():
            if count==0:
                sample_list=self.q_Kaan_sample_sets.get()
                self.q_Kaan_sample_sets.task_done()
                count+=1
                
            else:
                sample_list=np.concatenate((sample_list, self.q_Kaan_sample_sets.get()), axis=1)
                self.q_Kaan_sample_sets.task_done()
                
            if self.q_Kaan_sample_sets.empty():
                # to check if there is any nan and set them to zero
                # For this algorithm to work sample_list should be 
                # an array which become an array with np.concatenate
                con = np.isfinite(sample_list)
                # print("np.any(con)=",np.any(con),"All Values are finite / None of them are nan")
                sample_list[~con] = 0
                
                if sample_list.shape[1]==self.numb:
                    # if sample_size collected consist of 100
                    sample_wind=deepcopy(sample_list[:,:self.numb])
                    sample_time_stream=np.arange(self.numb*self.m,self.numb*(self.m+1))/self.sample_rate
                    self.m=self.m+1
                    # Call Detection Algorithm
                    self.EM_B_Detection_Algorithm(stream=sample_wind,time_stream=sample_time_stream)
                elif sample_list.shape[1]>self.numb:
                    # Sample_size collected consist of 100 elements sent to algorithm
                    sample_wind=deepcopy(sample_list[:100,:self.numb])
                    sample_time_stream=np.arange(self.numb*self.m,self.numb*(self.m+1))/self.sample_rate
                    self.m=self.m+1
                    self.EM_B_Detection_Algorithm(stream=sample_wind,time_stream=sample_time_stream)
                    # Inserting back reaming elements to Queue since it is higher than 100
                    # Break is requiered since q_Kaan_filtered_sample_sets is filled again
                    self.q_Kaan_sample_sets.put(deepcopy(sample_list[:,self.numb:]))
                    break
                else:
                    # Inserting back elements to Queue since it is less than 100
                    # Break is requiered since q_Kaan_sample_sets is filled again
                    self.q_Kaan_sample_sets.put(deepcopy(sample_list))
                    break
                
            

    def run(self): 
        """ Method that retrieves samples from the queue, reshapes them into 
            the desired format and filters the samples.
        """
        # Start measurement
        self.device.start_measurement()
        self.sampling = True
        
        while self.sampling:
            while not self.q_sample_sets.empty():
                
                #Read samples from queue 
                sd = self.q_sample_sets.get()
                self.q_sample_sets.task_done()
                
                # Reshape the samples retrieved from the queue
                samples = np.reshape(sd.samples, (sd.num_samples_per_sample_set, sd.num_sample_sets), order = 'F')
                
                #Filter data 
                for ch_type, ch_filter in self._filter_details.items():
                    if self.filter_specs[ch_type]['Enabled']:
                        # filter the data
                        samples[self.channels[ch_type]], self._filter_details[ch_type]['z_sos']=signal.sosfilt(self._filter_details[ch_type]['sos'], samples[self.channels[ch_type]], zi=self._filter_details[ch_type]['z_sos'])
                
                # print(samples.shape)         
                
                # Output sample data to queue
                self.q_filtered_sample_sets.put(deepcopy(samples))
                
                # Output Sample data to queue_Kaan
                self.q_Kaan_sample_sets.put(deepcopy(samples))
                self.kaan_function()
                
                # Pause the thread for a bit to update the plot
                time.sleep(0.01)

        
    def stop(self):
        """ Method that is executed when the thread is terminated. 
            This stop event stops the measurement.
        """
        self.sampling=False
        self.device.stop_measurement()
        
        if self._save_events:
            
            if sys.platform == "linux" or sys.platform == "linux2":
                newpath = '../../measurements'
            elif sys.platform == "win32": # Windows
                newpath = '../../measurements'           

            newpath = newpath + '/Algorithm_Catches_' + datetime.today().strftime('%m_%d_%Y')
            
            if not os.path.exists(newpath):
                os.makedirs(newpath)
            now = datetime.now()
            current_time = now.strftime("%H_%M_%S")
            
            file_name_1 = "/Number_of_Events_"+current_time+".txt"
            file_name_2 = "/Time_Occurences_"+current_time+".txt"
            file_name_3 = "/Event_Type_"+current_time+".txt"
            
            
            Num_Ev=[self.Number_of_Blinks, self.Number_of_Look_Left, self.Number_of_Look_Rights]
            Check=[]
            for i in range(len(self.Chosen_Window_times)):
                           Check.append((self.Chosen_Window_times[i][0],self.Chosen_Window_times[i][-1]))
            
            # # Saving the array in a text file
                    
            np.savetxt(newpath+file_name_1,Num_Ev)    
            np.savetxt(newpath+file_name_2, Check)
            np.savetxt(newpath+file_name_3, self.Type_Event)
            
            print("\nNumber of Events and Time Occurences Saved\n")
            
        
        
                
