# -*- coding: utf-8 -*-
"""
Created on 23/09/2022 at 17:47 

This is the Updated Version for easy access for multiple choses.
        
23/09/22  -> Possible To Choose to record detected events.
          -> "_ImpPlot=True" Plots the Impedance Plot
          -> "_SRT_Plot=True" Plots the Semi Real Time Data different class is used  
             kaan_GUI_Algorithm: "kaan_algorithm_detect" or "kaan_algorithm_detect_SRT" 
          -> "_save_Event=True" Save the detected events and its time stamps.
          -> " _save_Imp=True" Save the recorded impedance values during the setup
          
@author: Kaan Karas & TMSi
    
"""
import sys
sys.path.append("../")
import os

# TMSi Librares
from TMSiSDK import tmsi_device
from TMSiSDK.device import DeviceInterfaceType, ChannelType, ReferenceMethod, ReferenceSwitch, DeviceState
from TMSiSDK.error import TMSiError, TMSiErrorCode, DeviceErrorLookupTable
from TMSiSDK.file_writer import FileWriter, FileFormat


from TMSiSDK import plotters

# from TMSiSDK import filters
from TMSiSDK import kaan_GUI_Algorithm

from PySide2 import QtWidgets
import time
from datetime import datetime

class TMSi_Connect:
    def __init__(self,_ImpPlot=True,_SRT_Plot=False,_save_Event=False, _save_Imp=True):
        super(TMSi_Connect,self).__init__()
        try:
            tmsi_device.initialize()
            self.dev = tmsi_device.create(tmsi_device.DeviceType.saga, DeviceInterfaceType.docked, DeviceInterfaceType.usb)
            self.dev.open()
            # Set Channels to measure
            self.UNI_list = [0,1,2,3,4,5,6,7,8,33,34,35,36,38,39] 
            # UNI_list = [0,1,2,3,4,5,6,7,8,10,11,16,33,34,35,36,38,39,42,45,46]
            self.UNI_count = 0 # The counter is used to keep track of the number of UNI channels that have been encountered while looping over the channel list
            self.ch_list = self.dev.config.channels # Retrieve all channels from the device and update which should be enabled
            for idx, ch in enumerate(self.ch_list):
                if (ch.type == ChannelType.UNI):
                    if self.UNI_count in self.UNI_list:
                        ch.enabled = True
                    else:
                        ch.enabled = False
                    self.UNI_count += 1 # Update the UNI counter
                else:
                    ch.enabled = False # Close AUX, BIP and SENSOR channel types
            
            self.dev.config.channels = self.ch_list 
            # Set the sample rate to 500 Hz and Print the result
            self.dev.config.base_sample_rate = 4000
            self.dev.config.set_sample_rate(ChannelType.all_types, 8) # 1,2,4,8 are available
            # Set Reference method and Switch Method 
            self.dev.config.reference_method = ReferenceMethod.common,ReferenceSwitch.fixed  
            
            self._ImpPlot=_ImpPlot
            self._SRT_Plot=_SRT_Plot
            self._save_Event=_save_Event
            self._save_Imp=_save_Imp
            
            if self._ImpPlot: # !!!!  Impedance Plot Head Configuration
                
                plotter_app = QtWidgets.QApplication.instance()  # Check if there is already a plotter application in existence
                if not plotter_app: # Initialise the plotter application if there is no other plotter application
                    plotter_app = QtWidgets.QApplication(sys.argv)    
                # Define the GUI object and show it
                window = plotters.ImpedancePlot(figurename = 'An Impedance Plot', device = self.dev, layout = 'head', file_storage = self._save_Imp)
                window.show()
                plotter_app.exec_() # Enter the event loop
                print('\n Wait for a bit while we close the plot... \n') 
                time.sleep(1) # Pause for a while to properly close the GUI after completion
                
                if not self._SRT_Plot: # Close the QtWidgets
                    print("\nPlotter app is closed")
                    QtWidgets.QApplication.quit()
                    del plotter_app
             
        except TMSiError as e:
            print("!!! TMSiError !!! : ", e.code)
            if (e.code == TMSiErrorCode.device_error) :
                print("  => device error : ", hex(self.dev.status.error))
                DeviceErrorLookupTable(hex(self.dev.status.error))
                if self.dev.status.state == DeviceState.connected:
                    print("\nDevice is Closed")
                    self.dev.close()
        
            
    def Start_Measuring(self,queueS2_S3, queueS3_S2): 
        self.queueS3_S2 = queueS3_S2    
        self.queueS2_S3 = queueS2_S3    
    
        if sys.platform == "linux" or sys.platform == "linux2":
            newpath = 'measurements/'
        elif sys.platform == "win32": # Windows
            newpath = 'measurements/'           
   
        newpath = newpath + 'Experiments_' + datetime.today().strftime('%m_%d_%Y')

        if not os.path.exists(newpath):
            os.makedirs(newpath)
        file_name = newpath + "/0_Scenario_0.poly5"
        # Initialise a file-writer class (Poly5-format) and state its file path
        file_writer = FileWriter(FileFormat.poly5, file_name)
        # Define the handle to the device
        file_writer.open(self.dev)
        
        if not self._SRT_Plot:    
            # Initialise filter and Measurement Build in 
            filter_appl = kaan_GUI_Algorithm.Classification_Algorithm(self.dev, self._save_Event, self.queueS3_S2)
            print("To exit Ctrl+C")
            try:
                while True:
                    if not self.queueS2_S3.empty():
                        Condition_Close = self.queueS2_S3.get()
                        if Condition_Close=="Close":
                            filter_appl.stop()
                            print("GUI is Terminated, Condition send path: GUI -> Distributer -> Detect")
                            break
            except KeyboardInterrupt:
                print("KeyboardInterrupt")
                filter_appl.stop()
                pass
            
            finally:
                file_writer.close() # Close the file writer after GUI termination
                # Close the connection to the device when the device is opened
                if self.dev.status.state == DeviceState.connected:
                    print("\nDevice is Closed")
                    self.dev.close()
                    
        elif self._SRT_Plot: # Open a Widget and Plot  Semi Real Time Data
            try:
                plotter_app = QtWidgets.QApplication.instance()  # Check if there is already a plotter application in existence
                if not plotter_app: # Initialise the plotter application if there is no other plotter application
                    plotter_app = QtWidgets.QApplication(sys.argv)
                
                # Initialise filter and Measurement Build in 
                filter_appl = kaan_GUI_Algorithm.Classification_Algorithm_SRT(self.dev, self._save_Event, self.queueS2_S3, self.queueS3_S2)
                
                plot_window = plotters.RealTimePlot(figurename = 'A RealTimePlot',device = self.dev, 
                                                    channel_selection = [1,2,3,4,8],filter_app = filter_appl)  
                plot_window.show()
                plotter_app.exec_()
                QtWidgets.QApplication.quit()
                del plotter_app
                
            except TMSiError as e:
                print("!!! TMSiError !!! : ", e.code)
                if (e.code == TMSiErrorCode.device_error) :
                    print("  => device error : ", hex(self.dev.status.error))
                    DeviceErrorLookupTable(hex(self.dev.status.error))
                            
            finally:
                file_writer.close() # Close the file writer after GUI termination
                # Close the connection to the device when the device is opened
                if self.dev.status.state == DeviceState.connected:
                    print("\nDevice is Closed")
                    self.dev.close()        
             
                
###############################################################################          
if __name__=="__main__":
    TMSi_Connect()
           
        
   