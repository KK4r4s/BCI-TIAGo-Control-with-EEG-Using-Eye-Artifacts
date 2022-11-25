'''
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

TMSiSDK: Plotter object that displays an heatmap based on real-time computed
             RMS values, designed for HD-EMG applications.

'''

from PySide2 import QtGui, QtCore, QtWidgets
import numpy as np
import pyqtgraph as pg
import time
import queue
from scipy import signal, interpolate
import sys

from .. import tmsi_device
from .. import sample_data_server

from ..device import DeviceInterfaceType, ChannelType


class HDEMGPlot(pg.GraphicsLayoutWidget):
    """ Class that creates a GUI to display the impedance values in a gridded
        layout.
    """
    def __init__(self, figurename, device, tail_orientation, signal_lim):
        """ Setting up the GUI's elements
        """
        
        if sys.platform == "linux" or sys.platform == "linux2":
            print('This plotter is not compatible with the current version of the TMSi Python Interface on Linux (Ubuntu 18.04 LTS)\n')
            return
        
        pg.GraphicsLayoutWidget.__init__(self)
        pg.setConfigOptions(antialias = True)
        
        # Pass the device handle to the GUI
        self.device = device 
        
        ### ADD SOMETHING TO DETERMINE REFERENCE METHOD / NUMBER OF HW CHANNELS
        if self.device.config.num_channels > 64:
            self._EMG_chans = 64
        else:
            self._EMG_chans = 32
            
        if self.device.config.reference_method[0] == 'common':
            self._chan_offset = 1
        else:
            self._chan_offset = 0
        
        self.sample_rate = self.device.config.get_sample_rate(ChannelType.counter)
        
        # Update the title of the HD EMG plot
        self.setWindowTitle(figurename)
        self.setMinimumSize(600, 500)
        
        # Orientation of the grid on the body
        self.tail_orientation = tail_orientation
        
        # Upper limit colorbar
        self.signal_lim = signal_lim
        
        # Set up UI and thread
        self.initUI()
        self.setupThread()
        
    def initUI(self):
        """ Method responsible for constructing the basic elements in the plot.
            All viewboxes have a set size so that the information can be displayed
            correctly.
        """
        # Set view settings
        self.setBackground('w')
        # self.showMaximized()
        
        # Add plot window for the channels
        self.window = self.addPlot()
        self.window.setAspectLocked(lock=True, ratio = 1)
        
        # Write the ticks to the plot
        self.window.hideAxis('left')
        self.window.hideAxis('bottom')
        
        # Disable auto-scaling and menu
        self.window.hideButtons()
        self.window.setMenuEnabled(False)
        self.window.setMouseEnabled(x = False, y = False)
        
        # Ratio is slightly different between 32/64 channel setup
        if self._EMG_chans == 32:
            self._x_interpolate = np.arange(0, int((len(self.device.channels)-2 - self._chan_offset)/8) - 1 + .1, 0.1)
            self._y_interpolate = np.arange(0, int((len(self.device.channels)-2 - self._chan_offset)/(self._EMG_chans/8)) - 1 + .2, 0.1)
        else:
            self._x_interpolate = np.arange(0, int((len(self.device.channels)-2 - self._chan_offset)/8) - 1 + .2, 0.1)
            self._y_interpolate = np.arange(0, int((len(self.device.channels)-2 - self._chan_offset)/(self._EMG_chans/8)) - 1 + .2, 0.1)
        self._dummy_val = np.zeros((len(self._x_interpolate), len(self._y_interpolate)))
        
        if self.tail_orientation == 'Right' or self.tail_orientation == 'right':
            self._dummy_val = np.rot90(self._dummy_val, 1)
        elif self.tail_orientation == 'Left' or self.tail_orientation == 'left':
            self._dummy_val = np.rot90(self._dummy_val, 3)
            
        # Create image item
        self.img = pg.ImageItem(image = self._dummy_val) 
        self.window.addItem(self.img)
        
         # Prepare a linear color map
        cm = pg.colormap.get('CET-R4')
        
        # Insert a Colorbar, non-interactive with a label
        self.bar = pg.ColorBarItem(values = (0, self.signal_lim), colorMap=cm, interactive = False, label = 'RMS (\u03BCVolt)', )
        self.bar.setImageItem(self.img, insert_in = self.window)
        
        # Scale factor (number of interpolation points to cover a width of 3 or 7 columns)
        corr_x = len(self._x_interpolate) / (int((len(self.device.channels)-2-self._chan_offset)/8)-1)
        
        # Scale factor (number of interpolation points to cover a width of 7 rows)
        corr_y = len(self._y_interpolate) / 7
        
        # Initialise the standard format for the different indicators
        if self.tail_orientation == 'Left' or self.tail_orientation == 'left': 
            self.spots = [{'pos': ((i%8) * corr_y, int(i/8) * corr_x), 'size': 5, 'pen': 'k', 'brush': QtGui.QBrush(QtGui.QColor(128, 128, 128))} \
                              for i in range(self._EMG_chans)]
        elif self.tail_orientation == 'Up' or self.tail_orientation == 'up':
            self.spots = [{'pos': (int((-i + (self._EMG_chans-1))/8) * corr_y, i%8 * corr_x), 'size': 5, 'pen': 'k', 'brush': QtGui.QBrush(QtGui.QColor(128, 128, 128))} \
                              for i in range(self._EMG_chans)]
        elif self.tail_orientation == 'Right' or self.tail_orientation == 'right':
            self.spots = [{'pos': ((-i + (self._EMG_chans - 1))%8 * corr_y, int((-i + (self._EMG_chans-1))/8) * corr_x), 'size': 5, 'pen': 'k', 'brush': QtGui.QBrush(QtGui.QColor(128, 128, 128))} \
                              for i in range(self._EMG_chans)]
        elif self.tail_orientation == 'Down' or self.tail_orientation == 'down':
            self.spots = [{'pos': (int(i/8) * corr_y, (-i + (self._EMG_chans-1))%8 * corr_x), 'size': 5, 'pen': 'k', 'brush': QtGui.QBrush(QtGui.QColor(128, 128, 128))} \
                              for i in range(self._EMG_chans)]
    
            # Set the position for each indicator
        for i in range(len(self.device.channels)):
            if i > self._EMG_chans-1:
                break
            # Place the name of each channel below the respective indicator
            text = f'{self.device.channels[i+self._chan_offset].name: ^10}'
            t_item = pg.TextItem(text, (128, 128, 128), anchor=(0, 0))
            t_item.setPos(self.spots[i]['pos'][0] -.25, self.spots[i]['pos'][1] + .1)
            self.window.addItem(t_item)
         
                
        # Add all indicators to the plot
        self.c = pg.ScatterPlotItem(self.spots)
        self.window.addItem(self.c)

        self.window.invertY(True)
            
        
    @QtCore.Slot(object)
    def update_plot(self, data):
        """ Method that updates the indicators according to the measured impedance values
        """
        self.img.setImage(data, autoRange=False, autoLevels=False)


    def setupThread(self):
        """ Method that initialises the sampling thread of the device
        """
        # Create a Thread
        self.thread = QtCore.QThread()
        # Instantiate the worker class
        self.worker = SamplingThread(self)
        
        # Move the worker to a Thread
        self.worker.moveToThread(self.thread)
        
        # Connect signals to slots
        self.thread.started.connect(self.worker.update_samples)
        self.worker.output.connect(self.update_plot)
        
        # Start the thread
        self.thread.start()

    def closeEvent(self,event):
        """ Method that redefines the default close event of the GUI. This is
            needed to close the sampling thread when the figure is closed.
        """
        # Stop the worker and the thread
        self.worker.stop()
        self.thread.terminate()
        self.thread.wait()
        
        sample_data_server.unregisterConsumer(self.device.id, self.worker.q_sample_sets)
        

class SamplingThread(QtCore.QObject):
    """ Class responsible for sampling the data from the device
    """
    # Initialise the ouptut object
    output = QtCore.Signal(object)
    def __init__(self, main_class):
        QtCore.QObject.__init__(self)
        
        # Access initialised values from the GUI class
        self.device = main_class.device
        self.sample_rate = main_class.sample_rate
        self._EMG_chans = main_class._EMG_chans
        self._chan_offset = main_class._chan_offset
        
        self.window_buffer = np.zeros((len(self.device.channels)-2-self._chan_offset, self.sample_rate * 5))
        self.window_rms_size = self.sample_rate // 4
        self._add_final = 0
        
        self.sos = signal.butter(2, 10, 'highpass', fs=self.sample_rate, output='sos')
        z_sos0 = signal.sosfilt_zi(self.sos)
        self.z_sos = np.repeat(z_sos0[:, np.newaxis, :], len(self.device.channels)-2-self._chan_offset, axis=1)
        
        self._x_grid = np.arange(0, self._EMG_chans/8, dtype=int)
        self._y_grid = np.arange(0, 8, dtype=int)
        
        self._x_interpolate = main_class._x_interpolate
        self._y_interpolate = main_class._y_interpolate
        
        self.tail_orientation = main_class.tail_orientation
        
        # Prepare Queue
        self.q_sample_sets = queue.Queue(1000)
        
        # Register the consumer to the sample server
        sample_data_server.registerConsumer(self.device.id, self.q_sample_sets)
        
        # Start measurement
        self.device.start_measurement()
        self.sampling = True

    @QtCore.Slot()
    def update_samples(self): 
        """ Method that retrieves the sample data from the device. The method 
            gives the impedance value as output
        """
        while self.sampling:
            while not self.q_sample_sets.empty():
                
                # Retrieve sample data from the sample_data_server queue
                sd = self.q_sample_sets.get()
                self.q_sample_sets.task_done()
                
                # Reshape the samples retrieved from the queue
                samples = np.reshape(sd.samples, (sd.num_samples_per_sample_set, sd.num_sample_sets), order = 'F')
                self.new_samples = sd.num_sample_sets
                
                self.window_buffer[:, self._add_final:(self._add_final + self.new_samples)] = samples[self._chan_offset:-2,:]
                
                if self._add_final + self.new_samples > self.window_rms_size:
                    filt_data, self.z_sos = signal.sosfilt(self.sos, self.window_buffer[:, 0:self.window_rms_size], zi = self.z_sos)
                    
                    rms_data = np.sqrt(np.mean(filt_data**2, axis = 1))
                    
                    if self.tail_orientation == 'Left' or self.tail_orientation == 'left': 
                        rms_data = np.reshape(rms_data, (int(self._EMG_chans/8),8)).T
                        
                        f = interpolate.interp2d(self._x_grid, self._y_grid, rms_data, kind='linear')
                        output_heatmap = f(self._x_interpolate, self._y_interpolate)

                    elif self.tail_orientation == 'Up' or self.tail_orientation == 'up': 
                        rms_data = np.rot90(np.reshape(rms_data, (int(self._EMG_chans/8),8)).T, 1)
                        
                        f = interpolate.interp2d(self._y_grid, self._x_grid, rms_data, kind='linear')
                        output_heatmap = f(self._y_interpolate, self._x_interpolate)

                    elif self.tail_orientation == 'Right' or self.tail_orientation == 'right': 
                        rms_data = np.rot90(np.reshape(rms_data, (int(self._EMG_chans/8),8)).T, 2)
                        
                        f = interpolate.interp2d(self._x_grid, self._y_grid, rms_data, kind='linear')
                        output_heatmap = f(self._x_interpolate, self._y_interpolate)

                    elif self.tail_orientation == 'Down' or self.tail_orientation == 'down': 
                        rms_data = np.rot90(np.reshape(rms_data, (int(self._EMG_chans/8),8)).T, 3)
                        
                        f = interpolate.interp2d(self._y_grid, self._x_grid, rms_data, kind='linear')
                        output_heatmap = f(self._y_interpolate, self._x_interpolate)
                    
                    self._add_final = 0

                    self.window_buffer = np.hstack((self.window_buffer[:,self.window_rms_size:], np.zeros((len(self.device.channels)-2-self._chan_offset, self.window_rms_size)) ))
                    self.output.emit(output_heatmap)
                    
                else:
                    self._add_final += self.new_samples
                
            # Pause the thread so that the update does not happen too fast
            time.sleep(0.01)
            
    def stop(self):
        """ Method that is executed when the thread is terminated. 
            This stop event stops the measurement and closes the connection to 
            the device.
        """
        self.device.stop_measurement()
        self.sampling = False
        


if __name__ == "__main__":
    # Initialise the TMSi-SDK first before starting using it
    tmsi_device.initialize()
    
    # Create the device object to interface with the SAGA-system.
    dev = tmsi_device.create(tmsi_device.DeviceType.saga, DeviceInterfaceType.docked, DeviceInterfaceType.usb)

    # Find and open a connection to the SAGA-system and print its serial number
    dev.open()
    print("handle 1 " + str(dev.info.ds_serial_number))
    
    # Initialise the application
    app = QtWidgets.QApplication(sys.argv)
    # Define the GUI object and show it
    window = HDEMGPlot(figurename = 'An HDEMG Heatmap Plot', device = dev, tail_orientation='up')
    window.show()
    
    # Enter the event loop
    # sys.exit(app.exec_()) 
    app.exec_()
    dev.close()
