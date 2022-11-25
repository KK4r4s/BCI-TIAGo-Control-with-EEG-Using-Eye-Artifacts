# -*- coding: utf-8 -*-
"""
Created on Tue Aug  2 14:47:43 2022

@author: Lenovo
"""


from scipy import signal
import numpy as np
import math
import matplotlib.pyplot as plt
from random import randint,seed
import os
class tools:
    def __init__(self):
        self.my_dict={10:{0:{"Idle":[[1.73,5.014],[13.04,14.796]],"LL":[[7.5,9.5],[38,41]]}, 
             1:{"Idle":[[20.5,23.5]],"LL":[[3.12,5.18],[32.5,34.5]]},
             2:{"Blink":[[1.46,2.238],[26.51,27.014]],"LL":[[22,26],[32.5,35]]}, 
             3:{"LL":[[13,17],[33,35.5]]},
             4:{"LR":[[10,12.7],[43,47]]}, 
             5:{"LR":[[3,6],[32,35]]}, 
             6:{"Blink":[[44.75,45.50],[53,55]],"LR":[[4,6.2],[49.5,53]]},
             7:{"LR":[[5,7.2],[28.5,31]]}, 
             24:{"DBlink":[[23.2,24.4]]},
             25:{"DBlink":[[27.75,29]]}, 
             26:{"DBlink":[[26.16,27]]},
             27:{"DBlink":[[18.4,19.4]]}},
         
         11:{0:{"LL":[[4,6],[30.8,33]],"Idle":[[0,2.208]]}, 
             1:{"LL":[[9.6,11.5 ],[15,17 ]]},
             2:{"LL":[[4.5,6 ],[12.5,14 ]],"Blink":[[1.2,1.9],[9.2,9.9]]}, 
             3:{"LL":[[11.5,13 ],[21.25,22.75 ]]},
             4:{"LR":[[2.2,4.3 ],[11.25,12.75 ]]}, 
             5:{"LR":[[18,20 ],[24,26.5 ]],"Idle":[[16.04,18.14]]}, 
             6:{"LR":[[4.5,6.5 ],[35.3,37 ]]},
             7:{"LR":[[3,5 ],[39,43.2 ]],"Blink":[[1.56,2.34],[15.2,16]]}, 
             24:{"DBlink":[[13.6,14.6 ]]},
             25:{"DBlink":[[15.4,16.2 ]]}, 
             26:{"DBlink":[[18.1,18.8 ]]},
             27:{"DBlink":[[15.6,16.4 ]]}},
         
         12:{0:{"LL":[[2,3.75 ],[37,39 ]],"Blink":[[4.6,5.4],[20.26,20.99]]}, 
             1:{"LL":[[3.25,4.5 ],[29,30.5 ]]},
             2:{"LL":[[2,3.5 ],[33,34.5 ]],"Idle":[[3.296,4.56]]}, 
             3:{"LL":[[23.2,24.5 ],[45,47 ]]},
             4:{"LR":[[1.75,3.25 ],[16.2,17.6 ]]}, 
             5:{"LR":[[13,15.3 ],[32,34.06 ]]}, 
             6:{"LR":[[9.5,11.29 ],[26.75,28.4 ]]},
             7:{"LR":[[3.5,5 ],[23.75,25.5 ]],"Blink":[[1.666,2.5],[12.7,13.6]]}, 
             24:{"DBlink":[[17.25,18.25 ]],"Idle":[[15.164,17.238]]},
             25:{"DBlink":[[19.2,20.4 ]]}, 
             26:{"DBlink":[[19,20 ]]},
             27:{"DBlink":[[26.3,27.3 ]]}},
         
         14:{0:{"LL":[[29.5,31 ],[41.75,43.25 ]],"Blink":[[3.5,4.2],[24.6,25.6]]}, 
             1:{"LL":[[24.5,26 ],[49,51 ]]},
             2:{"LL":[[12.5,14.5 ],[40,42.75 ]]}, 
             3:{"LL":[[2.5,4.5 ],[30.6,34 ]]},
             4:{"LR":[[16,18 ],[49,51.3 ]]}, 
             5:{"LR":[[1.5,4 ],[34,36.5 ]],"Blink":[[0.5,1.5],[20.15,21]]}, 
             6:{"LR":[[2,3.8 ],[37.5,39.2 ]]},
             7:{"LR":[[2,3.75 ],[25.4,27.4 ]],"Idle":[[26.85,28.5]]}, 
             24:{"DBlink":[[14.6,15.6 ]]},
             25:{"DBlink":[[17.5,18.75 ]]}, 
             26:{"DBlink":[[19.4,20.4 ]]},
             27:{"DBlink":[[18.6,19.8 ]],"Idle":[[0,1.6]]}}
         }

    # Notch Filter
    def notch_filter(self,val, data, fs, order=3):
        notch_freq_Hz = np.array([float(val)])
        for freq_Hz in np.nditer(notch_freq_Hz):
            bp_stop_Hz = freq_Hz + 10.0 * np.array([-1, 1])
            b, a = signal.butter(order, bp_stop_Hz / (fs / 2.0), btype='bandstop')
            fin = data = signal.lfilter(b, a, data)
        return fin

    # High Pass Filter
    def butter_highpass_filtfilt(self,data, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = signal.butter(order, normal_cutoff, btype="highpass", analog=False)
        y = signal.filtfilt(b, a, data)
        return y

    # Low Pass Filter
    def butter_lowpass_filtfilt(self,data, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = signal.butter(order, normal_cutoff, btype="lowpass", analog=False)
        y = signal.filtfilt(b, a, data)
        return y

    # Band Pass Filter
    def butter_bandpass_lfilter(self,data, lowcut, highcut, fs, order=5):
        b, a = signal.butter(order, [lowcut, highcut], fs=fs, btype='band')
        y = signal.lfilter(b, a, data)
        return y
    def butter_bandpass_filtfilt(self,data, lowcut, highcut, fs, order=5):
        b, a = signal.butter(order, [lowcut, highcut], fs=fs, btype='band')
        y = signal.filtfilt(b, a, data)
        return y

    # Down Sample a Signale
    def DownSample_Signal(self,data, newfs, fs):
        const = fs/newfs
        if isinstance(const, float):
            const = round(const)
        newdata = np.empty((data.shape[0], math.floor(data.shape[1]/const)))
        for i in range(data.shape[0]):
            for j in range(0, newdata.shape[1]):
                newdata[i, j] = data[i, j*const]
        return newdata  
            
    # Scale the Signal
    def Scale_Signal(self,data):
        from sklearn.preprocessing import StandardScaler
        scaler = StandardScaler()
        scaled = scaler.fit_transform(data)
        return scaled

    #Color Switch for Plot
    def rgb_to_hex(self,r, g, b):
        """
        named_color = rgb_to_hex(randint(randint(40,100),200),randint(50,randint(140,200)),randint(50,200))
        """
        return ('#{:X}{:X}{:X}').format(r, g, b)

    def Signal_Plot(self,data,freq,Name_Sig,Ch_N,Chan_Num):
        time=np.arange(0,data.shape[1])/freq
        fig, ax = plt.subplots(1,1, figsize=[15,8])
        fig.suptitle(Name_Sig)
        
        ax.plot(time,data[Chan_Num],self.rgb_to_hex(120,100,100))
        ax.set_xlabel("Time [sec]")
        ax.set_ylabel("Amplitude [microVolt]")
        ax.set_title(Ch_N[Chan_Num])
     
    def Multiple_Ch_Plot(self,data,freq,Name_Sig,Ch_N,d_Ch=[0,15,16,2,18,17],SF=False):
        """
        Parameters
        ----------
        data : TYPE
            DESCRIPTION.
        freq : TYPE
            DESCRIPTION.
        Name_Sig : TYPE
            DESCRIPTION.
        Ch_N : TYPE
            DESCRIPTION.
        d_Ch : TYPE, optional
            DESCRIPTION. The default is [0,15,16,2,18,17].

        Returns
        -------
        Plot
        """
        time=np.arange(0,data.shape[1])/freq
        col=2
        row=math.ceil(len(d_Ch)/2)
        fig, ax = plt.subplots(row,col, figsize=[18,9])
        fig.suptitle(Name_Sig)
        k=0
        x=randint(randint(80,160),200)
        for j in range(col):
            for i in range(row):
                ax[i,j].plot(time,data[d_Ch[k]],self.rgb_to_hex(x,100,100))
                ax[i,j].set_xlabel("Time [sec]")
                ax[i,j].set_ylabel("Amplitude [$\mu$V]")
                ax[i,j].set_title(Ch_N[d_Ch[k]])
                fig.tight_layout()
                k+=1
                if k==len(d_Ch):
                    break
        if len(d_Ch)%2:
            fig.delaxes(ax[i+1,j])
        if SF:
            fig.savefig('C:/Users/Lenovo/Desktop/'+Name_Sig+str(d_Ch)+'.png')
            print('C:/Users/Lenovo/Desktop/'+Name_Sig+str(d_Ch)+'.png')
            
            
    def STFT(self,Data,Data_time,f_sample,Ch_Name,Nperseg=25,Vmax=25,Vmin=0): 
        """
        myTools.STFT(Data=Chosen_Windows[0][3,:],Data_time=Chosen_Window_times[0],f_sample=500,Ch_Name=Ch_Names[3])

        STFTs can be used as a way of quantifying the change of a nonstationary 
        signal’s frequency and phase content over time.
        
        The EEG signal was sampled using a sampling frequency of 50Hz and analyzed 
        using Short-time Fourier transform.
        """
        f_STFT, t_STFT, Zxx = signal.stft(Data, fs=f_sample, nperseg=Nperseg,boundary="even", padded=True)
        shift=Data_time[-1] - t_STFT[-1]
        t_STFT=t_STFT+shift
        
        fig,ax = plt.subplots(2,1,figsize=(15,10))
        c=ax[0].pcolormesh(t_STFT, f_STFT, np.abs(Zxx), vmin=Vmin, vmax=Vmax, shading='gouraud',cmap ='jet')
        #c=ax[0].pcolormesh(t_STFT, f_STFT, (np.abs(Zxx)), vmin=(np.abs(Zxx)).min(), vmax=(np.abs(Zxx)).max(), shading='gouraud',cmap ='jet')
        print(len(t_STFT),len(Data_time))
        cax = fig.add_axes([0.9,0.53,0.05,0.35])
        fig.colorbar(c, cax=cax, orientation='vertical')
        ax[0].set_title('STFT Magnitude')
        ax[0].set_ylabel('Frequency [Hz]')
        ax[0].set_xlabel('Time [sec]')
        ax[0].set_ylim([0,25])
         
        time_DS=Data_time
        ax[1].set_title('Raw Down Sampled Signal')
        ax[1].plot(time_DS,Data,label=Ch_Name)
        ax[1].set_xlabel("Time")
        ax[1].set_ylabel("microVolt")
        ax[1].set_xlim([time_DS[0],time_DS[-1]])
        ax[1].legend()
            



    def Retrieve(self,Path_Name,Directory_Name):
        for KDN in Directory_Name:
            KDN_F="".join(Path_Name+[KDN])
            
        f_P5 = []
        f_ex = []
        order = []
        for path, subdirs, files in os.walk(KDN_F):
            for filename in files:
                if filename.split(".")[-1].lower() == "csv":
                    f = os.path.join(path, filename)
                    f_ex.append(str(f) + os.linesep)
                    order.append(int(filename.split("-")[0].split("_")[-1]))
                if filename.split(".")[-1].lower() == "poly5":
                    f = os.path.join(path, filename)
                    f_P5.append(str(f) + os.linesep)
                    # order.append(int(filename.split("-")[0].split("_")[-1]))
                    
        ord_ind = []
        num = 0
        while True:
            ord_ind.append(order.index(num))
            if num == max(order):
                break
            num += 1
            
        filename_Poly5 = []
        filename_excel = []
        for i in ord_ind:
            filename_Poly5.append(f_P5[i])
            filename_excel.append(f_ex[i])
        
        f_P5_R = []
        f_ex_R = []
        for text in filename_Poly5:
            f_P5_R.append("/".join(text.split("\\")).split("\r")[0])
        for text in filename_excel:
            f_ex_R.append("/".join(text.split("\\")).split("\r")[0])
        
        return f_P5_R, f_ex_R 
  
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 

