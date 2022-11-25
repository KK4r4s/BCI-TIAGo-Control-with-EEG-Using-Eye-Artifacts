# -*- coding: utf-8 -*-
"""
Created on 19/09/2022 12:45 PM

@author: Kaan Karas
"""
import time
import PySimpleGUI as sg
import io 
import os
from PIL import Image

from sys import platform

class GUI_Image:
    def __init__(self):
        super(GUI_Image,self).__init__()
        self.main_page_button={"0":"RobotControl","1":"TaskSelection"}
        self.w_t=0.5 # Sleep when button pressed 

        # self.main()
        
    def main(self,queueS1_S2, queueS2_S1,back_case=None):
        sg.theme('DarkBlue12')
        queueS1_S2.put(["main","None",""])
        
        layout = [[sg.VPush()], 
                  [sg.Text("\t\tWelcome to Blink Detection Algorithm GUI", font = ("Arial", 16))],
                  [sg.VPush()],  
                  [sg.pin(sg.Button("Move TiaGo with \nEye Movements and Eye Blinks",
                                    size=(30, 2),font = ("Arial", 12),
                                    pad=(100, 0),
                                    button_color=('black', 'salmon'),
                                    enable_events=True,
                                    key="0")),
                  sg.pin(sg.Button("Activate Predefined Tasks",
                                   size=(30, 2),font = ("Arial", 12),
                                   pad=(0, 0),
                                   button_color=('black', 'lightsteelblue'),
                                   enable_events=True,
                                   key="1"))],
                  [sg.VPush()], 
                  [sg.Text("To Click a Button Blink Twice",font = ("Arial", 12))],
                  [sg.Text("To Move Between Buttons Look Right or Left",font = ("Arial", 12))],
                  [sg.VPush()]
                  ]
        
        window = sg.Window("Kaan's Blink Application", layout,size=(1000, 800),margins=(10,0),grab_anywhere=True)
        
        while True:
            event, values =window.read(timeout=50)
            if back_case==True:
                back_case=None
                queueS1_S2.put(["main","None","0"])
                
            if not queueS2_S1.empty():
                dist_value=queueS2_S1.get()
                # page=dist_value[0]
                main_event=dist_value[1]
                Pressed=dist_value[2]

                if main_event=='0':
                    window.find_element("0").Update(button_color=('black', 'salmon'))
                    window.find_element("1").Update(button_color=('black', 'lightsteelblue'))
                    queueS1_S2.put(["main","None","0"])
                elif main_event=='1':
                    window.find_element("1").Update(button_color=('black', 'salmon'))
                    window.find_element("0").Update(button_color=('black', 'lightsteelblue'))
                    queueS1_S2.put(["main","None","1"])
                
                if Pressed=="Pressed":
                    self.open_window(window,main_event,queueS1_S2, queueS2_S1)
                elif Pressed=="Done" :
                    break
                    
                window.refresh()
                time.sleep(0.1)

            if event in (sg.WIN_CLOSED, 'Exit'):
                queueS1_S2.put(["main","Exit",""])
                break
        window.close()   
#------------------------------------------------------------------------------
    def open_window(self,wndw,button,queueS1_S2, queueS2_S1):
        queueS1_S2.put([self.main_page_button[button],"None",""])
        self.CCW="00"
        self.CW="00"
        if button=="0":
            wndw.close()
            layout = [[sg.VPush()], 
                      [sg.Text("\t\tWelcome to Move TiaGo with Eye Movements and Eye Blinks", font = ("Arial", 14))],
                      [sg.VPush()],
                      [sg.VPush()],
                      [sg.Button("Forward",size=(20, 2),font = ("Arial", 12),
                                        pad=(375, 0), button_color=('black', 'lightsteelblue'),
                                        enable_events=True,
                                        key="00")],
                      [sg.VPush()], 
                      [sg.pin(sg.Button("Turn Left",
                                       size=(10, 2),font = ("Arial", 12),
                                       pad=(100, 0), button_color=('black', 'lightsteelblue'),
                                       enable_events=True, key="01")),
                       sg.pin(sg.Button("00",size=(10, 2),font = ("Arial", 12),
                                        pad=(100, 0), button_color=('black', 'salmon'),
                                        enable_events=False, key="02")),
                       sg.pin(sg.Button("Turn Right",
                                        size=(10, 2),font = ("Arial", 12),
                                        pad=(100, 0), button_color=('black', 'lightsteelblue'),
                                        enable_events=True, key="03"))],
                      [sg.VPush()],
                      [sg.VPush()], 
                      [sg.VPush()], 
                      [sg.Text("To Go Forward Blink Twice",font = ("Arial", 12))],
                      [sg.Text("To Right or Left, Look Right or Left",font = ("Arial", 12))],
                      [sg.Text("Blink Four Times To Return Previous Screen",font = ("Arial", 12))],
                      [sg.Button("Back",size=(10, 2),font = ("Arial", 12),
                                       pad=(0, 0),button_color=('black', 'lightsteelblue'),
                                       enable_events=True, key="5")],
                      [sg.VPush()]
                      ]
        elif button=="1":
            wndw.close()
            layout = [[sg.VPush()], 
                      [sg.Text("\t\tWelcome to Activate Predefined Tasks", font = ("Arial", 16))],
                      [sg.VPush()],  
                      [sg.VPush()],
                      [sg.VPush()],
                      [sg.pin(sg.Button("Call Nurse",size=(10, 2),font = ("Arial", 14),
                                       pad=(75, 0),button_color=('black', 'salmon'),
                                       enable_events=True, key="10")),
                       sg.pin(sg.Button("Go To Table",size=(10, 2),font = ("Arial", 14),
                                        pad=(50, 0),button_color=('black', 'lightsteelblue'),
                                        enable_events=True, key="11")),
                       sg.pin(sg.Button("Scan",size=(10, 2),font = ("Arial", 14),
                                        pad=(50, 0),button_color=('black', 'lightsteelblue'),
                                        enable_events=True, key="12")),
                      sg.pin(sg.Button("Dance",size=(10, 2),font = ("Arial", 14),
                                       pad=(50, 0),button_color=('black', 'lightsteelblue'),
                                       enable_events=True, key="13"))],
                      [sg.VPush()], 
                      [sg.VPush()],
                      [sg.Text("To Click a Button Blink Twice",font = ("Arial", 12))],
                      [sg.Text("To Move Between Buttons Look Right or Left",font = ("Arial", 12))],
                      [sg.Text("Blink Four Times To Return Previous Screen",font = ("Arial", 12))],
                      [sg.Button("Back",size=(10, 2),font = ("Arial", 12),
                                       pad=(0, 0),button_color=('black', 'lightsteelblue'),
                                       enable_events=True, key="5")],
                      [sg.VPush()]
                      ]
        
        window = sg.Window("Kaan's Blink Application", layout,size=(1000, 800),margins=(10,0),grab_anywhere=True, modal=True)
        
        while True:
            event, values = window.read(timeout=50)
            
            if not queueS2_S1.empty():
                dist_value=queueS2_S1.get()
                # page=dist_value[0]
                Task_event=dist_value[1]
                Pressed=dist_value[2]
#  Welcome to Move TiaGo with Eye Movements and Eye Blinks - button=="0"-------------------------
                if button=="0":
                    
                    if Task_event == "01" :
                        window.find_element("01").Update(button_color=('black', 'salmon'))
                        window.find_element("03").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("02").Update(button_color=('black', 'lightsteelblue'))
                        if self.CW=="00": self.CW="360" 
                        self.CCW=str(int(self.CCW)+15); self.CW=str(int(self.CW)-15)
                        if self.CCW=="0" or self.CCW=="360": self.CCW="00"; self.CW="00"
                        elif abs(int(self.CCW))>360: self.CCW=str(abs(int(self.CCW))-360); self.CW=str(int(self.CW)+360)
                        if int(self.CCW)<=180: text=self.CCW + " Left"
                        elif int(self.CCW)>180: text=self.CW + " Right"
                        if text.split(" ")[0]=="00": text="00"; 
                        window["02"].update(text)
                        window.refresh()

                        # Robot Turn Left
                                                
                        self.action(window, self.w_t,"01")
                        
                    elif Task_event =="00":
                        window.find_element("00").Update(button_color=('black', 'salmon'))
                        window.find_element("02").Update(button_color=('black', 'lightsteelblue'))
                        window.refresh()
                        
                        # Robot Move Forward
                        
                        self.action(window,self.w_t,"00")
                        
                    elif Task_event == "03" :
                        window.find_element("03").Update(button_color=('black', 'salmon'))
                        window.find_element("01").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("02").Update(button_color=('black', 'lightsteelblue'))
                        if self.CCW=="00":  self.CCW="360"
                        self.CCW=str(int(self.CCW)-15); self.CW=str(int(self.CW)+15)
                        if self.CW=="0" or self.CW=="360": self.CCW="00"; self.CW="00"
                        elif abs(int(self.CW))>360: self.CW=str(abs(int(self.CW))-360);self.CCW=str(int(self.CCW)+360)    
                        if int(self.CW)<=180: text=self.CW + " Right"
                        elif int(self.CW)>180: text=self.CCW + " Left"
                        if text.split(" ")[0]=="00": text="00"
                        window["02"].update(text)
                        window.refresh()
                        
                        # Robot Turn Right
                        
                        self.action(window, self.w_t,"03")      
            
#  Welcome to Activate Predefined Tasks SCREEN - button=="1" -------------------       
                elif button=="1":
                    if Pressed=="Pressed":
                        if Task_event=="10":
                             

                             self.action(window, self.w_t,Task_event)
                        elif Task_event=="11":
                             
                             self.action(window, self.w_t,Task_event)
                        elif Task_event=="12":
                             
                             self.scan(window,queueS1_S2, queueS2_S1)
                        elif Task_event=="13":
                             
                             self.action(window, self.w_t,Task_event)
                             
                    elif Task_event == "10":
                        window.find_element("10").Update(button_color=('black', 'salmon'))
                        window.find_element("13").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("11").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("12").Update(button_color=('black', 'lightsteelblue'))
                        queueS1_S2.put(["TaskSelection","None","10"])
                        window.refresh()
                    elif Task_event == "11":
                        window.find_element("11").Update(button_color=('black', 'salmon'))
                        window.find_element("10").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("12").Update(button_color=('black', 'lightsteelblue'))
                        queueS1_S2.put(["TaskSelection","None","11"])
                        window.refresh()
                    elif Task_event == "12":
                        window.find_element("12").Update(button_color=('black', 'salmon'))
                        window.find_element("11").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("13").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("10").Update(button_color=('black', 'lightsteelblue'))
                        queueS1_S2.put(["TaskSelection","None","12"])
                        window.refresh()
                    elif Task_event == "13":
                        window.find_element("13").Update(button_color=('black', 'salmon'))
                        window.find_element("12").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("10").Update(button_color=('black', 'lightsteelblue'))
                        window.find_element("11").Update(button_color=('black', 'lightsteelblue'))
                        queueS1_S2.put(["TaskSelection","None","13"])
                        window.refresh()
                
                time.sleep(0.1)
                
                if Pressed=="Done" :
                    break
                elif Pressed=="Back":
                    window.close()
                    self.main(queueS1_S2, queueS2_S1,back_case=True)
                    
            if event in (sg.WIN_CLOSED, 'Exit'):
                last_page = [k for k, v in self.main_page_button.items() if v == button]
                queueS1_S2.put([last_page,"Exit",""])
                break
                
        window.close()
        
    def scan(self,wndw_scan,queueS1_S2, queueS2_S1):
        queueS1_S2.put(["Scan","None",""])
        wndw_scan.close()
        
        img_path="../../"
        img_f_name = "0.jpg"
        img_info = "0"
        N_Objects = 3

        image = Image.open(img_f_name)
        image.thumbnail((640, 480)) # Size the Image
        bio = io.BytesIO()
        image.save(bio, format="PNG")

        if platform == "linux" or platform == "linux2":
            path =  os.getcwd()
        elif platform == "win32": # Windows
            path =  os.getcwd()
            
        time.sleep(0.5)

        layout = [[sg.Text("\t\tTIAGO View", size=(60, 1),font = ("Arial", 14) , justification="center")],
                  [sg.VPush()],
                  [sg.Image(bio.getvalue(), key="cam1",pad=(200,0))],
                  [sg.VPush()], 
                  [sg.pin(sg.Button("Left",size=(10, 2),font = ("Arial", 14),
                                   pad=(100, 0),button_color=('black', 'lightsteelblue'),
                                   enable_events=True, key="20")),
                   sg.pin(sg.Button("Pick",size=(10, 2),font = ("Arial", 14),
                                    pad=(75, 0),button_color=('black', 'salmon'),
                                    enable_events=True, key="21")),
                   sg.pin(sg.Button("Right",size=(10, 2),font = ("Arial", 14),
                                    pad=(100, 0),button_color=('black', 'lightsteelblue'),
                                    enable_events=True, key="22"))],
                  [sg.VPush()],
                  [sg.Text("To Click a Button Blink Twice",font = ("Arial", 12))],
                  [sg.Text("To Move Between Buttons Look Right or Left",font = ("Arial", 12))],
                  [sg.Text("Blink Four Times To Return Previous Screen",font = ("Arial", 12))],
                  [sg.Button("Back",size=(10, 2),font = ("Arial", 12),
                                   pad=(0, 0),button_color=('black', 'lightsteelblue'),
                                   enable_events=True, key="5")],
                  [sg.VPush()]
                  ]
        window = sg.Window("Kaan's Blink Application", layout,size=(1000, 800), 
                            grab_anywhere=True, modal=True)   
        time.sleep(1)
        while True:
            event, values = window.read(timeout=50)
            if not queueS2_S1.empty():
                dist_value=queueS2_S1.get()
                # page=dist_value[0]
                Scan_event=dist_value[1]
                Pressed=dist_value[2]

                # print(img_f_name,img_info)

                if Scan_event=="20": # Left
                    window.find_element("20").Update(button_color=('black', 'salmon'))
                    window.find_element("21").Update(button_color=('black', 'lightsteelblue'))
                    window.find_element("22").Update(button_color=('black', 'lightsteelblue'))
                    
                    # change image
                    if img_f_name.split(".")[0]=="0": # Special case only for left 
                        img_f_name = str(N_Objects-1) + "."+ img_f_name.split(".")[1]
                        img_info = str(N_Objects-1)
                    else:
                        img_f_name = str(abs(int(img_f_name.split(".")[0])-1)%N_Objects) + "."+ img_f_name.split(".")[1]
                        img_info = str(abs(int(img_info)-1)%N_Objects)
                                        
                    if os.path.exists(path+"/"+img_f_name):
                        image = Image.open(img_f_name)
                        image.thumbnail((640, 480)) # Size the Image
                        bio = io.BytesIO()
                        image.save(bio, format="PNG")
                        window["cam1"].update(data=bio.getvalue())


                    

                    self.action(window, self.w_t, "20")
                elif Pressed=="Pressed": # Scan_event=="21": #  Also okay to use 
                    window.find_element("21").Update(button_color=('black', 'salmon'))
                    window.find_element("20").Update(button_color=('black', 'lightsteelblue'))
                    window.find_element("22").Update(button_color=('black', 'lightsteelblue'))

                    

                    self.action(window, self.w_t, "21")
                elif Scan_event=="22": # Right
                    window.find_element("22").Update(button_color=('black', 'salmon'))
                    window.find_element("21").Update(button_color=('black', 'lightsteelblue'))
                    window.find_element("20").Update(button_color=('black', 'lightsteelblue'))

                    # change image
                    img_f_name = str(abs(int(img_f_name.split(".")[0])+1)%N_Objects) + "."+ img_f_name.split(".")[1]
                    img_info = str(abs(int(img_info)+1)%N_Objects)
                                        
                    if os.path.exists(path+"/"+img_f_name):
                        image = Image.open(img_f_name)
                        image.thumbnail((640, 480)) # Size the Image
                        bio = io.BytesIO()
                        image.save(bio, format="PNG")
                        window["cam1"].update(data=bio.getvalue())

                    self.action(window, self.w_t, "22")
                
                if Pressed=="Done" :
                    break
                elif Pressed=="Back":
                    self.open_window(window,"1",queueS1_S2, queueS2_S1)
        
            if event in (sg.WIN_CLOSED, 'Exit'):
                queueS1_S2.put(["Scan","Exit",""])
                break
                  
        window.close()
#------------------------------------------------------------------------------
        
    def action(self,window, wait_time, key):
                
        self.disable(window,key,wait_time)
        self.enable(window,key,wait_time)
        
        if list(key)[0]=="0":
            window.find_element("02").Update(button_color=('black', 'salmon'))
            window.find_element(key).Update(button_color=('black', 'lightsteelblue'))
        elif list(key)[0]=="1":
            window.find_element(key).Update(button_color=('black', 'salmon'))
        elif list(key)[0]=="2":
            window.find_element(key).Update(button_color=('black', 'lightsteelblue'))
            window.find_element("21").Update(button_color=('black', 'salmon'))
        window.refresh()
        time.sleep(wait_time)
            
    def disable(self,window,key,wait_time):
        b_N=list(key)[0]
        window[b_N+'0'].update(disabled=True);window[b_N+'1'].update(disabled=True)
        if b_N=="0":
            window[b_N+'3'].update(disabled=True);window['5'].update(disabled=True)
        elif b_N=="1":
            window[b_N+'2'].update(disabled=True);window[b_N+'3'].update(disabled=True);window['5'].update(disabled=True)
        elif b_N=="2":
            window[b_N+'2'].update(disabled=True)
        window.refresh()  
        time.sleep(wait_time)
        
    def enable(self,window,key,wait_time):
        b_N=list(key)[0]
        window[b_N+'0'].update(disabled=False);window[b_N+'1'].update(disabled=False)
        if b_N=="0":
            window[b_N+'3'].update(disabled=False);window['5'].update(disabled=False)
        elif b_N=="1":
            window[b_N+'2'].update(disabled=False);window[b_N+'3'].update(disabled=False);window['5'].update(disabled=False)
        elif b_N=="2":
            window[b_N+'2'].update(disabled=False)
        window.refresh()
        
    

if __name__ == "__main__":
    GUI()
          
            
           
            
       
        
        
        
            
     
      
      
