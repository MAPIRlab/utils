## SIMPLE GUI to view and process rosbag2 files
## ----------------------------------------------
import math
import threading
import os
import time
#import pyexcel
import operator
from pathlib import Path
import numpy as np
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk
)

# Images
from PIL import Image, ImageTk
from scipy.ndimage import zoom

# Geospatial data (GPS)
import rasterio     
from rasterio.plot import show, show_hist
from rasterio.enums import Resampling

# ROSbag reader/writter
from rosbags.highlevel import AnyReader 
from rosbags.rosbag2 import Reader

# TKinter GUI
import tkinter as tk
from tkinter import filedialog
import tkintermapview



class RosbagViewer(tk.Frame):

    def set_folder(self):
        dir = filedialog.askdirectory()
        if dir:
            self.label_dir.config(text="Folder: " + dir)
            self.dir=dir 
        else:
            self.label_dir.config(text="No data folder set")


    def process_gps(self):
        # create a rosbag reader instance and open it for reading
        topic_gps = "/hunter/fix"
        # variables
        gps_interval = 1
        cont_gps = 0
        
        with AnyReader([Path(self.bag_path)]) as reader:
            # Filter by Topic
            my_connections = [x for x in reader.connections if operator.contains([topic_gps], x.topic)]
            #total = my_connections[0].msgcount
            #print(f"Found {total} GPS messages")
   
            for connection, timestamp, rawdata in reader.messages(connections=my_connections):
                # deserialize data
                msg = reader.deserialize(rawdata, connection.msgtype)            
                cont_gps = cont_gps+1
                
                if (connection.msgtype=='sensor_msgs/msg/NavSatFix') and cont_gps >= gps_interval:
                    cont_gps = 0
                    gps = msg
                    self.markers.append([gps.latitude, gps.longitude])
                    #self.map_widget.set_marker(gps.latitude, gps.longitude, marker_color_circle="black", marker_color_outside="gray40")
                    #self.map_widget.set_path(self.markers)
                    # center map
                    #self.map_widget.set_position(gps.latitude, gps.longitude)
                    #self.markers.append(m)
                
                """
                if (connection.msgtype=='sensor_msgs/msg/Image') and connection.topic==topic_camera1 and cont_image>image_interval:
                    cont_image=0
                    w, h = msg.width, msg.height
                    
                    color_image = np.reshape(msg.data, (h, w, 3)) # for rgb image
                    
                    image = Image.fromarray(color_image)
                    image = image.resize((int(w/4),int(h/4)))
                    ##show_image    
                    self.axes[0].imshow(image)
                    self.figure_canvas.draw()
                        #show(color_image, cmap="RdYlGn",ax=self.axes[0],title='Camera#0')
                if (connection.msgtype=='sensor_msgs/msg/Image') and connection.topic==topic_camera2 and cont_image2>image_interval:
                    cont_image2=0
                    w, h = msg.width, msg.height
                    ##print(connection.msgtype)
                    
                    color_image = np.reshape(msg.data, (h, w, 3)) # for rgb image
                    
                    image = Image.fromarray(color_image)
                    
                    image = image.resize((int(w/4),int(h/4)))
                    ##show_image    
                    self.axes[1].imshow(image)
                    self.figure_canvas.draw()
                """    

        # Bag file complete (show path)
        self.map_widget.set_marker(self.markers[0][0], self.markers[0][1], marker_color_circle="green", marker_color_outside="gray40")
        self.map_widget.set_marker(self.markers[-1][0], self.markers[-1][1], marker_color_circle="red", marker_color_outside="gray40")
        self.map_widget.set_position(self.markers[0][0], self.markers[0][1])
        self.map_widget.set_path(self.markers)     
        
            
    def update_graph(self):
        # launch the rasbag process in a separate thread!
        thread = threading.Thread(target = self.process_gps)
        thread.start()


    def process_info(self):
        # clear previous content:
        self.rosbag_info.set("")
        with AnyReader([Path(self.bag_path)]) as reader:
            
            self.rosbag_info.set(self.rosbag_info.get() + "\n DURATION [s]: \t" + str(reader.duration/1e9))
            self.rosbag_info.set(self.rosbag_info.get() + "\n MSG COUNT: \t" + str(reader.message_count))
            self.rosbag_info.set(self.rosbag_info.get() + "\n TOPIC LIST [count] --> MSG_TYPE: ")
            for connection in reader.connections:
                #print(connection.topic, connection.msgtype)
                self.rosbag_info.set(self.rosbag_info.get() + "\n\t" + str(connection.topic) + "[" + str(connection.msgcount) +"]" + " --> " + str(connection.msgtype))                    


    def select_bag(self):
        # self.bag_path = filedialog.askdirectory()
        self.bag_path = '/home/jgmonroy/rosbag2_2024_02_20-13_02_11'                
        if self.bag_path:
            self.file_label.config(text="Selected file: " + self.bag_path)
            # Display bag info
            self.process_info()
        else:
            self.file_label.config(text="No file selected.")


    # Init Class
    def __init__(self, parent, *args, **kwargs):
        # Init tkinter
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent    # ??

        # configure GUI
        window = parent
        window.title("ROSBAG VIEWER")
        window.geometry('1280x720')        
        window["bg"] = "white"
        
        # Select Rosbag (Button)
        self.file_label = tk.Label(window, text="No file selected.")
        self.file_label.pack(pady=5)
        select_button = tk.Button(window, text="Select File", command=lambda: self.select_bag())
        select_button.pack(pady=5)

        # Rosbag Info (Text Display)
        self.rosbag_info = tk.StringVar()
        self.rosbag_info.set("")
        self.rosbag_info_label = tk.Label(window, textvariable=self.rosbag_info, anchor='w')
        self.rosbag_info_label.pack(pady=5, anchor='w')
        self.rosbag_info_label.pack(side=tk.LEFT)

        # Map viewer
        self.map_widget = tkintermapview.TkinterMapView(window, width=640, height=480, corner_radius=1)
        # tile sever:
        #self.map_widget.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")  # OpenStreetMap (default)
        #self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google normal
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite
        self.map_widget.set_position(36.71540989100557, -4.477567058872805)     # ETSI UMA
        self.map_widget.set_zoom(19)
        self.map_widget.pack(side=tk.RIGHT)
        self.map_widget["bg"] = "white"
        self.markers = []

        # Draw GPS path (Button)
        btn_start = tk.Button(window, text="SHOW GPS PATH", command=lambda: self.update_graph())
        btn_start.pack(side=tk.BOTTOM)

          # Images
        """
        self.figure = Figure(figsize=(12,6), dpi=100, tight_layout=True)
        self.figure_canvas = FigureCanvasTkAgg(self.figure, self)        
        self.axes = self.figure.subplots(2,1)  
        self.axes[0].set_xlabel('')
        self.axes[0].set_yticks([])
        self.axes[0].set_xticks([])
        self.axes[1].set_xlabel('')
        self.axes[1].set_yticks([])
        self.axes[1].set_xticks([])
        """

        # Add Logo :D
        """
        logo = Image.open('logo_mapir.png')
        displaylogo = ImageTk.PhotoImage(logo)
        labellogo = tk.Label(window, image=displaylogo)
        labellogo.image = displaylogo
        labellogo.pack(side=tk.TOP, fill=tk.X)
        labellogo["bg"] = "white"        
        
        self.figure_canvas.get_tk_widget().pack(side=tk.LEFT)
        self.figure_canvas.get_tk_widget()["bg"] = "white"
        """
        
        

if __name__ == "__main__":
    # Top level GUI
    root = tk.Tk()
    # Instance of RosbagViewer and initial GUI configuration
    RosbagViewer(root).pack(side="top", fill="both", expand=True)

    root.mainloop()