from tkinter import messagebox
import serial
import tkinter as tk
from tkinter import *
from tkinter import Canvas
from tkinter import ttk
import math
import serial.tools.list_ports
import struct

class Window(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.position_input = None
        self.master = master
        self.dark_mode = False
        self.serial_port = None
        self.init_window()

        self.connected = False
        self.bFreeWheel = True


        self.populate_combo_box()


    def set_needle_position(self, position):
        self.gauge.set_needle_position(position)

    def init_window(self):
        self.master.title("Motor Controller GUI")

        # Add a Windows drop-down menu
        menu = Menu(self.master)

        self.master.config(menu=menu)
        self.master.configure(background="#474757")  # set background to #474757

        ########################################################################################
        file = Menu(menu)
        file.add_command(label='Exit', command=self.client_exit)
        menu.add_cascade(label='File', menu=file)

        ########################################################################################
        # Add a label and drop-down menu for the control mode
        control_label = Label(self.master, text="Control Mode:", fg="white", bg="#474757")  # set foreground to white and background to #474757
        control_label.grid(row=0, column=0, padx=10, pady=10)
        self.control_var = StringVar()
        self.control_var.set("Free-Wheel")
        self.control_dropdown = OptionMenu(self.master, self.control_var, "Free-Wheel", "Position")
        self.control_dropdown.config(width=15, bg="gray", fg="white")  # set background to gray and foreground to white
        self.control_dropdown.grid(row=0, column=1, padx=10, pady=10)

        # Add a slider for position control and a text field for numerical input
        self.position_slider = ttk.Scale(self.master, from_=0, to=180, orient=HORIZONTAL, length=200)
        self.position_slider.grid(row=1, column=0, padx=10, pady=10)

        self.position_slider_label1 = Label(self.master, text="Use slider or textfield to set v or pos:", fg="white", bg="#474757")  # set foreground to white and background to #474757
        self.position_slider_label1.grid(row=1, column=1, padx=10, pady=10)
        self.position_slider_label2 = Label(self.master, text="Speed(Freewheel)/Position(pos):", fg="white", bg="#474757")  # set foreground to white and background to #474757
        self.position_slider_label2.grid(row=2, column=1, padx=10, pady=10)

        self.position_input = Entry(self.master, bg="gray", fg="white")  # set background to gray and foreground to white
        self.position_input.grid(row=2, column=0, padx=10, pady=10)
        self.position_input.insert(0,90)

        # Add text fields for KP and Ki
        self.kp_label = Label(self.master, text="KP:", fg="white", bg="#474757")  # set foreground to white and background to #474757
        self.kp_label.grid(row=3, column=0, padx=10, pady=10)
        self.kp_input = Entry(self.master, bg="gray", fg="white")  # set background to gray and foreground to white
        self.kp_input.grid(row=3, column=1, padx=10, pady=10)
        self.kp_input.insert(0,"1.0")

        self.ki_label = Label(self.master, text="Ki:", fg="white", bg="#474757")  # set foreground to white and background to #474757
        self.ki_label.grid(row=4, column=0, padx=10, pady=10)
        self.ki_input = Entry(self.master, bg="gray", fg="white")  # set background to gray and foreground to white
        self.ki_input.grid(row=4, column=1, padx=10, pady=10)
        self.ki_input.insert(0,"0.0")

        self.kd_label = Label(self.master, text="Kd:", fg="white", bg="#474757")  # set foreground to white and background to #474757
        self.kd_label.grid(row=5, column=0, padx=10, pady=10)
        self.kd_input = Entry(self.master, bg="gray", fg="white")
        self.kd_input.grid(row=5, column=1, padx=10, pady=10)
        self.kd_input.insert(0,"0.0")


        # Add a button to connect to the selected device
        self.connect_button = ttk.Button(self.master, text="Connect", command=self.connect_to_device)
        self.connect_button.grid(row=8, column=0, padx=10, pady=10)

        # Add a button to start the motor
        self.start_button = ttk.Button(self.master, text="Start Motor", command=self.start_motor)
        self.start_button.grid(row=8, column=1, padx=10, pady=10)

        # Add a label for the connection status
        self.connection_status = Label(self.master, text="Not Connected")
        self.connection_status.grid(row=9, column=0, padx=10, pady=10)

        # Add a button to toggle dark mode
        self.dark_mode_button = ttk.Button(self.master, text="Dark Mode", command=self.toggle_dark_mode)
        self.dark_mode_button.grid(row=9, column=1, padx=10, pady=10)

        #self.send_button = ttk.Button(self.master, text="send data", command=self.send_data())
        #self.send_button.grid(row=9, column=1, padx=10, pady=10)

        # Add a canvas for the gauge display
        self.gauge_canvas = Canvas(self.master, width=300, height=300)
        self.gauge = Gauge(self.master, width=200, height=200)
        self.gauge.grid(row=10, column=0, columnspan=2, padx=10, pady=10)



    def client_exit(self):
        exit()


    def populate_combo_box(self):
        # Populate the combo box with available serial ports
        self.ports = serial.tools.list_ports.comports()
        self.port_names = []
        for port in self.ports:
            self.port_names.append(port.device)
        self.port_var = StringVar()
        if self.port_names:
            self.port_var.set(self.port_names[0])
            self.port_dropdown = OptionMenu(self.master, self.port_var, self.port_names[0], *self.port_names)
        else:
            self.port_var.set("No Ports Found")
            self.port_dropdown = OptionMenu(self.master, self.port_var, "No Ports Found")
        self.port_dropdown.config(width=15)
        self.port_dropdown.grid(row=6, column=0, padx=10, pady=10)


    def connect_to_device(self):
        baud_rate = 115200
        port_name = self.port_var.get()
        if port_name == "No Ports Found":
            self.populate_combo_box()
            port_name = self.port_var.get()
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=1)
            self.connected = True
        except:
            self.connected = False

        if self.connected:
            self.connect_button["state"] = DISABLED
            self.start_button["state"] = NORMAL
            self.connection_status.config(text="Connected", fg="green")
        else:
            self.connection_status.config(text="Not Connected", fg="red")
            self.connect_button["state"] = NORMAL
            self.start_button["state"] = DISABLED





    def start_motor(self):
        command = ""

        # get params
        control_mode = self.control_var.get()
        if control_mode == "Free-Wheel":
            self.bFreeWheel = True
            command += "1,"
        elif control_mode == "Position":
            self.bFreeWheel = False
            command += "0,"
        else:
            self.bFreeWheel = True
            command += "1,"
            print("error default mode selected , check input")

        kp = self.kp_input.get()
        ki = self.ki_input.get()
        kd = self.kd_input.get()

        gains = "{:.8f},{:.8f},{:.8f}".format(float(kp), float(ki), float(kd))
        command += gains

        setpoint = self.position_input.get()
        command += ",{:.4f}".format(float(setpoint))
        print(command)
        print(len(command))

        #payload = struct.pack(">b4d",self.bFreeWheel,float(setpoint),float(kp),float(ki),float(kd))
        #print(payload)
        #print(len(payload))

        if self.serial_port:
            self.serial_port.write(command.encode())




    def toggle_dark_mode(self):
        self.dark_mode = not self.dark_mode
        self.init_window()
        if self.dark_mode:
            self.master.configure(background="#474757")
            #self.master.configure(fg="white")
        else:
            self.master.configure(background="light grey")
            #self.master.configure(fg="#474757")


    def client_exit(self):
        exit()


class Gauge(Canvas):from tkinter import Canvas
import math

class Gauge(Canvas):
    def __init__(self, master=None, width=300, height=300):
        Canvas.__init__(self, master, width=width, height=height, bg='#474757')
        self.width = width
        self.height = height
        self.create_oval(10, 10, self.width-10, self.height-10, fill='#474757', outline='#474757')
        self.create_oval(9, 9, self.width-9, self.height-9, fill='white', outline='#474757')
        self.needle = self.create_line(0, 0, 0, 0, width=2, fill="#FFC107")

    def set_needle_position(self, position):
        x = self.width / 2
        y = self.height / 2
        angle = math.radians(position)
        length = min(self.width, self.height) / 2.2
        x2 = x + math.sin(angle) * length
        y2 = y - math.cos(angle) * length
        self.coords(self.needle, x, y, x2, y2)



root = Tk()
app = Window(root)
root.mainloop()