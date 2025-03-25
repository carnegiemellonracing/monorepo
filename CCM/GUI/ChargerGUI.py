from tkinter import *
import tkinter.font
from time import sleep
from PIL import Image, ImageTk

root = Tk()
root.geometry('1024x600')
root.title("Charger Screen")
root.configure(background="black")


maxCellV = StringVar()
minCellV = StringVar()
chargeI = StringVar()
bmsState = StringVar()
bmsMode = StringVar()
cellT = StringVar()
ch1 = StringVar()
ch2 = StringVar()
ch3 = StringVar()
ch4 = StringVar()
starttext= StringVar()
seg1data = StringVar()
seg2data = StringVar()
seg2data = StringVar()
seg4data = StringVar()
seg5data = StringVar()
seg6data = StringVar()


maxCellV.set("2700")
minCellV.set("2100")
chargeI.set("0")
bmsState.set("1")
bmsMode.set("1")
cellT.set("120")
ch1.set("good")
ch2.set("good")
ch3.set("absent")
ch4.set("absent")
starttext.set("Select Charge Method")
seg1data.set("12")
seg2data.set("13")
seg2data.set("16")

load = Image.open('CMR_LOGO.png')
# Resize image to fit
load = load.resize((420, 170), Image.ANTIALIAS)
render = ImageTk.PhotoImage(load)
img = Label(image=render, width = 420, height = 170, bg="black")
img.image = render
img.place(x=580, y=5)

def quitALL():
    try: 
        infowin.destroy()
    except:
        pass
    root.destroy()

def slow():
    global starttext
    starttext.set("Start Slow Charging")
    start_button.configure(bg="#66ff66")
    print("slow")

def fast():
    global starttext
    starttext.set("Start Fast Charging")


def start():
    print("start")
    global maxCellV
    maxCellV.set("4")
    global starttext
    if starttext == "Start Slow Charging":
        starttext.set("Slow Charging")
    if starttext == "Start Fast Charging":
        starttext.set("Fast Charging")
    chargeI.set("6")

def info():
    infowin = Toplevel()
    infowin.geometry('600x450')
    infowin.title("Additional Information")
    infowin.configure(bg="black")

    spacelabel = Label(infowin, text="  ")
    spacelabel.grid(row=0,column=0)

    spacelabel2 = Label(infowin, text="  ")
    spacelabel2.grid(row=8,column=0)

    minVoltlabel = Label(infowin, text="Min Voltage  ")
    minVoltlabel.grid(row=1, column=2, sticky=W)

    maxVoltlabel = Label(infowin, text="Max Voltage  ")
    maxVoltlabel.grid(row=1, column=3, sticky=W)

    minTemplabel = Label(infowin, text="Min Temp  ")
    minTemplabel.grid(row=1, column=4, sticky=W)

    maxTemplabel = Label(infowin, text="Max Temp ")
    maxTemplabel.grid(row=1, column=5, sticky=W)

    segment1 = Label(infowin, text="Seg 1:")
    segment1.grid(row=2,column=1,sticky=E)

    segment2 = Label(infowin, text="Seg 2:")
    segment2.grid(row=3,column=1,sticky=E)

    segment3 = Label(infowin, text="Seg 3:")
    segment3.grid(row=4,column=1,sticky=E)

    segment4 = Label(infowin, text="Seg 4:")
    segment4.grid(row=5,column=1,sticky=E)

    segment5 = Label(infowin, text="Seg 5:")
    segment5.grid(row=6,column=1,sticky=E)

    segment6 = Label(infowin, text="Seg 6:")
    segment6.grid(row=7,column=1,sticky=E)

    dvoltage = Label(infowin, text="Voltage")
    dvoltage.grid(row=9,column=2,sticky=W)

    dcurrent = Label(infowin, text="Current")
    dcurrent.grid(row=9,column=3,sticky=W)

    charger1 = Label(infowin, text="Charger 1:")
    charger1.grid(row=10,column=1,sticky=E)

    charger2 = Label(infowin, text="Charger 2:")
    charger2.grid(row=11,column=1,sticky=E)

    charger3 = Label(infowin, text="Charger 3:")
    charger3.grid(row=12,column=1,sticky=E)
    
    charger4 = Label(infowin, text="Charger 4:")
    charger4.grid(row=13,column=1,sticky=E)
    
    s1data1 = Label(infowin, textvariable=seg1data)
    s1data1.grid(row=2, column=2)

    s1data2 = Label(infowin, textvariable=seg2data)
    s1data2.grid(row=3, column=2)

    s1data3 = Label(infowin, textvariable=seg2data)
    s1data3.grid(row=4, column=2)

    s1data4 = Label(infowin, textvariable=seg4data)
    s1data4.grid(row=5, column=2)

    s1data5 = Label(infowin, textvariable=seg5data)
    s1data5.grid(row=6, column=2)

    s1data6 = Label(infowin, textvariable=seg6data)
    s1data6.grid(row=7, column=2)

    s2data1 = Label(infowin, textvariable=seg1data)
    s2data1.grid(row=2, column=3)

    s2data2 = Label(infowin, textvariable=seg2data)
    s2data2.grid(row=3, column=3)

    s2data3 = Label(infowin, textvariable=seg2data)
    s2data3.grid(row=4, column=3)

    s2data4 = Label(infowin, textvariable=seg4data)
    s2data4.grid(row=5, column=3)

    s2data5 = Label(infowin, textvariable=seg5data)
    s2data5.grid(row=6, column=3)

    s2data6 = Label(infowin, textvariable=seg6data)
    s2data6.grid(row=7, column=4)

    s3data1 = Label(infowin, textvariable=seg1data)
    s3data1.grid(row=2, column=4)

    s3data2 = Label(infowin, textvariable=seg2data)
    s3data2.grid(row=3, column=4)

    s3data3 = Label(infowin, textvariable=seg2data)
    s3data3.grid(row=4, column=4)

    s3data4 = Label(infowin, textvariable=seg4data)
    s3data4.grid(row=5, column=4)

    s3data5 = Label(infowin, textvariable=seg5data)
    s3data5.grid(row=6, column=4)

    s3data6 = Label(infowin, textvariable=seg6data)
    s3data6.grid(row=7, column=4)

    s4data1 = Label(infowin, textvariable=seg1data)
    s4data1.grid(row=2, column=5)

    s4data2 = Label(infowin, textvariable=seg2data)
    s4data2.grid(row=3, column=5)

    s4data3 = Label(infowin, textvariable=seg2data)
    s4data3.grid(row=4, column=5)

    s4data4 = Label(infowin, textvariable=seg4data)
    s4data4.grid(row=5, column=5)

    s4data5 = Label(infowin, textvariable=seg5data)
    s4data5.grid(row=6, column=5)

    s4data6 = Label(infowin, textvariable=seg6data)
    s4data6.grid(row=7, column=5)


    infolabels = [spacelabel,spacelabel2,minVoltlabel,maxVoltlabel,minTemplabel,maxTemplabel,segment1,segment2,segment3,segment4,segment5,segment6,dvoltage,dcurrent,charger1,charger2,charger3,charger4]
    for wid in infolabels:
        wid.configure(bg = "black", fg="white", font=("Montserrat", 14))
    infodata = [s1data1,s1data2,s1data3,s1data4,s1data5,s1data6,s2data1,s2data2,s2data3,s2data4,s2data5,s2data6,s3data1,s3data2,s3data3,s3data4,s3data5,s3data6,s4data1,s4data2,s4data3,s4data4,s4data5,s4data6]
    for wid in infodata:
        wid.configure(bg = "black", fg="chartreuse", font=("Montserrat", 14))




titlelabel = Label(text="  ")
titlelabel.grid(row=0,column=0)

cVlabel = Label(text="Cell Voltage:    Max:")
cVlabel.grid(row=1, column=1, sticky=W)

cVminlabel = Label(text="  Min:")
cVminlabel.grid(row=1, column=3,sticky=W)
        
cIlabel = Label(text="Charge Current:")
cIlabel.grid(row=2, column=1, sticky=W)

bmslabel = Label(text="BMS State: ")
bmslabel.grid(row=3, column=1, sticky=W)

bmsModelabel = Label(text="  Mode:")
bmsModelabel.grid(row=3, column=3,sticky=W)

cTlabel = Label(text="Max Cell Temp: ")
cTlabel.grid(row=4, column=1, sticky=W)

cVdata = Label(textvariable=maxCellV)
cVdata.grid(row=1, column=2, sticky=E)

cVmindata = Label(textvariable=minCellV)
cVmindata.grid(row=1, column=4, sticky=E)
        
cIdata = Label(textvariable=chargeI)
cIdata.grid(row=2, column=2, sticky=E )

bmsdata = Label(textvariable=bmsState)
bmsdata.grid(row=3, column=2, sticky=E)

bmsModedata = Label(textvariable=bmsMode)
bmsModedata.grid(row=3, column=4, sticky=E)

cTdata = Label(textvariable=cellT)
cTdata.grid(row=4, column=2, sticky=E)

spacer = Label()
spacer.grid(row=5,column=1)

statuslabel = Label(text="Charger Status:")
statuslabel.grid(row=6, column=1, sticky=W)

onestatlabel = Label(text="Charger 1:")
onestatlabel.grid(row=7, column=1, sticky=E)

twostatlabel = Label(text="Charger 2:")
twostatlabel.grid(row=8, column=1, sticky=E)

thrstatlabel = Label(text="Charger 3:")
thrstatlabel.grid(row=9, column=1, sticky=E)

foustatlabel = Label(text="Charger 4:")
foustatlabel.grid(row=10, column=1, sticky=E)

conedata = Label(textvariable=ch1)
conedata.grid(row=7, column=2)

ctwodata = Label(textvariable=ch2)
ctwodata.grid(row=8, column=2)

cthrdata = Label(textvariable=ch3)
cthrdata.grid(row=9, column=2)

cfoudata = Label(textvariable=ch4)
cfoudata.grid(row=10, column=2)

stepsmessage = Message(text="Steps to charge:                                                                      1) charge it                                                                                        2) stop", width = 450)
stepsmessage.place(x=550, y= 200)

slow_button = Button(text="Slow Mode", command=slow, width=20, pady=40)
slow_button.place(x=80, y=375)
slow_button.configure(bg="#66ff66", font=("Monstserrat", 18), fg="black")

start_button = Button(textvariable=starttext, command=start, width=20, pady=40)
start_button.place(x=394, y=375)
start_button.configure(bg="#ffd24d", font=("Monstserrat", 18), fg="black")

fast_button = Button(text="Fast Mode", command=fast, state=DISABLED, width=20, pady=40)
fast_button.place(x=708, y=375)
fast_button.configure(bg="#ffd24d", font=("Monstserrat", 18))
#buttons are 235 by 110 pixels

info_button = Button(text="More Info", command=info, width=12, pady=10)
info_button.place(x=780, y=522)
info_button.configure(bg="white", font=("Monstserrat", 14), fg="black")

close_button = Button(text="Close", command=quitALL)
close_button.place(x=5, y=575)

myWidgets = [onestatlabel,twostatlabel,thrstatlabel,foustatlabel,statuslabel,stepsmessage,titlelabel,cVlabel,cVminlabel,cIlabel,bmslabel,bmsModelabel,cTlabel,spacer]
mydata = [cTdata,conedata,ctwodata,cthrdata,cfoudata,bmsdata,bmsModedata,cIdata,cVdata,cVmindata]
for wid in myWidgets:
    wid.configure(bg = "black", fg="white", font=("Montserrat", 18))
for wid in mydata:
    wid.configure(bg ="black", fg ="chartreuse", font=("Monstserrat", 18))
root.mainloop()
