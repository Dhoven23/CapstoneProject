import serial

import wx

def send(word):
    data = bytearray()
    word = list(word)
    Dcode = ord(word[0])
    Ncode = ''.join(word[1:])
    data.append(0x20)
    data.append(0x20)
    data.append(Dcode)
    t1 = (int(Ncode)&255)
    t2 = ((int(Ncode)>>8)&255)
    data.append(t1)
    data.append(t2)
    ser.write(data)

def forward(event=None):
    send('w1')
def left(event = None):
    send('a1')
def backward(event=None):
    send('s1')
def right(event=None):
    send('d1')
def hover(event=None):
    send('H0')
def up(event=None):
    send('r1')
def down(event=None):
    send('f1')
def KILL(event=None):
    send('K0')

def OnEnterPressed(event):
    send(event.GetString())


app = wx.App()

window = wx.Frame(None, title = "wxPython Frame", size = (300,350))
panel = wx.Panel(window)
btnw = wx.Button(panel, label = "\nFWD\n", pos = (110,5))
btna = wx.Button(panel, label = "\nLFT\n", pos = (25,60))
btns = wx.Button(panel, label = "\nBWD\n", pos = (110,115))
btnd = wx.Button(panel, label = "\nRGT\n", pos = (195,60))
btnH = wx.Button(panel, label = "\nHVR\n", pos = (110,60))
btnUp= wx.Button(panel, label = "\nUP\n",  pos = (25,225))
btnDn= wx.Button(panel, label = "\nDN\n", pos = (195 ,225))
btnK = wx.Button(panel, label = "\nKILL!\n",pos = (110,225))

btnw.Bind(wx.EVT_BUTTON,forward)
btna.Bind(wx.EVT_BUTTON,left)
btns.Bind(wx.EVT_BUTTON,backward)
btnd.Bind(wx.EVT_BUTTON,right)
btnH.Bind(wx.EVT_BUTTON,hover)
btnUp.Bind(wx.EVT_BUTTON,up)
btnDn.Bind(wx.EVT_BUTTON,down)
btnK.Bind(wx.EVT_BUTTON,KILL)
label = wx.StaticText(panel,-1,"Enter Command:",pos = (15,202))
cmd = wx.TextCtrl(panel,style=wx.TE_PROCESS_ENTER,size = (50,20),pos = (175,199))

cmd.Bind(wx.EVT_TEXT_ENTER,OnEnterPressed)



ser = serial.Serial('/dev/cu.usbserial-D308LP97',baudrate=57600)

window.Show(True)
app.MainLoop()
