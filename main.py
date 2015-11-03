#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ZetCode Tkinter tutorial

In this script, we use the grid manager
to create a skeleton of a calculator.

author: Jan Bodnar
last modified: December 2010
website: www.zetcode.com
"""

from Tkinter import Tk, W, E, Canvas
from Tkinter import *
from ttk import Button, Label, Style, Checkbutton, Treeview
from ttk import Frame, PanedWindow
from ttk import Entry


class Example(Frame):
  
    def __init__(self, parent):
        Frame.__init__(self, parent)   
        # Frames        
        self.top_frame = Frame(self)

        self.central_frame = Frame(self)

        self.bottom_frame = Frame(self)

        self.parent = parent        
        self.init_ui()
        
    def init_ui(self):
      
        self.parent.title("Rostopic GUI")
        
        Style().configure("TButton", padding=(0, 5, 0, 5), 
            font='serif 10')
        cb = Checkbutton(self.top_frame, text="publishers")

        # Main 
        self.init_frame_ui(self, 3, 1, 3)
        # Top
        self.init_frame_ui(self.top_frame, 1, 3, 3)
        # Bottom
        self.init_frame_ui(self.bottom_frame, 4, 1, 3)
        
        self.top_frame.grid(row=0, column=0)
        self.central_frame.grid(row=1, column=0, sticky="e")
        self.bottom_frame.grid(row=2, column=0)
        
        # Top content
        cb = Checkbutton(self.top_frame, text="publishers")
        cb.grid(row=0, column=0)
        cb = Checkbutton(self.top_frame, text="subscribers")
        cb.grid(row=0, column=1)
        
        # Left content
        w = Canvas(self.central_frame, bg='#FFFFFF', width=300,
                   height=300, scrollregion=(0,0,500,500))   
        hbar = Scrollbar(self.central_frame, orient=HORIZONTAL)
        hbar.pack(side=BOTTOM, fill=X)
        hbar.config(command=w.xview)     
        w.config(width=300, height=300)
        w.config(xscrollcommand=hbar.set)
        w.pack(side=LEFT, expand=True, fill=BOTH)

        w.create_line(0, 0, 200, 100)
        w.create_line(0, 100, 200, 0, fill="red", dash=(4, 4))

        w.create_rectangle(50, 25, 150, 75, fill="blue")

        

        
       
        self.pack()

    def init_frame_ui(self, newframe, nrows, ncolumns, padd):
        for i in xrange(0, nrows - 1):
            newframe.rowconfigure(i, pad=padd)
        for i in xrange(0, ncolumns - 1):
            newframe.columnconfigure(i, pad=padd)

    def create_topics_list(self, parent):
        tv = Treeview(parent)
        tv['columns'] = ('msg_type', 'pubs', 'subs', 'rate')
        tv.heading("#0", text='topic_name', anchor='w')
        tv.column("#0", anchor="w")
        tv.heading('msg_type', text='msg_type')
        tv.column('msg_type', anchor='center', width=100)
        tv.heading('pubs', text='pubs')
        tv.column('pubs', anchor='center', width=60)
        tv.heading('subs', text='subs')
        tv.column('subs', anchor='center', width=60)
        tv.heading('rate', text='rate')
        tv.column('rate', anchor='center', width=60)
        return tv

def main():
  
    root = Tk()
    app = Example(root)
    root.mainloop()  


if __name__ == '__main__':
    main()  
