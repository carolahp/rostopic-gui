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

from Tkinter import Tk, W, E
from ttk import Button, Label, Style, Checkbutton, Treeview
from ttk import Frame, PanedWindow
from ttk import Entry


class Example(Frame):
  
    def __init__(self, parent):
        Frame.__init__(self, parent)   
        # Frames        
        self.top_frame = Frame(self)

        self.central_frame = PanedWindow(self)

        self.left_frame = Frame(self)
        self.right_frame = Frame(self)

        self.parent = parent        
        self.init_ui()
        
    def init_ui(self):
      
        self.parent.title("Rostopic GUI")
        
        Style().configure("TButton", padding=(0, 5, 0, 5), 
            font='serif 10')
        cb = Checkbutton(self.top_frame, text="publishers")

        # Main 
        self.init_frame_ui(self, 2, 1, 3)
        # Top
        self.init_frame_ui(self.top_frame, 1, 3, 3)
        # Left
        self.init_frame_ui(self.left_frame, 3, 1, 3)
        # Right
        self.init_frame_ui(self.right_frame, 4, 1, 3)
        
        self.top_frame.grid(row=0, column=0)
        self.central_frame.grid(row=1, column=0)
        self.central_frame.add(self.left_frame)
        self.central_frame.add(self.right_frame)
        
        
        # Top content
        cb = Checkbutton(self.top_frame, text="publishers")
        cb.grid(row=0, column=0)
        cb = Checkbutton(self.top_frame, text="subscribers")
        cb.grid(row=0, column=1)
        
        # Left content
        e_search = Entry(self.left_frame)
        e_search.grid(row=0, column=0)
        tv = self.create_topics_list(self.left_frame)
        tv.grid(row=1, column=0)

        # Right content
        topic_title = Label(self.right_frame, text="Hello Caro")
        topic_title.grid(row=0, column=0)
        

        
       
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
