import os
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog as fd
from tkinter.messagebox import showinfo
from tkinter.messagebox import showerror
import pyperclip
import json

from FileLoad import FileLoad
from FileSave import FileSave
from VisualizePCD import VisualizePCD

class AppWithGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        # create the root window
        self.close_vis_button = None
        self.visualize_button = None
        self.save_button = None
        self.open_button = None
        self.title('Tkinter Open File Dialog')
        self.resizable(False, False)
        self.geometry('300x150')
        self.vis_pcd = None
        self.pcd = None

        # create the menu button
        self.create_menu_buttons()

        # Bind the Enter Key to the window
        self.bind('<Return>', self.save_json_file)

        # Call function to make the window stay on top
        # self.stay_on_top()

    # def stay_on_top(self):
    #     self.lift()
    #     self.after(2000, self.stay_on_top)

    def select_load_file(self):

        # self.points = None
        filetypes = (
            ('Points [x, y, z, i] (.pts)', '*.pts'),
            ('Point clouds[x, y, z, i] (.e57)', '*.e57'),
            ('Triangle mesh files (.ply, .stl, .fbx, .obj, .off, .gltf, .glb)', '.ply .stl .fbx .obj .off .gltf .glb'),
            ('Point cloud files (.xyz, .xyzn, .xyzrgb, .ply, .pcd, .pts)', '.xyz .xyzn .xyzrgb .ply .pcd .pts'),
            ('Polygon files (.ply)', '.ply'),
            ('Point clouds (.las, .laz)', '.las .laz'),
            ('text files', '*.txt'),
            ('All files', '*.*')
        )

        filename = fd.askopenfilename(
            title='Open a point cloud file',
            initialdir=os.getcwd(),
            filetypes=filetypes)

        if not filename:
            showerror(title='Error',
                      message='Point cloud filename is missing.')
        else:
            # showinfo(
            #     title='Selected File',
            #     message=filename
            # )
            load = FileLoad(filename)
            self.pcd = load.load_file()
            showinfo(
                title='Selected File successfully loaded',
                message=filename
            )
            print("-------PCD: \n",self.pcd)
            # print("------Points: \n",self.points)
            self.visualize_button.config(state="normal")
            self.close_vis_button.config(state="normal")
            self.save_button.config(state="normal")


    def select_save_file(self):
        filetypes = (
            ('Points [x, y, z, i] (.pts)', '*.pts'),
            ('Triangle mesh files (.ply, .stl, .fbx, .obj, .off, .gltf, .glb)', '.ply .stl .fbx .obj .off .gltf .glb'),
            ('Point cloud files (.xyz, .xyzn, .xyzrgb, .ply, .pcd, .pts)', '.xyz .xyzn .xyzrgb .ply .pcd .pts'),
            ('Polygon files (.ply)', '.ply'),
            ('text files', '*.txt'),
            ('All files', '*.*')
        )

        filename = fd.asksaveasfilename(
            title='Save a point cloud file',
            initialdir=os.getcwd(),
            filetypes=filetypes)

        if not filename:
            showerror(title='Error',
                      message='Point cloud filename is missing.')
        else:
            # showinfo(
            #     title='Selected File',
            #     message=filename
            # )
            save = FileSave(filename, self.pcd)
            save.save_file()
            showinfo(
                title='Selected File successfully saved',
                message=filename
            )
            print("-------PCD: \n", self.pcd)
            # print("------Points: \n", self.points)

    def save_json_file(self, event):
        filetypes = (
            ('JSON files', '*.json'),
            ('All files', '*.*')
        )

        filename = fd.asksaveasfilename(
            title='Save a JSON file',
            initialdir=os.getcwd(),
            filetypes=filetypes)

        if not filename:
            showerror(title='Error',
                      message='Point cloud filename is missing.')
        else:
            # showinfo(
            #     title='Selected File',
            #     message=filename
            # )
            clipboard = pyperclip.paste()
            # print("-------JSON: \n", clipboard)
            json_clip = json.loads(clipboard)
            with open(filename, "w") as j:
                json.dump(json_clip, j)
            showinfo(
                title='Selected File successfully saved',
                message=filename
            )
            print("-------JSON2: \n", json_clip)

    # TODO Visualization as thread?
    def visualize_pcd(self):
        self.vis_pcd = VisualizePCD(self.pcd)
        self.vis_pcd.visualize()

    def close_vis(self):
        self.vis_pcd.vis_close()
        self.pcd = None
        self.vis_pcd = None
        self.visualize_button.config(state="disabled")
        self.close_vis_button.config(state="disabled")
        self.save_button.config(state="disabled")


    def create_menu_buttons(self):
        """ create a menu buttons """
        # open button
        self.open_button = ttk.Button(
            self,
            text='Open a Point Cloud File',
            command=self.select_load_file
        )

        self.open_button.pack(expand=True)

        # save image button
        self.save_button = ttk.Button(
            self,
            text='Save a Point Cloud File',
            state="disabled",
            command=self.select_save_file
        )

        self.save_button.pack(expand=True)

        # visualize Point Cloud button
        self.visualize_button = ttk.Button(
            self,
            text='Visualize a Point Cloud File',
            state="disabled",
            command=self.visualize_pcd
        )

        # self.visualize_button["state"] = "disabled"

        self.visualize_button.pack(expand=True)

        # close visualization Point Cloud button
        self.close_vis_button = ttk.Button(
            self,
            text='Close visualization window',
            state="disabled",
            command=self.close_vis
        )

        # self.visualize_button["state"] = "disabled"

        self.close_vis_button.pack(expand=True)



if __name__ == "__main__":
    app = AppWithGUI()
    app.mainloop()