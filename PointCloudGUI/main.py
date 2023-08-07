import os
import tkinter as tk
from tkinter import ttk
from tkinter import *
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

        self.sabre_logo_image = PhotoImage(file="sabre_logo.png")
        self.sabre_logo_lbl = None
        self.open_button = None
        self.open_strip_match_button = None
        self.save_button = None
        self.visualize_button = None
        self.visualize_mob_strips_button = None
        self.visualize_ransac_button = None
        self.ransac_vox_size_text = None
        self.ransac_vox_size_lbl = None
        self.visualize_mw_button = None
        self.mw_vox_size_text = None
        self.mw_vox_size_lbl = None
        self.close_vis_button = None

        self.title('Point Cloud Visualization')
        self.resizable(False, False)
        self.geometry('350x330')
        self.vis_pcd = None
        self.pcd = None
        self.pcd1 = None
        self.ransac_vox_size = 0.3
        self.mw_vox_size = 0.0006

        # Visualisation class
        self.vis_pcd = VisualizePCD()
        # Point cloud load class
        self.load = FileLoad()

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
            ('kitti binary files', '*.bin'),
            ('All files', '*.*')
        )

        filename = fd.askopenfilename(
            title='Open a point cloud file',
            initialdir=os.getcwd(),
            filetypes=filetypes)

        if not filename:
            showerror(title='Error',
                      message='Point cloud filename is missing.')
            return
        else:
            # showinfo(
            #     title='Selected File',
            #     message=filename
            # )
            if self.pcd is None:
                self.pcd = self.load.load_file(filename)
                showinfo(
                    title='Selected File successfully loaded',
                    message=filename
                )
                print("-------PCD: \n", self.pcd)
                # print("------Points: \n",self.points)
                self.visualize_button.config(state="normal")
                #self.close_vis_button.config(state="normal")
                self.save_button.config(state="normal")
            else:
                self.pcd1 = self.load.load_file(filename)
                showinfo(
                    title='Selected File successfully loaded',
                    message=filename
                )
                print("-------PCD: \n", self.pcd1)
                # print("------Points: \n",self.points)
                #self.visualize_button.config(state="disabled")
                self.visualize_ransac_button.config(state="normal")
                self.visualize_mw_button.config(state="normal")
                self.visualize_mob_strips_button.config(state="normal")
                #self.close_vis_button.config(stcyber securiyate="normal")
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
            return
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
            return
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


    def visualize_pcd(self):
        self.close_vis_button.config(state="normal")
        self.vis_pcd.visualize(self.pcd)

    def visualize_mob_strips(self):
        self.close_vis_button.config(state="normal")
        self.vis_pcd.visualize_mob_strips(self.pcd, self.pcd1)

    def visualize_ransac_pcd(self):
        self.close_vis_button.config(state="normal")
        self.ransac_vox_size = float(self.ransac_vox_size_text.get())
        if self.ransac_vox_size != 0 or self.ransac_vox_size is not None:
            self.vis_pcd.visualize_ransac(self.pcd, self.pcd1, self.ransac_vox_size)
        else:
            print("Voxel size changed to default value 0.3")
            self.vis_pcd.visualize_ransac(self.pcd, self.pcd1, 0.3)

    def visualize_mw_pcd(self):
        self.close_vis_button.config(state="normal")
        self.mw_vox_size = float(self.mw_vox_size_text.get())
        if self.mw_vox_size != 0 or self.mw_vox_size is not None:
            self.vis_pcd.visualize_mw(self.pcd, self.pcd1, self.mw_vox_size)
        else:
            print("Voxel size changed to default value 0.0006")
            self.vis_pcd.visualize_mw(self.pcd, self.pcd1, self.vis_pcd.visualize_mw(self.pcd, self.pcd1, 0.0006))

    def validate(self, action, index, value_if_allowed,
                 prior_value, text, validation_type, trigger_type, widget_name):
        if value_if_allowed:
            try:
                float(value_if_allowed)
                return True
            except ValueError:
                return False

        else:
            return False

    def close_vis(self):
        self.vis_pcd.vis_close()
        self.pcd = None
        self.pcd1 = None
        self.vis_pcd = None
        self.visualize_button.config(state="disabled")
        self.visualize_mob_strips_button.config(state="disabled")
        self.visualize_ransac_button.config(state="disabled")
        self.visualize_mw_button.config(state="disabled")
        self.close_vis_button.config(state="disabled")
        self.save_button.config(state="disabled")


    def create_menu_buttons(self):
        """ create a menu buttons """
        # SABRE logo image
        self.sabre_logo_lbl = ttk.Label(self, image=self.sabre_logo_image)
        self.sabre_logo_lbl.grid(row=0, column=0, columnspan=3, rowspan=1, padx=128, pady=5, sticky='nsew')

        # open button
        self.open_button = ttk.Button(
            self,
            text='Open a Point Cloud File',
            command=self.select_load_file
        )
        self.open_button.grid(row=1, column=0, columnspan=3, sticky='nsew')

        # open target and source files button
        self.open_strip_match_button = ttk.Button(
            self,
            text='Open source and target Point Cloud Files',
            command=lambda: [self.select_load_file(), self.select_load_file()]
        )
        self.open_strip_match_button.grid(row=2, column=0, columnspan=3, sticky='nsew')

        # save image button
        self.save_button = ttk.Button(
            self,
            text='Save a Point Cloud File',
            state="disabled",
            command=self.select_save_file
        )
        self.save_button.grid(row=3, column=0, columnspan=3, sticky='nsew')

        # visualize Point Cloud button
        self.visualize_button = ttk.Button(
            self,
            text='Edit Point Cloud',
            state="disabled",
            command=self.visualize_pcd
        )
        self.visualize_button.grid(row=4, column=0, columnspan=3, sticky='nsew')

        # visualize Mobile Strips button
        self.visualize_mob_strips_button = ttk.Button(
            self,
            text='Visualize mobile strips',
            state="disabled",
            command=self.visualize_mob_strips
        )
        self.visualize_mob_strips_button.grid(row=5, column=0, columnspan=3, sticky='nsew')

        # visualize RANSAC Point Clouds Registration button
        self.visualize_ransac_button = ttk.Button(
            self,
            text='Visualize RANSAC registration',
            state="disabled",
            command=self.visualize_ransac_pcd
        )
        self.visualize_ransac_button.grid(row=6, column=0, sticky='nsew')

        # RANSAC Voxel size textbox label
        self.ransac_vox_size_lbl = Label(self, text="Voxel size")
        self.ransac_vox_size_lbl.grid(row=6, column=1)
        # RANSAC Voxel size textbox
        vcmd = (self.register(self.validate),
                '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.ransac_vox_size_text = tk.Entry(self, width= 7, validate='key', validatecommand=vcmd)
        self.ransac_vox_size_text.insert(0, str(self.ransac_vox_size))
        self.ransac_vox_size_text.grid(row=6, column=2, sticky='nsew')

        # visualize Multiway Point Clouds Registration button
        self.visualize_mw_button = ttk.Button(
            self,
            text='Visualize Multiway registration',
            state="disabled",
            command=self.visualize_mw_pcd
        )
        self.visualize_mw_button.grid(row=7, column=0, sticky='nsew')

        # Multiway Voxel size textbox label
        self.mw_vox_size_lbl = Label(self, text="Voxel size")
        self.mw_vox_size_lbl.grid(row=7, column=1)
        # Multiway Voxel size textbox
        self.mw_vox_size_text = tk.Entry(self, width=7, validate='key', validatecommand=vcmd)
        self.mw_vox_size_text.insert(0, str(self.mw_vox_size))
        self.mw_vox_size_text.grid(row=7, column=2, sticky='nsew')

        # close visualization Point Cloud button
        self.close_vis_button = ttk.Button(
            self,
            text='Close visualization window',
            state="disabled",
            command=self.close_vis
        )
        self.close_vis_button.grid(row=8, column=0, columnspan=3, sticky='nsew')


if __name__ == "__main__":
    app = AppWithGUI()
    app.mainloop()