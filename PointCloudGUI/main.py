import os
import tkinter as tk
from tkinter import ttk
from tkinter import *
from tkinter import filedialog as fd
import tkinter.font as tkFont
from tkinter.colorchooser import askcolor
from tkinter.messagebox import showinfo
from tkinter.messagebox import showerror

import numpy as np
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
        self.sabre_hw_image = PhotoImage(file="sabre_hardware_img599-470.png")
        self.sabre_logo_lbl = None
        self.sabre_hw_image_lbl = None
        self.open_button = None
        self.open_2point_clds_for_registration_button = None
        self.open_pt_cld_and_transfrm_mtrx_button = None
        self.save_button = None
        self.edit_pt_cld_button = None
        self.visualize_mob_strips_button = None
        self.manual_registration_button = None
        self.man_reg_threshold_lbl = None
        self.man_reg_threshold_text = None
        self.visualize_ransac_button = None
        self.ransac_vox_size_lbl = None
        self.ransac_vox_size_text = None
        self.visualize_mw_button = None
        self.mw_vox_size_lbl = None
        self.mw_vox_size_text = None
        self.visualize_real_time_button = None
        # self.split_along_kml_button = None
        self.visualize_segm_DBSCAN_button = None
        self.source_color_btn = None
        self.source_color_lbl = None
        self.source_color = [1, 0.706, 0]
        self.target_color_btn = None
        self.target_color_lbl = None
        self.target_color = [0, 0.651, 0.929]
        self.close_vis_button = None
        self.quit_button = None

        self.title('SABRE Point Cloud Visualization')
        self.resizable(False, False)
        self.geometry('1010x532')
        # Add fonts for all the widgets
        default_font = tkFont.nametofont("TkDefaultFont")
        default_font.configure(size=10)
        self.option_add("*Font", default_font)
        self.vis_pcd = None
        self.pcd = None
        self.pcd1 = None
        self.transf_mtrx = None
        self.kml_coordinates = None
        self.man_reg_threshold = 0.2
        self.ransac_vox_size = 0.3
        self.mw_vox_size = 0.0006

        # Visualisation class
        self.vis_pcd = VisualizePCD()
        # Point cloud load class
        self.load = FileLoad()

        # Point cloud save class
        self.save = FileSave()

        # create the menu button
        self.create_menu_buttons()

        # Bind the Enter Key to the window
        self.bind('<Return>', self.save_json_file)

        # Call function to make the window stay on top
        # self.stay_on_top()

    # def stay_on_top(self):
    #     self.lift()
    #     self.after(2000, self.stay_on_top)

    def change_sour_color(self):
        colors = askcolor(title="Color Chooser")
        self.source_color_lbl.configure(bg=colors[1])
        self.source_color = np.asarray(colors[0]) / 255.0

    def change_targ_color(self):
        colors = askcolor(title="Color Chooser")
        self.target_color_lbl.configure(bg=colors[1])
        self.target_color = np.asarray(colors[0]) / 255.0

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
            ('KML trajectory files', '*.kml'),
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
                self.edit_pt_cld_button.config(state="normal")
                self.visualize_real_time_button.config(state="normal")
                self.visualize_segm_DBSCAN_button.config(state="normal")
                self.save_button.config(state="normal")
            else:
                self.pcd1 = self.load.load_file(filename)
                showinfo(
                    title='Selected File successfully loaded',
                    message=filename
                )
                print("-------PCD: \n", self.pcd1)
                # print("------Points: \n",self.points)
                # self.visualize_button.config(state="disabled")
                self.manual_registration_button.config(state="normal")
                self.visualize_ransac_button.config(state="normal")
                self.visualize_mw_button.config(state="normal")
                self.visualize_mob_strips_button.config(state="normal")
                self.source_color_btn.config(state="normal")
                self.target_color_btn.config(state="normal")
                # self.close_vis_button.config(stcyber securiyate="normal")
                self.save_button.config(state="normal")

    def select_transf_file(self):
        # self.points = None
        filetypes = (
            ('text files', '*.txt'),
            ('All files', '*.*')
        )
        filename = fd.askopenfilename(
            title='Open a transformation matrix file',
            initialdir=os.getcwd(),
            filetypes=filetypes)

        if not filename:
            showerror(title='Error',
                      message='Transformation matrix filename is missing.')
            return
        else:
            if self.transf_mtrx is None:
                # file = open(filename, "r")
                self.transf_mtrx = np.loadtxt(filename)
                showinfo(
                    title='Selected File successfully loaded',
                    message=filename
                )
                print("Transformation matrix: \n", self.transf_mtrx)
            else:
                print("Transformation matrix error. Please restart the application")
                return

    def select_kml_file(self):
        # self.points = None
        filetypes = (
            ('KML trajectory file', '*.kml'),
            ('All files', '*.*')
        )
        filename = fd.askopenfilename(
            title='Open a KML trajectory file',
            initialdir=os.getcwd(),
            filetypes=filetypes)

        if not filename or not filename.endswith(".kml"):
            showerror(title='Error',
                      message='KML trajectory filename is missing.')
            return
        else:
            if self.kml_coordinates is None:
                # file = open(filename, "r")
                self.kml_coordinates = self.load.load_kml(filename)
                showinfo(
                    title='The KML trajectory File successfully loaded',
                    message=filename
                )
                print(f"KML coordinates matrix with {len(self.kml_coordinates)} points: \n", self.kml_coordinates)
            else:
                print("KML coordinates matrix: \n", self.kml_coordinates)
                print("KML file load error. Please restart the application")
                return

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
        if not filename or self.pcd is None:
            showerror(title='Error',
                      message='Point cloud filename or data is missing.')
            return
        else:
            # showinfo(
            #     title='Selected File',
            #     message=filename
            # )
            #save = FileSave(filename, self.pcd)
            self.save.save_pcd_file(filename, self.pcd)
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

    def visualize_pcd_with_transformation(self):
        if self.transf_mtrx is None or self.pcd is None:
            showerror(title='Error',
                      message='Point cloud or transformation matrix is missing.')
            return
        else:
            self.close_vis_button.config(state="normal")
            self.vis_pcd.visualize_pcd_transf(self.pcd, self.transf_mtrx)

    def visualize_mob_strips(self):
        self.close_vis_button.config(state="normal")
        self.vis_pcd.visualize_mob_strips(self.pcd, self.pcd1,
                                          self.source_color, self.target_color)

    def manual_registration_visualization(self):
        self.close_vis_button.config(state="normal")
        self.vis_pcd.manual_registration(self.pcd, self.pcd1, self.man_reg_threshold,
                                         self.source_color, self.target_color)

    def visualize_ransac_pcd(self):
        self.close_vis_button.config(state="normal")
        self.ransac_vox_size = float(self.ransac_vox_size_text.get())
        if self.ransac_vox_size != 0 or self.ransac_vox_size is not None:
            self.vis_pcd.my_registration(self.pcd, self.pcd1, self.ransac_vox_size,
                                          self.source_color, self.target_color)
        else:
            print("Voxel size changed to default value 0.3")
            self.vis_pcd.visualize_ransac(self.pcd, self.pcd1, 0.3,
                                          self.source_color, self.target_color)

    def visualize_mw_pcd(self):
        self.close_vis_button.config(state="normal")
        self.mw_vox_size = float(self.mw_vox_size_text.get())
        if self.mw_vox_size != 0 or self.mw_vox_size is not None:
            self.vis_pcd.visualize_mw(self.pcd, self.pcd1, self.mw_vox_size,
                                      self.source_color, self.target_color)
        else:
            print("Voxel size changed to default value 0.0006")
            self.vis_pcd.visualize_mw(self.pcd, self.pcd1, 0.0006,
                                      self.source_color, self.target_color)

    def visualize_real_time(self):
        self.close_vis_button.config(state="normal")
        self.vis_pcd.visualize_real_time(self.pcd)

    def visualize_segm_DBSCAN(self):
        self.close_vis_button.config(state="normal")
        self.vis_pcd.vis_segm_DBSCAN(self.pcd)


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
        self.edit_pt_cld_button.config(state="disabled")
        self.visualize_mob_strips_button.config(state="disabled")
        self.visualize_ransac_button.config(state="disabled")
        self.visualize_mw_button.config(state="disabled")
        self.visualize_real_time_button.config(state="disabled")
        self.close_vis_button.config(state="disabled")
        self.save_button.config(state="disabled")

    def create_menu_buttons(self):
        """ create a menu buttons """
        # SABRE logo image
        self.sabre_logo_lbl = ttk.Label(self, image=self.sabre_logo_image)
        self.sabre_logo_lbl.grid(row=0, column=0, columnspan=3, rowspan=1, padx=156, pady=5, sticky='nsew')

        # SABRE hardware image
        self.sabre_hw_image_lbl = ttk.Label(self, image=self.sabre_hw_image)
        self.sabre_hw_image_lbl.grid(row=0, column=4, columnspan=1, rowspan=15, padx=3, pady=15, sticky='nsew')

        # open button
        self.open_button = ttk.Button(
            self,
            text='Open Point Cloud File',
            command=self.select_load_file
        )
        self.open_button.grid(row=1, column=0, columnspan=3, sticky='nsew')

        # open target and source files button
        self.open_2point_clds_for_registration_button = ttk.Button(
            self,
            text='Open source and target Point Cloud Files',
            command=lambda: [self.select_load_file(), self.select_load_file()]
        )
        self.open_2point_clds_for_registration_button.grid(row=2, column=0, columnspan=3, sticky='nsew')

        # open and visualize point cloud with transformation matrix button
        self.open_pt_cld_and_transfrm_mtrx_button = ttk.Button(
            self,
            text='Open and Visualize Point Cloud with Transformation matrix',
            command=lambda: [self.select_load_file(), self.select_transf_file(),
                             self.visualize_pcd_with_transformation()]
        )
        self.open_pt_cld_and_transfrm_mtrx_button.grid(row=3, column=0, columnspan=3, sticky='nsew')

        # save image button
        self.save_button = ttk.Button(
            self,
            text='Save Point Cloud File',
            state="disabled",
            command=self.select_save_file
        )
        self.save_button.grid(row=4, column=0, columnspan=3, sticky='nsew')

        # visualize with editing Point Cloud button
        self.edit_pt_cld_button = ttk.Button(
            self,
            text='Edit Point Cloud',
            state="disabled",
            command=self.visualize_pcd
        )
        self.edit_pt_cld_button.grid(row=5, column=0, columnspan=3, sticky='nsew')

        # visualize Mobile Strips button
        self.visualize_mob_strips_button = ttk.Button(
            self,
            text='Visualize mobile strips',
            state="disabled",
            command=self.visualize_mob_strips
        )
        self.visualize_mob_strips_button.grid(row=6, column=0, columnspan=3, sticky='nsew')

        # Manual points based registration button
        self.manual_registration_button = ttk.Button(
            self,
            text='Manual points based registration',
            state="disabled",
            command=self.manual_registration_visualization
        )
        self.manual_registration_button.grid(row=7, column=0, columnspan=1, sticky='nsew')
        # Manual registration threshold textbox label
        self.man_reg_threshold_lbl = Label(self, text="Threshold")
        self.man_reg_threshold_lbl.grid(row=7, column=1)
        # Manual registration threshold textbox
        vcmd = (self.register(self.validate),
                '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.man_reg_threshold_text = tk.Entry(self, width=8, validate='key', validatecommand=vcmd)
        self.man_reg_threshold_text.insert(0, str(self.man_reg_threshold))
        self.man_reg_threshold_text.grid(row=7, column=2, sticky='nsew')

        # visualize RANSAC Point Clouds Registration button
        self.visualize_ransac_button = ttk.Button(
            self,
            text='Visualize RANSAC registration',
            state="disabled",
            command=self.visualize_ransac_pcd
        )
        self.visualize_ransac_button.grid(row=8, column=0, sticky='nsew')
        # RANSAC Voxel size textbox label
        self.ransac_vox_size_lbl = Label(self, text="Voxel size")
        self.ransac_vox_size_lbl.grid(row=8, column=1)
        # RANSAC Voxel size textbox
        vcmd = (self.register(self.validate),
                '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.ransac_vox_size_text = tk.Entry(self, width=8, validate='key', validatecommand=vcmd)
        self.ransac_vox_size_text.insert(0, str(self.ransac_vox_size))
        self.ransac_vox_size_text.grid(row=8, column=2, sticky='nsew')

        # visualize Multiway Point Clouds Registration button
        self.visualize_mw_button = ttk.Button(
            self,
            text='Visualize Multiway registration',
            state="disabled",
            command=self.visualize_mw_pcd
        )
        self.visualize_mw_button.grid(row=9, column=0, sticky='nsew')
        # Multiway Voxel size textbox label
        self.mw_vox_size_lbl = Label(self, text="Voxel size")
        self.mw_vox_size_lbl.grid(row=9, column=1)
        # Multiway Voxel size textbox
        self.mw_vox_size_text = tk.Entry(self, width=8, validate='key', validatecommand=vcmd)
        self.mw_vox_size_text.insert(0, str(self.mw_vox_size))
        self.mw_vox_size_text.grid(row=9, column=2, sticky='nsew')

        # Choose source Point Cloud color button
        self.source_color_btn = ttk.Button(
            self,
            text='Select Source PCD Color',
            state="disabled",
            command=self.change_sour_color
        )
        self.source_color_btn.grid(row=10, column=0, sticky='nsew')
        # Chosen source color label
        self.source_color_lbl = Label(self, text="Source", bg=("#FFB400"))
        self.source_color_lbl.grid(row=10, column=1)

        # Choose target Point Cloud color button
        self.target_color_btn = ttk.Button(
            self,
            text='Select Target PCD Color',
            state="disabled",
            command=self.change_targ_color
        )
        self.target_color_btn.grid(row=11, column=0, sticky='nsew')
        # Chosen target color label
        self.target_color_lbl = Label(self, text="Target", bg=("#00A6ED"))
        self.target_color_lbl.grid(row=11, column=1)

        # visualize Real-Time Model button
        self.visualize_real_time_button = ttk.Button(
            self,
            text='Real-time visualization',
            state="disabled",
            command=self.visualize_real_time
        )
        self.visualize_real_time_button.grid(row=12, column=0, columnspan=3, sticky='nsew')

        # Visualise Segmentation with DBSCAN
        self.visualize_segm_DBSCAN_button = ttk.Button(
            self,
            text='Visualise Segmentation with DBSCAN',
            state="disabled",
            command=self.visualize_segm_DBSCAN
        )
        self.visualize_segm_DBSCAN_button.grid(row=13, column=0, columnspan=3, sticky='nsew')

        # # open and save cropped point cloud along the KML trajectory button
        # self.split_along_kml_button = ttk.Button(
        #     self,
        #     text='Open PCD and KML then save PCD split along KML',
        #     command=lambda: [self.select_load_file(), self.select_kml_file(),
        #                      self.save.save_kml_split(self.pcd, self.kml_coordinates)]
        # )
        # self.split_along_kml_button.grid(row=13, column=0, columnspan=3, sticky='nsew')

        # close visualization Point Cloud button
        self.close_vis_button = ttk.Button(
            self,
            text='Close visualization window',
            state="disabled",
            command=self.close_vis
        )
        self.close_vis_button.grid(row=14, column=0, columnspan=3, sticky='nsew')

        # close visualization Point Cloud button
        self.quit_button = ttk.Button(
            self,
            text='Quit',
            state="normal",
            command=self.destroy_it
        )
        self.quit_button.grid(row=15, column=0, columnspan=3, sticky='nsew')

    def destroy_it(self):
        app.destroy()

if __name__ == "__main__":
    app = AppWithGUI()
    app.mainloop()
