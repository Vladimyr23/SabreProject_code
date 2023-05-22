# import Libraries
import numpy as np
import pandas as pd
import open3d as o3d
import tkinter
# import pptk


'''Read a .pts points file into Pandas dataframe'''
def read_pts_df(filepath):
    col_names = ['X', 'Y', 'Z', 'intensity']
    df = pd.read_csv(filepath, names=col_names, header=None, delimiter=' ')
    return df


'''Read a .pts points file into numpy array'''


def read_pts_np(file_path):
    with open(file_path, "r") as f:
        points_np = []
        for f_line in f:
            x, y, z, i = [num for num in f_line.split()]
            points_np.append([float(x), float(y), float(z), int(i)])

    points_arr = np.array(points_np).transpose()
    print(len(points_arr))
    point_xyz = points_arr[:3].transpose()
    points_intensity = points_arr[3].transpose()
    return point_xyz, points_intensity


def visualize(pcd):
    # Create a visualizer object
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    # opt = vis.get_render_option() # This one and the next lines are to place black background
    # opt.background_color = np.asarray([0, 0, 0])    # Add point cloud to visualizer
    vis.add_geometry(pcd)
    # Register callback function for mouse click event
    # vis.register_point_pick_callback(on_point_pick)
    # Start visualizer
    fig = vis.run()
    test = tkinter.PhotoImage(fig)

    label1 = tkinter.Label(image=test)
    label1.image = test
    label1.place()


def on_point_pick(vis, point_id):
    # Get selected point coordinates
    p = vis.get_picked_points()[0]
    # Add point to line list
    if len(line.points) == 0:
        line.points[0] = p
    elif len(line.points) == 1:
        line.points[1] = p
        vis.add_geometry(line)
    else:
        line.points = []
        line.points[0] = p

def donothing():
   x = 0


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    filepath_static = "D:/ResearchAssistRGU/Sabre3DPointCloudVis/SABRE - Selected Static Scan Data/SABRE - Selected Static Scan Data/"
    filename = "SABRE Static Scan_T17_004.pts"
    filepath = filepath_static + filename

    # points_df = read_pts_df(file_path)
    # #points_df.head(7)
    #
    point_xyz, points_intensity = read_pts_np(filepath)
    # points_rgb_intensity = np.vstack((points_intensity, points_intensity, points_intensity)).T
    # print(len(points_rgb_intensity))
    # visualization with open3d
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(point_xyz)
    # geom.colors = o3d.utility.Vector3dVector(points_rgb_intensity)
    # geom.intensity = o3d.utility.Vector3dVector(points_intensity)
    # geom.background_color = np.asarray([0, 0, 0]) # not working this way
    # o3d.visualization.draw_geometries_with_editing([geom]) #working vsualization
    line = o3d.geometry.LineSet()  # Create line pcd
    # Draw line between two mouse clicks
    window = tkinter.Tk()
    window.geometry("1024x768")
    window.title("Point Cloud Editor")
    #box = tkinter.Entry(window)

    menubar = tkinter.Menu(window)
    window.config(background="white", menu=menubar)

    filemenu = tkinter.Menu(menubar, tearoff=0)
    filemenu.add_command(label="New", command=visualize(geom))
    filemenu.add_command(label="Open", command=donothing)
    filemenu.add_command(label="Save", command=donothing)
    filemenu.add_separator()
    filemenu.add_command(label="Exit", command=window.destroy)
    menubar.add_cascade(label="File", menu=filemenu)

    helpmenu = tkinter.Menu(menubar, tearoff=0)
    helpmenu.add_command(label="Help Index", command=donothing)
    helpmenu.add_command(label="About...", command=donothing)
    menubar.add_cascade(label="Help", menu=helpmenu)

    #window.config(menu=menubar)
    #box.pack()
    window.mainloop()

    # # IO with open3d is not working
    # print("Testing IO for point cloud ...")
    # # sample_pcd_data = o3d.data.PCDPointCloud()
    # pcd = o3d.io.read_point_cloud(file_path)
    # print(pcd)
    # #o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)
    #
    # # visualization with open3d
    # geom = o3d.pcd.PointCloud()
    # geom.points = o3d.utility.Vector3dVector(pcd[:3])
    # o3d.visualization.draw_geometries_with_editing([geom])
