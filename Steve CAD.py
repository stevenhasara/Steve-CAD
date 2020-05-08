#Libraries
import PySimpleGUI as sg
import numpy as np
from skimage import measure
import open3d as o3d
import random



# GEOMETRY #
#####################################################################################################################
# Define shapes with implicit geometry.  Where relevant, rotate for 90deg rotational symmetry about all 3 axes

def sphere_coord(r):
    x, y, z = np.mgrid[-1.01:1.01:101j, -1.01:1.01:101j, -1.01:1.01:101j]
    return (x**2)+(y**2)+(z**2)-(r**2)

def cube_coord(length):
    x, y, z = np.mgrid[-1.01:1.01:101j, -1.01:1.01:101j, -1.01:1.01:101j]
    return np.maximum(np.maximum(np.abs(x),np.abs(y)),np.abs(z))-length

def add_cylinder(r,d):
    x, y, z = np.mgrid[-1.01:1.01:101j, -1.01:1.01:101j, -1.01:1.01:101j]
    coords=np.maximum(x*x+y*y-r*r,np.abs(z-0)-d)
    for i in range(3):
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(1,2)))
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(0,2)))
    return coords

def subtract_cylinder(r,d):
    x, y, z = np.mgrid[-1.01:1.01:101j, -1.01:1.01:101j, -1.01:1.01:101j]
    coords=np.maximum(x*x+y*y-r*r,np.abs(z-1)-d)
    for i in range(3):
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(1,2)))
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(0,2)))
    return coords

def add_torus(R,a):
    x, y, z = np.mgrid[-1.01:1.01:101j, -1.01:1.01:101j, -1.01:1.01:101j]
    coords=((x*x+y*y+z*z+R*R-a*a)**2)-4*R*R*(x*x+y*y)
    for i in range(3):
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(1,2)))
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(0,2)))
    return coords

def subtract_torus(R,a):
    x, y, z = np.mgrid[-1.01:1.01:101j, -1.01:1.01:101j, -1.01:1.01:101j]
    coords=((x*x+y*y+z*z+R*R-a*a)**2)-4*R*R*(x*x+y*y)
    for i in range(3):
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(1,2)))
        coords=np.minimum(coords,np.rot90(coords,i+1,axes=(0,2)))
    return coords


#While loop which iterates through each shape defined in GUI
def geometry2(shape):
    print(shape)
    step=0
    starting=0
    while step<len(shape):
        if shape[step]=='cube':
            coords=cube_coord(shape[step+1])
            print("start with cube")
            starting=1
        if shape[step]=='sphere':
            coords=sphere_coord(shape[step+1])
            print("start with sphere")
            starting=1
        if shape[step]=='none':
            print("start with nothing")
            starting=1
        if shape[step]=='add_cylinder':
            if shape[0]=='none' and step==2:
                coords=add_cylinder(shape[step+1],shape[step+2])
                print("adding starting cylinder")
            else:
                coords=np.minimum(coords,add_cylinder(shape[step+1],shape[step+2]))
                print("adding cylinder")
        if shape[step]=='subtract_cylinder':
            coords=np.maximum(coords,-subtract_cylinder(shape[step+1],shape[step+2]))
            print("subtracting cylinder")
        if shape[step]=='add_torus':
            if shape[0]=='none' and step==2:
                coords=add_torus(shape[step+1],shape[step+2])
                print("adding starting torus")
            else:
                coords=np.minimum(coords,add_torus(shape[step+1],shape[step+2]))
                print("adding cylinder")
        if shape[step]=='subtract_torus':
            coords=np.maximum(coords,-subtract_torus(shape[step+1],shape[step+2]))
            print("subtracting torus")
        if starting==1: step=2; starting=0;
        else: step+=3;
    verts, faces, normals, values = measure.marching_cubes_lewiner(coords, 0, spacing=(0.02,0.02,0.02))
    create_ply(verts,faces,"shape.ply")


#given Verts & Faces as outputs from measure.marching_cubes_lewiner, create a text output saved as a .ply for Open3D to read as a mesh.
#The filename argument is just the name of the .ply file that Open3D should look for (shape.ply in this case)
def create_ply(verts,faces,filename):
    comment="comment your mom" #temp / optional
    num_verts = (verts.shape[0])
    num_faces = (faces.shape[0])
    print("Making .ply file.  Contains %d vertices and %d faces" % (num_verts,num_faces))
    #this header is dictated by the .ply formatting as described at "http://paulbourke.net/dataformats/ply/"
    header="ply\nformat ascii 1.0\n%s\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nelement face %d\nproperty list uchar int vertex_index\nend_header\n" % (comment, num_verts, num_faces)
    text_file = open(filename,"wt")
    text_file.write(header)
    text_file = open(filename, "a")
    #np.savetxt(text_file, verts,fmt='%f')
    with open(filename, "a") as f:
            np.savetxt(f, verts, fmt='%f')
    #append faces array with new left-most column containing number of verts comprising a given face
    ply_faces=np.empty((faces.shape[0],4))
    for i in range(faces.shape[0]):
        a=np.array([faces[i].size])
        b=faces[i]
        ply_faces[i]=np.append(a,b)
        #np.savetxt(text_file, ply_faces[i],fmt='%u')
    with open(filename, "a") as f:
            np.savetxt(f, ply_faces, fmt='%u')





# VISUALIZER #
#####################################################################################################################

def visualize_ply(filename):
    mesh=o3d.io.read_triangle_mesh(filename)
    mesh.compute_vertex_normals()
    #mesh.paint_uniform_color([0.5,0.1,0.1])
    o3d.visualization.draw_geometries([mesh],window_name="Printer's Cube", width=1000, height=1000, left=625, top=30)

def save_stl(filepath):
    mesh=o3d.io.read_triangle_mesh("shape.ply")
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(filepath,mesh,print_progress=True)



# RANDOM MESH #
#####################################################################################################################
def random_mesh(num_shapes):
    starting_shape=['cube','sphere']
    shape_choice=['add_cylinder','subtract_cylinder','add_torus','subtract_torus']
    shape=[random.choice(starting_shape),random.random()]
    for i in range(num_shapes-1):
        add_shape=random.choice(shape_choice)
        if add_shape=='add_cylinder' or add_shape=='subtract_cylinder':
            shape+=[add_shape,random.random(),random.random()]
        else:
            value=random.random()
            shape+=([add_shape,value,1-value])
    print(shape)
    return(shape)

# GUI #
#######################################################################################################################

#transform the values produced by GUI inputs into the format read by my geometry2 while loop
def Submit(values):
    shape=[]
    if values[1]==True: shape.append('cube')
    elif values[2]==True: shape.append('sphere')
    elif values[3]==True: shape.append('none')
    shape.append(values[4])
    if values[5]==True:
        shape.append('add_cylinder')
        shape.append(values[8])
        shape.append(values[9])
    elif values[6]==True:
        shape.append('subtract_cylinder')
        shape.append(values[8])
        shape.append(values[9])
    if values[10]==True:
        shape.append('add_cylinder')
        shape.append(values[13])
        shape.append(values[14])
    elif values[11]==True:
        shape.append('subtract_cylinder')
        shape.append(values[13])
        shape.append(values[14])
    if values[15]==True:
        shape.append('add_cylinder')
        shape.append(values[8])
        shape.append(values[9])
    elif values[16]==True:
        shape.append('subtract_cylinder')
        shape.append(values[18])
        shape.append(values[19])
    if values[20]==True:
        shape.append('add_cylinder')
        shape.append(values[23])
        shape.append(values[24])
    elif values[21]==True:
        shape.append('subtract_cylinder')
        shape.append(values[23])
        shape.append(values[24])
    if values[25]==True:
        shape.append('add_torus')
        shape.append(values[28])
        shape.append(values[29])
    elif values[26]==True:
        shape.append('subtract_torus')
        shape.append(values[28])
        shape.append(values[29])
    if values[30]==True:
        shape.append('add_torus')
        shape.append(values[33])
        shape.append(values[34])
    elif values[31]==True:
        shape.append('subtract_torus')
        shape.append(values[33])
        shape.append(values[34])
    return shape

#Set color scheme to one of the PySimpleGUI defaults
sg.ChangeLookAndFeel('GreenTan')

#Menu currently doesn't do anything...
menu_def = [['File', ['Save', 'Exit']],
            ['Help', 'About...'], ]

shape_list=[]

layout = [
    [sg.Menu(menu_def, tearoff=True)],
    [sg.Text("Steve CAD", size=(30, 1), justification='center', font=("Helvetica", 25), relief=sg.RELIEF_RIDGE)],
    [sg.Text("Define shapes & Submit to visualize with Open3D.  Errors appear in the Python console.")],

    [sg.Frame(layout=[
    [sg.Radio('Cube', "StartingShape", default=True), sg.Radio('Sphere', "StartingShape"), sg.Radio('None', "StartingShape")],
    [sg.Text('Size:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01,default_value=1)]], title='Starting Shape', relief=sg.RELIEF_SUNKEN, tooltip='Recommended to leave at 1'),sg.Button('Random')],

    [sg.Frame(layout=[
    [sg.Radio('Add', "Cylinder 1"), sg.Radio('Subtract', "Cylinder 1"), sg.Radio('Off', "Cylinder 1", default=True)],
    [sg.Text('Radius:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)],
    [sg.Text('Depth:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)]], title='Cylinder 1', relief=sg.RELIEF_SUNKEN, tooltip='Define your modification',key='Cylinder1'),
    sg.Frame(layout=[
    [sg.Radio('Add', "Cylinder 2"), sg.Radio('Subtract', "Cylinder 2"), sg.Radio('Off', "Cylinder 2", default=True)],
    [sg.Text('Radius:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)],
    [sg.Text('Depth:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)]], title='Cylinder 2', relief=sg.RELIEF_SUNKEN, tooltip='Define your modification',key='Cylinder2')],

    [sg.Frame(layout=[
    [sg.Radio('Add', "Cylinder 3"), sg.Radio('Subtract', "Cylinder 3"), sg.Radio('Off', "Cylinder 3", default=True)],
    [sg.Text('Radius:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)],
    [sg.Text('Depth:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)]], title='Cylinder 3', relief=sg.RELIEF_SUNKEN, tooltip='Define your modification',key='Cylinder3'),
    sg.Frame(layout=[
    [sg.Radio('Add', "Cylinder 4"), sg.Radio('Subtract', "Cylinder 4"), sg.Radio('Off', "Cylinder 4", default=True)],
    [sg.Text('Radius:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)],
    [sg.Text('Depth:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)]], title='Cylinder 4', relief=sg.RELIEF_SUNKEN, tooltip='Define your modification',key='Cylinder4')],

    [sg.Frame(layout=[
    [sg.Radio('Add', "Torus 1"), sg.Radio('Subtract', "Torus 1"), sg.Radio('Off', "Torus 1", default=True)],
    [sg.Text('Radius:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)],
    [sg.Text('Cross:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)]], title='Torus 1', relief=sg.RELIEF_SUNKEN, tooltip='Define your modification',key='Torus1'),
    sg.Frame(layout=[
    [sg.Radio('Add', "Torus 2"), sg.Radio('Subtract', "Torus 2"), sg.Radio('Off', "Torus 2", default=True)],
    [sg.Text('Radius:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)],
    [sg.Text('Cross:'), sg.Slider(range=(0.02,1),orientation='h',resolution=.01)]], title='Torus 2', relief=sg.RELIEF_SUNKEN, tooltip='Define your modification',key='Torus2')],

    [sg.Text('_'  * 80)],

    [sg.Frame(layout=[
    [sg.Text('This will save the last shape you SUBMITTED, not current settings!', size=(50, 1))],
    [sg.Text('Save Location:', size=(15, 1), auto_size_text=False, justification='right'),
        sg.InputText('Default Folder'), sg.FolderBrowse()],
    [sg.Text('File Name:', size=(15, 1), auto_size_text=False, justification='right'),
        sg.InputText("Steve CAD"), sg.Button("Save")],
    ],title='Save .STL File')],

    [sg.Text(' '*80)],
    [sg.Submit(tooltip='Click to submit this window'), sg.Cancel()]
]

#GUI open while loop
window = sg.Window("Steve CAD", layout, default_element_size=(40, 1), grab_anywhere=False,location=(0,0))
submitted=0
while True:
    event, values = window.read()
    if event in (None, 'Cancel'):   # if user closes window or clicks cancel
        break
    if event in ('Random'):
        submitted=1
        shape=random_mesh(random.randint(2,25))
        print(shape)
        geometry2(shape)
        visualize_ply("shape.ply")
    if event in ('Cylinder'):
        window.Element('Cylinder1').Update(visible = True)
        shape_list.append('cylinder')
        update_shapes(shape_list)
    if event in ('Off'):
        window.Element('Cylinder1').Update(visible = False)
        shape_list.append('cylinder')
        update_shapes(shape_list)
    if event in ('Save'):
        if values[35]=='Default Folder': print("ERROR:  Choose a folder.");continue;
        if submitted==0: print("ERROR: Submit geometry before saving.");continue;
        print(r"%s/%s.stl" % (values[35],values[36]))
        save_stl(r"%s/%s.stl" % (values[35],values[36]))
    if event in ('Submit'):
        print(values)
        submitted=1
        shape=Submit(values)
        if shape[0]=='none' and len(shape)==2:
            print("ERROR: Starting shape NONE and all other shapes turned off.  No mesh generated.")
            continue
        if shape[0]=='none' and shape[2]=='subtract_cylinder':
            print("ERROR: Can't SUBTRACT geometry from starting shape NONE.  First ADD something.  No mesh generated.")
            continue
        if values[25]==True and values[28]+values[29]>1:
            print("ERROR: When ADDING a torus, Radius + Cross Section values can't be greater than 1.  No mesh generated.")
            continue
        if values[30]==True and values[33]+values[34]>1:
            print("ERROR: When ADDING a torus, Radius + Cross Section values can't be greater than 1.  No mesh generated.")
            continue
        geometry2(shape)
        visualize_ply("shape.ply")
window.close()




