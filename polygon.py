import numpy as np
from functions import *

def rotX(vec,theta):
    ct=np.cos(theta)
    st=np.sin(theta)
    mat=np.array([[1, 0,  0,   0],
                  [0, ct, -st, 0],
                  [0, st, ct,  0],
                  [0, 0,  0,   1]])
    return np.matmul(mat,vec)

def rotY(vec,theta):
    ct=np.cos(theta)
    st=np.sin(theta)
    mat=np.array([[ct,  0, st, 0],
                  [0,   1, 0,  0],
                  [-st, 0, ct, 0],
                  [0,   0, 0,  1]])
    return np.matmul(mat,vec)

def rotZ(vec,theta):
    ct=np.cos(theta)
    st=np.sin(theta)
    mat=np.array([[ct, -st, 0, 0],
                  [st, ct,  0, 0],
                  [0,  0,   1, 0],
                  [0,  0,   0, 1]])
    return np.matmul(mat,vec)

def trans(vec,coord=(0,0,0)):
    mat=np.array([[1,0,0,coord[0]],
                    [0,1,0,coord[1]],
                    [0,0,1,coord[2]],
                    [0,0,0,1]])
    return np.matmul(mat,vec)


class Polygon:
    def __init__(self):
        self.faces=[]
        self.colors=[]
        self.center=np.array([[1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,0],
                            [0,0,0,1]])

    def create_cube(self,len_edge=100):
        self.colors=['red','green','blue','gray', 'yellow','orange']
        n_face=6
        self.faces=[]
        theta=np.linspace(0,2*np.pi,4,endpoint=False)
        for k in range(n_face):
            vertices=[]
            if k==0: #Front face
                vertices=[[(len_edge/2)*(np.cos(i)-np.sin(i)),
                           (len_edge/2)*(np.cos(i)+np.sin(i)),
                           len_edge/2,
                           1] for i in theta]
            elif k==1: #Right face
                for i in self.faces[0]:
                    vertices.append(rotY(i,np.pi/2))
            elif k==2: #Behind face
                for i in self.faces[0]:
                    vertices.append(rotY(i,np.pi))
            elif k==3: #Left Face
                for i in self.faces[0]:
                    vertices.append(rotY(i,-np.pi/2))
            elif k==4: #Top face
                for i in self.faces[0]:
                    vertices.append(rotX(i,-np.pi/2))
            elif k==5: #Down face
                for i in self.faces[0]:
                    vertices.append(rotX(i,np.pi/2))
            self.faces.append(np.array(vertices))
    
    def create_cylinder(self,h=100,r0=50,r1=50,step=19):
        self.colors=['red','green']+['blue']*(step)
        self.faces=[]
        #Front face
        theta=np.linspace(0,2*np.pi,step)
        vertices=[[r0*np.cos(i),
                   r0*np.sin(i),
                   h/2,
                   1] for i in theta]
        self.faces.append(np.array(vertices))
        #Behind face
        vertices=[[r1*np.cos(i),
                   r1*np.sin(i),
                   -h/2,
                   1] for i in theta]
        self.faces.append(np.array(vertices))
        #Sides faces
        n=len(self.faces[0])
        for i in range(n):
            vertices=[self.faces[0][i],
                      self.faces[0][(i+1)%n],
                      self.faces[1][(i+1)%n],
                      self.faces[1][i]]
            self.faces.append(np.array(vertices))

    def create_semicylinder(self,h=20,r=5,l=15,step=10):
        self.colors=['red','green']+['blue']*(step+2)
        self.faces=[]
        #Front face
        theta=np.linspace(0,np.pi,step,endpoint=True)
        vertices=[[r*np.cos(i),
                   r*np.sin(i),
                   h/2,
                   1] for i in theta]
        vertices+=[[-r,-l,h/2,1],
                   [r,-l,h/2,1]]
        self.faces.append(np.array(vertices))
        #Behind side
        vertices=[[r*np.cos(i),
                   r*np.sin(i),
                   -h/2,
                   1] for i in theta]
        vertices+=[[-r,-l,-h/2,1],
                   [r,-l,-h/2,1]]
        self.faces.append(np.array(vertices))
        #Sides faces
        n=len(self.faces[0])
        for i in range(n):
            vertices=[self.faces[0][i],
                      self.faces[0][(i+1)%n],
                      self.faces[1][(i+1)%n],
                      self.faces[1][i]]
            self.faces.append(np.array(vertices))
     
    def create_cuboid(self,h=100,l=100):
        self.colors=['red','green']+['blue']*4
        self.faces=[]
        #Front face
        theta=np.linspace(0,2*np.pi,4,endpoint=False)
        vertices=[[(l/2)*(np.cos(i)-np.sin(i)),
                   (l/2)*(np.cos(i)+np.sin(i)),
                   h/2,
                   1] for i in theta]
        self.faces.append(np.array(vertices))
        #Behind face
        vertices=[[(l/2)*(np.cos(i)-np.sin(i)),
                   (l/2)*(np.cos(i)+np.sin(i)),
                   -h/2,
                   1] for i in theta]
        self.faces.append(np.array(vertices))
        #Sides faces
        n=len(self.faces[0])
        for i in range(n):
            vertices=[self.faces[0][i],
                      self.faces[0][(i+1)%n],
                      self.faces[1][(i+1)%n],
                      self.faces[1][i]]
            self.faces.append(np.array(vertices))


    @staticmethod
    def draw_face(face,color,pen):
        depth=1200
        z=face[0][2]
        xp=convert_depth(face[0][0],z,depth)
        yp=convert_depth(face[0][1],z,depth)
        pen.setpos(xp,yp)
        pen.down()
        #pen.color(color,color)
        pen.fillcolor(color)
        pen.begin_fill()
        for point in face[1:]:
            z=point[2]
            xp=convert_depth(point[0],z,depth)
            yp=convert_depth(point[1],z,depth)
            pen.goto(xp,yp)
        z=face[0][2]
        xp=convert_depth(face[0][0],z,depth)
        yp=convert_depth(face[0][1],z,depth)
        pen.goto(xp,yp)
        pen.end_fill()
        pen.up()
    
    
    def draw_polygon(self,pen,vec_cam):
        mean_z=[]
        for i in self.faces:
            mean_z.append(np.mean(np.matmul(i,vec_cam.T)[:,2]))
        order=np.argsort(mean_z)#[::-1]
        for i in order:
            self.draw_face(np.matmul(self.faces[i],vec_cam.T),self.colors[i],pen)
    
    
    def translation(self,coord=(0,0,0)):
        self.center=trans(self.center,coord)
        for i in range(len(self.faces)):
            for j in range(len(self.faces[i])):
                self.faces[i][j]=trans(self.faces[i][j],coord)

    def rotation(self,A,B,C):
        self.center=rotX(self.center,A)
        self.center=rotY(self.center,B)
        self.center=rotZ(self.center,C)
        for k,face in enumerate(self.faces):
            for i in range(face.shape[0]):
                self.faces[k][i]=rotX(self.faces[k][i],A)
                self.faces[k][i]=rotY(self.faces[k][i],B)
                self.faces[k][i]=rotZ(self.faces[k][i],C)
    
    def faces2triangles(self,max=100):
        new_faces=[]
        new_colors=[]
        for i,face in enumerate(self.faces):
            for tri in polygon2triangles(face):
                stack=triangle2triangles(tri,max=max,stack=[])
                for new_face_i in stack:
                    new_face=[]
                    for each_point in new_face_i:
                        new_face.append(each_point)
                    new_faces.append(np.array(new_face))
                    new_colors.append(self.colors[i])
        self.faces=new_faces
        self.colors=new_colors