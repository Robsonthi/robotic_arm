import numpy as np
from polygon import Polygon

def rotAxis(vec,ref,theta,n_axis):
    a,b,c=ref[0][-1],ref[1][-1],ref[2][-1]
    u,v,w=ref[0][n_axis],ref[1][n_axis],ref[2][n_axis]
    ct=np.cos(theta)
    st=np.sin(theta)
    R=np.array([[u**2+(v**2+w**2)*ct, u*v*(1-ct)-w*st, u*w*(1-ct)+v*st, (a*(v**2+w**2)-u*(b*v+c*w))*(1-ct)+(b*w-c*v)*st],
               [u*v*(1-ct)+w*st, v**2+(u**2+w**2)*ct, v*w*(1-ct)-u*st, (b*(u**2+w**2)-v*(a*u+c*w))*(1-ct)+(c*u-a*w)*st],
               [u*w*(1-ct)-v*st, v*w*(1-ct)+u*st, w**2+(u**2+v**2)*ct, (c*(u**2+v**2)-w*(a*u+b*v))*(1-ct)+(a*v-b*u)*st],
               [0,0,0,1]])
    return np.matmul(R,vec)

class Arm(Polygon):
    def __init__(self,polygons,axis,stages,dh_table):
        self.axis=axis #Axis each articulation
        self.stages=stages #indice of beginner polygon each axis
        self.faces=[]
        self.colors=[]
        self.each_group=[] #Face of all forward polygons
        for i,pol in enumerate(polygons):
            self.faces+=pol.faces
            self.colors+=pol.colors
            if i==0:
                self.each_group.append(0)
            else:
                self.each_group.append(self.each_group[i-1]+len(polygons[i-1].faces))
            
        self.center=np.array([[1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,0],
                            [0,0,0,1]])
        self.dh_table=dh_table #Actual state (origin)
        self.position=self.forward_kinematics() #Actual position (origin)
 
    @staticmethod
    def dh_matrix(theta,l,d,alpha):
        ct = np.cos(theta) 
        st = np.sin(theta) 
        ca = np.cos(alpha) 
        sa = np.sin(alpha) 
        return np.array([[ct, -st*ca, st*sa, l*ct], 
                        [st, ct*ca,  -ct*sa, l*st], 
                        [0,  sa,     ca,     d], 
                        [0,  0 ,     0 ,     1]])
    
    #Position of state actual
    def forward_kinematics(self):
        #P0=trans((0,40,0))*rot(y,-pi/2)*Po
        T_U_H = self.axis[0]
        for line_dh in self.dh_table:
            T_U_H = np.matmul(T_U_H, self.dh_matrix(line_dh[0], line_dh[1], line_dh[2], line_dh[3]))
        self.position=T_U_H
        return self.position

    def move(self,stage,dtheta,n_axis=2):
        #stage = 0 to number axis (less H)
        ref=self.axis[stage]
        pol_ref=self.stages[stage]
        face_ref=self.each_group[pol_ref]
        #Moving the faces of polygon
        for i in range(face_ref,len(self.faces)):
            for j in range(len(self.faces[i])):
                self.faces[i][j]=rotAxis(self.faces[i][j],ref,dtheta,n_axis)
        for i in range(stage+1,len(self.axis)):
            self.axis[i]=rotAxis(self.axis[i],ref,dtheta,n_axis)
        #Update of position
        self.dh_table[stage][0]+=dtheta
        self.forward_kinematics()