import numpy as np
from polygon import Polygon
from arm import Arm
import turtle
from functions import trajectory
import time

class Arm4dof(Arm):
    def __init__(self, polygons, axis, stages, dh_table):
        super().__init__(polygons, axis, stages, dh_table)

    def inverse_kinematics(self,position): #Custom
        nx,ny,nz=position[0][0],position[1][0],position[2][0]
        ox,oy,oz=position[0][1],position[1][1],position[2][1]
        ax,ay,az=position[0][2],position[1][2],position[2][2]
        px,py,pz=position[0][-1],position[1][-1],position[2][-1]
        
        d = [row[2] for row in self.dh_table]
        a = [row[1] for row in self.dh_table]

        #General
        #theta1=np.arctan2(-az,ax)
        theta1=np.arctan2(-pz,px)
        c1,s1=np.cos(theta1),np.sin(theta1)
        theta4=np.arctan2(-oy,ny)
  
        s2=(-d[3]*ay+py-d[0]-40)/300
        c2=(-d[3]*ax*c1+d[3]*az*s1+c1*px-s1*pz)/300
        c2=(310*az-pz)/(300*s1+1e-8)
        c2=(-310*ax+px)/(300*c1+1e-8)
        theta2=np.arctan2(s2,c2)
        
        '''
        r = np.sqrt(px**2 + pz**2)  # Horizontal projection
        s = py - (d[0]+40)  # Vertical offset
        #c**2=r**2 + s**2
        D = (r**2 + s**2 - a[1]**2 - d[3]**2) / (2 * a[1] * d[3])
        theta3 = -np.arccos(D)
        #theta3 = np.arctan2(np.sqrt(1 - D**2), D)
        theta2 = np.arctan2(s, r) - np.arctan2(d[3]*np.sin(theta3), a[1]+d[3]*np.cos(theta3))
        '''

        theta23=np.arccos(-ay)
        print(-ay,theta23)
        theta3=theta23-theta2-np.pi/2 #off set of angle
        #print(theta1,theta2,theta3,theta4)
        return [theta1,theta2,theta3,theta4]

if __name__=='__main__': 
    #Creating an Arm
    pols=[]
    #p0
    p0=Polygon()
    p0.create_cylinder(h=40,r0=60,r1=60,step=10)
    #p0.faces2triangles()
    p0.rotation(-np.pi/2,0,0)
    p0.translation((0,20,0))
    #p1
    p1=Polygon()
    p1.create_cylinder(h=250,r0=40,r1=40,step=10)
    #p1.faces2triangles()
    p1.rotation(-np.pi/2,0,0)
    p1.translation((0,165,0))
    #p2
    p2=Polygon()
    p2.create_cylinder(h=380,r0=40,r1=40,step=10)
    #p2.faces2triangles()
    p2.rotation(0,np.pi/2,0)
    p2.translation((150,250,80))
    #p3
    p3=Polygon()
    p3.create_cylinder(h=300,r0=40,r1=40,step=10)
    #p3.faces2triangles()
    p3.rotation(0,np.pi/2,0)
    p3.translation((410,250,0))
    #p4
    p4=Polygon()
    p4.create_cylinder(h=50,r0=30,r1=30,step=10)
    #p4.faces2triangles()
    p4.rotation(0,np.pi/2,0)
    p4.translation((585,250,0))
    #p5
    p5=Polygon()
    p5.create_cylinder(h=40,r0=10,r1=10,step=10)
    #p5.faces2triangles()
    p5.rotation(0,np.pi/2,0)
    p5.translation((630,270,0))
    #p6
    p6=Polygon()
    p6.create_cylinder(h=40,r0=10,r1=10,step=10)
    #p6.faces2triangles()
    p6.rotation(0,np.pi/2,0)
    p6.translation((630,230,0))

    #Actual state (origin)
    axis=[np.array([[1, 0, 0, 0],
                    [0, 0, 1, 40], #40+210
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]),
        np.array([[1, 0, 0, 0],
                    [0, 1, 0, 250],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
        np.array([[1, 0, 0, 300],
                    [0, 1, 0, 250],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
        np.array([[0, 0, 1, 300],
                    [1, 0, 0, 250],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]),
        np.array([[0, 0, 1, 610],
                    [1, 0, 0, 250],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])]

    #Actual state (origin)
    dh_table=np.array([[0,           0,   210,    np.pi/2], #d1=0
                       [0,           300, 0,      0],
                       [0+np.pi/2, 0,   0,      np.pi/2],
                       [0,           0,   260+50, 0]])

    pol=Arm4dof(polygons=[p0,p1,p2,p3,p4,p5,p6],
            axis=axis,
            stages=[1,2,3,4],
            dh_table=dh_table)

    print('Origin pos:')
    print(pol.position)
    print('Origin DH table:')
    print(pol.dh_table)
    #End arm creation

    #Initialing the screend
    width_screen=1600
    height_screen=1200
    win=turtle.Screen()
    win.setup(width_screen,height_screen)
    win.bgcolor("white")
    win.title('Robotic')
    win.tracer(0)

    pos_cam=np.array([[0,0,0,500],
                    [0,0,0,500],
                    [0,0,0,2000],
                    [0,0,0,1]])

    vec_cam=np.eye(4)# - pos_cam

    my_pen=turtle.Turtle()
    my_pen.hideturtle()
    my_pen.up()

    #''' Testing trajectory
    sec=5
    step=100
    t1=-np.pi
    #dt1=[t1/step]*step
    dt1=trajectory(ti=0,tf=t1,time=sec,step=step)
    t2=np.pi/4
    #dt2=[t2/step]*step
    dt2=trajectory(ti=0,tf=t2,time=sec,step=step)
    t3=-np.pi/4
    #dt3=[t3/step]*step
    dt3=trajectory(ti=0,tf=t3,time=sec,step=step)
    t4=np.pi/4
    #dt4=[t4/step]*step
    dt4=trajectory(ti=0,tf=t4,time=sec,step=step)

    pol.draw_polygon(my_pen,vec_cam)
    win.update()
    #while 1:
    for dA,dB,dC,dD in zip(dt1,dt2,dt3,dt4): #dt
        #print(dA,dB,dC,dD)
        my_pen.clear()
        pol.move(0,dA,n_axis=2)
        pol.move(1,dB,n_axis=2)
        pol.move(2,dC,n_axis=2)
        pol.move(3,dD,n_axis=2)
        pol.draw_polygon(my_pen,vec_cam)
        win.update()
        #break
        time.sleep(0.05)
    turtle.done()
    exit()
    #'''

    #old code
    if 1: #Forward
        pol.move(0,-np.pi/4,n_axis=2)
        pol.move(1,np.pi/8,n_axis=2)
        pol.move(2,-np.pi/4,n_axis=2)
        pol.move(3,np.pi/4,n_axis=2)
        pol.draw_polygon(my_pen,vec_cam)
        print('New pos:')
        print(pol.position)
        print(pol.axis[-1])
        print('New DH table:')
        print(pol.dh_table)
        win.update()
        turtle.done()
        exit()
    else: #Inverse
        #New pos
        #2 - 45
        #new_pos=np.array([[-7.07106781e-01, -4.32978028e-17,  7.07106781e-01,  5.19203102e+02],
        #                  [ 7.07106781e-01,  1.79345371e-17,  7.07106781e-01,  4.69203102e+02],
        #                  [-4.32978028e-17,  1.00000000e+00,  1.79345371e-17,  5.55970652e-15],
        #                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        #1- 45, 2 - 90
        #new_pos=np.array([[-7.07106781e-01,  4.32978028e-17, -7.07106781e-01, -7.07106781e+00],
        #                  [-7.07106781e-01,  1.79345371e-17,  7.07106781e-01,  6.81335137e+02],
        #                  [ 4.32978028e-17,  1.00000000e+00,  1.79345371e-17, -7.42963433e-15],
        #                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        #0 0 45 45
        #new_pos=np.array([[-5.00000000e-01,  5.00000000e-01,  7.07106781e-01,  5.19203102e+02],
        #                  [ 5.00000000e-01, -5.00000000e-01,  7.07106781e-01,  4.69203102e+02],
        #                  [ 7.07106781e-01,  7.07106781e-01,  0.00000000e+00,  0.00000000e+00],
        #                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        #180 45 -45 90
        #new_pos=np.array([[ 1.83697020e-16,  4.26642159e-17, -1.00000000e+00, -5.22132034e+02],
        #                [ 1.22464680e-16, -1.00000000e+00,  4.26642159e-17,  4.62132034e+02],
        #                [-1.00000000e+00, -1.22464680e-16, -1.83697020e-16, -6.99354170e-14],
        #                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        #180 -45 45 90
        #new_pos=np.array([[ 3.71362481e-17, -4.26642159e-17, -1.00000000e+00, -5.22132034e+02],
        #                [ 6.12323400e-17, -1.00000000e+00,  4.26642159e-17,  3.78679656e+01],
        #                [-1.00000000e+00, -6.12323400e-17, -1.22464680e-16, -4.97379915e-14],
        #                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        #-45 22.5 -45 45
        new_pos=np.array([[-3.087000e-01, -6.913000e-01,  6.533000e-01,  3.985017e+02],
                        [ 6.533000e-01, -6.533000e-01, -3.827000e-01,  2.461732e+02],
                        [ 6.913000e-01,  3.087000e-01,  6.533000e-01,  3.985017e+02],
                        [ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]])
        thetas=pol.inverse_kinematics(new_pos)
        print('Thetas:')
        print(thetas)
        pol.move(0,thetas[0],n_axis=2)
        pol.move(1,thetas[1],n_axis=2)
        pol.move(2,thetas[2],n_axis=2)
        pol.move(3,thetas[3],n_axis=2)
        pol.draw_polygon(my_pen,vec_cam)
        print('New pos:')
        print(pol.position)
        print('New DH table:')
        print(pol.dh_table)
        win.update()
        turtle.done()
        exit()