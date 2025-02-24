import numpy as np
from polygon import Polygon, rotX, rotY, rotZ
from arm import Arm
import turtle
from functions import trajectory
import time

class Arm6dof(Arm):
    def __init__(self, polygons, axis, stages, dh_table):
        super().__init__(polygons, axis, stages, dh_table)

if __name__=='__main__': 
    #Creating an Arm
    pols=[]
    #p0
    p0=Polygon()
    p0.create_cylinder(h=40,r0=120,r1=120,step=10)
    #p0.faces2triangles()
    p0.rotation(-np.pi/2,0,0)
    p0.translation((0,20,0))
    #p1
    p1=Polygon()
    p1.create_cuboid(h=240,l=120)
    #p1.faces2triangles()
    p1.rotation(-np.pi/2,0,0)
    p1.translation((0,160,0))
    #p2
    p2=Polygon()
    p2.create_semicylinder(h=40,r=60,l=70,step=6)
    #p2.faces2triangles()
    p2.rotation(0,0,0)
    p2.translation((0,350,40))
    #p3
    p3=Polygon()
    p3.create_semicylinder(h=40,r=60,l=70,step=6)
    #p3.faces2triangles()
    p3.rotation(0,0,0)
    p3.translation((0,350,-40))
    #p4
    p4=Polygon()
    p4.create_semicylinder(h=40,r=50,l=80,step=6)
    #p4.faces2triangles()
    p4.rotation(0,0,np.pi/2)
    p4.translation((0,350,0))
    #p5
    p5=Polygon()
    p5.create_cuboid(h=230,l=100)
    #p5.faces2triangles()
    p5.rotation(0,np.pi/2,0)
    p5.translation((195,350,0))
    #p6
    p6=Polygon()
    p6.create_semicylinder(h=100/3,r=50,l=60,step=6)
    #p6.faces2triangles()
    p6.rotation(0,0,-np.pi/2)
    p6.translation((370,350,100/3))
    #p7
    p7=Polygon()
    p7.create_semicylinder(h=100/3,r=50,l=60,step=6)
    #p7.faces2triangles()
    p7.rotation(0,0,-np.pi/2)
    p7.translation((370,350,-100/3))
    #p8
    p8=Polygon()
    p8.create_semicylinder(h=100/3,r=40,l=70,step=6)
    #p8.faces2triangles()
    p8.rotation(0,0,np.pi/2)
    p8.translation((370,350,0))
    #p9
    p9=Polygon()
    p9.create_cuboid(h=150,l=80)
    #p9.faces2triangles()
    p9.rotation(0,np.pi/2,0)
    p9.translation((515,350,0))
    #p10
    p10=Polygon()
    p10.create_semicylinder(h=80/3,r=40,l=50,step=6)
    #p10.faces2triangles()
    p10.rotation(0,0,-np.pi/2)
    p10.translation((640,350,80/3))
    #p11
    p11=Polygon()
    p11.create_semicylinder(h=80/3,r=40,l=50,step=6)
    #p11.faces2triangles()
    p11.rotation(0,0,-np.pi/2)
    p11.translation((640,350,-80/3))
    #p12
    p12=Polygon()
    p12.create_semicylinder(h=80/3,r=30,l=60,step=6)
    #p12.faces2triangles()
    p12.rotation(0,0,np.pi/2)
    p12.translation((640,350,0))
    #p13
    p13=Polygon()
    p13.create_cuboid(h=80,l=60)
    #p13.faces2triangles()
    p13.rotation(0,np.pi/2,0)
    p13.translation((740,350,0))
    #p14
    p14=Polygon()
    p14.create_semicylinder(h=20,r=30,l=40,step=6)
    #p14.faces2triangles()
    p14.rotation(-np.pi/2,-np.pi/2,0)
    p14.translation((820,370,0))
    #p15
    p15=Polygon()
    p15.create_semicylinder(h=20,r=30,l=40,step=6)
    #p15.faces2triangles()
    p15.rotation(-np.pi/2,-np.pi/2,0)
    p15.translation((820,330,0))
    #p16
    p16=Polygon()
    p16.create_semicylinder(h=20,r=20,l=50,step=6)
    #p16.faces2triangles()
    p16.rotation(-np.pi/2,np.pi/2,0)
    p16.translation((820,350,0))
    #p17
    p17=Polygon()
    p17.create_cuboid(h=40,l=40)
    #p17.faces2triangles()
    p17.rotation(0,np.pi/2,0)
    p17.translation((890,350,0))
    #p18
    p18=Polygon()
    p18.create_cylinder(h=40,r0=20,r1=20,step=10)
    #p18.faces2triangles()
    p18.rotation(0,np.pi/2,0)
    p18.translation((930,350,0))
    #p19
    p19=Polygon()
    p19.create_cylinder(h=40,r0=5,r1=5,step=10)
    #p19.faces2triangles()
    p19.rotation(0,np.pi/2,0)
    p19.translation((970,365,0))
    #p20
    p20=Polygon()
    p20.create_cylinder(h=40,r0=5,r1=5,step=10)
    #p20.faces2triangles()
    p20.rotation(0,np.pi/2,0)
    p20.translation((970,335,0))

    #Actual state (origin)
    axis=[np.array([[1, 0, 0, 0],
                    [0, 0, 1, 40], #40+310
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]),
        np.array([[1, 0, 0, 0],
                    [0, 1, 0, 350],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
        np.array([[1, 0, 0, 370],
                    [0, 1, 0, 350],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
        np.array([[1, 0, 0, 640],
                    [0, 1, 0, 350],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
        np.array([[1, 0, 0, 820],
                    [0, 0, 1, 250],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]),
        np.array([[0, 0, 1, 820],
                    [0, 1, 0, 350],
                    [-1, 0, 0, 0],
                    [0, 0, 0, 1]]),
        np.array([[0, 0, 1, 950],
                    [0, 1, 0, 350],
                    [-1, 0, 0, 0],
                    [0, 0, 0, 1]])]

    #Actual state (origin)
    dh_table=np.array([[0,           0,   310,    np.pi/2], #d1=0
                       [0,           370, 0,      0],
                       [0,           270, 0,      0],
                       [0,           180, 0,      -np.pi/2],
                       [0+np.pi/2, 0,   0,      np.pi/2],
                       [0,           0,   90+40, 0]])

    pol=Arm6dof(polygons=[p0,p1,p2,p3,p4,p5,
                    p6,p7,p8,p9,p10,p11,
                    p12,p13,p14,p15,p16,
                    p17,p18,p19,p20],
            axis=axis,
            stages=[1,4,8,12,16,18],
            dh_table=dh_table)

    print('Origin pos:')
    print(pol.position)
    print('Origin DH table:')
    print(pol.dh_table)
    #End arm creation

    #Initialing the screend
    width_screen=2400
    height_screen=1300
    win=turtle.Screen()
    win.setup(width_screen,height_screen)
    win.bgcolor("white")
    win.title('Robotic')
    win.tracer(0)

    pos_cam=np.array([[0,0,0,200],
                    [0,0,0,-200],
                    [0,0,0,0],
                    [0,0,0,1]])
    #vec_cam=np.eye(4)
    vec_cam=rotZ(rotY(rotX(np.eye(4),np.pi/6),0),0)-pos_cam

    my_pen=turtle.Turtle()
    my_pen.hideturtle()
    my_pen.up()

    if 0: #forward instant
        pol.move(0,-np.pi/1.5,n_axis=2)
        pol.move(1,np.pi/6,n_axis=2)
        pol.move(2,-np.pi/4,n_axis=2)
        pol.move(3,-np.pi/4,n_axis=2)
        pol.move(4,np.pi/4,n_axis=2)
        pol.draw_polygon(my_pen,vec_cam)
        print('New pos:')
        print(pol.position)
        win.update()
        turtle.done()
    else:#forward with trajectory
        sec=5
        step=100
        t1=-np.pi#/1.5
        dt1=trajectory(ti=0,tf=t1,time=sec,step=step)
        t1_=-np.pi/6
        dt1+=trajectory(ti=t1,tf=t1_,time=sec,step=step)
        
        t2=np.pi/6
        dt2=trajectory(ti=0,tf=t2,time=sec,step=step)
        t2_=-np.pi/6
        dt2+=trajectory(ti=t2,tf=t2_,time=sec,step=step)

        t3=-np.pi/4
        dt3=trajectory(ti=0,tf=t3,time=sec,step=step)
        t3_=np.pi/6
        dt3+=trajectory(ti=t3,tf=t3_,time=sec,step=step)
        
        t4=-np.pi/4
        dt4=trajectory(ti=0,tf=t4,time=sec,step=step)
        t4_=np.pi/8
        dt4+=trajectory(ti=t4,tf=t4_,time=sec,step=step)
        
        t5=np.pi/4
        dt5=trajectory(ti=0,tf=t5,time=sec,step=step)
        t5_=-np.pi/4
        dt5+=trajectory(ti=t5,tf=t5_,time=sec,step=step)
        
        pol.draw_polygon(my_pen,vec_cam)
        win.update()

        for dA,dB,dC,dD,dE in zip(dt1,dt2,dt3,dt4,dt5): #dt
            #print(dA,dB,dC,dD)
            my_pen.clear()
            pol.move(0,dA,n_axis=2)
            pol.move(1,dB,n_axis=2)
            pol.move(2,dC,n_axis=2)
            pol.move(3,dD,n_axis=2)
            pol.move(4,dE,n_axis=2)
            pol.draw_polygon(my_pen,vec_cam)
            win.update()
            #break
            time.sleep(0.05)
        turtle.done()