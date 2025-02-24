import matplotlib.pyplot as plt
import numpy as np

def convert_depth(x,z,depth):
    return 1.5*depth*x/(-z+2*depth) #1, 1.5 or 1.5,  2.0

def plot_face(face):
    n=len(face)
    for i in range(n):
        plt.plot([face[i][0],face[(i+1)%n][0]],
                 [face[i][1],face[(i+1)%n][1]],color='black')
    #plt.show()

def calculate_centroid_3d(vertices):
    vertices=vertices[:,:-1]
    # Step 1: Compute the normal vector of the plane
    v1, v2 = vertices[1] - vertices[0], vertices[2] - vertices[0] #Warning
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)  # Normalize
    # Step 2: Define basis vectors for the 2D plane
    basis_x = v1 / np.linalg.norm(v1)  # First basis vector
    basis_y = np.cross(normal, basis_x)  # Orthogonal vector
    # Step 3: Project vertices onto 2D plane
    vertices_2d = np.array([
        [np.dot(v - vertices[0], basis_x), np.dot(v - vertices[0], basis_y)]
        for v in vertices
    ])
    # Step 4: Calculate 2D centroid
    n = len(vertices_2d)
    A = 0
    C_x, C_y = 0, 0
    for i in range(n):
        x1, y1 = vertices_2d[i]
        x2, y2 = vertices_2d[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        A += cross
        C_x += (x1 + x2) * cross
        C_y += (y1 + y2) * cross
    A *= 0.5
    C_x /= (6 * A)
    C_y /= (6 * A)
    # Step 5: Map back to 3D
    centroid_2d = np.array([C_x, C_y])
    centroid_3d = vertices[0] + centroid_2d[0] * basis_x + centroid_2d[1] * basis_y
    return np.append(centroid_3d,1) #[px, py, pz, 1]

def length_verification(face,max):
    n=len(face)
    for i in range(n):
        length=np.sqrt(sum((face[i]-face[(i+1)%n])**2))
        if length>max:
            return True
    return False

def polygon2triangles(face):
    center=calculate_centroid_3d(face)
    n=len(face)
    new_faces=[]
    for i in range(n):
        new_face=np.array([face[i],
                           face[(i+1)%n],
                           center])
        new_faces.append(new_face)
    return new_faces #return N triangles

def triangle2triangles(face,max,stack):
    n=len(face)
    max_len=-1
    for i in range(n):
        length=np.sqrt(sum((face[i]-face[(i+1)%n])**2))
        if length>max_len:
            max_len=length
            j=i
    #Length verification
    if max_len<=max:
        return stack+[face] #return same triangle
    center=(face[j]+face[(j+1)%n])/2
    for i in range(n):
        if i!=j:
            new_face=np.array([face[i],
                            face[(i+1)%n],
                            center])
            stack=triangle2triangles(new_face,max=max,stack=stack)
    return stack

def trajectory(ti,tf,time,step): #third degree trajectory
    c0=ti
    c1=0
    c3=-(tf-c0-c1*time/2)/(1/2*time**3)
    c2=-(3*c3*time**2+c1)/(2*time)
    t=np.linspace(0,time,step,endpoint=True)
    derivative=c1+2*c2*t+3*c3*t**2
    dt=[p*(time/step) for p in derivative[1:]] #time/step = d_time
    #pos=c0+c1*t+c2*t**2+c3*t**3
    #dt=[p-pos[i] for i,p in enumerate(pos[1:])]
    return dt

if __name__=='__main__':
    vertices = np.array([
        [0, 0, 0,1],
        [10,-1,0,1],
        [20,0,0,1],
        [20, 5, 0,1],
        [0, 5, 0,1],
    ])

    r=10
    #vertices=np.array([[r*np.cos(t),r*np.sin(t),0,1] for t in np.linspace(0,2*np.pi,21)]) #circle
    #vertices=np.array([[r*np.cos(t),r*np.sin(t),0,1] for t in np.linspace(0,np.pi,10,endpoint=True)]) #semi circle
    vertices=np.array([[r*np.cos(t),r*np.sin(t),0,1] for t in np.linspace(0,np.pi,10,endpoint=True)]
                    +[[-r,-r,0,1],[r,-r,0,1]]) #semi circle with square
    centroid = calculate_centroid_3d(vertices)
    #print("Centroid:", centroid)
    faces=[]
    for tri in polygon2triangles(vertices):
        stack=triangle2triangles(tri,max=11,stack=[])
        for face in stack:
            plot_face(face)
    plt.axis('scaled')
    plt.show()