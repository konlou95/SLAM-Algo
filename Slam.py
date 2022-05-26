"""Slam controller."""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from pylab import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *

pi=3.14
max_speed=3.14
distance_between_wheels=0.052 #metra  

aktina_zantas=0.0205 #metra
perifereia_zantas=2*pi*aktina_zantas #metra
encoder_tick=perifereia_zantas/6.28 #metra
#reference cylinders
reference_cylinders=[(-0.5,-0.125),(-0.25,0.5),(0.125,0),(0.5,0.5)]
cylinder_radious=0.05


def Drive_robot():                               # oi kinhseis pou lew na kanei to robot
        left_motor.setVelocity(max_speed*0.85)
        right_motor.setVelocity(max_speed)   
           
    
    
def odometry(encoder_values,pose):         # upologismos thesis x y theta modelo odometrias

        #pairnw times apo encoders
        encoder_values[0]=left_encoder.getValue()
        encoder_values[1]=right_encoder.getValue()
        
        l=encoder_values[0]*encoder_tick #metra
        r=encoder_values[1]*encoder_tick #metra
        
        #print('-----------------------------------------')
       # print('oi times twn encoder einai aktinia kai exw aristerh timh:{} deksia timh:{}'.format(encoder_values[0],encoder_values[1]))
        #print('oi times twn  to3wn se metra einai aristerh timh:{} deksia timh:{}'.format(l,r))
        
        if l<r:
            alpha=(r-l)/distance_between_wheels
            R=l/alpha
            cx=pose[0] - (R + distance_between_wheels/2)*math.sin(pose[2])
            cy=pose[1] + (R + distance_between_wheels/2)*cos(pose[2])
            theta=(pose[2]+alpha)%(2*pi)
            x = cx + (R + distance_between_wheels/2)*math.sin(theta);
            y = cy - (R + distance_between_wheels/2)*math.cos(theta);
        
        elif l>r:
            alpha=(l-r)/distance_between_wheels
            R=r/alpha
            cx=pose[0] + (R + distance_between_wheels/2)*math.sin(pose[2])
            cy=pose[1] + (R + distance_between_wheels/2)*cos(pose[2])
            theta=(pose[2]+alpha)%(2*pi)
            x = cx - (R + distance_between_wheels/2)*math.sin(theta);
            y = cy - (R + distance_between_wheels/2)*math.cos(theta);
            
        elif l==r:
            theta=pose[2]
            x=pose[0]+l*math.cos(theta)
            y=pose[1]+r*math.sin(theta)
        
        print('pose of robot {} {} {}'.format(-y,x,theta))    
        return -y,x,theta
        
        
def compute_derivative(range_image):
        der=[]
        i=1
        
        for i in range(len(range_image)-1):                    
            value=((range_image[i+1]-range_image[i-1])/2)*10
            der.append(value)
        #print('values of derivative: {}'.format(der))
        #plot(der)        
        #show()
        return(der)

def find_cylinders(range_image,der):  #upologismos ari8mou aktinas anagnwrishs kulindrou apo to lidar kai apostash ths apo to lidar proston kulindro 
        threshold=0.75
        cylinder=[]
        on_cylinder = False
        sum_ray, sum_depth, rays = 0.0, 0.0, 0
        
        for i in range(len(der)):
                if der[i]<-threshold:
                        on_cylinder = True
                        sum_ray, sum_depth, rays = 0.0, 0.0, 0
                elif abs(der[i])<= threshold and on_cylinder:
                       sum_ray+=i
                       sum_depth=range_image[i]
                       rays+=1
                elif der[i]>threshold and on_cylinder:
                        cylinder.append((sum_ray/(rays+0.00001),sum_depth))
                        on_cylinder = False
                        
        print(cylinder)    
        return cylinder
            
def sc_cartesian_cylinders(cylinders,offset):   #metatroph se kartesianes suntetagmenes ws pros to susthma aksonwn tou LIDAR
        
        angle_unit=0.02453 #gwnia se rad metaksu ka8e aktinas tou LIDAR
        sc_cylinders=[]
        for c in cylinders:
            ray,depth=c
            temp= round(ray+0.1)*angle_unit
            angle=3.14-temp
            x=(depth+offset)*math.cos(angle)
            y=(depth+offset)*math.sin(angle)
            
            #print('position of cylinders  {} {} angle {} '.format(x,y,angle))
            sc_cylinders.append((x,y,angle))
 
        return sc_cylinders

def world_cartesian_cylinder(pose,point): #metafora apo to susthma suntetagmenwn tou LIDAR sto susthma tou kosmou tou robot
        x = point[0]
        y=point[1]
        sunistamenh=sqrt(x*x+y*y)
        dx=sunistamenh*math.cos(pose[2]+point[2])
        dy=sunistamenh*math.sin(pose[2]+point[2])
        return pose[0]+dx,pose[1]+dy


def cylinder_pairs(world_cylinders): #edw ginetai match ta gnwsta shmeia tvn kulindrwn me auta pou diavazontai
 
        cylinder_pairs=[]
        for i,c in enumerate(world_cylinders):
            for j,k in enumerate(reference_cylinders):
                xx=c[0]-k[0]
                yy=c[1]-k[1]
                if sqrt(xx*xx+yy*yy)<0.1:
                    cylinder_pairs.append((i,j))
    
        
        print('{}'.format(cylinder_pairs))
        
def compute_center_of_mass(point_list):
        
        if not point_list:
            return 0.0,0.0
        sx = sum([p[0] for p in point_list])      
        sy = sum([p[1] for p in point_list])
        return (float(sx) / len(point_list), float(sy) / len(point_list))


            
                       
def similarity_transform(left_list,right_list,fix_scale = False): #edw olo to ma8hmatiko modelo tou similarity transform
        if len(left_list)==len(right_list):
                lc=compute_center_of_mass(left_list)
                rc=compute_center_of_mass(right_list)           
                for (lx, ly), (rx, ry) in zip(left_list, right_list):             
                    rx -= rc[0]
                    ry -= rc[1]
                    lx -= lc[0]
                    ly -= lc[1]
                    cs =  rx*lx + ry*ly
                    ss = -rx*ly + ry*lx
                    rr =  rx*rx + ry*ry
                    ll =  lx*lx + ly*ly
                if rr < 1e-4 and ll < 1e-4:
                    return None
                if fix_scale:
                    la = 1
                else:
                    la = sqrt(rr/ll)

                c = cs/sqrt(cs**2 + ss**2)
                s = ss/sqrt(cs**2 + ss**2)

                tx = rc[0] - la*(c*lc[0] - s*lc[1])
                ty = rc[1] - la*(s*lc[0] + c*lc[1])

                return la, c, s, tx, ty
                
def apply_transform(trafo, p):
        la, c, s, tx, ty = trafo
        lac = la * c
        las = la * s
        x = lac * p[0] - las * p[1] + tx
        y = las * p[0] + lac * p[1] + ty
        return (x, y)   
def correct_pose(pose, trafo):

        pose2 = apply_transform(trafo, pose)
        c, s = trafo[1], trafo[2]
        ang = math.atan2(s, c)
        return (pose2[0], pose2[1], pose[2] + ang)
    
    
def get_subsampled_points(scan,position, sampling):
        # Subsample from scan
        index_range_tuples = []
        for i in range(0, len(scan), sampling):
            index_range_tuples.append( (i, scan[i]) )
            
        temp=sc_cartesian_cylinders(index_range_tuples, 0.0)
        #print('antwnhs {} {}'.format(temp,len(temp)))    
        suntetagmenes_toixou=[world_cartesian_cylinder(position, c) for c in temp] 
        #print('toixos reeeeee {}'.format(suntetagmenes_toixou))
        return suntetagmenes_toixou
        
        
def wall_pairs(points,arena_left = -0.75, arena_right = 0.75,arena_bottom = -0.75, arena_top = 0.75,eps = 0.05):
        # pairing the points behind cylinders with the walls 
        left_list = []
        right_list = []
        #looking for pairs for the known coordinates of wall                              
        for x, y in points:
            if x < arena_left + eps:  
                left_list.append((x, y))
                right_list.append((arena_left, y))
            elif x > arena_right - eps:
                left_list.append((x, y))
                right_list.append((arena_right, y))
            elif y < arena_bottom + eps:
                left_list.append((x, y))
                right_list.append((x, arena_bottom))
            elif y > arena_top - eps:
                left_list.append((x, y))
                right_list.append((x, arena_top))
        
        return left_list, right_list                            
                                     
def concatenate_transform(a, b):
        laa, ca, sa, txa, tya = a
        lab, cb, sb, txb, tyb = b

    
        la = laa * lab

    
        c = ca*cb - sa*sb
        s = sa*cb + ca*sb

    
        tx = txa + laa * ca * txb - laa * sa * tyb
        ty = tya + laa * sa * txb + laa * ca * tyb

        return (la, c, s, tx, ty)  
    
    
                                       
def get_icp_transform(world_points, iterations):

    
        overall_trafo = (1.0, 1.0, 0.0, 0.0, 0.0)

        for j in range(iterations):
            trafo_pts = [apply_transform(overall_trafo, p) for p in world_points]
            left, right = wall_pairs(trafo_pts)
            trafo = similarity_transform(left, right, fix_scale = True)
            if trafo:
                overall_trafo = concatenate_transform(trafo, overall_trafo)                                     
                                     
                                     
        return overall_trafo   
        
        
def Jac_matrix(position, encoder_values): # dhmiourgia iakomvianou pinaka 
        theta = position[2]
        w=distance_between_wheels
        l=encoder_values[0]*encoder_tick #metra
        r=encoder_values[1]*encoder_tick #metra
        if r != l:

            # This is for the case r != l.
            # g has 3 components and the state has 3 components, so the
            # derivative of g with respect to all state variables is a
            # 3x3 matrix.
            alpha = (r - l) / w
            rad = l/alpha
            Rw2 = rad + w/2
            m = array([ [1, 0, Rw2*(cos(theta + alpha) - cos(theta))],
                        [0, 1, Rw2*(sin(theta + alpha) - sin(theta))],
                        [0, 0, 1]
                ])

        else:
            # This is for the special case r == l.
            m = array([ [1, 0, -l*sin(theta)],
                        [0, 1,  l*cos(theta)],
                        [0, 0, 1]
                ])
        return m 
        
        
def V_matrix(position, encoder_values):  #dhmiourgia pinaka V
        theta = position[2]
        w=distance_between_wheels
        l=encoder_values[0]*encoder_tick #metra
        r=encoder_values[1]*encoder_tick #metra
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.
            dg1dl = (w*r)*( sin(theta+alpha) - sin(theta))/(r-l)**2 \
                   -(r+l)*cos(theta+alpha)/(2*(r-l))
            dg2dl = (w*r)*(-cos(theta+alpha) + cos(theta))/(r-l)**2 \
                   -(r+l)*sin(theta+alpha)/(2*(r-l))


            dg1dr = (-w*l)*( sin(theta+alpha) - sin(theta))/(r-l)**2 \
                   +(r+l)*cos(theta+alpha)/(2*(r-l))
            dg2dr = (-w*l)*(-cos(theta+alpha) + cos(theta))/(r-l)**2 \
                   +(r+l)*sin(theta+alpha)/(2*(r-l))
        else:
            # This is for the special case l == r.
            dg1dl = 0.5*(cos(theta) + (l/w)*sin(theta))
            dg2dl = 0.5*(sin(theta) - (l/w)*cos(theta))
            dg1dr = 0.5*(-(l/w)*sin(theta) + cos(theta))
            dg2dr = 0.5*( (l/w)*cos(theta) + sin(theta))

        dg3dl = -1/w
        dg3dr =  1/w

        m = array([[dg1dl, dg1dr],
                   [dg2dl, dg2dr],
                   [dg3dl, dg3dr]])

        return m  
        
def predict(encoder_values,jac_matrix,V,covariance):    #prediction step

        l=encoder_values[0]*encoder_tick #metra
        r=encoder_values[1]*encoder_tick #metra  
        control_motion_factor = 0.35  # Error in motor control.
        control_turn_factor = 0.6  # Additional error due to slip when turning.
        sigL2 = (control_motion_factor*l)**2 + (control_turn_factor*(r-l))**2
        sigR2 = (control_motion_factor*r)**2 + (control_turn_factor*(r-l))**2
        control_cov = diag([sigL2, sigR2])
        
       
        covariance= jac_matrix @ covariance @ jac_matrix.T + V @ control_cov @ V.T
        
        return covariance
 
def h(position, landmark):       #non linear function of measurment
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - position[0]
        dy = landmark[1] - position[1] 
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - position[2] + pi) % (2*pi) - pi

        return array([r, alpha]) 
        
def dh_dstate(position, landmark,):      # Jacobian matrix of h non linear function of measurment
        x, y, theta = position
        xm, ym = landmark
        d = 0

        xl = x + d*cos(theta)
        yl = y + d*sin(theta)
        q = (xm - xl)**2 + (ym - yl)**2

        dx = xm - xl
        dy = ym - yl

        # Note that:
        # x y theta is state[0] state[1] state[2]
        # x_m y_m is landmark[0] landmark[1]
        # The Jacobian of h is a 2x3 matrix.
        drdx = -dx/sqrt(q)
        drdy = -dy/sqrt(q)
        drdtheta = d*(dx*sin(theta) - dy*cos(theta))/sqrt(q)

        dadx = dy/q
        dady = -dx/q
        dadtheta = - d/q * (dx*cos(theta) + dy*sin(theta)) - 1
        return array([[drdx, drdy, drdtheta],
                      [dadx, dady, dadtheta]])   
                      
def correct(measurement, landmark,h,H,covariance):
        """The correction step of the Kalman filter."""
        Z = array(measurement)
        Z[1] = atan2(sin(Z[1]), cos(Z[1]))

        K = covariance @ H.T @ linalg.inv(H @ covariance @ H.T + self.Q)
        innov = Z - h
        innov[1] = atan2(sin(innov[1]), cos(innov[1]))

        self.state = self.state + K @ innov
        covariance = (eye(3) - K @ H) @ covariance                
    
                                          
if __name__== "__main__":
    # create the Robot instance.
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    # motor instances
    left_motor=robot.getDevice('left wheel motor')
    right_motor=robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)  
    #sensor instances
    left_encoder=robot.getDevice('left wheel sensor')
    left_encoder.enable(timestep)
    right_encoder=robot.getDevice('right wheel sensor')
    right_encoder.enable(timestep)
    lidar=robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
    camera=robot.getDevice('camera')
    camera.enable(timestep)
    
    camera.recognitionEnable(timestep)
    encoder_values=[0,0]
    pose=[-0.25,-0.5,0] # 8esh tou robot prepei omws na dwsw [-z,-x,theta] apo to translation to epuck
    covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    
    
    while robot.step(timestep) != -1:
        Drive_robot()
        print('------------')
        position=odometry(encoder_values,pose)
        range_image=lidar.getRangeImage() 
        der=compute_derivative(range_image) 
        cylinders=find_cylinders(range_image,der)      
        sc_cylinders=sc_cartesian_cylinders(cylinders,0.05)
        world_cylinders = [world_cartesian_cylinder(position, c) for c in sc_cylinders]
      
        cylinder_pairs(world_cylinders)
        trafo=similarity_transform(world_cylinders, reference_cylinders,fix_scale = True)
        if trafo:
            transformed_world_cylinders=[apply_transform(trafo,t)for t in world_cylinders]  
            #print('dior8wmenoi kulindroi {}'.format(transformed_world_cylinders))
            position=correct_pose(position, trafo)
            #print('dior8wmenh troxia {}'.format(position))
        suntetagmenes_toixou=get_subsampled_points(range_image,position, sampling = 10)  
        wall_trafo = get_icp_transform(suntetagmenes_toixou, iterations = 40)
        if wall_trafo:
            position=correct_pose(position, wall_trafo)
            #print('akoma pio dior8wmenh troxia {}'.format(position))
            
       
        jac_matrix= Jac_matrix(position, encoder_values)        
        V = V_matrix(position, encoder_values)
        covariance=predict(encoder_values,jac_matrix,V,covariance)
        
        
        for i in range(len(world_cylinders)):
            
            landmark=world_cylinders[i]
            #print('land  {}'.format(landmark))
            #h=h(position, landmark)
            #H=dh_dstate(position, landmark)
            #correct(measurement, landmark,h,H,covariance)
        #print(' {}'.format(range_image))
        
        
       
        
        
