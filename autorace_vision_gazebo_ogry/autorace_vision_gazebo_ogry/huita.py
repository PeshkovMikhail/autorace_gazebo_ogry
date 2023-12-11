import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import struct
import cv2
import cv_bridge

CAM_ANGLE = 0.11
CAM_FOV_HOR = 1.51843645
CAM_H = 480
CAM_W = 848
CAM_FOV_VER = 2 * np.arctan(np.tan(CAM_FOV_HOR/2)*CAM_H/CAM_W)

def image_remap(img, ind):
    return img[int(CAM_H-ind[1]-1), int(ind[0])]

def to_real_angles(coords):
    coords[1] = CAM_H - coords[1]-1
    res = np.zeros_like(coords,dtype=np.float32)
    res[0] = np.arctan(np.tan(CAM_FOV_HOR/2)*(2*coords[0]/(CAM_W-1)-1))
    res[1] = CAM_ANGLE + np.arctan(np.tan(CAM_FOV_VER/2)*(2*coords[1]/(CAM_H-1)-1))
    return res

def img_coords_translate_2_road(coords, depth):
    cds = image_remap(depth,coords)
    
    angles = to_real_angles(coords.copy())
    
    res = np.zeros_like(coords,dtype=np.float32)
    
    res[0] = np.cos(angles[1])*np.sin(angles[0])*cds
    res[1] = np.cos(angles[0])*np.cos(angles[1])*cds
    z = np.cos(angles[0])*np.sin(angles[1])*cds
   
    if abs(z-0.11) < 0.03:
        coef = 0.11/z
        res *= coef
    return res


def isroad(coords, depth):
    # return mask[CAM_H-1-coords[1] , coords[0]] == 0 and not np.isinf(depth[coords[1] , coords[0]])

    cds = image_remap(depth,coords)
    if np.isinf(cds):
        return False
    angles = to_real_angles(coords.copy())
    
    res = np.zeros_like(coords,dtype=np.float32)
    
    res[0] = np.cos(angles[1])*np.sin(angles[0])*cds
    res[1] = np.cos(angles[0])*np.cos(angles[1])*cds
    z = np.cos(angles[0])*np.sin(angles[1])*cds

    return abs(z-0.11) < 0.03



def road_coords_2_img(coords):
    i = (coords[0]/(coords[1]*np.tan(CAM_FOV_HOR/2))+1)/2*(CAM_W-1)
    j = (CAM_H-1)/2*(np.tan(np.arctan2(0.11,coords[1])-CAM_ANGLE)/np.tan(CAM_FOV_VER/2)+1)
    j = CAM_H-j-1
    return np.array([np.round(i),np.round(j)])


def get_line(f_p,s_p):
    if np.abs(f_p[0]-s_p[0])<0.01:
        return "V",(f_p[0]+s_p[0])/2
    k = (f_p[1]-s_p[1])/(f_p[0]-s_p[0])
    b = f_p[1]-k*f_p[0]
    return "H", (k,b)

def get_fucked(coord1,coord2, depth, road_t):
    s_pt =  5
    l_i = [np.array([424,0]),np.array([424,s_pt])]
    r_i = [np.array([424,0]),np.array([424,s_pt])]
    dat = [l_i,r_i]
    for j in [0,1]:
        for i in range(424,848):
            if not isroad(np.array([i,j*s_pt]), depth):
                break
            r_i[j][0] = i
        
        for i in range(424,-1,-1):
            if not isroad(np.array([i,j*s_pt]), depth):
                break
            l_i[j][0] = i
    
    road = get_line(img_coords_translate_2_road(dat[road_t][0], depth),img_coords_translate_2_road(dat[road_t][1], depth))
    
    my_line = get_line(coord1,coord2)

    if road[0] == "V":
        x = road[1]
        if my_line[0]=="V":
            return [np.nan]
        y = my_line[1][0]*x+my_line[1][1]
        return np.array([x,y])
    
    k1,b1 = road[1]
    k2,b2 = my_line[1]

    if k1==k2:
        return [np.nan]
    
    x = (b2-b1)/(k1-k2)
    y = k1*x+b1
    return np.array([x,y])


def check_down(coord1,coord2, depth):
    c1,c2 = img_coords_translate_2_road(coord1, depth),img_coords_translate_2_road(coord2, depth)
    if (coord1[1]==0):
        n_coord = get_fucked(c1,c2,depth,0)
        if np.isnan(n_coord[0]):
            return (c1,c2)
        return (n_coord,c2)
    if (coord2[1]==0):
        n_coord = get_fucked(c1,c2,depth,1)
        if np.isnan(n_coord[0]):
            return (c1,c2)
        return (c1,n_coord)
    return (c1,c2)

def build_perpendicular_line(v, p):
    nv = (v[1],-v[0])
    if nv[1]!=0 and abs(nv[0]/nv[1])<1:
        return "V", lambda y: nv[0]*(y-p[1])/nv[1]+p[0]  
    
    return "H",lambda x: nv[1]*(x-p[0])/(nv[0]) + p[1]

def draw_line(nv,p):
    if nv[1]!=0 and abs(nv[0]/nv[1])<1:
        return "V", lambda y: nv[0]*(y-p[1])/nv[1]+p[0]  
    
    return "H",lambda x: nv[1]*(x-p[0])/(nv[0]) + p[1]


def normalize(vec):
    return vec/(np.sqrt(np.sum(vec**2)))

hue_white_l =  0
hue_white_h =  179
saturation_white_l =  0
saturation_white_h =  70
lightness_white_l =  230
lightness_white_h =  255

hue_yellow_l = 10
hue_yellow_h = 40
saturation_yellow_l = 100
saturation_yellow_h = 255
lightness_yellow_l = 180
lightness_yellow_h = 255

reliability_white_line = 100
reliability_yellow_line = 100

is_calibration_mode = True

def maskWhiteLane(image):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    Hue_l = hue_white_l
    Hue_h = hue_white_h
    Saturation_l = saturation_white_l
    Saturation_h = saturation_white_h
    Lightness_l = lightness_white_l
    Lightness_h = lightness_white_h

    # define range of white color in HSV
    lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
    upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask = mask)

    fraction_num = np.count_nonzero(mask)

    how_much_short = 0

    for i in range(0, 480):
        if np.count_nonzero(mask[i,::]) > 0:
            how_much_short += 1

    how_much_short = 480 - how_much_short

    return fraction_num, mask

def maskYellowLane(image):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    Hue_l = hue_yellow_l
    Hue_h = hue_yellow_h
    Saturation_l = saturation_yellow_l
    Saturation_h = saturation_yellow_h
    Lightness_l = 95
    Lightness_h = lightness_yellow_h

    # define range of yellow color in HSV
    lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
    upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask = mask)

    fraction_num = np.count_nonzero(mask)

    if is_calibration_mode == False:
        if fraction_num > 35000:
            if lightness_yellow_l < 250:
                lightness_yellow_l += 20
        elif fraction_num < 5000:
            if lightness_yellow_l > 90:
                lightness_yellow_l -= 20

    how_much_short = 0

    for i in range(0, 480):
        if np.count_nonzero(mask[i,::]) > 0:
            how_much_short += 1
    
    how_much_short = 480 - how_much_short

    return fraction_num, mask


class Huita(Node):
    def __init__(self):
        super().__init__('huita')

        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image_sub = self.create_subscription(Image, "/color/image", self.get_image, 10)
        self.depth_sub = self.create_subscription(Image, "/depth/image", self.get_depth, 10)

        self.cv_image_output = self.create_publisher(Image, "/detect/img", 10)

        self.depth = None #np.zeros((480, 848))
        self.cvBridge = cv_bridge.CvBridge()

    def get_image(self, msg):
        if self.depth is None:
            return
        img = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        _, ym = maskYellowLane(img.copy())
        _, wm = maskWhiteLane(img.copy())

        self.mask = np.logical_or(ym, wm)

        ITER_COUNT = 10
        VEC_SIZE = 10
        VEC_COEF = 0
        VEC_FADING = [0.5 for i in range(ITER_COUNT)]

        points = [
            np.array([394, 0])
        ]

        vectors = [
            np.array([0, VEC_SIZE])
        ]

        perp = build_perpendicular_line(vectors[-1],points[-1]+vectors[-1])
        show_line = img.copy()
        for iter in range(ITER_COUNT):
            gp = points[-1]+vectors[-1]
            pp = points[-1]
            vec = vectors[-1]

            f = perp
            if f[0]=="H":
                f = f[1]
                iwy = gp[1]
                for iwx in range(int(np.round(gp[0])),848):
                    pt = int(f(iwx))
                    if (not 0<=pt<480) or  not isroad(np.array([iwx,pt]), self.depth):
                        iwx-=1
                        break
                    iwy = int(np.round(pt))
                    show_line[479-int(pt), iwx] = (255, 0, 255)

                iyy = gp[1]
                for iyx in range(int(np.round(gp[0])),-1,-1):
                    pt = int(f(iyx))
                    if not 0<=pt<480 or not isroad(np.array([iyx,pt]), self.depth):
                        iyx+=1
                        break
                    iyy = int(np.round(pt))
                    show_line[479-int(pt), iyx] = (255, 0, 255)
                    
            else:
                f= f[1]
                iwx = gp[0]
                for iwy in range(int(np.round(gp[1])),480):
                    pt = int(f(iwy))
                    if (not 0<=pt<848) or  not isroad(np.array([pt,iwy]),  self.depth):
                        iwy-=1
                        break
                    iwx = int(np.round(pt))
                    show_line[479-iwy, iwx] = (255, 0, 255)

                iyx = gp[0]
                for iyy in range(int(np.round(gp[1])),-1,-1):
                    pt = int(f(iyy))
                    if not 0<=pt<848 or not isroad(np.array([pt,iyy]), self.depth):
                        iyy+=1
                        break
                    iyx = int(np.round(pt))
                    show_line[479-iyy, iyx] = (255, 0, 255)

            y = np.array((iyx, iyy))
            w = np.array((iwx, iwy))

            
            
            print("Y",y, "W", w)
            # print("NO CHECK Y",img_coords_translate_2_road(y),"W", img_coords_translate_2_road(w))
            y,w = check_down(y,w, self.depth)
            
            p_new = road_coords_2_img((y + w)/2)

            n_w = p_new-pp
            print(p_new)
            print(n_w)
            n_w = normalize(n_w)
            n_w *= VEC_SIZE
            
            
            
            # n_w = n_w*(VEC_FADING[iter])+vec*(1-VEC_FADING[iter])
            print("NEW VEC",n_w)
            points.append(n_w+pp)
            show_line[479-int(points[-1][1])-5:479-int(points[-1][1])+5,int(points[-1][0])-5:int(points[-1][0])+5]=(255,0,255)
            print("NEW POINT",points[-1])
            vectors.append(n_w)
            print("LAST POINT",points[-1])

            

            perp_vec = (y+w)/2-img_coords_translate_2_road(points[-2], self.depth)
            print("NOT PERP VEC",perp_vec)
            perp_vec[0],perp_vec[1] = perp_vec[1],-perp_vec[0]
            print("PEPR",perp_vec)
            perp_vec += img_coords_translate_2_road(points[-1], self.depth)
            print("POINT",perp_vec)
            perp_vec = road_coords_2_img(perp_vec)-points[-1]
            print("RESULT VECTOR",perp_vec)
            
            perp = draw_line(perp_vec,points[-1]+vectors[-1])
            print(perp[1](points[-1][0]))
        print(perp_vec)

        show_line[479-int(points[-1][1])-5:479-int(points[-1][1])+5,int(points[-1][0])-5:int(points[-1][0])+5]=(255,0,0)
            
        res = Twist()

        
        PIZDEC = normalize(img_coords_translate_2_road(points[-1],self.depth)-img_coords_translate_2_road(points[0],self.depth))
        print("PIZDEC",type(PIZDEC[0]))
        self.cv_image_output.publish(self.cvBridge.cv2_to_imgmsg(show_line, "rgb8"))
        res.linear.x = float(PIZDEC[1])*0.1
        res.angular.z = float(-np.arctan2(PIZDEC[0], PIZDEC[1]))
        self.cmd_vel.publish(res)
        



    def get_depth(self, msg):
        if self.depth is None:
            self.depth = np.zeros((480, 848))
        for y in range(480):
            for x in range(848):
                offset = msg.step*y + x*4
                self.depth[y, x] = struct.unpack("f", msg.data[offset: offset + 4])[0]


def main(args=None):
    rclpy.init(args=args)

    driver = Huita()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()