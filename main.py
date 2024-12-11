from vpython import *


#畫面
scene.width = scene.height = 500
L = 0.5
scene.range = 1.0*L
scene.center = vector(0.5*L, -0.25*L, 0*L)
#座標軸
R = L/100
d = 0.5*L
xaxis = cylinder(pos=vec(0,0,0), axis=vec(d,0,0), radius=R, color=color.yellow)
yaxis = cylinder(pos=vec(0,-L,0), axis=vec(0,3*d,0), radius=R, color=color.yellow)
zaxis = cylinder(pos=vec(0,0,0), axis=vec(0,0,d), radius=0.1*R, color=color.yellow)
#座標軸文字
k = 1.1
h= 0.1*L
text(pos=xaxis.pos+k*xaxis.axis, text='x', height=h, align='center', billboard=True, emissive=True)
text(pos=yaxis.pos+k*yaxis.axis, text='y', height=h, align='center', billboard=True, emissive=True)
# text(pos=zaxis.pos+k*zaxis.axis, text='z', height=h, align='center', billboard=True, emissive=True)

scene.camera.pos=vec(0,-0.3*L,1.5*L)

#物體
mass=0.2
R=0.05
ks=50000
kball=2000
Ls0=0.4
velocity=vector(0,0,0)
g=vector(0,-9.8,0)


ball=sphere(pos=vector(0,0,0),color=color.orange,radius=R,make_trail=True,trail_type='points',interval=2,retain=10)
Pivot=sphere(pos=vector(0,0,0),color=color.green,radius=0.2*R)
ball2=sphere(pos=vector(-ball.pos.x,ball.pos.y,ball.pos.z),color=color.orange,radius=R)

ball_arrow = arrow(pos=ball.pos,axis=velocity,color=color.green,shaftwidth=0.02)
string = cylinder(pos=vector(0,0,0), axis=ball.pos, radius=0.1*R)
string2 = cylinder(pos=vector(0,0,0), axis=ball2.pos, radius=0.1*R)

Lcurve = gcurve(color=color.red)

#初始角度
angle0=10*(2*pi/360)
ball.pos=Ls0*vec(sin(angle0),-cos(angle0),0)

#運動
periodT=2*pi*sqrt(Ls0/mag(g))
#Pivot
PivotA=0.2
PivotW=2*(2*pi/periodT)*1.2

Bresist=0.05

delta_t=periodT/20000


t=0
while t<30*periodT:
    rate(20000)

    Pivot.pos.y=PivotA*sin(PivotW*t)
    string.pos.y=Pivot.pos.y
    string2.pos.y=Pivot.pos.y
    Fg=mass*g

    Fwire=-ks*(mag(ball.pos-Pivot.pos)-Ls0)*(ball.pos-Pivot.pos)/mag(ball.pos-Pivot.pos)


    Fresist =- Bresist*velocity
    if (ball.pos.x-R)<0:
        Fwall =- kball*(ball.pos.x-R)*vector(1,0,0)
    else:
        Fwall=vector(0,0,0)

    force = Fg+Fwire+Fresist+Fwall

    last_velocity=velocity
    last_vtan=last_velocity-(last_velocity.dot(string.axis.norm()))*string.axis.norm()
    velocity=velocity+force/mass*delta_t
    vtan=velocity-(velocity.dot(string.axis.norm()))*string.axis.norm()

    ball.pos=ball.pos+velocity*delta_t
    ball2.pos=vector(-ball.pos.x,ball.pos.y,ball.pos.z)
    string.axis=ball.pos-Pivot.pos
    string2.axis=ball2.pos-Pivot.pos
    ball_arrow.pos=ball.pos
    ball_arrow.axis=velocity

    angle=atan(-ball.pos.x/ball.pos.y)*180/pi

    if (last_vtan.dot(vtan)) < 0.0 and (ball.pos.x-R) > 0:
        print(f"time={t}", "\t", f"angle={angle}")
        Lcurve.plot(t,angle)
    t += delta_t

ball.color = color.green

def test():
    
if __name__ == '__main__':





