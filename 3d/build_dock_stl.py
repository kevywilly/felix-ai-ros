"""Build a quick REFERENCE STL of the charging dock (not print-final).
Coords: X=lateral (centerline 0), Y=depth (mouth at Y=0, robot backs in -Y),
Z=up (floor 0). Units mm. Edit PARAMS and re-run."""
import numpy as np, struct

# ---------- PARAMS (mm) ----------
MOUTH_W=160.0; THROAT_W=113.0; FUNNEL_DEPTH=70.0   # funnel taper (entry->seat)
WALL_T=6.0; WALL_Z0=5.0; WALL_Z1=86.0              # funnel wall thickness & height
BASE_T=5.0                                         # base plate thickness
SEAT_Y=-FUNNEL_DEPTH                               # battery rear face seats here (-70)
BACKWALL_Y0=SEAT_Y-5; BACKWALL_Y1=SEAT_Y           # back wall -75..-70
BACKWALL_HALF=66.0
SHELF_Y=BACKWALL_Y0-70                             # brick shelf back to -145
LIP_Y0=SHELF_Y-5; LIP_Y1=SHELF_Y; LIP_Z=30        # back lip to stop brick
PAD_X=[-33.0,0.0,33.0]; PIN_Z=[42.0,54.0]; PIN_R=1.6; PIN_PROUD=4.0  # pogo pins
OUT="dock_reference.stl"

tris=[]
def quad(a,b,c,d): tris.append((a,b,c)); tris.append((a,c,d))
def prism(loop,z0,z1):
    n=len(loop); bot=[(x,y,z0) for x,y in loop]; top=[(x,y,z1) for x,y in loop]
    for i in range(n):
        j=(i+1)%n; quad(bot[i],bot[j],top[j],top[i])      # sides
    # caps (fan)
    for i in range(1,n-1):
        tris.append((bot[0],bot[i+1],bot[i]))             # bottom
        tris.append((top[0],top[i],top[i+1]))             # top
def cyl_y(cx,cz,y0,y1,r,n=14):
    ring=[(cx+r*np.cos(t),cz+r*np.sin(t)) for t in np.linspace(0,2*np.pi,n,endpoint=False)]
    for i in range(n):
        j=(i+1)%n
        a=(ring[i][0],y0,ring[i][1]); b=(ring[j][0],y0,ring[j][1])
        c=(ring[j][0],y1,ring[j][1]); d=(ring[i][0],y1,ring[i][1])
        quad(a,b,c,d)
    c0=(cx,y0,cz); c1=(cx,y1,cz)
    for i in range(n):
        j=(i+1)%n
        tris.append((c0,(ring[i][0],y0,ring[i][1]),(ring[j][0],y0,ring[j][1])))
        tris.append((c1,(ring[j][0],y1,ring[j][1]),(ring[i][0],y1,ring[i][1])))

# base plate (full footprint, mouth Y=0 back to shelf)
prism([(-MOUTH_W/2-WALL_T,0),(MOUTH_W/2+WALL_T,0),(MOUTH_W/2+WALL_T,SHELF_Y),(-MOUTH_W/2-WALL_T,SHELF_Y)],0,BASE_T)
# funnel walls (tapered): inner mouth ±MOUTH/2, inner throat ±THROAT/2
mi=MOUTH_W/2; ti=THROAT_W/2
Lwall=[(-mi,0),(-mi-WALL_T,0),(-ti-WALL_T,SEAT_Y),(-ti,SEAT_Y)]
Rwall=[( mi,0),( ti,SEAT_Y),( ti+WALL_T,SEAT_Y),( mi+WALL_T,0)]
prism(Lwall,WALL_Z0,WALL_Z1); prism(Rwall,WALL_Z0,WALL_Z1)
# back wall (holds pogo block)
prism([(-BACKWALL_HALF,BACKWALL_Y1),(BACKWALL_HALF,BACKWALL_Y1),(BACKWALL_HALF,BACKWALL_Y0),(-BACKWALL_HALF,BACKWALL_Y0)],WALL_Z0,WALL_Z1)
# brick stop lip
prism([(-MOUTH_W/2-WALL_T,LIP_Y1),(MOUTH_W/2+WALL_T,LIP_Y1),(MOUTH_W/2+WALL_T,LIP_Y0),(-MOUTH_W/2-WALL_T,LIP_Y0)],0,LIP_Z)
# pogo pins (5): power lanes X=-33,0 doubled vertically; CC X=+33 single @48
cyl_y(-33,42,SEAT_Y,SEAT_Y+PIN_PROUD,PIN_R); cyl_y(-33,54,SEAT_Y,SEAT_Y+PIN_PROUD,PIN_R)
cyl_y(  0,42,SEAT_Y,SEAT_Y+PIN_PROUD,PIN_R); cyl_y(  0,54,SEAT_Y,SEAT_Y+PIN_PROUD,PIN_R)
cyl_y( 33,48,SEAT_Y,SEAT_Y+PIN_PROUD,PIN_R)
# two hard-stop bumper posts (battery face seats on these, not the pins)
for sx in (-50,50): cyl_y(sx,48,SEAT_Y,SEAT_Y+2.0,5.0,n=16)

# write binary STL
T=np.array(tris,dtype=np.float32)
e1=T[:,1]-T[:,0]; e2=T[:,2]-T[:,0]; nrm=np.cross(e1,e2)
nrm=nrm/(np.linalg.norm(nrm,axis=1,keepdims=True)+1e-9)
with open(OUT,"wb") as f:
    f.write(b"dock_reference felix charging dock".ljust(80,b" "))
    f.write(struct.pack("<I",len(T)))
    for i in range(len(T)):
        f.write(struct.pack("<3f",*nrm[i]))
        for v in T[i]: f.write(struct.pack("<3f",*v))
        f.write(struct.pack("<H",0))
mn=T.reshape(-1,3).min(0); mx=T.reshape(-1,3).max(0)
print(f"wrote {OUT}: {len(T)} triangles")
print("bbox X %.0f..%.0f  Y %.0f..%.0f  Z %.0f..%.0f"%(mn[0],mx[0],mn[1],mx[1],mn[2],mx[2]))
print("overall %.0f x %.0f x %.0f mm"%(mx[0]-mn[0],mx[1]-mn[1],mx[2]-mn[2]))
