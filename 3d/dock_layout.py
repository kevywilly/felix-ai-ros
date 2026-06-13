"""Felix charging-dock layout — dimensioned 2D views, built from real robot numbers.
Edit the PARAMS block and re-run to regenerate. Coordinates: X=lateral(R-L),
Y=fore/aft(depth, +Y into robot), Z=up. Floor z=0, centerline x=0, seated
battery rear face at y=0."""
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

# ---- ROBOT (measured) ----
WHEEL_D=96; WHEEL_W=38; TRACK=265            # wheel dia/width, center-to-center
AXLE_Z=WHEEL_D/2                              # 48
BATT_W=107; BATT_H=72; BATT_DEPTH=40         # rear face W x H, depth into robot
BATT_CZ=AXLE_Z                               # battery centered on axle -> 48
BATT_Z0=BATT_CZ-BATT_H/2; BATT_Z1=BATT_CZ+BATT_H/2   # 12 .. 84
FRAME_W=192; FRAME_H=43; FRAME_Z0=34.5; FRAME_Z1=FRAME_Z0+FRAME_H  # 34.5..77.5
wheel_cx=TRACK/2                             # 132.5
wheel_in=wheel_cx-WHEEL_W/2                  # 113.5 inner face
gap_in=2*wheel_in                            # 227 between inner faces

# ---- DOCK (design params) ----
PAD_Z=AXLE_Z                                 # pad/pin center height 48
PAD_PITCH=33; PAD_W=[25,25,14]               # VBUS,GND,CC widths (CC narrower)
MOUTH_W=160; THROAT_W=BATT_W+6               # 113 -> 3mm side clearance
FUNNEL_DEPTH=70                              # y depth of taper
WALL_Z0=8; WALL_Z1=88                        # funnel wall vertical span (catches batt edges)
WALL_T=5; BASE_T=4
POGO_PROUD=3                                 # pins stick out past seat plane
SHELF_DEPTH=70                               # brick shelf behind back wall
half_angle=np.degrees(np.arctan((MOUTH_W-THROAT_W)/2/FUNNEL_DEPTH))

def dim(ax,x0,y0,x1,y1,txt,off=0,va='center',ha='center',c='b'):
    ax.annotate('',xy=(x1,y1),xytext=(x0,y0),arrowprops=dict(arrowstyle='<->',color=c,lw=1))
    ax.text((x0+x1)/2,(y0+y1)/2+off,txt,color=c,ha=ha,va=va,fontsize=8,
            bbox=dict(fc='white',ec='none',alpha=0.8,pad=0.5))

fig,axes=plt.subplots(1,3,figsize=(20,7))

# ===== REAR VIEW (X-Z): what backs into the dock =====
ax=axes[0]; ax.set_title("REAR VIEW (looking at robot's back / dock entrance)")
ax.axhline(0,color='saddlebrown',lw=2); ax.text(-150,-6,"floor",color='saddlebrown')
for s in (-1,1):
    ax.add_patch(Circle((s*wheel_cx,AXLE_Z),WHEEL_D/2,fc='0.85',ec='k'))
    ax.text(s*wheel_cx,AXLE_Z,"wheel\nØ96",ha='center',va='center',fontsize=8)
ax.add_patch(Rectangle((-FRAME_W/2,FRAME_Z0),FRAME_W,FRAME_H,fc='0.93',ec='k',ls='--'))
ax.text(-FRAME_W/2+4,FRAME_Z1-7,"frame 192 wide",fontsize=8)
ax.add_patch(Rectangle((-BATT_W/2,BATT_Z0),BATT_W,BATT_H,fc='#cfe',ec='k',lw=1.5))
ax.text(0,BATT_Z1-8,"battery rear face\n107 x 72",ha='center',fontsize=8)
# pads
xs=[-PAD_PITCH,0,PAD_PITCH]; lbl=['VBUS','GND','CC']
for x,w,l in zip(xs,PAD_W,lbl):
    ax.add_patch(Rectangle((x-w/2,PAD_Z-9),w,18,fc='#fd6',ec='k'))
    ax.text(x,PAD_Z,l,ha='center',va='center',fontsize=7)
# dock mouth footprint (dashed) entering between wheels
ax.add_patch(Rectangle((-MOUTH_W/2,WALL_Z0),MOUTH_W,WALL_Z1-WALL_Z0,fc='none',ec='r',ls=':',lw=1.5))
ax.text(-MOUTH_W/2,WALL_Z1+3,"dock mouth 160 (clears wheels)",color='r',fontsize=8)
dim(ax,-wheel_cx,-22,wheel_cx,-22,"track 265 c-c",off=-7)
dim(ax,-wheel_in,-12,wheel_in,-12,"gap 227",off=4,c='g')
dim(ax,BATT_W/2+30,BATT_Z0,BATT_W/2+30,BATT_Z1,"72")
dim(ax,BATT_W/2+45,0,BATT_W/2+45,BATT_Z0,"12 clr",c='g')
ax.plot([-BATT_W/2-60,BATT_W/2+60],[PAD_Z,PAD_Z],'r--',lw=0.7); ax.text(BATT_W/2+62,PAD_Z,"Z=48\npad ctr",color='r',fontsize=7,va='center')
ax.set_aspect('equal'); ax.set_xlim(-200,210); ax.set_ylim(-30,110); ax.set_xlabel("X lateral (mm)"); ax.set_ylabel("Z up (mm)"); ax.grid(alpha=.3)

# ===== TOP VIEW (X-Y): funnel taper =====
ax=axes[1]; ax.set_title(f"TOP VIEW  (funnel half-angle ~{half_angle:.0f}°)")
# robot battery footprint into +y
ax.add_patch(Rectangle((-BATT_W/2,0),BATT_W,BATT_DEPTH,fc='#cfe',ec='k'))
ax.text(0,BATT_DEPTH/2,"battery\n(seated)",ha='center',va='center',fontsize=8)
for s in (-1,1):
    ax.add_patch(Rectangle((s*wheel_cx-WHEEL_W/2,-10),WHEEL_W,40,fc='0.85',ec='k'))
    ax.text(s*wheel_cx,30,"wheel",ha='center',fontsize=7)
# funnel walls (dock at y<=0)
L=[Polygon([(-THROAT_W/2,0),(-MOUTH_W/2,-FUNNEL_DEPTH),(-MOUTH_W/2-WALL_T,-FUNNEL_DEPTH),(-THROAT_W/2-WALL_T,0)],closed=True,fc='#fcc',ec='r'),
   Polygon([( THROAT_W/2,0),( MOUTH_W/2,-FUNNEL_DEPTH),( MOUTH_W/2+WALL_T,-FUNNEL_DEPTH),( THROAT_W/2+WALL_T,0)],closed=True,fc='#fcc',ec='r')]
for p in L: ax.add_patch(p)
ax.add_patch(Rectangle((-MOUTH_W/2-WALL_T,-FUNNEL_DEPTH-BASE_T),MOUTH_W+2*WALL_T,BASE_T,fc='r',ec='r')) # back wall
ax.add_patch(Rectangle((-15,-6),30,6,fc='#888',ec='k')); ax.text(0,-12,"pogo block (floats ±5)",ha='center',color='r',fontsize=7)
ax.add_patch(Rectangle((-60,-FUNNEL_DEPTH-BASE_T-SHELF_DEPTH),120,SHELF_DEPTH,fc='none',ec='gray',ls='--'))
ax.text(0,-FUNNEL_DEPTH-BASE_T-SHELF_DEPTH/2,"PD brick shelf",ha='center',color='gray',fontsize=8)
dim(ax,-MOUTH_W/2,-FUNNEL_DEPTH-14,MOUTH_W/2,-FUNNEL_DEPTH-14,"mouth 160",off=-6,c='r')
dim(ax,-THROAT_W/2,8,THROAT_W/2,8,"throat 113",off=4,c='r')
dim(ax,MOUTH_W/2+20,0,MOUTH_W/2+20,-FUNNEL_DEPTH,"depth 70",c='r')
ax.set_aspect('equal'); ax.set_xlim(-170,170); ax.set_ylim(-FUNNEL_DEPTH-BASE_T-SHELF_DEPTH-15,55); ax.set_xlabel("X lateral (mm)"); ax.set_ylabel("Y depth (+ into robot)"); ax.grid(alpha=.3)

# ===== SIDE VIEW (Y-Z) =====
ax=axes[2]; ax.set_title("SIDE VIEW")
ax.axhline(0,color='saddlebrown',lw=2)
ax.add_patch(Circle((BATT_DEPTH+10,AXLE_Z),WHEEL_D/2,fc='0.85',ec='k')); ax.text(BATT_DEPTH+10,AXLE_Z,"wheel",ha='center',va='center',fontsize=7)
ax.add_patch(Rectangle((0,BATT_Z0),BATT_DEPTH,BATT_H,fc='#cfe',ec='k')); ax.text(BATT_DEPTH/2,BATT_CZ,"battery",ha='center',va='center',fontsize=8)
# dock: base + back wall + funnel wall profile + pogo
ax.add_patch(Rectangle((-FUNNEL_DEPTH-BASE_T,0),FUNNEL_DEPTH+BASE_T+2,BASE_T,fc='r')) # base
ax.add_patch(Rectangle((-FUNNEL_DEPTH-BASE_T,WALL_Z0),BASE_T,WALL_Z1-WALL_Z0,fc='r')) # back wall
ax.add_patch(Rectangle((-FUNNEL_DEPTH,WALL_Z0),2,WALL_Z1-WALL_Z0,fc='#fcc',ec='r')) # wall (mouth)
ax.add_patch(Rectangle((-15,PAD_Z-9,),15+POGO_PROUD,18,fc='#888',ec='k')); ax.text(-30,PAD_Z+16,"pogo @Z48",color='r',fontsize=7)
ax.add_patch(Rectangle((-FUNNEL_DEPTH-BASE_T-SHELF_DEPTH,0),SHELF_DEPTH,BASE_T+30,fc='none',ec='gray',ls='--')); ax.text(-FUNNEL_DEPTH-SHELF_DEPTH,40,"brick",color='gray',fontsize=8)
dim(ax,BATT_DEPTH+62,0,BATT_DEPTH+62,PAD_Z,"48",c='r')
dim(ax,BATT_DEPTH+50,0,BATT_DEPTH+50,BATT_Z0,"12",c='g')
dim(ax,-FUNNEL_DEPTH-BASE_T,WALL_Z1+6,-2,WALL_Z1+6,"funnel depth 70",off=4,c='r')
ax.set_aspect('equal'); ax.set_xlim(-FUNNEL_DEPTH-BASE_T-SHELF_DEPTH-15,BATT_DEPTH+80); ax.set_ylim(-12,108); ax.set_xlabel("Y depth (mm)"); ax.set_ylabel("Z up (mm)"); ax.grid(alpha=.3)

plt.tight_layout(); plt.savefig("_dock_layout.png",dpi=85,bbox_inches='tight')
print(f"half-angle {half_angle:.1f} deg | gap between wheels {gap_in:.0f} | mouth {MOUTH_W} (margin {(gap_in-MOUTH_W)/2:.0f}mm/side)")
print(f"battery rear face z {BATT_Z0:.0f}..{BATT_Z1:.0f}, pad center z {PAD_Z}")
