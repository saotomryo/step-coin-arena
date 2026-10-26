"""Build a reproducible visual-only derivative of the competition MJCF.

All added geoms have zero mass and disabled contact; competition assets are
never overwritten. Components are illustrative packaging, not manufacturing CAD.
"""
import json,math,hashlib,sys
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'xrobocon/assets/robot_passive_front_bogie_r18_steering.xml'
TARGET=ROOT/'xrobocon/assets/visual/robot_front_bogie_r18_detailed.xml'
COLORS={'metal':'.48 .55 .62 1','dark':'.055 .07 .09 1','blue':'.04 .34 .63 1','orange':'.95 .44 .07 1','silver':'.72 .77 .8 1','rubber':'.12 .135 .15 1','pcb':'.02 .27 .16 1','red':'.7 .045 .035 1','wire':'.05 .06 .07 1'}

def build():
    tree=ET.parse(SOURCE);root=tree.getroot();root.set('model','front_bogie_r18_visual_concept')
    materials=ET.SubElement(root,'asset')
    for name,rgba in COLORS.items():ET.SubElement(materials,'material',name='v_'+name,rgba=rgba,specular='.4' if name in ('metal','silver') else '.15',shininess='.35')
    counter=0
    def geom(body,kind,size,pos=(0,0,0),color='metal',**kwargs):
        nonlocal counter
        counter+=1
        attrs=dict(name=f'visual_{counter:04d}',type=kind,size=' '.join(map(str,size)),pos=' '.join(map(str,pos)),material='v_'+color,mass='0',contype='0',conaffinity='0',group='2')
        attrs.update({key:' '.join(map(str,value)) if isinstance(value,(tuple,list)) else str(value) for key,value in kwargs.items()})
        return ET.SubElement(body,'geom',attrs)
    def beam(body,a,b,r=.007,color='metal'):return geom(body,'capsule',[r],color=color,fromto=list(a)+list(b))
    def bolt(body,pos,axis=(0,0,1),r=.006):geom(body,'cylinder',[r,.003],pos,'silver',zaxis=axis)
    base=root.find(".//body[@name='base_link']")
    # Two extrusion rails, cross-members, lower tray and corner fasteners.
    for y in (-.102,.102):geom(base,'box',[.19,.012,.012],[-.01,y,-.028],'metal')
    for x in (-.17,-.02,.15):geom(base,'box',[.012,.11,.012],[x,0,-.028],'metal')
    geom(base,'box',[.19,.11,.004],[-.01,0,-.045],'dark')
    for x in (-.17,.15):
        for y in (-.102,.102):bolt(base,[x,y,-.012])
    # Removable blue upper panels flank an exposed electronics bay.
    for y in (-.09,.09):
        geom(base,'box',[.16,.018,.006],[-.01,y,.025],'blue')
        for x in (-.15,.12):bolt(base,[x,y,.034],r=.004)
    geom(base,'box',[.055,.062,.025],[-.115,0,-.002],'dark') # battery case
    for x in (-.14,-.09):geom(base,'box',[.006,.065,.004],[x,0,.027],'orange') # straps
    geom(base,'box',[.042,.057,.004],[.012,0,.004],'pcb')
    for x in (-.02,.042):
        for y in (-.045,.045):geom(base,'cylinder',[.003,.012],[x,y,-.006],'silver')
    for i in range(4):geom(base,'box',[.009,.018,.006],[.002+i*.013,-.015,.013],'dark')
    for i in range(8):geom(base,'box',[.002,.004,.003],[.006+i*.005,.04,.011],'silver')
    geom(base,'box',[.027,.045,.018],[.108,0,.001],'dark') # power distribution
    for y in (-.027,0,.027):geom(base,'box',[.013,.009,.004],[.108,y,.024],'orange')
    # Power and harness runs, following the frame rather than floating wires.
    for sign in (-1,1):
        points=[(-.1,sign*.06,.015),(-.07,sign*.075,.02),(.07,sign*.075,.025),(.12,sign*.045,.025)]
        for a,b in zip(points,points[1:]):beam(base,a,b,.0025,'red' if sign>0 else 'wire')
        for x in (-.06,.02,.08):geom(base,'box',[.004,.006,.004],[x,sign*.075,.025],'dark')
    # Front bumper, status lamps, sensor tower and rear antenna.
    beam(base,(.185,-.11,-.012),(.185,.11,-.012),.01,'dark')
    for y in (-.08,.08):geom(base,'sphere',[.008],[.191,y,.008],'orange')
    geom(base,'cylinder',[.008,.035],[.04,0,.053],'metal')
    geom(base,'cylinder',[.034,.014],[.04,0,.097],'dark')
    geom(base,'cylinder',[.035,.003],[.04,0,.111],'blue')
    geom(base,'cylinder',[.036,.002],[.04,0,.087],'silver')
    beam(base,(-.18,-.07,.025),(-.18,-.07,.17),.002,'wire')
    for i in range(3):geom(base,'box',[.01,.003,.001],[-.03+i*.025,.111,.012],'silver')
    # Exposed rocker/bogie arms: layered side plates and visible pivot bearings.
    for side,sign in (('left',1),('right',-1)):
        rocker=root.find(f".//body[@name='{side}_rocker']")
        bogie=root.find(f".//body[@name='{side}_bogie']")
        for offset in (-.008,.008):
            beam(rocker,(-.26,offset,-.1),(.13,offset,-.04),.009,'metal')
            beam(bogie,(-.13,offset,-.06),(.13,offset,-.06),.009,'metal')
        for body in (rocker,bogie):
            geom(body,'cylinder',[.025,.011],[0,sign*.01,0],'dark',zaxis=(0,1,0))
            geom(body,'cylinder',[.015,.014],[0,sign*.015,0],'silver',zaxis=(0,1,0))
            bolt(body,[0,sign*.033,0],(0,1,0),.007)
        for position in ('front','middle','rear'):
            steering=root.find(f".//body[@name='{side}_wheel_{position}_steering']")
            wheel=root.find(f".//body[@name='{side}_wheel_{position}']")
            # Steering gearbox and mounting fork; dimensions are concept packaging.
            geom(steering,'box',[.026,.021,.015],[0,-sign*.034,.02],'dark')
            geom(steering,'cylinder',[.017,.023],[0,-sign*.03,-.004],'metal',zaxis=(0,1,0))
            for x in (-.022,.022):beam(steering,(x,-sign*.026,.015),(x,0,0),.005,'metal')
            # Axial gearmotor body sits inboard of the wheel.
            geom(steering,'cylinder',[.025,.027],[0,-sign*.058,0],'dark',zaxis=(0,1,0))
            geom(steering,'cylinder',[.027,.009],[0,-sign*.025,0],'silver',zaxis=(0,1,0))
            # Machined rim, spokes, axle nut, and tread blocks on the tire.
            wheel.find('geom').set('rgba','.075 .085 .095 1')
            for face in (-1,1):
                z=face*.027
                geom(wheel,'cylinder',[.108,.003],[0,0,z],'dark')
                geom(wheel,'cylinder',[.081,.004],[0,0,z+face*.004],'metal')
                geom(wheel,'cylinder',[.031,.007],[0,0,z+face*.008],'orange')
                for i in range(8):
                    angle=i*math.tau/8
                    c,s=math.cos(angle),math.sin(angle)
                    geom(wheel,'box',[.031,.008,.004],[.061*c,.061*s,z+face*.008],'silver',euler=(0,0,math.degrees(angle)))
                    bolt(wheel,[.045*c,.045*s,z+face*.015],r=.004)
                bolt(wheel,[0,0,z+face*.02],r=.012)
            for i in range(40):
                angle=i*math.tau/40;c,s=math.cos(angle),math.sin(angle)
                geom(wheel,'box',[.006,.011,.028],[.179*c,.179*s,0],'rubber',euler=(0,0,math.degrees(angle)))
    TARGET.parent.mkdir(exist_ok=True);ET.indent(tree,space='  ');tree.write(TARGET,encoding='utf-8',xml_declaration=True)
    manifest=dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),visual=str(TARGET.relative_to(ROOT)),visual_sha256=hashlib.sha256(TARGET.read_bytes()).hexdigest(),added_geometries=counter,kind='visual_concept',physical_asset_modified=False)
    TARGET.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest))
if __name__=='__main__':build()
