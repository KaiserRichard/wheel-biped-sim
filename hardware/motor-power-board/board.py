import os,sys,json,xml.etree.ElementTree as ET
from pathlib import Path
import pcbnew as pcb

root=Path(__file__).resolve().parent
work=root/'.work'
parts=json.loads((work/'parts.json').read_text(encoding='utf8'))
mm=pcb.FromMM
ox,oy=48,43
width,height=96,99
def xy(x,y): return pcb.VECTOR2I(mm(ox+x),mm(oy+y))
def xyz(d): return pcb.VECTOR2I(mm(d[0]),mm(d[1]))
def outline(board,x,y,w,h,layer,line_width=0.1):
    a=pcb.PCB_SHAPE(board);a.SetShape(pcb.SHAPE_T_RECT);a.SetStart(xy(x,y));a.SetEnd(xy(x+w,y+h))
    a.SetLayer(layer);a.SetWidth(mm(line_width));board.Add(a)
def silk(board,label,x,y,h=1):
    s=pcb.PCB_TEXT(board);s.SetText(label);s.SetTextSize(xyz((h,h)));s.SetTextThickness(mm(h*.14))
    s.SetLayer(pcb.F_SilkS);s.SetPosition(xy(x,y));s.SetHorizJustify(pcb.GR_TEXT_H_ALIGN_LEFT);board.Add(s)
def shunt_lib():
    folder=root/'MotorPower.pretty';folder.mkdir(exist_ok=True)
    raw='''(footprint "WSBS8518_Kelvin" (version 20241229) (generator pcbnew) (layer "F.Cu") (attr through_hole)
 (descr "Vishay WSBS8518...35 500 uohm, separate lugs and sense pins")
 (property "Reference" "REF**" (at 0 -9.65) (layer "F.SilkS") (effects (font (size 1 1) (thickness .15))))
 (property "Value" "WSBS8518_Kelvin" (at 0 10) (layer "F.Fab") (effects (font (size 1 1) (thickness .15))))
 (fp_rect (start -42.5 -9) (end 42.5 9) (stroke (width .15) (type solid)) (fill no) (layer "F.Fab"))
 (fp_rect (start -43 -9.5) (end 43 9.5) (stroke (width .05) (type solid)) (fill no) (layer "F.CrtYd"))
 (pad "1" thru_hole circle (at -30 0) (size 10 10) (drill 7.4) (layers "*.Cu" "*.Mask"))
 (pad "2" thru_hole circle (at 30 0) (size 10 10) (drill 7.4) (layers "*.Cu" "*.Mask"))
 (pad "3" thru_hole circle (at -8.825 0) (size 3.8 3.8) (drill 2.25) (layers "*.Cu" "*.Mask"))
 (pad "4" thru_hole circle (at 8.825 0) (size 3.8 3.8) (drill 2.25) (layers "*.Cu" "*.Mask"))
)'''
    (folder/'WSBS8518_Kelvin.kicad_mod').write_text(raw,encoding='utf8')
    (root/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "MotorPower") (type "KiCad") (uri "${KIPRJMOD}/MotorPower.pretty") (options "") (descr "HW2 shunt mounting pattern")))\n',encoding='utf8')
    return folder
def track(board,net,points,layer,w):
    for a,b in zip(points,points[1:]):
        t=pcb.PCB_TRACK(board);t.SetStart(xy(*a));t.SetEnd(xy(*b));t.SetWidth(mm(w));t.SetLayer(layer);t.SetNet(net);board.Add(t)
def fprefer(ref):
    placement={'J8':(64,75),'J9':(77,66),'Q13':(69,64),'RS1':(48,82.05),
       'R82':(86,52),'R83':(89,56),'R84':(86,61), 'R64':(77,47.2), 'R63':(80,51.1),
       'Q1':(9.46,70.6),'J6':(11.4,63.8),'J7':(21.1,69.3),'R79':(23,72), 'R75':(26.7,74.2), 'R76':(31,72),
       'D11':(22,63.9),'D9':(7,66.5),'J5':(8,55),'R80':(43,54),'U16':(48,68),'R81':(40,68),
       'Q6':(60,14),'Q7':(70,14),'Q8':(80,14),'R26':(66,18),'R27':(73,19),'R32':(60,17.5),'R28':(83,13.8),
       'U2':(62,22), 'J3':(76,7),'R31':(86,20),'J4':(7,24),'R24':(28,27),
       'U1':(50,17),'C2':(55,23),'C1':(47,23),'C3':(53,26),
       'C13':(74,56),'J1':(11,7),'J2':(46,7),'J3':(77,7)}
    return placement.get(ref)
def separated(a,b,gap=.3):
    return a[2]+gap<=b[0] or b[2]+gap<=a[0] or a[3]+gap<=b[1] or b[3]+gap<=a[1]
def footprint_extents(f):
    box=f.GetCourtyard(pcb.F_Cu).BBox()
    if box.GetWidth()==0: box=f.GetBoundingBox(False,False)
    p=f.GetPosition()
    return tuple(pcb.ToMM(z) for z in [box.GetLeft()-p.x,box.GetTop()-p.y,box.GetRight()-p.x,box.GetBottom()-p.y])
def pack(board,entries):
    used=[(.01,.01,7.01,7.01),(89,.01,95.99,7.01),(.01,92,7.01,98.99),(89,92,95.99,98.99),
          (10.6,63,13,71.6),(13.3,68.2,22.3,71.8)]
    locked=['RS1','Q1','J6','J7']
    ext={ref:footprint_extents(f) for ref,f,want in entries}
    def area(ref):
        l,t,r,b=ext[ref]; return (r-l)*(b-t)
    order=sorted(entries,key=lambda n:(n[0] not in locked,-area(n[0])))
    grid=(0,)+tuple(z*.55*i for i in range(1,48) for z in (-1,1))
    offsets=sorted(((dx,dy) for dx in grid for dy in grid),key=lambda a:a[0]**2+a[1]**2)
    for ref,f,(x0,y0) in order:
        l,t,r,b=ext[ref]
        for dx,dy in offsets:
            x,y=x0+dx,y0+dy
            box=(x+l,y+t,x+r,y+b)
            if min(box[0],box[1])<.9 or box[2]>width-.9 or box[3]>height-.9: continue
            obstacles=used[:4]+used[6:] if ref in locked else used
            if any(not separated(o,box) for o in obstacles): continue
            f.SetPosition(xy(x,y));used.append(box);board.Add(f)
            break
        else: raise RuntimeError(('No placement',ref))
def source():
    shunt=shunt_lib()
    b=pcb.BOARD();b.SetCopperLayerCount(4)
    cfg=b.GetDesignSettings()
    cfg.m_MinClearance=mm(.20);cfg.m_TrackMinWidth=mm(.2);cfg.m_MinThroughDrill=mm(.25)
    cfg.m_ViasMinSize=mm(.55);cfg.m_CopperEdgeClearance=mm(.35)
    et=ET.parse(work/'motor_power.xml').getroot()
    nets={}
    pin_net={}
    bmap={p['ref']:p for p in parts if p['fp']}
    for nx in et.findall('./nets/net'):
        name=nx.attrib['name']
        present=[n for n in nx.findall('./node') if n.attrib['ref'] in bmap]
        if not present: continue
        net=pcb.NETINFO_ITEM(b,name);b.Add(net);nets[name]=net
        for n in present: pin_net[(n.attrib['ref'],n.attrib['pin'])]=net
    comps={n.attrib['ref']:n for n in et.findall('./components/comp')}
    entries=[]
    for ref,p in bmap.items():
        lib,name=p['fp'].split(':')
        libdir=str(shunt if lib=='MotorPower' else Path(os.environ['KICAD9_FOOTPRINT_DIR'])/(lib+'.pretty'))
        f=pcb.FootprintLoad(libdir,name)
        if f is None: raise ValueError(p['fp'])
        f.SetFPID(pcb.LIB_ID(lib,name));f.SetReference(ref);f.SetValue(p['value'])
        f.SetAttributes(f.GetAttributes() & ~pcb.FP_EXCLUDE_FROM_BOM)
        f.SetPosition(xy(*(fprefer(ref) or p['pos'])));f.SetOrientationDegrees(0)
        ids=comps[ref]; path=pcb.KIID_PATH()
        for id in ids.find('sheetpath').attrib['tstamps'].strip('/').split('/'):
            if id: path.push_back(pcb.KIID(id))
        path.push_back(pcb.KIID(ids.findtext('tstamps').split()[0]))
        f.SetPath(path)
        if p['dnp']: f.SetAttributes(f.GetAttributes() | pcb.FP_DNP)
        f.Value().SetVisible(False)
        f.Reference().SetTextSize(xyz((.8,.8)));f.Reference().SetTextThickness(mm(.12))
        pads={pad.GetNumber() for pad in f.Pads()}
        if not set(p['nets']).issubset(pads): raise RuntimeError((ref,p['fp'],pads,list(p['nets'])))
        for pad in f.Pads():
            net=pin_net.get((ref,pad.GetNumber()))
            if net: pad.SetNet(net)
        f.Reference().SetVisible(False)
        entries.append((ref,f,fprefer(ref) or p['pos']))
    pack(b,entries)
    outline(b,0,0,width,height,pcb.Edge_Cuts,.05)
    for i,(x,y) in enumerate([(3.5,3.5),(92.5,3.5),(3.5,95.5),(92.5,95.5)],1):
        lib='MountingHole';name='MountingHole_3.2mm_M3'
        foot=pcb.FootprintLoad(str(Path(os.environ['KICAD9_FOOTPRINT_DIR'])/(lib+'.pretty')),name)
        foot.SetFPID(pcb.LIB_ID(lib,name));foot.SetReference(f'H{i}');foot.SetValue('M3 mounting')
        foot.SetAttributes(foot.GetAttributes() | pcb.FP_BOARD_ONLY | pcb.FP_EXCLUDE_FROM_BOM)
        foot.SetPosition(xy(x,y));foot.Reference().SetVisible(False);foot.Value().SetVisible(False);b.Add(foot)
    silk(b,'WBR  /  MOTOR POWER',37,88.1,1.55)
    silk(b,'6S   HARDWIRED STOP   REV 0.1',35,93,1.05)
    silk(b,'SHUNT OPTION / DNP',35,77.5,.95)
    silk(b,'STOP I/O',15,14);silk(b,'COIL I/O',53,3.2)
    silk(b,'SENSE',6,37.5);silk(b,'CLAMP',5,47.5);silk(b,'CTRL',65,74);silk(b,'AUX FB',80,15.0)
    b.GetTitleBlock().SetTitle('WBR motor power / 6S')
    pcb.SaveBoard(str(work/'placed.kicad_pcb'),b)

def add_power(board):
    nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()}
    pads={(f.GetReference(),p.GetNumber()):p.GetPosition() for f in board.GetFootprints() for p in f.Pads()}
    def rel(ref,p):
        z=pads[ref,str(p)]
        return round(pcb.ToMM(z.x)-ox,4),round(pcb.ToMM(z.y)-oy,4)
    j,q6,q7,q8=[rel(f,n) for f,n in [('J2',2),('Q6',3),('Q7',3),('Q8',3)]]
    # A solid 2 mm ground backbone carries the three 24 V coil return pulses.
    for path in [[j,(j[0]+3,10),(q6[0]-1,10),(q6[0],11),q6],
                 [q6,(q6[0]+3,13.2),(q7[0],13.2),q7],
                 [q7,(q8[0],q7[1]-(q8[0]-q7[0])),q8]]:
        track(board,nets['GND'],path,pcb.In1_Cu,2.0)
    for layer in (pcb.F_Cu,pcb.B_Cu):
        track(board,nets['DUMP_SW'],[(12,70.6),(12,64.4),(11.4,63.8)],layer,1.6)
        track(board,nets['CLAMP_GND'],[(14.54,70.6),(19.8,70.6),(21.1,69.3)],layer,2.4)
def route_board(stage):
    b=pcb.LoadBoard(str(work/'placed.kicad_pcb'))
    if stage=='export':
        add_power(b)
        pcb.SaveBoard(str(work/'fixed.kicad_pcb'),b)
        dsn=work/'motor_power.dsn'
        assert pcb.ExportSpecctraDSN(b,str(dsn))
        txt=dsn.read_text(encoding='utf8').replace('(type route)','(type fix)')
        txt=txt.replace('(clearance 50 (type smd_smd))','(clearance 200 (type smd_smd))')
        import re
        coil_names='COIL1_RET COIL2_RET COILP_RET'
        txt=re.sub(r'\(class (?:kicad_default|Default)\s+([^()]+)',lambda m:re.sub(r'\bCOIL[12P]_RET\b','',m.group()),txt,count=1)
        class_start=txt.index('    (class ')
        txt=txt[:class_start]+f'    (class Coil {coil_names}\n      (rule (width 900) (clearance 250))\n    )\n'+txt[class_start:]
        dsn.write_text(txt,encoding='utf8')
    else:
        assert pcb.ImportSpecctraSES(b,str(work/'motor_power.ses'))
        add_power(b)
        pcb.SaveBoard(str(root/'motor_power.kicad_pcb'),b)
if __name__=='__main__':
    if len(sys.argv)>1: route_board(sys.argv[1])
    else: source()



