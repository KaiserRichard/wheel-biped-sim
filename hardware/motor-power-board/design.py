import re,json,uuid,os
from pathlib import Path
from collections import defaultdict
from functools import lru_cache

ROOT=Path(__file__).resolve().parent
LIB=Path(os.environ['KICAD9_SYMBOL_DIR'])
PROJ='motor_power'
PAGES={
 'power': ('Motor feeder and external disconnects', ['Bolted off-board 6S feeder / contactors / six fused branches', 'Low-current sense taps and independent AUX/CTRL logic feed']),
 'stop': ('Dual-channel stop circuit and coil switches', ['External safety latch in monitored falling-edge restart mode', 'Separate safety contacts in the two main coil supply paths']),
 'precharge': ('Differential precharge and hardware sequencing', ['K2 first, KP after proof, K1 after a bidirectional differential check', 'Non-retriggerable hardware window and delayed feedback fault']),
 'clamp': ('Self-powered link clamp', ['Kelvin taps at the link busbars and separate source-reference return', 'Independent analog regulation, comparator and pulsed dump switch']),
 'interface': ('CTRL interface and optional current monitor', ['CTRL logic reference joins board ground at the link star return', 'Open-drain status, default-off permit, optional high-side monitor']),
}
parts=[]
notes=defaultdict(list)
current='power'
def page(key):
    global current
    current=key
def note(line):
    notes[current].append(line)
def add(ref,value,lib,fp,nets,pos=None,mpn='',dnp=False,rot=0,desc=''):
    parts.append(dict(ref=ref,value=value,lib=lib,fp=fp,nets={str(k):v for k,v in nets.items()},pos=pos,mpn=mpn or value,dnp=dnp,rot=rot,desc=desc,page=current))
def seq(values):
    return dict(enumerate(values,1))
def custom(name,value,ref,fp,pins,nets,pos=None,**kw):
    key='MotorPower:'+name
    old=custom_defs.setdefault(key,(ref,fp,pins,kw.pop('url','')))
    assert old[2]==pins,(key,old,pins)
    add(ref,value,key,fp,nets,pos,**kw)
def external(ref,val,pins,nets,**kw):
    custom(ref+'_Harness',val,ref,'',pins,nets,**kw)

def q(s): return json.dumps(str(s),ensure_ascii=False)
def uid(s): return str(uuid.uuid5(uuid.NAMESPACE_URL,'hw2-design/'+s))
def parse(raw):
    toks=re.finditer(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+',raw)
    stack=[]
    for z in toks:
        t=z.group()
        if t=='(':
            stack.append([])
        elif t==')':
            value=stack.pop()
            if not stack: return value
            stack[-1].append(value)
        else: stack[-1].append(t)
def sub(node,key):
    return [x for x in node if isinstance(x,list) and x[0]==key]
def val(t): return json.loads(t) if t.startswith('"') else t
def wirefmt(t): return '('+' '.join(wirefmt(x) if isinstance(x,list) else x for x in t)+')'
@lru_cache(None)
def library(name):
    return {val(x[1]):x for x in sub(parse((LIB/(name+'.kicad_sym')).read_text(encoding='utf8')),'symbol')}
@lru_cache(None)
def stock(lib_id):
    family,name=lib_id.split(':')
    entry=library(family)[name]
    parent=sub(entry,'extends')
    if parent:
        base=stock(family+':'+val(parent[0][1]))
        base_name=val(base[1]).split(':')[1]
        entry=['symbol',q(name)]+[x for x in base[2:] if x[0] not in ('property','symbol')]+sub(entry,'property')
        entry += [[x[0],q(val(x[1]).replace(base_name+'_',name+'_',1)),*x[2:]] for x in sub(base,'symbol')]
    return [entry[0],q(lib_id),*entry[2:]]
custom_defs={}
def custom_symbol(lib_id,full=True):
    legend,fp,pins,url=custom_defs[lib_id]
    name=lib_id.split(':')[1]
    left=[(n,p,t) for n,p,t,side in pins if side=='L']
    right=[(n,p,t) for n,p,t,side in pins if side=='R']
    limit=max(len(left),len(right))
    top=2.54*(limit//2+1)
    ref=re.match(r'[A-Z]+',legend)[0]
    y=top+2.54
    body=f'(rectangle (start -15.24 {top}) (end 15.24 {-top}) (stroke (width 0.254) (type default)) (fill (type background)))'
    funcs=''
    for g,sign in [(left,-1),(right,1)]:
        for j,(n,p,t) in enumerate(g):
            py=2.54*(len(g)//2-j)
            funcs+=f'(pin {t} line (at {sign*20.32} {py} {0 if sign==-1 else 180}) (length 5.08) (name {q(p)} (effects (font (size 0.90 0.90)))) (number {q(n)} (effects (font (size 0.85 0.85)))))'
    raw=f'''(symbol {q(lib_id if full else name)} (pin_names (offset 0.7)) (exclude_from_sim no) (in_bom yes) (on_board yes)
      (property "Reference" {q(ref)} (at 0 {y} 0) (effects (font (size 1.0 1.0))))
      (property "Value" {q(name)} (at 0 {y+2.54} 0) (effects (font (size 1.0 1.0))))
      (property "Footprint" {q(fp)} (at 0 0 0) (effects (font (size 1 1)) (hide yes)))
      (property "Datasheet" {q(url)} (at 0 0 0) (effects (font (size 1 1)) (hide yes)))
      (symbol {q(name+'_0_1')} {body}) (symbol {q(name+'_1_1')} {funcs}))'''
    return parse(raw)
def symbol(lib): return custom_symbol(lib) if lib in custom_defs else stock(lib)
def boxpins(names,left=None,types=None):
    if isinstance(names,list):
        names=seq(names)
    left=set(left or ())
    return [(str(k),v,(types or {}).get(k,'passive'),'L' if k in left else 'R') for k,v in names.items()]
def fp_res(s='0805'):
    return 'Resistor_SMD:R_'+s+{'0805':'_2012Metric','1206':'_3216Metric','2512':'_6332Metric'}[s]
def r(ref,value,a,b,pos,mpn='',s='0805',dnp=False,desc=''):
    add(ref,value,'Device:R',fp_res(s),{1:a,2:b},pos,mpn,dnp,90,desc)
def c(ref,value,a,b,pos,mpn='',s='0805',dnp=False,desc=''):
    size={'0805':'2012','1206':'3216','1210':'3225'}[s]
    add(ref,value,'Device:C','Capacitor_SMD:C_'+s+'_'+size+'Metric',{1:a,2:b},pos,mpn,dnp,90,desc)
def d(ref,val,a,k,pos,lib='Device:D',fp='Diode_SMD:D_SOD-123',mpn='',desc=''):
    add(ref,val,lib,fp,{1:k,2:a},pos,mpn,False,90,desc)
def j(ref,val,nets,pos,style='term',rot=0,desc=''):
    n=len(nets)
    if style=='term':
        fp=f'TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-{n}-5.08_1x{n:02}_P5.08mm_Horizontal'
    elif style=='jst':
        fp=f'Connector_JST:JST_XH_B{n}B-XH-A_1x{n:02}_P2.50mm_Vertical'
    elif style=='lug':
        fp=f'Connector_Wire:SolderWire-2.5sqmm_1x01_D2.4mm_OD4.4mm'
    add(ref,val,f'Connector_Generic:Conn_01x{n:02}',fp,seq(nets),pos,rot=rot,desc=desc)
def flag(ref,net):
    add('#FLG'+str(ref),'PWR_FLAG','power:PWR_FLAG','',{'1':net})
def nmos(ref,val,g,d,s,pos,power=False,desc=''):
    fp='Package_TO_SOT_THT:TO-220-3_Vertical' if power else 'Package_TO_SOT_SMD:SOT-23'
    if power:
        custom('IPP023N10N5',val,ref,fp,boxpins(['G','D','S'],[1]),seq([g,d,s]),pos,url='https://www.infineon.com/assets/row/public/documents/24/49/infineon-ipp023n10n5-ds-en.pdf',desc=desc)
    else:
        add(ref,val,'Transistor_FET:2N7002',fp,seq([g,s,d]),pos,desc=desc)
def optic(ref,anode,cathode,collector,emitter,pos):
    add(ref,'LTV-817S-TA1-C','Isolator:LTV-817S','Package_DIP:SMDIP-4_W7.62mm',seq([anode,cathode,emitter,collector]),pos,desc='CTR bin C; 4-pin SMD gullwing')
def relay(ref,meaning,coil,sw,pos):
    # Omron G6K-2F-Y diagram / stock KiCad symbol and footprint
    add(ref,'G6K-2F-Y DC5','Relay:G6K-2','Relay_SMD:Relay_DPDT_Omron_G6K-2F-Y',
        seq(['+5_CTL',None,sw[0],sw[1],sw[3],sw[2],None,coil]),pos,desc=meaning)
def dec(ref,rail,pos,ground='GND'):
    c(ref,'100 nF / 50 V',rail,ground,pos,'GRM21BR71H104KA01L')
def ef(ref,val,a,b,desc=''):
    add(ref,val,'Device:Fuse','',{1:a,2:b},desc=desc)
def es(ref,val,a,b,nc=True,desc=''):
    add(ref,val,'Switch:SW_Push_Open' if nc else 'Switch:SW_Push','',{1:a,2:b},desc=desc)
def ext_r(ref,value,a,b,mpn,desc=''):
    add(ref,value,'Device:R','',{1:a,2:b},mpn=mpn,desc=desc)

exec(compile((ROOT/'circuit.py').read_text(encoding='utf-8-sig'),'circuit.py','exec'))

def drawtext(name,text,x,y,size,**kwargs):
    return f'(text {q(text)} (at {x} {y} 0) (effects (font (size {size} {size})) (justify left)) (uuid "{uid(name)}"))'
def title(text,paper='A3'):
    return f'(paper "{paper}") (title_block (title {q(text)}) (date "2026-11-22") (rev "0.1") (company "WBR laboratory / design only"))'
def coord(v): return f'{v:.4f}'.rstrip('0').rstrip('.')
def xy(x,y): return coord(x)+' '+coord(y)
def pin_list(sym):
    pin_table={}
    for u in sub(sym,'symbol'):
        for p in sub(u,'pin'):
            at=sub(p,'at')[0]
            pin_table[val(sub(p,'number')[0][1])]=(float(at[1]),float(at[2]),int(at[3])%360,p[1])
    return pin_table
def drawpart(part,x,y,pg):
    ref=part['ref']
    sym=symbol(part['lib'])
    pins=pin_list(sym)
    schrot=90 if part['lib'] in ('Device:R','Device:C','Device:Fuse','Device:C_Polarized') else 0
    if schrot:
        pins={p:(-b,a,(angle+schrot)%360,t) for p,(a,b,angle,t) in pins.items()}
    if set(pins)!=set(part['nets']): raise ValueError((ref,part['lib'],pins,part['nets']))
    place_id=uid('part/'+ref)
    sid=' (on_board yes)' if part['fp'] or ref.startswith('#') else ' (on_board no)'
    bom='no' if ref.startswith('#') else 'yes'
    fp=part['fp']
    off='yes' if part['dnp'] else 'no'
    path=f'/{uid("root")}/{uid("sheet/"+pg)}'
    val_index=0
    props=[]
    for tag,value in [('Reference',ref),('Value',part['value']),('Footprint',fp),('MPN',part['mpn']),('Assembly',('DNP' if off=='yes' else 'PCB') if fp else 'HARNESS'),('Description',part['desc'])]:
        vis=tag in ('Reference','Value') and not ref.startswith('#')
        px=x if vis else x+1
        py=y+max((-p[1] for p in pins.values() if p[2]==90),default=0)+5.08+2.54*val_index
        if vis: val_index+=1
        eff='(effects (font (size .94 .94)) (justify left)'+(')' if vis else ' (hide yes))')
        props.append(f'(property {q(tag)} {q(value)} (at {xy(px,py)} 0) {eff})')
    rows=[f'(symbol (lib_id {q(part["lib"])}) (at {xy(x,y)} {schrot}) (unit 1) (in_bom {bom}){sid} (dnp {off}) (uuid {q(place_id)}) {" ".join(props)}']
    for pin in pins:
        rows.append(f'(pin {q(pin)} (uuid {q(uid(ref+"/"+pin))}))')
    rows.append(f'(instances (project "{PROJ}" (path {q(path)} (reference {q(ref)}) (unit 1)))))')
    for pin,(px,py,angle,typ) in pins.items():
        a,b=x+px,y-py
        net=part['nets'][pin]
        if net is None:
            if typ!='no_connect': rows.append(f'(no_connect (at {xy(a,b)}) (uuid {q(uid(ref+"/nc/"+pin))}))')
            continue
        dx,dy={0:(-1,0),90:(0,1),180:(1,0),270:(0,-1)}[angle]
        cx,cy=a+dx*5.08,b+dy*5.08
        end=(angle+180)%360
        rows.append(f'(wire (pts (xy {xy(a,b)}) (xy {xy(cx,cy)})) (stroke (width 0) (type default)) (uuid {q(uid(ref+"/wire/"+pin))}))')
        rows.append(f'(global_label {q(net)} (shape passive) (at {xy(cx,cy)} {end}) (effects (font (size 0.82 0.82)) (justify {"right" if end in (0,90) else "left"})) (uuid {q(uid(ref+"/label/"+pin))}))')
    return '\n'.join(rows)

def export():
    grouped=defaultdict(list)
    for part in parts: grouped[part['page']].append(part)
    root=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid "{uid("root")}") {title("WBR motor power / external safety chain","A4")} (lib_symbols)']
    for i,(pg,(caption,info)) in enumerate(PAGES.items(),2):
        now=grouped[pg]
        cols=5 if pg=='precharge' else 4
        height=40.64 if pg in ('stop','clamp') else 33.02
        rows=(len(now)+cols-1)//cols
        paper='A2' if rows>=7 else 'A3'
        xstart=63.5 if cols==5 else 55.88
        width=106.68 if cols==5 else 101.6
        out=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid "{uid("page/"+pg)}") {title(caption,paper)} (lib_symbols']
        lib_ids=sorted({p['lib'] for p in now})
        for lib in lib_ids: out.append(wirefmt(symbol(lib)))
        out.append(')')
        out.append(drawtext(pg+' heading',f'{i-1:02d}   {caption}',16,14,2.6))
        for ri,line in enumerate(info+notes[pg]):
            out.append(drawtext(pg+' note'+str(ri),line,16,21+ri*4,1.12))
        for k,part in enumerate(now):
            out.append(drawpart(part,xstart+(k%cols)*width,53.34+(k//cols)*height,pg))
        out.append(')')
        (ROOT/(pg+'.kicad_sch')).write_text('\n'.join(out)+'\n',encoding='utf8')
        y=36+24*(i-2)
        sid=uid('sheet/'+pg)
        root.append(f'''(sheet (at 19.05 {y}) (size 117 14) (stroke (width .254) (type default)) (fill (color 0 0 0 0)) (uuid "{sid}")
          (property "Sheetname" "{i-1:02d}  {caption}" (at 19.05 {y-1.8} 0) (effects (font (size 1.35 1.35)) (justify left)))
          (property "Sheetfile" "{pg}.kicad_sch" (at 19.05 {y+16} 0) (effects (font (size .9 .9)) (justify left)))
          (instances (project "{PROJ}" (path "/{uid("root")}" (page "{i}")))))''')
    root.extend([drawtext('root title','WBR / 6S motor power',19.05,19,3),drawtext('root context','Controller PCB, bolted feeder and stop-loop harness',19.05,25,1.45),
       drawtext('root scope','18.0–25.2 V nominal pack / 30 A hot feeder design screen',152,46,1.45),
       drawtext('root safety','Independent hardwired disconnect and downstream dump',152,54,1.45),
       drawtext('root release','Design-only / consult README.md before any build',152,62,1.45),'(sheet_instances (path "/" (page "1")))',')'])
    (ROOT/(PROJ+'.kicad_sch')).write_text('\n'.join(root)+'\n',encoding='utf8')
    (ROOT/'motor_power_symbols.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator kicad_symbol_editor)\n'+'\n'.join(wirefmt(custom_symbol(n,False)) for n in sorted(custom_defs))+'\n)\n', encoding='utf8')
    (ROOT/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "MotorPower") (type "KiCad") (uri "${KIPRJMOD}/motor_power_symbols.kicad_sym") (options "") (descr "Datasheet-mapped HW2 parts and harness devices")))\n',encoding='utf8')
    (ROOT/'.work'/'parts.json').write_text(json.dumps(parts,indent=2),encoding='utf8')
if __name__=='__main__': export()



