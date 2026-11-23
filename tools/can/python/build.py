"""Build text databases, source-derived C oracle and portable test fixtures."""
import csv
import hashlib
import json
from pathlib import Path
import random
import re
import sys
from dm_codec import BITS, FIELDS, limits, quantize, pack_codes, feedback, special

ROOT = Path(__file__).resolve().parents[1]
MANUAL = 'https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf'
SDK = 'https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py'
SPECIAL_MANUAL = 'https://aifitlab-wiki.super.site/damiao-docs/dm-j4310-2ec-v12-motor-instruction-manual'
AXES = ['L_front','L_rear','L_wheel','R_front','R_rear','R_wheel']
FWCMD = [8,6,1,8,6,0]
FWFB = [4,3,0,4,3,0]
BUSES = [1,2,3,1,2,3]
HEADER = 'VERSION "N1 offline codec; ranges are not ratings"\nNS_ :\n CM_\n VAL_\nBS_:\nBU_: Controller Motor\n\n'


def sig(name, start, bits, lo, hi, unit):
    scale = (hi-lo)/((1 << bits)-1)
    return f' SG_ {name} : {start}|{bits}@0+ ({scale:.17g},{lo}) [{lo}|{hi}] "{unit}" Controller\n'


def dbc(messages):
    text, comments = HEADER, []
    for identifier, name, profile, kind in messages:
        text += f'BO_ {identifier} {name}: 8 Motor\n'
        if kind == 'command':
            for field, start, bits, bounds, unit in zip(FIELDS, [7,23,27,47,51], BITS,
                    limits(profile), ['rad','rad/s','N m/rad','N m s/rad','N m']):
                text += sig(field,start,bits,*bounds,unit)
                comments.append(f'CM_ SG_ {identifier} {field} "FROM WEB: {MANUAL} p.10; SDK codec ranges, not ratings.";')
            comments.append(f'CM_ BO_ {identifier} "Lifecycle override: FFFFFFFFFFFFFF + FB clear_error / FC enable / FD disable / FE zero; classify before MIT decode. Companion special DBC. {SPECIAL_MANUAL}";')
        elif kind == 'feedback':
            text += sig('MotorID',3,4,0,15,'') + sig('State',7,4,0,15,'')
            for field,start,bits,bounds,unit in [('Position',15,16,limits(profile)[0],'rad'),
                    ('Velocity',31,12,limits(profile)[1],'rad/s'),('Torque',35,12,limits(profile)[4],'N m'),
                    ('MOS_Temperature',55,8,(0,255),'degC'),('Rotor_Temperature',63,8,(0,255),'degC')]:
                text += sig(field,start,bits,*bounds,unit)
            for field in ['MotorID','State','Position','Velocity','Torque','MOS_Temperature','Rotor_Temperature']:
                comments.append(f'CM_ SG_ {identifier} {field} "FROM WEB: {MANUAL} p.9; MotorID low nibble confirmed by SDK/DM-jump; rotor byte described as internal coil temperature.";')
        else:
            for i in range(7):
                text += sig(f'Prefix{i}',8*i+7,8,0,255,'')
            text += sig('Opcode',63,8,0,255,'')
            comments.append(f'VAL_ {identifier} Opcode 251 "clear_error" 252 "enable" 253 "disable" 254 "zero";')
            comments.append(f'CM_ BO_ {identifier} "Require Prefix0..6 all 255. FROM WEB: DAMIAO V1.2 manual, Parameter Settings: {SPECIAL_MANUAL}";')
        text += '\n'
    return text+'\n'.join(c for c in comments if c.startswith('CM_'))+'\n'+'\n'.join(c for c in comments if c.startswith('VAL_'))+'\n'


def function(source, name):
    m = re.search(r'^(?:int|float|void) '+name+r'\([^;]*?\)\s*\{',source,re.M)
    start, depth, i = m.start(), 1, m.end()
    while depth:
        depth += (source[i]=='{') - (source[i]=='}')
        i += 1
    # Preserve executable source exactly; strip comments to keep the excerpt portable.
    return re.sub(r'/\*.*?\*/','',source[start:i],flags=re.S)+'\n'


def main(firmware):
    for folder in ['dbcs','tests','out','matlab','reference']:
        (ROOT/folder).mkdir(exist_ok=True)
    mapping = []
    msgs, specials = [], []
    for a, axis in enumerate(AXES):
        profile = 'wheel' if a in (2,5) else 'hip'
        mapping.append(dict(axis=axis,bus=BUSES[a],study_command_id=a+1,
            study_feedback_id=a+17,firmware_command_id=FWCMD[a],firmware_feedback_id=FWFB[a],
            firmware_bus=2 if a<3 else 1,profile=profile))
        msgs += [(a+1,axis+'_Command',profile,'command'),(a+17,axis+'_Feedback',profile,'feedback')]
        specials.append((a+1,axis+'_Special',profile,'special'))
    (ROOT/'wbr_dm.dbc').write_text(dbc(msgs),encoding='utf8')
    (ROOT/'dbcs'/'wbr_dm_special.dbc').write_text(dbc(specials),encoding='utf8')
    (ROOT/'dbcs'/'codec_firmware_wheel.dbc').write_text(dbc([
        (3,'Firmware_Wheel_Command','firmware_wheel','command'),
        (19,'Firmware_Wheel_Feedback','firmware_wheel','feedback')]),encoding='utf8')
    # DBC has no bus/direction key: keep firmware directions separate, no fake IDs.
    for side, indices in [('left',range(3)),('right',range(3,6))]:
        for kind in ['command','feedback','special']:
            items=[]
            for a in indices:
                p='wheel' if a in (2,5) else 'hip'
                identifier=FWFB[a] if kind=='feedback' else FWCMD[a]
                items.append((identifier,AXES[a]+'_'+kind,p,kind))
            (ROOT/'dbcs'/f'firmware_{side}_{kind}.dbc').write_text(dbc(items),encoding='utf8')
    (ROOT/'reference'/'motor_map.json').write_text(json.dumps(mapping,indent=2)+'\n')
    src=(firmware/'User/Devices/DM_Motor/dm4310_drv.c').read_text(encoding='utf8')
    hdr=(firmware/'User/Devices/DM_Motor/dm4310_drv.h').read_text(encoding='utf8')
    constants='\n'.join(re.findall(r'^#define\s+(?:P_MIN|P_MAX|V_MIN|V_MAX|KP_MIN|KP_MAX|KD_MIN|KD_MAX|T_MIN|T_MAX|MIT_MODE)\s+[^\s]+',hdr,re.M))
    pre='''/* Port of DM-jump driver by Tran Nguyen Binh, MIT license; see THIRD_PARTY.md.
   The six function bodies below are extracted mechanically, comments removed.
   Wheel constants become variables to test BOTH original and SDK ranges. */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
typedef int hcan_t;
#define FDCAN_DLC_BYTES_8 8
typedef struct { uint16_t id,state; int p_int,v_int,t_int; float pos,vel,tor,Tmos,Tcoil; } motor_fbpara_t;
typedef struct { motor_fbpara_t para; } Joint_Motor_t;
typedef Joint_Motor_t Wheel_Motor_t;
static uint8_t captured[8];
static float P_MIN2=-12, P_MAX2=12, V_MIN2=-45, V_MAX2=45, T_MIN2=-18, T_MAX2=18;
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
void canx_send_data(hcan_t* h, uint16_t id, uint8_t* d, int n) { (void)h; (void)id; for(int i=0;i<n;i++) captured[i]=d[i]; }
'''
    names=['float_to_uint','uint_to_float','mit_ctrl','mit_ctrl2','dm4310_fbdata','dm6215_fbdata']
    body='\n'.join(function(src,n) for n in names)
    main_c='''
int main(void) {
 int idx,profile; float x[5];
 puts("case,hex,position,velocity,torque");
 while(scanf("%d %d %f %f %f %f %f",&idx,&profile,&x[0],&x[1],&x[2],&x[3],&x[4])==7) {
   P_MIN2=profile==1?-12.5f:-12.0f; P_MAX2=-P_MIN2;
   T_MIN2=profile==1?-10.0f:-18.0f; T_MAX2=-T_MIN2;
   float lo[]={profile?P_MIN2:P_MIN,profile?V_MIN2:V_MIN,0,0,profile?T_MIN2:T_MIN};
   float hi[]={-lo[0],-lo[1],500,5,-lo[4]};
   for(int k=0;k<5;k++) { if(x[k]<lo[k]) x[k]=lo[k]; if(x[k]>hi[k]) x[k]=hi[k]; }
   if(profile) mit_ctrl2(0,1,x[0],x[1],x[2],x[3],x[4]);
   else mit_ctrl(0,1,x[0],x[1],x[2],x[3],x[4]);
   printf("%d,",idx); for(int k=0;k<8;k++) printf("%02X",captured[k]);
   uint8_t fb[]={0x11,captured[0],captured[1],captured[2],(uint8_t)((captured[3]&0xF0)|(captured[6]&15)),captured[7],30,31};
   Joint_Motor_t m={0};
   if(profile) dm6215_fbdata(&m,fb,8); else dm4310_fbdata(&m,fb,8);
   printf(",%.9g,%.9g,%.9g\\n",m.para.pos,m.para.vel,m.para.tor);
 }
 return 0;
}
'''
    (ROOT/'tests'/'dm_oracle.c').write_text(pre+constants+'\n'+body+main_c,encoding='utf8')
    license_file=next((p for p in firmware.glob('LICENSE*') if p.is_file()),None)
    lic=license_file.read_text(encoding='utf8') if license_file else 'MIT license; consult upstream LICENSE.'
    lic=re.sub(r'\s*\([^()\s]+@[^()\s]+\)', '', lic)
    (ROOT/'THIRD_PARTY.md').write_text('# Source attribution\n\n[FROM FILE] Mechanically extracted driver functions from DM-jump, User/Devices/DM_Motor/dm4310_drv.c.\n\n[FROM WEB] Upstream: https://github.com/nguyenbinh-shark/DM-jump .\n\n'+lic,encoding='utf8')
    hashes={str(p.relative_to(firmware)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [firmware/'User/Devices/DM_Motor/dm4310_drv.c',firmware/'User/Devices/DM_Motor/dm4310_drv.h',
                      firmware/'User/APP/chassisL_task.c',firmware/'User/APP/chassisR_task.c',firmware/'User/Bsp/can_bsp.c']}
    (ROOT/'reference'/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    # Freeze small S12 inputs so the portable decoder tests need no sibling tickets.
    s12=ROOT.parent/'S12_comms_in_loop'
    (ROOT/'reference/s12_can_rta.csv').write_bytes((s12/'out/can_rta.csv').read_bytes())
    text=(s12/'can_timing.py').read_text(encoding='utf8')
    snippet=text[text.index('def bits('):text.index('\ndef payloads(')]
    (ROOT/'reference/s12_bits.py').write_text('# FROM FILE: S12_comms_in_loop/can_timing.py, bits; frozen oracle excerpt.\n'+snippet)
    rng=random.Random(20261123)
    rows=[]
    for profile in ['hip','wheel','firmware_wheel']:
        bounds=limits(profile)
        values=[[a for a,b in bounds],[b for a,b in bounds],[0]*5,
                [a-(b-a) for a,b in bounds],[b+(b-a) for a,b in bounds]]
        for j,((lo,hi),nb) in enumerate(zip(bounds,BITS)):
            for k in [0,1,2,2047,2048,(1<<nb)-2,(1<<nb)-1]:
                if k >= 1<<nb: continue
                for eps in [-0.01,0,0.01]:
                    v=[0]*5; v[j]=lo+(hi-lo)*(k+eps)/((1<<nb)-1); values.append(v)
        values += [[rng.uniform(lo-(hi-lo)*.1,hi+(hi-lo)*.1) for lo,hi in bounds] for _ in range(160)]
        for v in values:
            q=quantize(v,profile)
            rows.append(dict(case=len(rows),profile=profile,**dict(zip(FIELDS,v)),
                             **{f'q{k}':c for k,c in enumerate(q)},hex=pack_codes(q).hex().upper()))
    with (ROOT/'tests'/'vectors.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
    with (ROOT/'tests'/'oracle_input.txt').open('w') as f:
        for r in rows:
            p=['hip','wheel','firmware_wheel'].index(r['profile'])
            f.write(f"{r['case']} {p} "+' '.join(format(r[k],'.17g') for k in FIELDS)+'\n')
    print(f'Built 9 databases and {len(rows)} vectors; extracted six C functions.')


if __name__=='__main__':
    main(Path(sys.argv[1]))
