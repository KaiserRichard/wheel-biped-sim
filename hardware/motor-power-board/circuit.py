G='GND'
C='CLAMP_GND'
CTL='+5_CTL'

page('power')
note('Fuses, harness gauge and battery capacity are TBD after pack and DC fault-current inspection.')
note('PK+ and motor busbars are external. All six branch returns terminate at the return star.')
external('BT1','6S pack + BMS',boxpins(['PACK+','PACK-'],[2]),seq(['PACK_RAW',G]),desc='18.0–25.2 V; capacity and fault current TBD')
ef('F1','Main fuse TBD','PACK_RAW','FUSED','>=60 V DC holder; first fuse at pack')
ef('F2','Motor feeder fuse TBD','FUSED','MOTOR_FUSED','>=60 V DC; coordinated with branch and BMS')
ef('F3','Logic fuse TBD','FUSED','LOGIC_TAP','Independent AUX supply branch')
ef('F4','Stop supply fuse TBD','FUSED','STOP_TAP','Independently protected supply input')
external('PS1','PYH200-72WS24 carrier',boxpins(['VIN+','VIN-','OUT+','OUT-'],[1,2]),seq(['STOP_TAP',G,'STOP_24',G]),
         url='https://product.tdk.com/system/files/dam/doc/product/power/switching-power/dc-dc-converter/catalog/pyh200_e.pdf', mpn='TDK-Lambda PYH200-72WS24',desc='External 24 V, 8.4 A nominal; apply vendor input/output carrier circuit')
external('X1','AUX auxiliary power',boxpins(['AUX+','AUX-'],[1,2]),seq(['LOGIC_TAP',G]),desc='AUX logic only')
ext_r('W1','Removable motor busbar','MOTOR_FUSED','MOTOR_IN','Bolted link; remove only when shunt option installed','No feeder current through PCB copper')
coil_names={'1':'Main in','2':'Main out','A1':'Coil +','A2':'Coil -','11':'EDM COM','12':'EDM NC','14':'EDM NO','21':'Status COM','22':'Status NC','24':'Status NO'}
for k,a,b,enable,ret,e1,e2,fb in [(2,'MOTOR_IN','PRE_IN','COIL2_24','COIL2_RET','EDM_1','EDM_2','K2_FB'),
      (1,'PRE_IN','LINK_BUS','COIL1_24','COIL1_RET','EDM_2','EDM_3','K1_FB'),
      (3,'PRE_IN','PRE_R','COILP_24','COILP_RET','EDM_3','EDM_4',None)]:
    names=coil_names if fb else {n:coil_names[n] for n in ['1','2','A1','A2','11','12','14']}
    pinmap={'1':a,'2':b,'A1':enable,'A2':ret,'11':e1,'12':e2,'14':None}
    if fb: pinmap.update({'21':CTL,'22':None,'24':fb})
    external('K'+str(k),'C310S/150 '+('precharge' if k==3 else 'main'),
      boxpins(names,['1','A1','11','21']),pinmap,
      url='https://schaltbau.com/en/product/contactors/c310/',mpn=('C310S/150 24I-V2' if fb else 'C310S/150 24I-V1'),
      desc='C310S catalog p. 8 terminal map, external bolted high-current contacts')
ext_r('R1','3.0 Ω / ±5%','PRE_R','LINK_BUS','Vishay LPS600 3R0','Heatsink-mounted, ≥150 J / 0.65 s design screen')
ext_r('R2','330 Ω / 5 W','LINK_BUS',G,'Vishay AC05 330R class','External bleeder near link')
ef('F12','Dump fuse TBD','LINK_BUS','DUMP_FUSED','Coordinate healthy pulse, short interruption and thermal latch')
ext_r('R3','1.5 Ω / ±10%','DUMP_FUSED','DUMP_SW','Vishay LPS600 1R5','External heatsink and NC thermal switch')
ext_r('W2','Link+ Kelvin takeoff','LINK_BUS','LINK_SENSE','Harness sense tap','Fused or protected sense lead near link busbar')
ext_r('W3','Return star takeoff',G,C,'Harness separate return','Return star at drive link negative, paired with link sense')
for i in range(6):
    ef('F'+str(i+5),'Branch fuse TBD','LINK_BUS','DRIVE'+str(i+1)+'_P','≥60 V DC fuse position; allocation TBD')
    external('X'+str(i+2),['Hip left 1','Hip left 2','Hip right 1','Hip right 2','Wheel left','Wheel right'][i],
             boxpins(['DC+','DC-'],[1,2]),seq(['DRIVE'+str(i+1)+'_P',G]),desc='External DAMIAO drive / verify installed revision')

page('stop')
note('S1 opens both NC channels. Radio uses two supervised, energized-healthy dry contacts.')
note('K1/K2/K3 mirror NC chain is in series with the reset/arm button. K4 permit defaults open.')
stopn={'A1':'24 V','A2':'0 V','S11':'Test 1','S12':'Input A','S21':'Test 2','S22':'Input B','S34':'Reset','13':'NO 1 in','14':'NO 1 out','23':'NO 2 in','24':'NO 2 out','33':'NO 3 in','34':'NO 3 out','41':'NC in','42':'NC out','Y32':'Status'}
external('SR1','PNOZ s4 750104',boxpins(stopn,['A1','A2','S11','S12','S21','S22','S34','Y32']),
 {'A1':'STOP_24','A2':G,'S11':'SD_A','S12':'EDM_1','S21':'SD_B','S22':'SR_B','S34':'RESET_IN',
  '13':'STOP_24','14':'COIL2_24','23':'STOP_24','24':'COIL1_24','33':'STOP_24','34':'COILP_24','41':None,'42':None,'Y32':None},
 url='https://www.pilz.com/download/open/PNOZ_s4_Operat_Man_21396-EN-17.pdf',desc='24 V DC, cross-short detection and monitored falling-edge start')
es('SW1','Stop mushroom NC A','SD_A','W_A',desc='Mechanically linked to SW2, positive opening')
es('SW2','Stop mushroom NC B','SD_B','W_B',desc='Mechanically linked to SW1, positive opening')
external('RX1','Supervised radio receiver',boxpins(['A COM','A NO','B COM','B NO'],[1,3]),seq(['W_A','RADIO_A','W_B','RADIO_B']),
         desc='Dual healthy dry contacts; open on stop, brownout or watchdog loss. Receiver model TBD')
es('SW3','Local reset / arm NO','RESET_OUT','RESET_IN',False,'Momentary button wired for monitored falling-edge')
es('TH1','Dump heatsink NC thermal','RADIO_A','THERM_A',True,'Temperature and load TBD; open trips hardware A channel')
j('J1','SAFETY CONTACTS',['THERM_A','EDM_1','RADIO_B','SR_B','EDM_4','RESET_OUT'],(8,7),'term',desc='Low-current dry contacts. Feed EDM_1 to both S12 and K1 NC.')
j('J2','COIL CONTROL',['STOP_24',G,'COILP_24','COIL2_RET','COIL1_RET','COILP_RET'],(44,7),'term',desc='External coil positives routed directly through safety relay outputs')
j('J3','AUX NO FEEDBACK',[CTL,'K1_FB','K2_FB',G],(78,7),'term',desc='Second auxiliaries; use verified dry-circuit contacts')
relay('K4','Dual-pole CTRL permit / timeout inhibiting dry relay','K4_COIL',['THERM_A','EDM_1','RADIO_B','SR_B'],(22,20))
relay('K5','Zero-link permissive in series with independent EDM','K5_COIL',['EDM_4','RESET_OUT',None,None],(39,19))
for i,n,post,p in [(4,'PERMIT_RELAY','K4_COIL',(24,27)),(5,'LINK_LOW','K5_COIL',(40,26))]:
    nmos('Q'+str(i),'2N7002',n,post,G,p)
    r('R'+str(20+i),'47k',n,G,(p[0]+4,p[1]+.8))
    d('D'+str(i),'1N4148W',post,CTL,(p[0]-5,p[1]))
# Low-side coil logic never supplies energy to a de-energized PNOZ output.
for i,s,n,p in [(6,'K2_CMD','COIL2_RET',(62,13)),(7,'RUN','COIL1_RET',(71,13)),(8,'KPRE_CMD','COILP_RET',(80,13))]:
    nmos('Q'+str(i),'IPP023N10N5',s,n,G,p,True,desc='C310S has integrated suppression. External harness to A2, no additional coil suppressor')
    r('R'+str(20+i),'47k',s,G,(p[0]+1,p[1]+5))
# Local 5-V logic rail, independent of downstream clamp
add('U1','LM2936Z-5.0/NOPB','Regulator_Linear:LM2936-5.0_TO92','Package_TO_SOT_THT:TO-92_Inline',
    seq(['STOP_24',G,CTL]),(51,18),desc='40 V operating at input')
c('C1','100 nF / 50 V','STOP_24',G,(49,23),'GRM21BR71H104KA01L')
add('C2','22 µF / 16 V','Device:C_Polarized','Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B',
    {1:CTL,2:G},(54,23),desc='Control reservoir at U1 output, check regulator temperature and transient droop')
dec('C3',CTL,(57,23))
r('R32','5.6k / 0.25 W','COILP_24','SAFE_LED',(58,13),s='1206')
optic('U2','SAFE_LED',G,'SAFE_N',G,(60,21))
r('R29','10k',CTL,'SAFE_N',(62,27))
r('R30','680 Ω','K1_FB',G,(78,20))
r('R31','680 Ω','K2_FB',G,(82,20))
flag(1,G);flag(3,'STOP_24')

page('precharge')
note('The matched summing dividers detect either polarity of pack-to-link difference.')
note('Nominal close window +/-0.85 V. The READY condition includes K2 auxiliary and switched-node proof.')
j('J4','KELVIN VOLTAGE TAPS',['PRE_IN','LINK_SENSE',G],(7,33),'term',desc='Separate sense wires and protected taps; 5.08 mm pitch')
def rs(i,value,a,b,p,prec=True):
    r('R'+str(i),value,a,b,p,desc='0.1% ratio-matching grade' if prec else '')
rs(40,'100k','PRE_IN','MID_S',(14,30));rs(41,'10k','MID_S',G,(18,30))
rs(42,'100k','LINK_SENSE','LINK_S',(14,33));rs(43,'10k','LINK_S',G,(18,33))
rs(44,'100k','PRE_IN','MID_B',(14,36));rs(45,'10.35k','MID_B',G,(18,36));rs(46,'294k','MID_B','REF2V5',(22,36))
rs(47,'100k','LINK_SENSE','LINK_B',(14,39));rs(48,'10.35k','LINK_B',G,(18,39));rs(49,'294k','LINK_B','REF2V5',(22,39))
rs(50,'10.0k','REF2V5','MID_REF',(18,42));rs(51,'10.2k','MID_REF',G,(22,42))
rs(52,'45.3k','REF2V5','LOW_REF',(18,45));rs(53,'10.0k','LOW_REF',G,(22,45))
r('R54','2.49k',CTL,'REF2V5',(14,45))
# Stock LM4040DBZ symbol also matches the tighter grade.
add('D6','LM4040A25IDBZR','Reference_Voltage:LM4040DBZ-2.5','Package_TO_SOT_SMD:SOT-23',
    {1:'REF2V5',2:G,3:None},(14,42),desc='2.500 V A-grade shunt reference')
cmp_lib='https://www.ti.com/lit/ds/symlink/tlv1704.pdf'
cmp_pins=boxpins(['2OUT','1OUT','V+','1IN−','1IN+','2IN−','2IN+','3IN−','3IN+','4IN−','4IN+','V−','4OUT','3OUT'],
                 [3,4,5,6,7,8,9,10,11,12],{1:'open_collector',2:'open_collector',13:'open_collector',14:'open_collector',3:'power_in',12:'power_in',
                    **dict.fromkeys(range(4,12),'input')})
cmpfp='Package_SO:TSSOP-14_4.4x5mm_P0.65mm'
custom('TLV1704PW','TLV1704IPWR','U3',cmpfp,cmp_pins,seq(['DV_REV','DV_FWD',CTL,'MID_S','LINK_B','LINK_S','MID_B','MID_REF','MID_S','LINK_S','LOW_REF',G,'LINK_LOW','MID_HIGH']), (28,37),url=cmp_lib)
for i,n,p in [(55,'DV_FWD',(31,31)),(56,'DV_REV',(35,31)),(57,'MID_HIGH',(31,42)),(58,'LINK_LOW',(35,42))]: r('R'+str(i),'4.7k',CTL,n,p)
dec('C4',CTL,(31,34))
add('U4','LTC6993CS6-2#TRMPBF','Timer:LTC6993xS6-2','Package_TO_SOT_SMD:SOT-23-6',
    seq(['SAFE',G,'T_SET','T_DIV',CTL,'WINDOW']),(42,31),desc='Rising edge, non-retriggerable one-shot')
r('R59','113k / 0.1%','T_SET',G,(37,34))
r('R60','1.00M',CTL,'T_DIV',(41,36));r('R61','681k','T_DIV',G,(45,36));dec('C5',CTL,(46,31))
logic_fp='Package_SO:SOIC-14_3.9x8.7mm_P1.27mm'
quadmap=[(1,2,3),(4,5,6),(9,10,8),(12,13,11)]
invmap=[(1,2),(3,4),(5,6),(9,8),(11,10),(13,12)]
def logic(ref,typ,fun,xy,pk):
    nets={7:G,14:CTL}
    pnames={7:'GND',14:'VCC'}
    pin_types={7:'power_in',14:'power_in'}
    for k,p in enumerate(pk,1):
        states=fun[k-1] if k<=len(fun) else [G]*(len(p)-1)+[None]
        for n,net,role in zip(p,states,('A','B','Y') if len(p)==3 else ('A','Y')):
            nets[n]=net;pnames[n]=str(k)+role;pin_types[n]='output' if role=='Y' else 'input'
    name=typ[:-1]
    custom(name,typ,ref,logic_fp,boxpins(dict(sorted(pnames.items())),[n for n,t in pin_types.items() if t!='output'],pin_types),
        nets,xy,url='https://www.ti.com/lit/ds/symlink/'+name.lower()+'.pdf')
def ands(ref,fun,xy): logic(ref,'SN74HCS08D',fun,xy,quadmap)
def ors(ref,fun,xy): logic(ref,'SN74HCS32D',fun,xy,quadmap)
def inv(ref,fun,xy): logic(ref,'SN74HCT14D',fun,xy,invmap)
ands('U5',[('DV_FWD','DV_REV','DELTA_OK'),('K2_FB','MID_HIGH','K2_PROVED'),('K2_PROVED','DELTA_OK','READY'),('SAFE','WINDOW','PRE_WINDOW')],(40,41))
ands('U6',[('PRE_WINDOW','READY','RUN_CLK'),('PRE_WINDOW','K2_PROVED','KP_READY'),('KP_READY','K1_NOT','KPRE_CMD'),('SAFE','K2_HOLD','K2_CMD')],(51,41))
ands('U7',[('K1_FB','K2_PROVED','MAINS_FB'),('MAINS_FB','RUN','LINK_GOOD'),('SAFE','UNPROVED','FAULT_RAW'),('T11_PERMIT','FAULT_N','PERMIT_RELAY')],(63,37))
ors('U8',[('RUN','WINDOW','K2_HOLD'),('WINDOW','LINK_GOOD','STAGE_OK'),('FAULT_LOCAL','CLAMP_FAULT','TRIP_REQ')],(63,48))
inv('U9',[('SAFE_N','SAFE'),('K1_FB','K1_NOT'),('STAGE_OK','UNPROVED'),('FAULT_DELAY','FAULT_DELAY_N'),('FAULT_DELAY_N','FAULT_LOCAL')],(52,49))
custom('SN74HC74','SN74HC74D','U10',logic_fp,boxpins(['1CLR#','1D','1CLK','1PRE#','1Q','1Q#','GND','2Q#','2Q','2PRE#','2CLK','2D','2CLR#','VCC'],
       [1,2,3,4,7,10,11,12,13,14],{**dict.fromkeys([1,2,3,4,10,11,12,13],'input'),
          **dict.fromkeys([5,6,8,9],'output'),7:'power_in',14:'power_in'}),
       seq(['SAFE',CTL,'RUN_CLK',CTL,'RUN',None,G,None,None,CTL,G,G,G,CTL]),(74,38),
       url='https://www.ti.com/lit/ds/symlink/sn74hc74.pdf')
r('R62','100k','FAULT_RAW','FAULT_DELAY',(70,45))
c('C6','220 nF','FAULT_DELAY',G,(74,45))
d('D7','1N4148W','FAULT_DELAY','FAULT_RAW',(78,45))
for i,p in enumerate([(42,46),(52,46),(62,42),(63,54),(52,54),(74,33)],7): dec('C'+str(i),CTL,p)
inv('U11',[('PERMIT_IN','PERMIT_N'),('PERMIT_N','T11_PERMIT'),('FAULT_LOCAL','FAULT_N'),('CLAMP_OPT_N','CLAMP_FAULT')],(74,52))
dec('C13',CTL,(79,54))
r('R63','10k','PERMIT_IN',G,(79,51))
r('R64','10k',CTL,'CLAMP_OPT_N',(79,48))
# The OR feeds a common qualification delay for sequencing and clamp diagnostics.
next(p for p in parts if p['ref']=='U8')['nets']['9']='FAULT_RAW'
next(p for p in parts if p['ref']=='R62')['nets']['1']='TRIP_REQ'
next(p for p in parts if p['ref']=='D7')['nets']['1']='TRIP_REQ'
for p in parts:
    if p['ref'] in ['Q6','Q7','Q8']:
        p['lib']='MotorPower:IRL540N';p['value']=p['mpn']='IRL540NPBF'

custom_defs['MotorPower:IRL540N']=('Q','Package_TO_SOT_THT:TO-220-3_Vertical',boxpins(['G','D','S'],[1]),'https://www.infineon.com/part/IRL540N')
page('clamp')
note('Q1 power source return is separate from the link negative Kelvin wire at J5.')
note('The optocouplers report persistent OV and invalid off-state drain voltage; TH1 trips the stop chain.')
j('J5','CLAMP POWER / KELVIN -',['CLAMP_FEED','LINK_KMIN'],(8,56),'term')
j('J6','DUMP SW / R3 low',['DUMP_SW'],(7,71),'lug')
j('J7','DUMP POWER RETURN',[C],(18,71),'lug')
custom('LM317HV_TO220','LM317HVT/NOPB','U12','Package_TO_SOT_THT:TO-220-3_Vertical',
  boxpins(['ADJ','OUT','IN'],[1,3],{1:'input',2:'power_out',3:'power_in'}),
  seq(['CL_ADJ','+10_CL','CLAMP_FEED']),(9,61),url='https://www.ti.com/lit/ds/symlink/lm317hv-mil.pdf')
r('R65','100 Ω','+10_CL','CL_ADJ',(14,60))
r('R66','715 Ω','CL_ADJ',C,(17,60))
c('C14','100 nF / 100 V','CLAMP_FEED',C,(6,63),'GRM21BR72A104KA01L')
c('C15','2.2 µF / 25 V','+10_CL',C,(20,58),s='1206')
d('D8','1N4148W','+10_CL','CLAMP_FEED',(18,55))
d('D9','SMBJ28A',C,'CLAMP_FEED',(8,66),fp='Diode_SMD:D_SMB')
r('R67','6.8k','+10_CL','CL_REF',(22,60))
add('D10','LM4040A25IDBZR','Reference_Voltage:LM4040DBZ-2.5','Package_TO_SOT_SMD:SOT-23',
    {1:'CL_REF',2:C,3:None},(25,60))
rs(68,'1.00k','CL_REF','CL_NEG',(22,55));rs(69,'10.7k','LINK_KMIN','CL_NEG',(26,55))
rs(70,'100k','LINK_SENSE','CL_POS',(29,53));rs(71,'9.49k','CL_POS',C,(33,53));rs(72,'2.05M','CL_ON','CL_POS',(37,53))
rs(73,'100k','LINK_SENSE','OV_S',(29,57));rs(74,'8.91k','OV_S',C,(33,57))
rs(75,'100k','DUMP_SW','DRAIN_S',(27,72));rs(76,'24.9k','DRAIN_S',C,(31,72))
r('R77','2.2k','+10_CL','CL_ON',(35,58))
custom('TLV1704PW','TLV1704IPWR','U13',cmpfp,cmp_pins,
    seq(['OV_OPTO_K','CL_ON','+10_CL','CL_NEG','CL_POS','OV_S','CL_NEG','CL_REF','DRAIN_S',C,'CL_REF',C,None,'CONT_OPTO_K']),
    (36,63),url=cmp_lib)
dec('C16','+10_CL',(36,68),C)
custom('UCC27517DBV','UCC27517DBVR','U14','Package_TO_SOT_SMD:SOT-23-5',
    boxpins(['VDD','GND','IN+','IN−','OUT'],[1,2,3,4],{1:'power_in',2:'power_in',3:'input',4:'input',5:'output'}),
    seq(['+10_CL',C,'CL_ON',C,'DUMP_GATE_RAW']),(22,65),url='https://www.ti.com/lit/ds/symlink/ucc27517.pdf')
dec('C17','+10_CL',(25,65),C)
r('R78','10 Ω','DUMP_GATE_RAW','DUMP_GATE',(20,68))
r('R79','10k','DUMP_GATE',C,(23,70))
nmos('Q1','IPP023N10N5','DUMP_GATE','DUMP_SW',C,(10,71),True,desc='100 V, TO-220; mounted at dump wire pads')
d('D11','SMBJ48A',C,'DUMP_SW',(15,66),fp='Diode_SMD:D_SMB',desc='Validate drain turn-off transient against 100 V rating')
r('R80','2.2k','+10_CL','OV_LED',(42,56))
optic('U15','OV_LED','OV_OPTO_K','CLAMP_OPT_N',G,(48,60))
add('Q2','BSS84','Transistor_FET:BSS84','Package_TO_SOT_SMD:SOT-23',seq(['CL_ON','+10_CL','CONT_EN']),(42,66))
r('R81','2.2k','CONT_EN','CONT_LED',(42,71))
optic('U16','CONT_LED','CONT_OPTO_K','CLAMP_OPT_N',G,(48,70))
flag(4,C);flag(5,'CLAMP_FEED')
# Physically distinct harness connections make the Kelvin taps explicit in PCB nets.
page('power')
ef('F11','Clamp control tap fuse TBD','LINK_BUS','CLAMP_FEED','Rated 60 V DC, independent tap')
ext_r('W4','Link− Kelvin takeoff',G,'LINK_KMIN','Harness sense return','Star termination independent of dump current wire')

page('interface')
note('CTRL uses receiver-side 3.3 V pull-ups on all five open-drain status lines.')
note('W1 remains installed for the default assembly. The optional shunt bolts into the feeder by cable, with separate Kelvin pins.')
j('J8','CTRL STATUS / PERMIT',[G,'PERMIT_IN','K1_FB_N','K2_FB_N','LINK_GOOD_N','CLAMP_FAULT_N','ESTOP_OK_N'],(68,70),'jst')
for i,gg,dd,p in [(9,'K1_FB','K1_FB_N',(56,62)),(10,'K2_FB','K2_FB_N',(59,62)),(11,'LINK_GOOD','LINK_GOOD_N',(62,62)),
            (12,'CLAMP_FAULT','CLAMP_FAULT_N',(65,62)),(13,'SAFE','ESTOP_OK_N',(68,62))]:
    nmos('Q'+str(i),'2N7002',gg,dd,G,p)
j('J9','DNP MONITOR I2C',[G,'T11_3V3','INA_SDA','INA_SCL','INA_ALERT'],(78,60),'jst',desc='Separate optional monitor harness, 3.3 V pull-ups at host')
parts[-1]['dnp']=True
custom('Shunt_Bolted_Kelvin','WSBS8518L5000JT35','RS1','MotorPower:WSBS8518_Kelvin',
       boxpins(['I+ stud','I− stud','S+','S−'],[1,3]),seq(['MOTOR_FUSED','MOTOR_IN','IS_P','IS_N']),(84,71),
       dnp=True,url='https://www.vishay.com/docs/30355/wsbs8518-35.pdf',
       desc='500 µΩ / stud-to-stud harness current; mechanical approval required')
add('U17','INA228AIDGSR','Sensor_Energy:INA228','Package_SO:VSSOP-10_3x3mm_P0.5mm',
    seq([G,G,'INA_ALERT','INA_SDA','INA_SCL','T11_3V3',G,'INA_VBUS','INA_N','INA_P']),(83,54),dnp=True,desc='85 V common mode, address 0x40 with A0/A1 low')
r('R82','10 Ω','IS_P','INA_P',(88,53),dnp=True)
r('R83','10 Ω','IS_N','INA_N',(88,56),dnp=True)
c('C18','100 nF','INA_P','INA_N',(88,59),dnp=True)
r('R84','4.7 Ω','IS_N','INA_VBUS',(87,62),dnp=True)
c('C19','100 nF','T11_3V3',G,(79,53),dnp=True)
flag(6,'T11_3V3')

p=next(p for p in parts if p['ref']=='U1')
p.update(value='LM317HVT/NOPB',mpn='LM317HVT/NOPB',lib='MotorPower:LM317HV_TO220',
    fp='Package_TO_SOT_THT:TO-220-3_Vertical',nets={'1':'CTL_ADJ','2':CTL,'3':'STOP_24'},
    desc='60 V input-output differential; mount small heatsink and verify thermal budget')
page('stop')
r('R85','100 Ω',CTL,'CTL_ADJ',(50,27))
r('R86','316 Ω / 0.1%','CTL_ADJ',G,(54,27))
d('D12','1N4148W',CTL,'STOP_24',(57,26))





