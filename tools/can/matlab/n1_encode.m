function message = n1_encode(db, name, values, profile)
% Clamp and reproduce DM-jump float32 operation order before VNT encoding.
% FROM FILE: DM-jump dm4310_drv.c float_to_uint; DBC rounding alone differs.
arguments
    db
    name
    values (1,5) double {mustBeFinite}
    profile (1,1) string
end
switch profile
    case "hip", p=12.5; v=30; t=10;
    case "wheel", p=12.5; v=45; t=10;
    case "firmware_wheel", p=12; v=45; t=18;
    otherwise, error('N1:Profile','Unknown codec profile');
end
lo=single([-p -v 0 0 -t]); hi=single([p v 500 5 t]);
bits=[16 12 12 12 12]; fields=["Position" "Velocity" "Kp" "Kd" "Torque"];
x=min(hi,max(lo,single(values)));
q=fix(((x-lo).*single(2.^bits-1))./(hi-lo));
message=canMessage(db,char(name));
% Use DBC factors to encode exact integer codes with canMessage.Signals.
info=signalInfo(db,char(name));
for k=1:5
    s=info(strcmp({info.Name},fields(k)));
    message.Signals.(fields(k))=double(q(k))*s.Factor+s.Offset;
end
if all(message.Data(1:7)==255) && ismember(message.Data(8),251:254)
    error('N1:Reserved','Normal values collide with lifecycle override');
end
end
