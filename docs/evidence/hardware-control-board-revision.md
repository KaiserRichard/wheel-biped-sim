**Tóm tắt [FROM FILE / COMPUTED].** Công bố AUX rev5 và CTRL rev4 cùng nguồn KiCad, PDF sơ đồ và báo cáo ERC/DRC. Ghi chú shunt và cực LED đã sửa. Cả hai bo chưa được chế tạo hoặc cấp nguồn.

# AUX and CTRL hardware revisions

[FROM FILE] This page is an edited, reduced version of the supplied hardware draft. Public source copies are [AUX rev5](../../hardware/AUX/) and [CTRL rev4](../../hardware/CTRL/). Earlier board revisions remain in the hardware archive.

[FROM FILE / COMPUTED] The shunt is 13 mOhm. Its corrected nominal calculation is `I = V/R = 0.050/0.013 = 3.846 A`; tolerances still apply. The CTRL LED symbol/cache now use pin 1 = K and pin 2 = A. Pad net IDs, copper and placement are preserved from the corrected source; internal net-name labels use the public board names.

[COMPUTED] Public-copy KiCad ERC and DRC report zero violations on both boards, including zero unconnected items and zero schematic-parity issues. Reports: [AUX ERC](../../hardware/AUX/checks/erc.json), [AUX DRC](../../hardware/AUX/checks/drc.json), [CTRL ERC](../../hardware/CTRL/checks/erc.json), [CTRL DRC](../../hardware/CTRL/checks/drc.json). Titles, revision labels and old internal board names were edited for publication; electrical topology is unchanged.

## Figures

[COMPUTED] Public schematics were regenerated from the renamed sources: [AUX schematic, seven pages](../../hardware/AUX/schematic.pdf) and [CTRL schematic, eight pages](../../hardware/CTRL/schematic.pdf). No earlier board render is presented as a render of these revisions.

## What this teaches

[STANDARD PRACTICE] A corrected symbol pin name must agree with the cached schematic and PCB pad metadata. `I_trip = V_threshold / R_shunt` describes a nominal design threshold; a clean electrical-rule report does not measure trip accuracy or establish physical safety.

## What the real robot must confirm

[UNKNOWN] Board assembly, polarity, regulation, shunt trip tolerance, receiver and wired-stop behavior, USB signal quality, watchdog timing and motor-power interfaces need bench verification. The boards remain unbuilt and unpowered.
