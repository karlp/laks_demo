define flash
file demo.elf
load
end

define restart
run
end

define attach_swd
mon swdp_scan
attach 1
end

define attach_jtag
mon jtag_scan
attach 1
end

file demo.elf

set mem inaccessible-by-default off
