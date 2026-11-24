/* STM32F446RE: 512 KiB flash, 128 KiB SRAM. Reserve 48 KiB heap and 8 KiB stack. */
ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
__stack_reserve = 8K;
__heap_size = 48K;
PHDRS {
  flash PT_LOAD FLAGS(5); /* read + execute */
  ram PT_LOAD FLAGS(6);   /* read + write */
  exidx 0x70000001 FLAGS(4);
}
SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH :flash
  .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH :flash
  .ARM.extab : { *(.ARM.extab*) } > FLASH :flash
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH :flash :exidx
  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH :flash
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH :flash
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH :flash
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) . = ALIGN(8); _edata = .; } > RAM AT > FLASH :ram
  .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(8); _ebss = .; } > RAM :ram
  . = ALIGN(8);
  __heap_start = .;
  __heap_limit = __heap_start + __heap_size;
  __stack_limit = _estack - __stack_reserve;
  _end = __heap_start;
  ASSERT(__heap_limit <= __stack_limit, "RAM overflow: data+bss+48KiB heap+8KiB stack exceeds SRAM budget")
}
