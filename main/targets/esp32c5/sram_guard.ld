/* The modem owns this entire SRAM bank while adctrig runs. Heap reservation
 * alone does not prevent a future firmware's static sections growing into it. */
ASSERT(_iram_end <= 0x40820000, "C5 RF dump overlaps IRAM")
ASSERT(_data_end <= 0x40820000, "C5 RF dump overlaps data")
ASSERT(_bss_end <= 0x40820000, "C5 RF dump overlaps BSS")
/* Spectrum workspace (spectrum.c) in the upper quarter of the dump bank: above
 * every spectrum snapshot (<= 2048 words). Longer captures may overwrite it, so
 * spectrum.c rebuilds it for every SPEC run. */
SECTIONS {
    .c5_spectrum 0x40838000 (NOLOAD) : ALIGN(16) {
        *(.c5_spectrum)
    }
}
ASSERT(SIZEOF(.c5_spectrum) <= 0x8000, "C5 spectrum workspace exceeds reserved bank slot")
