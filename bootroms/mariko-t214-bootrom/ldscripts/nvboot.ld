MEMORY
{
    ROM (rx):  org = 0x00100000, len = 0x17000
    IRAM (rw): org = 0x40000000, len = 0x4000
    /* RCM header (0x3E0) bytes encroaches into part of IRAM_A to 
     * allow for RCM binary beginning/entry-point to be exactly
     * IRAM_B i.e. 0x4002_0000. Make sure stack cannot wander into this buffer
     */
    RAM (rw):  org = 0x40004000, len = (0xC000-0x400)
}

SECTIONS
{
    .text : {
	KEEP(*(.text.boot));
	*(.text.nonsecure);
	*(.rodata.nonsecure);
    . = ALIGN(0x80);
    PROVIDE_HIDDEN (__secure_region__ = .); 
	*(.text .text.*);
	*(.rodata .rodata.*);
	. = ALIGN(0x04);
    } >ROM

    __data_start_rom = .;
    .data : AT (__data_start_rom) {
	__data_start = .;
	*(.data .data.*);
	. = ALIGN(0x04);
        LONG(0x59504324);
	__data_end = .;
	*(.nozi.data);
    __nozi_start = .;
	. = ALIGN(0x04);
    __nozi_end = .;
    } >RAM
    __bss_start = .;
    .bss : {
	*(.bss .bss.*);
    __bss_end = .;
    } >RAM
    .stack ALIGN(0x04) : {
	_end = .;
	. += 0x2000;
	__stack_top = .;
    } > RAM
    /* Strip unnecessary stuff */
    /DISCARD/ : { *(.comment .note .eh_frame) }

    /*
        http://ftp.gnu.org/old-gnu/Manuals/ld-2.9.1/html_node/ld_21.html
        The `(NOLOAD)' directive will mark a section to not be loaded at run time.
        The linker will process the section normally, but will mark it so that a
        program loader will not load it into memory. For example, in the script
        sample below, the ROM section is addressed at memory location `0' and does
        not need to be loaded when the program is run. The contents of the ROM section
        will appear in the linker output file as usual.

        Basically we do not want our ROM area to be filled with the size of the IRAM
        section as above. Removing NOLOAD will cause your resulting binary to grow with
        the size of the IRAM section defined above.
        The .IRAM area will be uninitialized.
    */
    .IRAM (NOLOAD):
    {
    /* Note: ALIGN just aligns the start of this section to the next word */
    /*       boundary. If you want to align structs or variables within this
    /*       section, use the __attribute__(aligned(x)) keyword when defining */
    /*       them. */
        .iram = ALIGN(0x04);
         __bit_start = .;
         /* .data.* is removed from MainBIT, MainBCT and .CryptoBuffer, to avoid conflict with
            the (.data.data*) line in the RAM section, which takes precedence.
            Note the changes in nvboot_section_defs.h to MainBIT, MainBCT and CryptoBuffer as well.*/
        *(.MainBIT);
        __bct_start = .;
        *(.MainBCT);
	    . = ALIGN(0x04);
        __data_after_bct = .;
        __crypto_buffer_start = .;
    	*(.CryptoBuffer);
	    . = ALIGN(0x04);
        __crypto_buffer_end = .;
        *(.iram);
    } >IRAM
}

