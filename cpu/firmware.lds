SECTIONS {
	. = 0;
	/DISCARD/ : {*(.note*)}
	startup : {*(startup)}
	.text : {*(.text*)}
	.rodata : {*(.rodata*)}
	.data : {*(.data*)}
	.bss : {*(.bss*)}
}
