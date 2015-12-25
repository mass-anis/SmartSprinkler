


void __attribute__((naked))
Delay10Cycle(unsigned long ulCount)
{
	__asm(	"    subs    r0, #1\n"
			"	 mov	 r0, r0\n"
			"	 mov	 r0, r0\n"
			"	 mov	 r0, r0\n"
			"	 mov	 r0, r0\n"
			"	 mov	 r0, r0\n"
			"	 mov	 r0, r0\n"
			"	 mov	 r0, r0\n"
			"    bne     Delay10Cycle\n"
			"    bx      lr");
}
