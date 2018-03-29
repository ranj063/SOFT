divert(-1)

dnl Define macro for byte control

dnl CONTROLBYTES_MAX(comment, value)
define(`CONTROLBYTES_MAX',
`#$1'
`	max STR($2)')

dnl CONTROLMIXER_TLV(comment, value)
define(`CONTROLBYTES_TLV',
`#$1'
`	tlv STR($2)')

dnl CONTROLMIXER_OPS(info, comment, get, put)
define(`CONTROLBYTES_OPS',
`ops."ctl" {'
`		info STR($1)'
`		#$2'
`		get STR($3)'
`		put STR($4)'
`	}')

dnl C_CONTROLMIXER(name, index, ops, base, num_regs, mask, max, tlv)
define(`C_CONTROLBYTES',
`SectionControlBytes.STR($1) {'
`'
`	# control belongs to this index group'
`	index STR($2)'
`'
`	# control uses bespoke driver get/put/info ID'
`	$3'
`'
`	base STR($4)'
`	num_regs STR($5)'
`	mask STR($6)'
`	$7'
`	$8'

`}')

divert(0)dnl
