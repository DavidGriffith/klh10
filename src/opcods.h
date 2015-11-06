/* OPCODS.H - Definitions of all PDP-10 instruction opcodes
*/
/* $Id: opcods.h,v 2.4 2002/05/21 16:54:32 klh Exp $
*/
/*  Copyright � 1992, 1993, 2001 Kenneth L. Harrenstien
**  All Rights Reserved
**
**  This file is part of the KLH10 Distribution.  Use, modification, and
**  re-distribution is permitted subject to the terms in the file
**  named "LICENSE", which contains the full text of the legal notices
**  and should always accompany this Distribution.
**
**  This software is provided "AS IS" with NO WARRANTY OF ANY KIND.
**
**  This notice (including the copyright and warranty disclaimer)
**  must be included in all copies or derivations of this software.
*/
/*
 * $Log: opcods.h,v $
 * Revision 2.4  2002/05/21 16:54:32  klh
 * Add KLH10_I_CIRC to allow any sys to have CIRC
 *
 * Revision 2.3  2001/11/10 21:28:59  klh
 * Final 2.0 distribution checkin
 *
 */

#ifndef OPCODS_RCSID
# define OPCODS_RCSID \
    RCSID(opcods_h,"$Id: opcods.h,v 2.4 2002/05/21 16:54:32 klh Exp $")
#endif

/* This file is intended to be included multiple times, always within
** a specific context that defines the macros "idef", "ixdef", and "iodef"
** suitably so as to declare or define various parts of the bindings
** represented in this file.
**
** Normal instructions are defined with:
**	idef(opval, name, enum, rtn, flags)
**
** and EXTEND opcodes are defined with:
**	ixdef(xopval, name, enum, rtn, flags)
**
** IO instructions (with internal or external devices) are
** defined with:
**	iodef(iocod, name, enum, rtn, flags)
**		where "iocod" is built with IOINOP or IOEXOP.
**
** Operations not specified here are all initialized at startup to
**	an appropriate default value, usually "i_muuo" so the monitor
**	can handle them.
** Only one opcode is truly illegal: 0 in EXEC mode, which currently
**	stops the KLH10 rather than trapping.  To change this,
**	remove the definition for "ILLEG".
*/

idef(000, "ILLEG", I_ILLEG, i_illegal,	IF_SPEC)
idef(001, "LUUO",  I_LUUO,  i_luuo,	IF_OPN|IF_1X1)	/* 001-037 inclusive */
idef(040, "MUUO",  I_MUUO,  i_muuo,	IF_OPN|IF_1X1)	/* all others */

/* Opcodes 040-0101 inclusive default to i_muuo */

	/* Late KL additions, note opcodes formerly UUOs "reserved for DEC" */
#if KLH10_CPU_KLX
 idef(052, "PMOVE", I_PMOVE, i_pmove, IF_1S)	/* KL PMOVE */
 idef(053, "PMOVEM",I_PMOVEM,i_pmovem,IF_1SM)	/* KL PMOVEM */
#endif

#if KLH10_SYS_ITS	/* ITS pager XCT */
 idef(0102, "XCTRI", I_XCTRI, i_pxct, IF_SPEC|IF_MR|IF_MIN)
 idef(0103, "XCTR",  I_XCTR,  i_pxct, IF_SPEC|IF_MR|IF_MIN)
#elif KLH10_CPU_KL
 idef(0102, "GFAD",  I_GFAD,  i_gfad, IF_2X)	/* KL GFAD */
 idef(0103, "GFSB",  I_GFSB,  i_gfsb, IF_2X)	/* KL GFSB */
#endif

#if KLH10_SYS_T10 || KLH10_SYS_T20
 idef(0104, "JSYS", I_JSYS, i_muuo, IF_ME)	/* BBN PAGER INSTRUCTION */
#endif
#if KLH10_CPU_KS || KLH10_CPU_KL
 idef(0105, "ADJSP", I_ADJSP, i_adjsp, IF_1XI)	/* KL/KS */
#endif
#if KLH10_CPU_KL
 idef(0106, "GFMP", I_GFMP, i_gfmp, IF_2X)	/* KL GFMP */
 idef(0107, "GFDV", I_GFDV, i_gfdv, IF_2X)	/* KL GFDV */
#endif
#if !KLH10_CPU_KA
 idef(0110, "DFAD",  I_DFAD,  i_dfad,	IF_2X)	/* KI+ */
 idef(0111, "DFSB",  I_DFSB,  i_dfsb,	IF_2X)	/* KI+ */
 idef(0112, "DFMP",  I_DFMP,  i_dfmp,	IF_2X)	/* KI+ */
 idef(0113, "DFDV",  I_DFDV,  i_dfdv,	IF_2X)	/* KI+ */
# if KLH10_CPU_KS || KLH10_CPU_KL
  idef(0114, "DADD",  I_DADD,  i_dadd,	IF_2X)	/* KL/KS */
  idef(0115, "DSUB",  I_DSUB,  i_dsub,	IF_2X)	/* KL/KS */
  idef(0116, "DMUL",  I_DMUL,  i_dmul, IF_AS|IF_A4|IF_MR|IF_M2)	/* KL/KS */
  idef(0117, "DDIV",  I_DDIV,  i_ddiv, IF_AS|IF_A4|IF_MR|IF_M2)	/* KL/KS */
# endif /* KL/KS */
 idef(0120, "DMOVE", I_DMOVE, i_dmove,	IF_2S)	/* KI+ */
 idef(0121, "DMOVN", I_DMOVN, i_dmovn,	IF_2S)	/* KI+ */
 idef(0122, "FIX",   I_FIX,   i_fix,	IF_1S)	/* KI+ */
# if KLH10_CPU_KS || KLH10_CPU_KL
  idef(0123,"EXTEND",I_EXTEND,i_extend,IF_SPEC|IF_AS|IF_A4|IF_MR|IF_M1) /* KL/KS */
# endif /* KL/KS */
 idef(0124, "DMOVEM",I_DMOVEM,i_dmovem,	IF_2SM)	/* KI+ */
 idef(0125, "DMOVNM",I_DMOVNM,i_dmovnm,	IF_2SM)	/* KI+ */
 idef(0126, "FIXR",  I_FIXR,  i_fixr,	IF_1S)	/* KI+ */
 idef(0127, "FLTR",  I_FLTR,  i_fltr,	IF_1S)	/* KI+ */
#endif /* !KA */
idef(0130, "UFA",   I_UFA,   i_ufa,	IF_AS|IF_A2|IF_MR|IF_M1) /* KA/KI */
idef(0131, "DFN",   I_DFN,   i_dfn,	IF_1XB)		/* KA/KI only */
idef(0132, "FSC",   I_FSC,   i_fsc,	IF_1XI)
idef(0133, "IBP",   I_IBP,   i_ibp,	IF_AS|IF_A0|IF_MS|IF_M1)
idef(0134, "ILDB",  I_ILDB,  i_ildb,	IF_AW|IF_A1|IF_MS|IF_M1)
idef(0135, "LDB",   I_LDB,   i_ldb,	IF_AW|IF_A1|IF_MR|IF_M1)
idef(0136, "IDPB",  I_IDPB,  i_idpb,	IF_AR|IF_A1|IF_MS|IF_M1)
idef(0137, "DPB",   I_DPB,   i_dpb,	IF_AR|IF_A1|IF_MS|IF_M1)
idef(0140, "FAD",   I_FAD,   i_fad,	IF_1X)
idef(0141, "FADL",  I_FADL,  i_fadl,	IF_1XFL) /* PDP6/KA/KI (not KL/KS) */
idef(0142, "FADM",  I_FADM,  i_fadm,	IF_1XM)
idef(0143, "FADB",  I_FADB,  i_fadb,	IF_1XB)
idef(0144, "FADR",  I_FADR,  i_fadr,	IF_1X)
idef(0145, "FADRI", I_FADRI, i_fadri,	IF_1XFI) /* KA+ INSTR (PDP6: FADRL) */
idef(0146, "FADRM", I_FADRM, i_fadrm,	IF_1XM)
idef(0147, "FADRB", I_FADRB, i_fadrb,	IF_1XB)
idef(0150, "FSB",   I_FSB,   i_fsb,	IF_1X)
idef(0151, "FSBL",  I_FSBL,  i_fsbl,	IF_1XFL) /* PDP6/KA/KI (not KL/KS) */
idef(0152, "FSBM",  I_FSBM,  i_fsbm,	IF_1XM)
idef(0153, "FSBB",  I_FSBB,  i_fsbb,	IF_1XB)
idef(0154, "FSBR",  I_FSBR,  i_fsbr,	IF_1X)
idef(0155, "FSBRI", I_FSBRI, i_fsbri,	IF_1XFI) /* KA+ INSTR (PDP6: FSBRL) */
idef(0156, "FSBRM", I_FSBRM, i_fsbrm,	IF_1XM)
idef(0157, "FSBRB", I_FSBRB, i_fsbrb,	IF_1XB)
idef(0160, "FMP",   I_FMP,   i_fmp,	IF_1X)
idef(0161, "FMPL",  I_FMPL,  i_fmpl,	IF_1XFL) /* PDP6/KA/KI (not KL/KS) */
idef(0162, "FMPM",  I_FMPM,  i_fmpm,	IF_1XM)
idef(0163, "FMPB",  I_FMPB,  i_fmpb,	IF_1XB)
idef(0164, "FMPR",  I_FMPR,  i_fmpr,	IF_1X)
idef(0165, "FMPRI", I_FMPRI, i_fmpri,	IF_1XFI) /* KA+ INSTR (PDP6: FMPRL) */
idef(0166, "FMPRM", I_FMPRM, i_fmprm,	IF_1XM)
idef(0167, "FMPRB", I_FMPRB, i_fmprb,	IF_1XB)
idef(0170, "FDV",   I_FDV,   i_fdv,	IF_1X)
idef(0171, "FDVL",  I_FDVL,  i_fdvl,	IF_1XFL) /* PDP6/KA/KI (not KL/KS) */
idef(0172, "FDVM",  I_FDVM,  i_fdvm,	IF_1XM)
idef(0173, "FDVB",  I_FDVB,  i_fdvb,	IF_1XB)
idef(0174, "FDVR",  I_FDVR,  i_fdvr,	IF_1X)
idef(0175, "FDVRI", I_FDVRI, i_fdvri,	IF_1XFI) /* KA+ INSTR (PDP6: FDVRL) */
idef(0176, "FDVRM", I_FDVRM, i_fdvrm,	IF_1XM)
idef(0177, "FDVRB", I_FDVRB, i_fdvrb,	IF_1XB)

	/* ; 200-277 (MOVE - SUBB) */
idef(0200, "MOVE",  I_MOVE,  i_move,	IF_1S)
idef(0201, "MOVEI", I_MOVEI, i_movei,	IF_1SI)
idef(0202, "MOVEM", I_MOVEM, i_movem,	IF_1SM)
idef(0203, "MOVES", I_MOVES, i_moves,	IF_1SS)
idef(0204, "MOVS",  I_MOVS,  i_movs,	IF_1S)
idef(0205, "MOVSI", I_MOVSI, i_movsi,	IF_1SI)
idef(0206, "MOVSM", I_MOVSM, i_movsm,	IF_1SM)
idef(0207, "MOVSS", I_MOVSS, i_movss,	IF_1SS)
idef(0210, "MOVN",  I_MOVN,  i_movn,	IF_1S)
idef(0211, "MOVNI", I_MOVNI, i_movni,	IF_1SI)
idef(0212, "MOVNM", I_MOVNM, i_movnm,	IF_1SM)
idef(0213, "MOVNS", I_MOVNS, i_movns,	IF_1SS)
idef(0214, "MOVM",  I_MOVM,  i_movm,	IF_1S)
idef(0215, "MOVMI", I_MOVMI, i_movei,	IF_1SI)	/* Same as MOVEI */
idef(0216, "MOVMM", I_MOVMM, i_movmm,	IF_1SM)
idef(0217, "MOVMS", I_MOVMS, i_movms,	IF_1SS)
idef(0220, "IMUL",  I_IMUL,  i_imul,	IF_1X)
idef(0221, "IMULI", I_IMULI, i_imuli,	IF_1XI)
idef(0222, "IMULM", I_IMULM, i_imulm,	IF_1XM)
idef(0223, "IMULB", I_IMULB, i_imulb,	IF_1XB)
idef(0224, "MUL",   I_MUL,   i_mul,	IF_AS|IF_A2|IF_X1)
idef(0225, "MULI",  I_MULI,  i_muli,	IF_AS|IF_A2|IF_XI)
idef(0226, "MULM",  I_MULM,  i_mulm,	IF_1XM)
idef(0227, "MULB",  I_MULB,  i_mulb,	IF_AS|IF_A2|IF_MS|IF_M1)
idef(0230, "IDIV",  I_IDIV,  i_idiv,	IF_AS|IF_A2|IF_X1)
idef(0231, "IDIVI", I_IDIVI, i_idivi,	IF_AS|IF_A2|IF_XI)
idef(0232, "IDIVM", I_IDIVM, i_idivm,	IF_AR|IF_A1|IF_MS|IF_M1)
idef(0233, "IDIVB", I_IDIVB, i_idivb,	IF_AS|IF_A2|IF_MS|IF_M1)
idef(0234, "DIV",   I_DIV,   i_div,	IF_AS|IF_A2|IF_X1)
idef(0235, "DIVI",  I_DIVI,  i_divi,	IF_AS|IF_A2|IF_XI)
idef(0236, "DIVM",  I_DIVM,  i_divm,	IF_AR|IF_A1|IF_MS|IF_M1)
idef(0237, "DIVB",  I_DIVB,  i_divb,	IF_AS|IF_A2|IF_MS|IF_M1)
idef(0240, "ASH",   I_ASH,   i_ash,	IF_AS|IF_A1|IF_ME8)
idef(0241, "ROT",   I_ROT,   i_rot,	IF_AS|IF_A1|IF_ME8)
idef(0242, "LSH",   I_LSH,   i_lsh,	IF_AS|IF_A1|IF_ME8)
idef(0243, "JFFO",  I_JFFO,  i_jffo,	IF_AR|IF_A1|IF_ME|IF_JMP) /* KA+ */
idef(0244, "ASHC",  I_ASHC,  i_ashc,	IF_AR|IF_A2|IF_ME8)
idef(0245, "ROTC",  I_ROTC,  i_rotc,	IF_AR|IF_A2|IF_ME8)
idef(0246, "LSHC",  I_LSHC,  i_lshc,	IF_AR|IF_A2|IF_ME8)

#if KLH10_I_CIRC /* AI-KA and KL/KS:  ROTC WITH AC+1 GOING THE WRONG WAY */
 idef(0247, "CIRC", I_CIRC, i_circ,	IF_AR|IF_A2|IF_ME8)
#endif
idef(0250, "EXCH",  I_EXCH,  i_exch,	IF_1XB)
idef(0251, "BLT",   I_BLT,   i_blt,	IF_SPEC|IF_1XB)
idef(0252, "AOBJP", I_AOBJP, i_aobjp,	IF_AS|IF_A1|IF_ME|IF_JMP)
idef(0253, "AOBJN", I_AOBJN, i_aobjn,	IF_AS|IF_A1|IF_ME|IF_JMP)
idef(0254, "JRST",  I_JRST,  i_jrst,	IF_SPEC|IF_ME|IF_JMP)
idef(0255, "JFCL",  I_JFCL,  i_jfcl,	IF_SPEC|IF_ME|IF_JMP)
idef(0256, "XCT",   I_XCT,   i_xct,	IF_SPEC|IF_MR|IF_MIN)
#if !KLH10_CPU_KA && (KLH10_PAG_KI || KLH10_PAG_KL)
 idef(0257, "MAP", I_MAP, i_map,	IF_1C|IF_X1)	/* KI+ (DEC pager) */
#endif
idef(0260, "PUSHJ", I_PUSHJ, i_pushj,	IF_SPEC|IF_1C|IF_ME|IF_JMP)
idef(0261, "PUSH",  I_PUSH,  i_push,	IF_SPEC|IF_1X)
idef(0262, "POP",   I_POP,   i_pop,	IF_SPEC|IF_AS|IF_A1|IF_MW|IF_M1)
idef(0263, "POPJ",  I_POPJ,  i_popj,	IF_SPEC|IF_AS|IF_A1|IF_ME|IF_JMP)
idef(0264, "JSR",   I_JSR,   i_jsr,	IF_MW|IF_M1|IF_JMP)
idef(0265, "JSP",   I_JSP,   i_jsp,	IF_1C|IF_ME|IF_JMP)
idef(0266, "JSA",   I_JSA,   i_jsa,	IF_AS|IF_A1|IF_MW|IF_M1|IF_JMP)
idef(0267, "JRA",   I_JRA,   i_jra,	IF_SPEC|IF_AS|IF_A1|IF_ME|IF_JMP)
idef(0270, "ADD",   I_ADD,   i_add,	IF_1X)
idef(0271, "ADDI",  I_ADDI,  i_addi,	IF_1XI)
idef(0272, "ADDM",  I_ADDM,  i_addm,	IF_1XM)
idef(0273, "ADDB",  I_ADDB,  i_addb,	IF_1XB)
idef(0274, "SUB",   I_SUB,   i_sub,	IF_1X)
idef(0275, "SUBI",  I_SUBI,  i_subi,	IF_1XI)
idef(0276, "SUBM",  I_SUBM,  i_subm,	IF_1XM)
idef(0277, "SUBB",  I_SUBB,  i_subb,	IF_1XB)

	/* ; 300-377 (CAI - SOSG) */
idef(0300, "CAI",   I_CAI,   i_cai,	IF_NOP)
idef(0301, "CAIL",  I_CAIL,  i_cail,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0302, "CAIE",  I_CAIE,  i_caie,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0303, "CAILE", I_CAILE, i_caile,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0304, "CAIA",  I_CAIA,  i_caia,	IF_SKP)
idef(0305, "CAIGE", I_CAIGE, i_caige,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0306, "CAIN",  I_CAIN,  i_cain,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0307, "CAIG",  I_CAIG,  i_caig,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0310, "CAM",   I_CAM,   i_cam,	       IF_MR|IF_M1)
idef(0311, "CAML",  I_CAML,  i_caml,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0312, "CAME",  I_CAME,  i_came,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0313, "CAMLE", I_CAMLE, i_camle,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0314, "CAMA",  I_CAMA,  i_cama,	IF_SKP|IF_MR|IF_M1)
idef(0315, "CAMGE", I_CAMGE, i_camge,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0316, "CAMN",  I_CAMN,  i_camn,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0317, "CAMG",  I_CAMG,  i_camg,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0320, "JUMP",  I_JUMP,  i_jump,	IF_NOP)
idef(0321, "JUMPL", I_JUMPL, i_jumpl,	IF_JMP|IF_AR|IF_A1|IF_ME)
idef(0322, "JUMPE", I_JUMPE, i_jumpe,	IF_JMP|IF_AR|IF_A1|IF_ME)
idef(0323, "JUMPLE",I_JUMPLE,i_jumple,	IF_JMP|IF_AR|IF_A1|IF_ME)
idef(0324, "JUMPA", I_JUMPA, i_jumpa,	IF_JMP|IF_ME)
idef(0325, "JUMPGE",I_JUMPGE,i_jumpge,	IF_JMP|IF_AR|IF_A1|IF_ME)
idef(0326, "JUMPN", I_JUMPN, i_jumpn,	IF_JMP|IF_AR|IF_A1|IF_ME)
idef(0327, "JUMPG", I_JUMPG, i_jumpg,	IF_JMP|IF_AR|IF_A1|IF_ME)
idef(0330, "SKIP",  I_SKIP,  i_skip,	       IF_AW|IF_A0|IF_MR|IF_M1)
idef(0331, "SKIPL", I_SKIPL, i_skipl,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0332, "SKIPE", I_SKIPE, i_skipe,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0333, "SKIPLE",I_SKIPLE,i_skiple,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0334, "SKIPA", I_SKIPA, i_skipa,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0335, "SKIPGE",I_SKIPGE,i_skipge,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0336, "SKIPN", I_SKIPN, i_skipn,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0337, "SKIPG", I_SKIPG, i_skipg,	IF_SKP|IF_AW|IF_A0|IF_MR|IF_M1)
idef(0340, "AOJ",   I_AOJ,   i_aoj,	       IF_AS|IF_A1|IF_ME)
idef(0341, "AOJL",  I_AOJL,  i_aojl,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0342, "AOJE",  I_AOJE,  i_aoje,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0343, "AOJLE", I_AOJLE, i_aojle,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0344, "AOJA",  I_AOJA,  i_aoja,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0345, "AOJGE", I_AOJGE, i_aojge,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0346, "AOJN",  I_AOJN,  i_aojn,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0347, "AOJG",  I_AOJG,  i_aojg,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0350, "AOS",   I_AOS,   i_aos,	       IF_AW|IF_A0|IF_MS|IF_M1)
idef(0351, "AOSL",  I_AOSL,  i_aosl,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0352, "AOSE",  I_AOSE,  i_aose,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0353, "AOSLE", I_AOSLE, i_aosle,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0354, "AOSA",  I_AOSA,  i_aosa,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0355, "AOSGE", I_AOSGE, i_aosge,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0356, "AOSN",  I_AOSN,  i_aosn,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0357, "AOSG",  I_AOSG,  i_aosg,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0360, "SOJ",   I_SOJ,   i_soj,	       IF_AS|IF_A1|IF_ME)
idef(0361, "SOJL",  I_SOJL,  i_sojl,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0362, "SOJE",  I_SOJE,  i_soje,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0363, "SOJLE", I_SOJLE, i_sojle,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0364, "SOJA",  I_SOJA,  i_soja,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0365, "SOJGE", I_SOJGE, i_sojge,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0366, "SOJN",  I_SOJN,  i_sojn,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0367, "SOJG",  I_SOJG,  i_sojg,	IF_JMP|IF_AS|IF_A1|IF_ME)
idef(0370, "SOS",   I_SOS,   i_sos,	       IF_AW|IF_A0|IF_MS|IF_M1)
idef(0371, "SOSL",  I_SOSL,  i_sosl,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0372, "SOSE",  I_SOSE,  i_sose,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0373, "SOSLE", I_SOSLE, i_sosle,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0374, "SOSA",  I_SOSA,  i_sosa,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0375, "SOSGE", I_SOSGE, i_sosge,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0376, "SOSN",  I_SOSN,  i_sosn,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)
idef(0377, "SOSG",  I_SOSG,  i_sosg,	IF_SKP|IF_AW|IF_A0|IF_MS|IF_M1)

	/* ; 400-477 (SETZ - SETOB) */
idef(0400, "SETZ",  I_SETZ,  i_setz,	IF_1C)
idef(0401, "SETZI", I_SETZI, i_setzi,	IF_1C)
idef(0402, "SETZM", I_SETZM, i_setzm,	IF_1CM)
idef(0403, "SETZB", I_SETZB, i_setzb,	IF_1CB)
idef(0404, "AND",   I_AND,   i_and,	IF_1X)
idef(0405, "ANDI",  I_ANDI,  i_andi,	IF_1XI)
idef(0406, "ANDM",  I_ANDM,  i_andm,	IF_1XM)
idef(0407, "ANDB",  I_ANDB,  i_andb,	IF_1XB)
idef(0410, "ANDCA", I_ANDCA, i_andca,	IF_1X)
idef(0411, "ANDCAI",I_ANDCAI,i_andcai,	IF_1XI)
idef(0412, "ANDCAM",I_ANDCAM,i_andcam,	IF_1XM)
idef(0413, "ANDCAB",I_ANDCAB,i_andcab,	IF_1XB)
idef(0414, "SETM",  I_SETM,  i_setm,	IF_1S)
#if KLH10_EXTADR
 idef(0415,"XMOVEI",I_XMOVEI,i_xmovei,	IF_1SI)	/* Extended version */
#else
 idef(0415,"SETMI", I_SETMI, i_setmi,	IF_1SI)
#endif
idef(0416, "SETMM", I_SETMM, i_setmm,	IF_MS|IF_M1)
idef(0417, "SETMB", I_SETMB, i_setmb,	IF_1SB)
idef(0420, "ANDCM", I_ANDCM, i_andcm,	IF_1X)
idef(0421, "ANDCMI",I_ANDCMI,i_andcmi,	IF_1XI)
idef(0422, "ANDCMM",I_ANDCMM,i_andcmm,	IF_1XM)
idef(0423, "ANDCMB",I_ANDCMB,i_andcmb,	IF_1XB)
idef(0424, "SETA",  I_SETA,  i_seta,	IF_NOP)
idef(0425, "SETAI", I_SETAI, i_setai,	IF_NOP)
idef(0426, "SETAM", I_SETAM, i_setam,	IF_1SM)
idef(0427, "SETAB", I_SETAB, i_setab,	IF_1SM)
idef(0430, "XOR",   I_XOR,   i_xor,	IF_1X)
idef(0431, "XORI",  I_XORI,  i_xori,	IF_1XI)
idef(0432, "XORM",  I_XORM,  i_xorm,	IF_1XM)
idef(0433, "XORB",  I_XORB,  i_xorb,	IF_1XB)
idef(0434, "IOR",   I_IOR,   i_ior,	IF_1X)
idef(0435, "IORI",  I_IORI,  i_iori,	IF_1XI)
idef(0436, "IORM",  I_IORM,  i_iorm,	IF_1XM)
idef(0437, "IORB",  I_IORB,  i_iorb,	IF_1XB)
idef(0440, "ANDCB", I_ANDCB, i_andcb,	IF_1X)
idef(0441, "ANDCBI",I_ANDCBI,i_andcbi,	IF_1XI)
idef(0442, "ANDCBM",I_ANDCBM,i_andcbm,	IF_1XM)
idef(0443, "ANDCBB",I_ANDCBB,i_andcbb,	IF_1XB)
idef(0444, "EQV",   I_EQV,   i_eqv,	IF_1X)
idef(0445, "EQVI",  I_EQVI,  i_eqvi,	IF_1XI)
idef(0446, "EQVM",  I_EQVM,  i_eqvm,	IF_1XM)
idef(0447, "EQVB",  I_EQVB,  i_eqvb,	IF_1XB)
idef(0450, "SETCA", I_SETCA, i_setca,	IF_AS|IF_A1)
idef(0451, "SETCAI",I_SETCAI,i_setcai,	IF_AS|IF_A1)
idef(0452, "SETCAM",I_SETCAM,i_setcam,	IF_1SM)
idef(0453, "SETCAB",I_SETCAB,i_setcab,	IF_AS|IF_A1|IF_MW|IF_M1)
idef(0454, "ORCA",  I_ORCA,  i_orca,	IF_1X)
idef(0455, "ORCAI", I_ORCAI, i_orcai,	IF_1XI)
idef(0456, "ORCAM", I_ORCAM, i_orcam,	IF_1XM)
idef(0457, "ORCAB", I_ORCAB, i_orcab,	IF_1XB)
idef(0460, "SETCM", I_SETCM, i_setcm,	IF_1X)
idef(0461, "SETCMI",I_SETCMI,i_setcmi,	IF_1XI)
idef(0462, "SETCMM",I_SETCMM,i_setcmm,	IF_1XM)
idef(0463, "SETCMB",I_SETCMB,i_setcmb,	IF_1XB)
idef(0464, "ORCM",  I_ORCM,  i_orcm,	IF_1X)
idef(0465, "ORCMI", I_ORCMI, i_orcmi,	IF_1XI)
idef(0466, "ORCMM", I_ORCMM, i_orcmm,	IF_1XM)
idef(0467, "ORCMB", I_ORCMB, i_orcmb,	IF_1XB)
idef(0470, "ORCB",  I_ORCB,  i_orcb,	IF_1X)
idef(0471, "ORCBI", I_ORCBI, i_orcbi,	IF_1XI)
idef(0472, "ORCBM", I_ORCBM, i_orcbm,	IF_1XM)
idef(0473, "ORCBB", I_ORCBB, i_orcbb,	IF_1XB)
idef(0474, "SETO",  I_SETO,  i_seto,	IF_1C)
idef(0475, "SETOI", I_SETOI, i_setoi,	IF_1C)
idef(0476, "SETOM", I_SETOM, i_setom,	IF_1CM)
idef(0477, "SETOB", I_SETOB, i_setob,	IF_1CB)

	/* ; 500-577 (HLL - HLRES) */
idef(0500, "HLL",   I_HLL,   i_hll,	IF_1S)
#if KLH10_EXTADR
 idef(0501,"XHLLI", I_XHLLI, i_xhlli,	IF_1C)	/* Extended version */
#else
 idef(0501,"HLLI",  I_HLLI,  i_hlli,	IF_1C)
#endif
idef(0502, "HLLM",  I_HLLM,  i_hllm,	IF_1SM)
idef(0503, "HLLS",  I_HLLS,  i_hlls,	IF_1SS)
idef(0504, "HRL",   I_HRL,   i_hrl,	IF_1S)
idef(0505, "HRLI",  I_HRLI,  i_hrli,	IF_1SI)
idef(0506, "HRLM",  I_HRLM,  i_hrlm,	IF_1SM)
idef(0507, "HRLS",  I_HRLS,  i_hrls,	IF_1SS)
idef(0510, "HLLZ",  I_HLLZ,  i_hllz,	IF_1S)
idef(0511, "HLLZI", I_HLLZI, i_hllzi,	IF_1C)
idef(0512, "HLLZM", I_HLLZM, i_hllzm,	IF_1SM)
idef(0513, "HLLZS", I_HLLZS, i_hllzs,	IF_1SS)
idef(0514, "HRLZ",  I_HRLZ,  i_hrlz,	IF_1S)
idef(0515, "HRLZI", I_HRLZI, i_hrlzi,	IF_1C)
idef(0516, "HRLZM", I_HRLZM, i_hrlzm,	IF_1SM)
idef(0517, "HRLZS", I_HRLZS, i_hrlzs,	IF_1SS)
idef(0520, "HLLO",  I_HLLO,  i_hllo,	IF_1S)
idef(0521, "HLLOI", I_HLLOI, i_hlloi,	IF_1C)
idef(0522, "HLLOM", I_HLLOM, i_hllom,	IF_1SM)
idef(0523, "HLLOS", I_HLLOS, i_hllos,	IF_1SS)
idef(0524, "HRLO",  I_HRLO,  i_hrlo,	IF_1S)
idef(0525, "HRLOI", I_HRLOI, i_hrloi,	IF_1C)
idef(0526, "HRLOM", I_HRLOM, i_hrlom,	IF_1SM)
idef(0527, "HRLOS", I_HRLOS, i_hrlos,	IF_1SS)
idef(0530, "HLLE",  I_HLLE,  i_hlle,	IF_1S)
idef(0531, "HLLEI", I_HLLEI, i_hllei,	IF_1C)
idef(0532, "HLLEM", I_HLLEM, i_hllem,	IF_1SM)
idef(0533, "HLLES", I_HLLES, i_hlles,	IF_1SS)
idef(0534, "HRLE",  I_HRLE,  i_hrle,	IF_1S)
idef(0535, "HRLEI", I_HRLEI, i_hrlei,	IF_1SI)
idef(0536, "HRLEM", I_HRLEM, i_hrlem,	IF_1SM)
idef(0537, "HRLES", I_HRLES, i_hrles,	IF_1SS)
idef(0540, "HRR",   I_HRR,   i_hrr,	IF_1S)
idef(0541, "HRRI",  I_HRRI,  i_hrri,	IF_1SI)
idef(0542, "HRRM",  I_HRRM,  i_hrrm,	IF_1SM)
idef(0543, "HRRS",  I_HRRS,  i_hrrs,	IF_1SS)
idef(0544, "HLR",   I_HLR,   i_hlr,	IF_1S)
idef(0545, "HLRI",  I_HLRI,  i_hlri,	IF_1C)
idef(0546, "HLRM",  I_HLRM,  i_hlrm,	IF_1SM)
idef(0547, "HLRS",  I_HLRS,  i_hlrs,	IF_1SS)
idef(0550, "HRRZ",  I_HRRZ,  i_hrrz,	IF_1S)
idef(0551, "HRRZI", I_HRRZI, i_hrrzi,	IF_1SI)
idef(0552, "HRRZM", I_HRRZM, i_hrrzm,	IF_1SM)
idef(0553, "HRRZS", I_HRRZS, i_hrrzs,	IF_1SS)
idef(0554, "HLRZ",  I_HLRZ,  i_hlrz,	IF_1S)
idef(0555, "HLRZI", I_HLRZI, i_hlrzi,	IF_1C)
idef(0556, "HLRZM", I_HLRZM, i_hlrzm,	IF_1SM)
idef(0557, "HLRZS", I_HLRZS, i_hlrzs,	IF_1SS)
idef(0560, "HRRO",  I_HRRO,  i_hrro,	IF_1S)
idef(0561, "HRROI", I_HRROI, i_hrroi,	IF_1SI)
idef(0562, "HRROM", I_HRROM, i_hrrom,	IF_1SM)
idef(0563, "HRROS", I_HRROS, i_hrros,	IF_1SS)
idef(0564, "HLRO",  I_HLRO,  i_hlro,	IF_1S)
idef(0565, "HLROI", I_HLROI, i_hlroi,	IF_1C)
idef(0566, "HLROM", I_HLROM, i_hlrom,	IF_1SM)
idef(0567, "HLROS", I_HLROS, i_hlros,	IF_1SS)
idef(0570, "HRRE",  I_HRRE,  i_hrre,	IF_1S)
idef(0571, "HRREI", I_HRREI, i_hrrei,	IF_1SI)
idef(0572, "HRREM", I_HRREM, i_hrrem,	IF_1SM)
idef(0573, "HRRES", I_HRRES, i_hrres,	IF_1SS)
idef(0574, "HLRE",  I_HLRE,  i_hlre,	IF_1S)
idef(0575, "HLREI", I_HLREI, i_hlrei,	IF_1C)
idef(0576, "HLREM", I_HLREM, i_hlrem,	IF_1SM)
idef(0577, "HLRES", I_HLRES, i_hlres,	IF_1SS)

	/* ; 600-677 (TRN - TSON) */
idef(0600, "TRN",  I_TRN,  i_trn,	IF_NOP)
idef(0601, "TLN",  I_TLN,  i_tln,	IF_NOP)
idef(0602, "TRNE", I_TRNE, i_trne,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0603, "TLNE", I_TLNE, i_tlne,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0604, "TRNA", I_TRNA, i_trna,	IF_SKP)
idef(0605, "TLNA", I_TLNA, i_tlna,	IF_SKP)
idef(0606, "TRNN", I_TRNN, i_trnn,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0607, "TLNN", I_TLNN, i_tlnn,	IF_SKP|IF_AR|IF_A1|IF_ME)
idef(0610, "TDN",  I_TDN,  i_tdn,	       IF_MR|IF_M1)
idef(0611, "TSN",  I_TSN,  i_tsn,	       IF_MR|IF_M1)
idef(0612, "TDNE", I_TDNE, i_tdne,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0613, "TSNE", I_TSNE, i_tsne,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0614, "TDNA", I_TDNA, i_tdna,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0615, "TSNA", I_TSNA, i_tsna,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0616, "TDNN", I_TDNN, i_tdnn,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0617, "TSNN", I_TSNN, i_tsnn,	IF_SKP|IF_AR|IF_A1|IF_MR|IF_M1)
idef(0620, "TRZ",  I_TRZ,  i_trz,	       IF_AS|IF_A1|IF_ME)
idef(0621, "TLZ",  I_TLZ,  i_tlz,	       IF_AS|IF_A1|IF_ME)
idef(0622, "TRZE", I_TRZE, i_trze,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0623, "TLZE", I_TLZE, i_tlze,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0624, "TRZA", I_TRZA, i_trza,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0625, "TLZA", I_TLZA, i_tlza,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0626, "TRZN", I_TRZN, i_trzn,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0627, "TLZN", I_TLZN, i_tlzn,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0630, "TDZ",  I_TDZ,  i_tdz,	       IF_AS|IF_A1|IF_MR|IF_M1)
idef(0631, "TSZ",  I_TSZ,  i_tsz,	       IF_AS|IF_A1|IF_MR|IF_M1)
idef(0632, "TDZE", I_TDZE, i_tdze, 	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0633, "TSZE", I_TSZE, i_tsze,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0634, "TDZA", I_TDZA, i_tdza,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0635, "TSZA", I_TSZA, i_tsza,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0636, "TDZN", I_TDZN, i_tdzn,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0637, "TSZN", I_TSZN, i_tszn,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0640, "TRC",  I_TRC,  i_trc,	       IF_AS|IF_A1|IF_ME)
idef(0641, "TLC",  I_TLC,  i_tlc,	       IF_AS|IF_A1|IF_ME)
idef(0642, "TRCE", I_TRCE, i_trce,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0643, "TLCE", I_TLCE, i_tlce,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0644, "TRCA", I_TRCA, i_trca,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0645, "TLCA", I_TLCA, i_tlca,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0646, "TRCN", I_TRCN, i_trcn,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0647, "TLCN", I_TLCN, i_tlcn,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0650, "TDC",  I_TDC,  i_tdc,	       IF_AS|IF_A1|IF_MR|IF_M1)
idef(0651, "TSC",  I_TSC,  i_tsc,	       IF_AS|IF_A1|IF_MR|IF_M1)
idef(0652, "TDCE", I_TDCE, i_tdce,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0653, "TSCE", I_TSCE, i_tsce,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0654, "TDCA", I_TDCA, i_tdca,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0655, "TSCA", I_TSCA, i_tsca,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0656, "TDCN", I_TDCN, i_tdcn,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0657, "TSCN", I_TSCN, i_tscn,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0660, "TRO",  I_TRO,  i_tro,	       IF_AS|IF_A1|IF_ME)
idef(0661, "TLO",  I_TLO,  i_tlo,	       IF_AS|IF_A1|IF_ME)
idef(0662, "TROE", I_TROE, i_troe,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0663, "TLOE", I_TLOE, i_tloe,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0664, "TROA", I_TROA, i_troa,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0665, "TLOA", I_TLOA, i_tloa,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0666, "TRON", I_TRON, i_tron,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0667, "TLON", I_TLON, i_tlon,	IF_SKP|IF_AS|IF_A1|IF_ME)
idef(0670, "TDO",  I_TDO,  i_tdo,	       IF_AS|IF_A1|IF_MR|IF_M1)
idef(0671, "TSO",  I_TSO,  i_tso,	       IF_AS|IF_A1|IF_MR|IF_M1)
idef(0672, "TDOE", I_TDOE, i_tdoe,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0673, "TSOE", I_TSOE, i_tsoe,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0674, "TDOA", I_TDOA, i_tdoa,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0675, "TSOA", I_TSOA, i_tsoa,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0676, "TDON", I_TDON, i_tdon,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)
idef(0677, "TSON", I_TSON, i_tson,	IF_SKP|IF_AS|IF_A1|IF_MR|IF_M1)

/* I/O INSTRUCTION DISPATCH */

/* Opcodes 0700-0777 inclusive default to either i_muuo or i_diodisp */

#if KLH10_CPU_KS || KLH10_CPU_KL	/* "Internal" devices; AC-dispatched */
 idef(0700, "IO700", I_IOV0, i_io700disp, IF_IO)
 idef(0701, "IO701", I_IOV1, i_io701disp, IF_IO)
 idef(0702, "IO702", I_IOV2, i_io702disp, IF_IO)
#endif /* KS || KL */

/* idef(0703, NULL, I_IO03, i_703, IF_IO) */

#if KLH10_CPU_KS	/* Only KS, sigh... */
 idef(0704, "UMOVE",  I_UMOVE,  i_umove,  IF_SPEC) /* KS = PXCT 4,[MOVE] */
 idef(0705, "UMOVEM", I_UMOVEM, i_umovem, IF_SPEC) /* KS = PXCT 4,[MOVEM] */
#endif /* KS */

/* idef(0706, NULL, I_IO06, i_706, IF_IO) */
/* idef(0707, NULL, I_IO07, i_707, IF_IO) */

#if KLH10_CPU_KS
# if KLH10_SYS_ITS
  idef(0710, "IORDI", I_IORDI, i_iordi, IF_MEIO)
  idef(0711, "IORDQ", I_IORDQ, i_iordq, IF_MEIO)
  idef(0712, "IORD",  I_IORD,  i_iord,  IF_MEIO)
  idef(0713, "IOWR",  I_IOWR,  i_iowr,  IF_MEIO)
  idef(0714, "IOWRI", I_IOWRI, i_iowri, IF_MEIO)
  idef(0715, "IOWRQ", I_IOWRQ, i_iowrq, IF_MEIO)
# else	/* DEC */
  idef(0710, "TIOE", I_TIOE, i_tioe, IF_MEIO)
  idef(0711, "TION", I_TION, i_tion, IF_MEIO)
  idef(0712, "RDIO", I_RDIO, i_rdio, IF_MEIO)
  idef(0713, "WRIO", I_WRIO, i_wrio, IF_MEIO)
  idef(0714, "BSIO", I_BSIO, i_bsio, IF_MEIO)
  idef(0715, "BCIO", I_BCIO, i_bcio, IF_MEIO)
# endif /* DEC */

 idef(0716, "BLTBU", I_BLTBU, i_bltbu,	IF_SPEC|IF_AS|IF_A1|IF_MW|IF_M1)
 idef(0717, "BLTUB", I_BLTUB, i_bltub,	IF_SPEC|IF_AS|IF_A1|IF_MW|IF_M1)

# if KLH10_SYS_ITS
  idef(0720, "IORDBI", I_IORDBI, i_iordbi, IF_MEIO)
  idef(0721, "IORDBQ", I_IORDBQ, i_iordbq, IF_MEIO)
  idef(0722, "IORDB",  I_IORDB,  i_iordb,  IF_MEIO)
  idef(0723, "IOWRB",  I_IOWRB,  i_iowrb,  IF_MEIO)
  idef(0724, "IOWRBI", I_IOWRBI, i_iowrbi, IF_MEIO)
  idef(0725, "IOWRBQ", I_IOWRBQ, i_iowrbq, IF_MEIO)
# else	/* DEC */
  idef(0720, "TIOEB", I_TIOEB, i_tioeb, IF_MEIO)
  idef(0721, "TIONB", I_TIONB, i_tionb, IF_MEIO)
  idef(0722, "RDIOB", I_RDIOB, i_rdiob, IF_MEIO)
  idef(0723, "WRIOB", I_WRIOB, i_wriob, IF_MEIO)
  idef(0724, "BSIOB", I_BSIOB, i_bsiob, IF_MEIO)
  idef(0725, "BCIOB", I_BCIOB, i_bciob, IF_MEIO)
# endif /* DEC */
#endif /* KLH10_CPU_KS */

/*
idef(0726, NULL, I_IO26, i_726, IF_IO)
idef(0727, NULL, I_IO27, i_727, IF_IO)
idef(0730, NULL, I_IO30, i_730, IF_IO)
idef(0731, NULL, I_IO31, i_731, IF_IO)
idef(0732, NULL, I_IO32, i_732, IF_IO)
idef(0733, NULL, I_IO33, i_733, IF_IO)
idef(0734, NULL, I_IO34, i_734, IF_IO)
idef(0735, NULL, I_IO35, i_735, IF_IO)
idef(0736, NULL, I_IO36, i_736, IF_IO)
idef(0737, NULL, I_IO37, i_737, IF_IO)
idef(0740, NULL, I_IO40, i_740, IF_IO)
idef(0741, NULL, I_IO41, i_741, IF_IO)
idef(0742, NULL, I_IO42, i_742, IF_IO)
idef(0743, NULL, I_IO43, i_743, IF_IO)
idef(0744, NULL, I_IO44, i_744, IF_IO)
idef(0745, NULL, I_IO45, i_745, IF_IO)
idef(0746, NULL, I_IO46, i_746, IF_IO)
idef(0747, NULL, I_IO47, i_747, IF_IO)
idef(0750, NULL, I_IO50, i_750, IF_IO)
idef(0751, NULL, I_IO51, i_751, IF_IO)
idef(0752, NULL, I_IO52, i_752, IF_IO)
idef(0753, NULL, I_IO53, i_753, IF_IO)
idef(0754, NULL, I_IO54, i_754, IF_IO)
idef(0755, NULL, I_IO55, i_755, IF_IO)
idef(0756, NULL, I_IO56, i_756, IF_IO)
idef(0757, NULL, I_IO57, i_757, IF_IO)
idef(0760, NULL, I_IO60, i_760, IF_IO)
idef(0761, NULL, I_IO61, i_761, IF_IO)
idef(0762, NULL, I_IO62, i_762, IF_IO)
idef(0763, NULL, I_IO63, i_763, IF_IO)
idef(0764, NULL, I_IO64, i_764, IF_IO)
idef(0765, NULL, I_IO65, i_765, IF_IO)
idef(0766, NULL, I_IO66, i_766, IF_IO)
idef(0767, NULL, I_IO67, i_767, IF_IO)
idef(0770, NULL, I_IO70, i_770, IF_IO)
idef(0771, NULL, I_IO71, i_771, IF_IO)
idef(0772, NULL, I_IO72, i_772, IF_IO)
idef(0773, NULL, I_IO73, i_773, IF_IO)
idef(0774, NULL, I_IO74, i_774, IF_IO)
idef(0775, NULL, I_IO75, i_775, IF_IO)
idef(0776, NULL, I_IO76, i_776, IF_IO)
idef(0777, NULL, I_IO77, i_777, IF_IO)
*/

/* "INTERNAL" DEVICE IO INSTRUCTIONS (0700-0702 inclusive) */

	/* APR */
iodef(IOINOP(0700, 0), "APRID", IO_APRID, io_aprid, IF_IO)	/* BI APR, */
#if !KLH10_CPU_KS
	/* DATAI APR, - (KL: Read address break)
			(KA/KI: Read console switches) */
  iodef(IOINOP(0700, 01), NULL, IO_DI_APR, io_di_apr, IF_IO)	/* DI APR, */
#endif
#if KLH10_CPU_KL
  iodef(IOINOP(0700, 02), "WRFIL", IO_WRFIL, io_wrfil, IF_IO)	/* BO APR, */
#endif
#if !KLH10_CPU_KS
	/* DATAO APR, - (KL: Set Address Break)
			(KI: set maint)
	   		(KA: set relocs) */
  iodef(IOINOP(0700, 03), NULL, IO_DO_APR, io_do_apr, IF_IO)	/* DO APR, */
#endif
iodef(IOINOP(0700, 04), "WRAPR", IO_WRAPR,  io_wrapr,  IF_IO)	/* CO APR, */
iodef(IOINOP(0700, 05), "RDAPR", IO_RDAPR,  io_rdapr,  IF_IO)	/* CI APR, */
iodef(IOINOP(0700, 06), NULL,    IO_SZ_APR, io_sz_apr, IF_IO)	/* SZ APR, */
iodef(IOINOP(0700, 07), NULL,    IO_SO_APR, io_so_apr, IF_IO)	/* SO APR, */

	/* PI */
#if KLH10_CPU_KL
  iodef(IOINOP(0700, 010), "RDERA", IO_RDERA, io_rdera, IF_IO)	/* BI PI, */
#endif
#if 0
  iodef(IOINOP(0700, 011), NULL, IO_DI_PI, NULL, IF_IO)		/* DI PI, */
#endif
#if KLH10_CPU_KL
  iodef(IOINOP(0700, 012),"SBDIAG",IO_SBDIAG,io_sbdiag,IF_IO)	/* BO PI, */
#endif
	 /* DATAO PI, - (KA/KI: Disp data on console lites) */
iodef(IOINOP(0700, 013), NULL, IO_DO_PI, io_do_pi, IF_IO)	/* DO PI, */
iodef(IOINOP(0700, 014), "WRPI", IO_WRPI, io_wrpi, IF_IO)	/* CO PI, */
iodef(IOINOP(0700, 015), "RDPI", IO_RDPI, io_rdpi, IF_IO)	/* CI PI, */
iodef(IOINOP(0700, 016), NULL, IO_SZ_PI, io_sz_pi, IF_IO)	/* SZ PI, */
iodef(IOINOP(0700, 017), NULL, IO_SO_PI, io_so_pi, IF_IO)	/* SO PI, */

	/* PAG */
#if KLH10_CPU_KS || KLH10_CPU_KL
# if KLH10_SYS_ITS
   iodef(IOINOP(0701, 0),"CLRCSH",IO_CLRCSH,io_clrcsh,IF_IO)	/* BI PAG, */
# endif
 iodef(IOINOP(0701, 01), "RDUBR", IO_RDUBR, io_rdubr, IF_IO)	/* DI PAG, */
 iodef(IOINOP(0701, 02), "CLRPT", IO_CLRPT, io_clrpt, IF_IO)	/* BO PAG, */
 iodef(IOINOP(0701, 03), "WRUBR", IO_WRUBR, io_wrubr, IF_IO)	/* DO PAG, */
 iodef(IOINOP(0701, 04), "WREBR", IO_WREBR, io_wrebr, IF_IO)	/* CO PAG, */
 iodef(IOINOP(0701, 05), "RDEBR", IO_RDEBR, io_rdebr, IF_IO)	/* CI PAG, */
# if KLH10_CPU_KL
   iodef(IOINOP(0701, 06), NULL, IO_SZ_PAG, io_sz_pag, IF_IO)	/* SZ PAG, */
   iodef(IOINOP(0701, 07), NULL, IO_SO_PAG, io_so_pag, IF_IO)	/* SO PAG, */
# endif
#endif /* KS || KL */

	/* CCA */
#if KLH10_CPU_KL
  iodef(IOINOP(0701, 010),   NULL,  IO_BI_CCA,io_swp,   IF_IO)	/* BI CCA, */
  iodef(IOINOP(0701, 011), "SWPIA", IO_SWPIA, io_swpia, IF_IO)	/* DI CCA, */
  iodef(IOINOP(0701, 012), "SWPVA", IO_SWPVA, io_swpva, IF_IO)	/* BO CCA, */
  iodef(IOINOP(0701, 013), "SWPUA", IO_SWPUA, io_swpua, IF_IO)	/* DO CCA, */
  iodef(IOINOP(0701, 014),   NULL,  IO_CO_CCA,io_swp,   IF_IO)	/* CO CCA, */
  iodef(IOINOP(0701, 015), "SWPIO", IO_SWPIO, io_swpio, IF_IO)	/* CI CCA, */
  iodef(IOINOP(0701, 016), "SWPVO", IO_SWPVO, io_swpvo, IF_IO)	/* SZ CCA, */
  iodef(IOINOP(0701, 017), "SWPUO", IO_SWPUO, io_swpuo, IF_IO)	/* SO CCA, */
#elif KLH10_SYS_ITS && KLH10_CPU_KS
  iodef(IOINOP(0701, 011), "RDPCST",IO_RDPCST,io_rdpcst,IF_IO)	/* DI CCA, */
  iodef(IOINOP(0701, 013), "WRPCST",IO_WRPCST,io_wrpcst,IF_IO)	/* DO CCA, */
#endif

	/* TIM */
#if KLH10_CPU_KS
# if KLH10_PAG_ITS
   iodef(IOINOP(0702, 0),  "SDBR1", IO_SDBR1, io_sdbr1, IF_IO)	/* BI TIM, */
   iodef(IOINOP(0702, 01), "SDBR2", IO_SDBR2, io_sdbr2, IF_IO)	/* DI TIM, */
   iodef(IOINOP(0702, 02), "SDBR3", IO_SDBR3, io_sdbr3, IF_IO)	/* BO TIM, */
   iodef(IOINOP(0702, 03), "SDBR4", IO_SDBR4, io_sdbr4, IF_IO)	/* DO TIM, */
   iodef(IOINOP(0702, 07), "SPM",   IO_SPM,   io_spm,   IF_IO)	/* SO TIM, */
# elif KLH10_PAG_KL	/* DEC-specific hacks */
   iodef(IOINOP(0702, 0), "RDSPB",  IO_RDSPB, io_rdspb, IF_IO)	/* BI TIM, */
   iodef(IOINOP(0702, 01), "RDCSB", IO_RDCSB, io_rdcsb, IF_IO)	/* DI TIM, */
   iodef(IOINOP(0702, 02), "RDPUR", IO_RDPUR, io_rdpur, IF_IO)	/* BO TIM, */
   iodef(IOINOP(0702, 03), "RDCSTM",IO_RDCSTM,io_rdcstm,IF_IO)	/* DO TIM, */
# endif
 iodef(IOINOP(0702, 04), "RDTIM", IO_RDTIM, io_rdtim, IF_IO)	/* CO TIM, */
 iodef(IOINOP(0702, 05), "RDINT", IO_RDINT, io_rdint, IF_IO)	/* CI TIM, */
 iodef(IOINOP(0702, 06), "RDHSB", IO_RDHSB, io_rdhsb, IF_IO)	/* SZ TIM, */
#endif /* KS */
#if KLH10_CPU_KL
 iodef(IOINOP(0702, 0),  "RDPERF", IO_RDPERF, io_rdperf, IF_IO)	/* BI TIM, */
 iodef(IOINOP(0702, 01), "RDTIME", IO_RDTIME, io_rdtime, IF_IO)	/* DI TIM, */
 iodef(IOINOP(0702, 02), "WRPAE",  IO_WRPAE,  io_wrpae,  IF_IO)	/* BO TIM, */
# if 0
   iodef(IOINOP(0702, 03), NULL, IO_DO_TIM, NULL, IF_IO)	/* DO TIM, */
# endif
 iodef(IOINOP(0702, 04), NULL,   IO_CO_TIM, io_co_tim, IF_IO)	/* CO TIM, */
 iodef(IOINOP(0702, 05), NULL,   IO_CI_TIM, io_ci_tim, IF_IO)	/* CI TIM, */
 iodef(IOINOP(0702, 06), NULL,   IO_SZ_TIM, io_sz_tim, IF_IO)	/* SZ TIM, */
 iodef(IOINOP(0702, 07), NULL,   IO_SO_TIM, io_so_tim, IF_IO)	/* SO TIM, */
#endif /* KL */

	/* MTR */
#if KLH10_CPU_KS
# if KLH10_PAG_ITS
   iodef(IOINOP(0702, 010), "LDBR1", IO_LDBR1, io_ldbr1, IF_IO)	/* BI MTR, */
   iodef(IOINOP(0702, 011), "LDBR2", IO_LDBR2, io_ldbr2, IF_IO)	/* DI MTR, */
   iodef(IOINOP(0702, 012), "LDBR3", IO_LDBR3, io_ldbr3, IF_IO)	/* BO MTR, */
   iodef(IOINOP(0702, 013), "LDBR4", IO_LDBR4, io_ldbr4, IF_IO)	/* DO MTR, */
   iodef(IOINOP(0702, 017), "LPMR",  IO_LPMR,  io_lpmr,  IF_IO)	/* SO MTR, */
# elif KLH10_PAG_KL	/* DEC-specific hacks */
   iodef(IOINOP(0702, 010), "WRSPB", IO_WRSPB, io_wrspb, IF_IO)	/* BI MTR, */
   iodef(IOINOP(0702, 011), "WRCSB", IO_WRCSB, io_wrcsb, IF_IO)	/* DI MTR, */
   iodef(IOINOP(0702, 012), "WRPUR", IO_WRPUR, io_wrpur, IF_IO)	/* BO MTR, */
   iodef(IOINOP(0702, 013), "WRCSTM",IO_WRCSTM,io_wrcstm,IF_IO)	/* DO MTR, */
# endif
 iodef(IOINOP(0702, 014), "WRTIM", IO_WRTIM, io_wrtim, IF_IO)	/* CO MTR, */
 iodef(IOINOP(0702, 015), "WRINT", IO_WRINT, io_wrint, IF_IO)	/* CI MTR, */
 iodef(IOINOP(0702, 016), "WRHSB", IO_WRHSB, io_wrhsb, IF_IO)	/* SZ MTR, */
#endif /* KS */
#if KLH10_CPU_KL
 iodef(IOINOP(0702, 010), "RDMACT",IO_RDMACT, io_rdmact, IF_IO)	/* BI MTR, */
 iodef(IOINOP(0702, 011), "RDEACT",IO_RDEACT, io_rdeact, IF_IO)	/* DI MTR, */
# if 0
 iodef(IOINOP(0702, 012), NULL, IO_BO_MTR, NULL, IF_IO)		/* BO MTR, */
 iodef(IOINOP(0702, 013), NULL, IO_DO_MTR, NULL, IF_IO)		/* DO MTR, */
# endif
 iodef(IOINOP(0702, 014), "WRTIME",IO_WRTIME, io_wrtime, IF_IO)	/* CO MTR, */
 iodef(IOINOP(0702, 015), NULL,    IO_CI_MTR, io_ci_mtr, IF_IO)	/* CI MTR, */
 iodef(IOINOP(0702, 016), NULL,    IO_SZ_MTR, io_sz_mtr, IF_IO)	/* SZ MTR, */
 iodef(IOINOP(0702, 017), NULL,    IO_SO_MTR, io_so_mtr, IF_IO)	/* SO MTR, */
#endif /* KL */

/* NOTE!! Check the IO_N def in opdefs.h if more IO-class ops are added! */

/* EXTENDED INSTRUCTIONS */

#if (KLH10_CPU_KS || KLH10_CPU_KL) \
	&& (KLH10_SYS_T10 || KLH10_SYS_T20)	/* DEC systems only */
ixdef(IXOP(000), "ILLEG",  IX_ILLEG,  ix_undef,	IF_SPEC)
ixdef(IXOP(001), "CMPSL",  IX_CMPSL,  ix_cmps,	IF_SPEC) /* Common rtn */
ixdef(IXOP(002), "CMPSE",  IX_CMPSE,  ix_cmps,	IF_SPEC) /*   "  */
ixdef(IXOP(003), "CMPSLE", IX_CMPSLE, ix_cmps,	IF_SPEC) /*   "  */
ixdef(IXOP(004), "EDIT",   IX_EDIT,   ix_edit,	IF_SPEC)
ixdef(IXOP(005), "CMPSGE", IX_CMPSGE, ix_cmps,	IF_SPEC) /* Common rtn */
ixdef(IXOP(006), "CMPSN",  IX_CMPSN,  ix_cmps,	IF_SPEC) /*   "  */
ixdef(IXOP(007), "CMPSG",  IX_CMPSG,  ix_cmps,	IF_SPEC) /*   "  */
ixdef(IXOP(010), "CVTDBO", IX_CVTDBO, ix_cvtdb,	IF_SPEC) /* Common rtn */
ixdef(IXOP(011), "CVTDBT", IX_CVTDBT, ix_cvtdb,	IF_SPEC) /*   "  */
ixdef(IXOP(012), "CVTBDO", IX_CVTBDO, ix_cvtbd,	IF_SPEC) /* Common rtn */
ixdef(IXOP(013), "CVTBDT", IX_CVTBDT, ix_cvtbd,	IF_SPEC) /*  "   */
ixdef(IXOP(014), "MOVSO",  IX_MOVSO,  ix_movso,	IF_SPEC)
ixdef(IXOP(015), "MOVST",  IX_MOVST,  ix_movst,	IF_SPEC)
ixdef(IXOP(016), "MOVSLJ", IX_MOVSLJ, ix_movslj,IF_SPEC)
ixdef(IXOP(017), "MOVSRJ", IX_MOVSRJ, ix_movsrj,IF_SPEC)
#if KLH10_CPU_KLX	/* Always MUUO if not on extended KL */
 ixdef(IXOP(020), "XBLT",   IX_XBLT,   ix_xblt,	IF_SPEC)
#endif
#if KLH10_CPU_KL
 ixdef(IXOP(021), "GSNGL",  IX_GSNGL,  ix_gsngl, IF_SPEC)
 ixdef(IXOP(022), "GDBLE",  IX_GDBLE,  ix_gdble, IF_SPEC)
# if 0 /* Simulated in T10+T20 monitor */
  ixdef(IXOP(023), "GDFIX",  IX_GDFIX,  ix_gdfix, IF_SPEC)
  ixdef(IXOP(024), "GFIX",   IX_GFIX,   ix_gfix,  IF_SPEC)
  ixdef(IXOP(025), "GDFIXR", IX_GDFIXR, ix_gdfixr,IF_SPEC)
  ixdef(IXOP(026), "GFIXR",  IX_GFIXR,  ix_gfixr, IF_SPEC)
# endif /* 0 */
 ixdef(IXOP(027), "DGFLTR", IX_DGFLTR, ix_dgfltr,IF_SPEC)
 ixdef(IXOP(030), "GFLTR",  IX_GFLTR,  ix_gfltr, IF_SPEC)
 ixdef(IXOP(031), "GFSC",   IX_GFSC,   ix_gfsc,  IF_SPEC)
#endif /* KL only */
#endif /* (KL||KS) && (T10||T20) */

/* NOTE!! Check the IX_N def in opdefs.h if more EXTEND ops are added! */

/*	Other cruft for dubious posterity */

/*
;OLD PROGRAMS USE THESE NAMES

CLEAR==SETZ
CLEARI==SETZI
CLEARM==SETZM
CLEARB==SETZB

;RANDOM ALIAS NAMES

ERJMP==JUMP 16,		; TOPS-20 JSYS-error dispatch (becomes JRST)
ERCAL==JUMP 17,		; TOPS-20 JSYS-error call (becomes PUSHJ 17,)
ADJBP==IBP		;KL10 FORM OF IBP WITH VARIABLE NUMBER TO INCREMENT
JFOV==JFCL 1,		;PDP10 INSTRUCTION (PC CHANGE ON PDP6)
JCRY1==JFCL 2,
JCRY0==JFCL 4,
JCRY==JFCL 6,
JOV==JFCL 10,
PORTAL==JRST 1,		; KI/KL
JRSTF==JRST 2,
HALT==JRST 4,
XJRSTF==JRST 5,		; KL/KS
XJEN==JRST 6,		; KL/KS
XPCW==JRST 7,		; KL/KS
JEN==JRST 12,
SFM==JRST 14,		; KL/KS
XMOVEI==SETMI		; KL only
XHLLI==HLLI		; KL only

;PDP6 HAS LONG FORM ROUNDED INSTEAD OF IMMEDIATES (FADRL, not FADRI, etc)

*/
