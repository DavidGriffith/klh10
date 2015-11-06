/* OSDNET.C - OS Dependent Network facilities
*/
/* $Id: osdnet.c,v 2.8 2003/02/23 18:22:08 klh Exp $
*/
/*  Copyright � 1999, 2001 Kenneth L. Harrenstien
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
 * $Log: osdnet.c,v $
 * Revision 2.8  2003/02/23 18:22:08  klh
 * Fix various problems for NetBSD/Alpha 1.6.
 *
 * Revision 2.7  2002/03/18 04:25:03  klh
 * Add support for promiscuous mode, fix minor ARP error output
 *
 * Revision 2.6  2001/11/19 10:32:34  klh
 * Solaris port fixups.
 *
 * Revision 2.5  2001/11/10 21:28:59  klh
 * Final 2.0 distribution checkin
 *
 */

/*	This file, like DPSUP.C, is intended to be included directly
    into source code rather than compiled separately and then linked 
    together.  The reason for this is that the actual configuration
    of code will vary depending on its intended use, so a single .o
    file cannot satisfy all programs.
 */

#include <unistd.h>	/* For basic Unix syscalls */

/* The possible configuration macro definitions, with defaults:
 */
#ifndef NETIFC_MAX
# define NETIFC_MAX 20
#endif

#ifndef OSDNET_INCLUDED
# include "osdnet.h"	/* Insurance to make sure our defs are there */
#endif

#ifdef RCSID
 RCSID(osdnet_c,"$Id: osdnet.c,v 2.8 2003/02/23 18:22:08 klh Exp $")
#endif

/* Local predeclarations */

struct ifent *osn_iflookup(char *ifnam);
int osn_ifealookup(char *ifnam, unsigned char *eap);

/* Get a socket descriptor suitable for general net interface
   examination and manipulation; this is not necessarily suitable for
   use as a packetfilter.
   This may only make sense on Unix.
 */
int
osn_ifsock(char *ifnam, ossock_t *as)
{
#if (KLH10_NET_NIT || KLH10_NET_DLPI || KLH10_NET_BPF || KLH10_NET_PFLT || \
	KLH10_NET_TUN || KLH10_NET_LNX)
    return ((*as = socket(AF_INET, SOCK_DGRAM, 0)) >= 0);
#else
# error OSD implementation needed for osn_ifsock
#endif
}

int
osn_ifclose(ossock_t s)
{
#if (KLH10_NET_NIT || KLH10_NET_DLPI || KLH10_NET_BPF || KLH10_NET_PFLT || \
	KLH10_NET_TUN || KLH10_NET_LNX)
    return (close(s) >= 0);
#else
# error OSD implementation needed for osn_ifclose
#endif
}


/* Minor utilities */

char *
eth_adrsprint(char *cp, unsigned char *ea)
{
    sprintf(cp, "%x:%x:%x:%x:%x:%x", ea[0], ea[1], ea[2], ea[3], ea[4], ea[5]);
    return cp;
}

char *
ip_adrsprint(char *cp, unsigned char *ia)
{
    sprintf(cp, "%d.%d.%d.%d", ia[0], ia[1], ia[2], ia[3]);
    return cp;
}


/* Interface Table initialization
**	Gets info about all net interfaces known to the native system.
**	Used for several purposes, hence fairly general.
**
*/
/*
  SIOCGIFCONF documentation is virtually non-existent.  The following
  info comes from looking at source code.
	struct ifconf ifconf;
	s = socket(AF_INET, SOCK_DGRAM, 0);
	ioctl(s, SIOCGIFCONF, &ifconf);

  ifconf has only two elements:
	<sys/if.h>: struct ifconf
		caddr_t	ifcu_buf;	Buffer to use
		int ifc_len;		Buffer size in bytes (or # bytes used)

  On return, the buffer holds a series of "struct ifreq" entries, each of
  which holds an interface name and address.  On older systems, each
  such entry was a fixed size, but this was later changed in order to
  accomodate longer addresses and consequently,
		=====>> EACH ENTRY MAY BE OF VARYING SIZE!!!! <<======

  This, by the way, is why SIOCGIFCONF appeared to be broken on OSF/1 when
  used with a single ifreq's worth of buffer -- since it needed a larger
  "ifreq" than the buffer had room for, it returned nothing.

  Although the second element of "struct ifreq" is a union, SIOCGIFCONF always
  uses it to return a sockaddr:
	<sys/if.h>: struct ifreq
		char ifr_name[16]		interface name, eg "de0"
		struct sockaddr ifr_addr	interface address
	<sys/socket.h>: struct sockaddr
		unsigned char sa_len		# bytes in this sockaddr
		unsigned char sa_family		address type, AF_xxx
		char sa_data[14]		actually up to 253 bytes!

  Note that the sa_len field is new; older versions of sockaddr had
  just sa_family and sa_data.

  To repeat, the actual size of an ifreq depends on the size of the
  sockaddr it contains.  This is ALWAYS at least sizeof(struct sockaddr),
  so trickiness is only needed when sa_len is greater than this default
  size.

#define	_SIZEOF_ADDR_IFREQ(p) \
	((p)->ifr_addr.sa_len <= sizeof(struct sockaddr) \
	?  sizeof(struct ifreq) \
	: (sizeof(struct ifreq)-sizeof(struct sockaddr)+(p)->ifr_addr.sa_len))

or alternatively:

#define	_SIZEOF_ADDR_IFREQ(ifr) \
	((ifr).ifr_addr.sa_len > sizeof(struct sockaddr) \
	? (sizeof(struct ifreq) - sizeof(struct sockaddr) + (ifr).ifr_addr.sa_len) \
	: sizeof(struct ifreq))

*/
/*
  Note that searching for AF_INET or IP addresses only finds interfaces that
  are presently configured with IP addresses by "ifconfig", typically those
  that are also up.  An interface that is dedicated to the emulator will
  normally be both "down" and have no IP address bound to it.

  Another way to look at this is that until an interface is configured
  up, it has no IP address bound to it.  This is the same reason that
  SIOCGIFADDR cannot be used to find the IP address of a dedicated
  interface; there is none.
*/

/* Complete table as returned from system by SIOCGIFCONF
 * Currently static for debugging, could be dynamic.
 */
static struct ifconf ifctab;
static char ifcbuf[sizeof(struct ifreq) * NETIFC_MAX];

/* Our own internal table of interface entries, filtered
   to keep only the ones we're interested in.
 */
static int iftab_initf = 0;
static int iftab_nifs = 0;
static struct ifent iftab[NETIFC_MAX];

static void osn_iftab_pass(int opts, int npass, int s, struct ifconf *ifc);

/* Get table of all interfaces, using our own generic entry format.
 * Routine works as follows:
 *
 * First, grabs a IFCONF table of all interfaces from the kernel.
 *	Each IFR entry contains a name and an address.
 *
 * Second, scan this IFCONF table (pass 1) to build our own table of IFE
 * entries, made of all IFR entries that pass the following filter
 * flags:
 *	IFTAB_IPS   - if set, accepts AF_INET interfaces (with IP address)
 *	IFTAB_ETHS  - if set, accepts LINK interfaces with Ether-like addrs.
 *	IFTAB_OTH   - if set, accepts all others (with unknown addr)
 *
 * Third, make another scan of IFCONF (pass 2) that grabs *all* information
 * about any interface present in our IFE table, regardless of what caused
 * it to be inserted.
 */
int
osn_iftab_init(int opts)
{
    int s;
    struct ifconf *ifc = &ifctab;
    register struct ifent *ifet;

    /* Start out with empty table */
    iftab_nifs = 0;
    ifet = &iftab[0];

    /* Open socket with AF_INET family to get at IP stuff.
     */
    if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	esfatal(1, "osn_iftab_init socket()");
    }

    /* Gobble table of config info about all interfaces, including name
       and IP address.
     */
    ifc->ifc_len = sizeof(ifcbuf);
    ifc->ifc_buf = (caddr_t)ifcbuf;
    if (ioctl(s, SIOCGIFCONF, (char *)ifc) < 0) {
	close(s);
	esfatal(1, "osn_iftab_init SIOCGIFCONF");
    }
    if (ifc->ifc_len < sizeof(struct ifreq)) {
	if (DP_DBGFLG)
	    dbprintln("SIOCGIFCONF only got %d bytes (need %d)",
			(int)ifc->ifc_len, (int)sizeof(struct ifreq));
	close(s);
	return 0;			/* Assume no interfaces */
    }
    if (DP_DBGFLG)
	dbprintln("SIOCGIFCONF returned %d bytes (%d ifcs?)",
		 (int)ifc->ifc_len, (int)(ifc->ifc_len/sizeof(struct ifreq)));

    if (DP_DBGFLG)
	osn_ifctab_show(stdout, ifc);

    /* Grabbed everything from OS, now grovel through it */
    osn_iftab_pass(opts, 1, s, ifc);	/* Do pass 1 scan */
    osn_iftab_pass(opts, 2, s, ifc);	/* Do pass 2 scan */
    iftab_initf = opts;			/* Inited! */

    close(s);
    if (DP_DBGFLG)
	osn_iftab_show(stdout, &iftab[0], iftab_nifs);
    return iftab_nifs;
}

int
osn_nifents(void)
{
    return iftab_nifs;
}

static void
osn_iftab_pass(int opts, int npass, int s, struct ifconf *ifc)
{
    register int i;
    int offset;
    struct ifreq ifr;
    register struct ifreq *ifp, *ifend, *ifnext;
    register struct ifent *ife, *ifet;

    /* Start out with empty table */
    if (npass == 1)
	iftab_nifs = 0;

    ifet = &iftab[0];

    ifp = ifc->ifc_req;
    ifend = (struct ifreq *)((char *)ifp + ifc->ifc_len);
    for (; ifp < ifend; ifp = ifnext) {

	/* Find next pointer now, to simplify structure of code.
	   This is complicated by the fact that the returned table format
	   has changed under different OS versions.  The "new regime"
	   uses a variable-size "ifreq" entry!  Choke...
	 */
	ifnext = ifp + 1;	/* Assume normal entry at first */
#if NETIF_HAS_SALEN
	if (ifp->ifr_addr.sa_len > sizeof(struct sockaddr)) {
	    offset = ifp->ifr_addr.sa_len - sizeof(struct sockaddr);
	    ifnext = (struct ifreq *)((char *)ifnext + offset);
	}
#endif
	/* See whether this entry is already in table.
	*/
	for (i = 0; i < iftab_nifs; ++i) {
	    if (strncmp(ifp->ifr_name, ifet[i].ife_name, sizeof(ifp->ifr_name))
		== 0) {
		/* Found existing entry!  Use its pointer */
		ife = &ifet[i];
		break;
	    }
	}
	if (i >= iftab_nifs) {
	    if (npass == 2)
		continue;
	    /* Pass1, decide now whether to create entry or not */
	    /* See if it's something we're interested in. */
	    if ((opts & IFTAB_IPS) && (ifp->ifr_addr.sa_family == AF_INET)) {
		/* yes */
#if NETIF_HAS_ETHLINK
	    } else if ((opts & IFTAB_ETHS)
		       && (ifp->ifr_addr.sa_family == AF_LINK)
		       && (((struct sockaddr_dl *)&ifp->ifr_addr)->sdl_alen
			   == ETHER_ADRSIZ)){
		/* yes */
#endif
	    } else if (opts & IFTAB_OTH) {
		/* yes */
	    } else {
		/* Nope, ignore this one */
		continue;
	    }

	    /* Yes, create this entry */
	    if (iftab_nifs >= NETIFC_MAX-1) {
		error("ifc table overflow, max %d", NETIFC_MAX);
		return;			/* Stop now */
	    }
	    ife = &ifet[iftab_nifs++];	/* New table entry to fill out */

	    /* Copy interface name and ensure null-terminated
	       even if sizes someday get out of synch.
	     */
	    strncpy(ife->ife_name, ifp->ifr_name,
		    (sizeof(ife->ife_name) < sizeof(ifp->ifr_name))
		    ? sizeof(ife->ife_name) : sizeof(ifp->ifr_name));
	    ife->ife_name[sizeof(ife->ife_name)-1] = '\0';

	    /* These ioctls should work for all interfaces */
	    strncpy(ifr.ifr_name, ifp->ifr_name, sizeof(ifr.ifr_name));
	    if (ioctl(s, SIOCGIFFLAGS, (char *)&ifr) < 0) {
		syserr(errno, "SIOCGIFFLAGS for \"%s\"", ife->ife_name);
		continue;			/* Discard entry */
	    }
	    ife->ife_flags = ifr.ifr_flags;

	    /* SIOCGIFMTU: ifr_mtu */
	    /* SIOCGIFMETRIC: ifr_metric */
	    continue;
	}
	if (npass == 1)
	    continue;

	/* Pass 2 and dealing with an existing entry.  Flesh it out! */
	switch (ifp->ifr_addr.sa_family) {
	case AF_INET:
	    ife->ife_pinet = ifp;	/* Remember pointer */
	    ife->ife_ipint = ((struct sockaddr_in *)
				&ifp->ifr_addr)->sin_addr.s_addr;
	    break;
#if NETIF_HAS_ETHLINK
	case AF_LINK:
	    if (((struct sockaddr_dl *)&ifp->ifr_addr)->sdl_alen
		== ETHER_ADRSIZ) {
		/* Looks like Ethernet physical link, save addr */
		struct sockaddr_dl *dla = (struct sockaddr_dl *)&ifp->ifr_addr;
		memcpy(ife->ife_ea,  LLADDR(dla), dla->sdl_alen);
		ife->ife_gotea = TRUE;
		ife->ife_plink = ifp;
		break;
	    }
#endif
	    /* Else drop through to "other" case */
	default:
    	    ife->ife_pother = ifp;
	    break;
	}
    }
}

void
osn_ifctab_show(FILE *f, struct ifconf *ifc)
{
    register struct ifreq *ifr;
    struct ifreq *ifrend;
    int len;
    int i;
    unsigned char *cp;
    int nents = 0;
    int nvary = 0;

    fprintf(f, "Interface table: %ld bytes (%d entries if std addr len %d)\n",
	    (long)ifc->ifc_len, ifc->ifc_len/sizeof(struct ifreq),
	    (int)sizeof(struct sockaddr));

    ifr = ifc->ifc_req;
    ifrend = (struct ifreq *)((char *)(ifc->ifc_req) + ifc->ifc_len);
    for (i = 0; ifr < ifrend; ++i) {
	++nents;

#if NETIF_HAS_SALEN
	len = ifr->ifr_addr.sa_len;
#else
	len = sizeof(struct sockaddr);
#endif

	/* Output entry data */
	fprintf(f, "%2d: \"%.*s\" fam %d, len %d",
		i, (int)sizeof(ifr->ifr_name), ifr->ifr_name,
		ifr->ifr_addr.sa_family, len);
	if (len) {
	    cp = (unsigned char *) ifr->ifr_addr.sa_data;
	    fprintf(f, " = %x", *cp);
	    for (--len; len > 0; --len) {
		fprintf(f, ":%x", *++cp);
	    }
	}
	fprintf(f, "\n");

	cp = (unsigned char *) ifr->ifr_addr.sa_data;
	switch (ifr->ifr_addr.sa_family) {
	case AF_INET:
	  {
	    struct sockaddr_in *skin = (struct sockaddr_in *) &ifr->ifr_addr;
	    struct in_addr *in = &skin->sin_addr;
	    unsigned char *ucp = (unsigned char *) &in->s_addr;

	    fprintf(f, "        AF_INET = port %d, IP %d.%d.%d.%d\n",
		    (int)skin->sin_port,
		    ucp[0], ucp[1], ucp[2], ucp[3]);
	  }
	    break;

#if NETIF_HAS_ETHLINK
	case AF_LINK:
	  {
	    struct sockaddr_dl *dla = (struct sockaddr_dl *) &ifr->ifr_addr;
	    fprintf(f, "        AF_LINK = type %d, alen %d",
		    dla->sdl_type, dla->sdl_alen);
	    if (len = dla->sdl_alen) {
		cp = (unsigned char *) LLADDR(dla);
		fprintf(f, " = %x", *cp);
		for (--len; len > 0; --len) {
		    fprintf(f, ":%x", *++cp);
		}
	    }
	    fprintf(f, "\n");
	  }
	    break;
#endif

	default:
	    fprintf(f, "        No handler for this family\n");
	}


	/* Move onto next entry */
#if NETIF_HAS_SALEN
	if (ifr->ifr_addr.sa_len > sizeof(struct sockaddr)) {
	    ++nvary;
	    ifr = (struct ifreq *)((char *)(ifr + 1) +
			(ifr->ifr_addr.sa_len - sizeof(struct sockaddr)));
	} else
#endif
	ifr++;
    }
    if (nvary)
	fprintf(f, "Interface summary: %d entries of varying length\n",
		nents);
    else
	fprintf(f, "Interface summary: %d entries of std length %d\n",
		nents, (int)sizeof(struct ifreq));
}


void
osn_iftab_show(FILE *f, struct ifent *ifents, int nents)
{
    register struct ifent *ife;
    int i;

    fprintf(f, "Filtered IFE table: %d entries\n", nents);

    for (i = 0, ife = ifents; i < nents; ++i, ++ife) {
	fprintf(f, "%2d: \"%s\"", i, ife->ife_name);
	if (ife->ife_pinet) {
	    unsigned char *ucp = ife->ife_ipchr;
	    fprintf(f, " (IP %d.%d.%d.%d)",
		    ucp[0], ucp[1], ucp[2], ucp[3]);
	}
	if (ife->ife_plink || ife->ife_gotea) {
	    unsigned char *ucp = ife->ife_ea;
	    fprintf(f, " (%sEther %x:%x:%x:%x:%x:%x)",
		    (ife->ife_plink ? "" : "Extracted "),
		    ucp[0], ucp[1], ucp[2], ucp[3], ucp[4], ucp[5]);
	}
	if (ife->ife_pother) {
	    fprintf(f, " (Other: fam %d)",
		    ife->ife_pother->ifr_addr.sa_family);
	}
	fprintf(f, "\n");
    }
}

/* OSN_IFTAB_ARP - Look up an entry ARP-style (given IP address)
 */
struct ifent *
osn_iftab_arp(struct in_addr ia)
{
    register int i = 0;
    register struct ifent *ife = iftab;

    for (; i < iftab_nifs; ++i, ++ife)
	if (ife->ife_ipia.s_addr == ia.s_addr)
	    return ife;
    return NULL;
}

/* OSN_IFTAB_ARPGET - As above, but returns EA if found.
 */
int
osn_iftab_arpget(struct in_addr ia, unsigned char *eap)
{
    register struct ifent *ife;

    if ((ife = osn_iftab_arp(ia)) && ife->ife_gotea) {
	ea_set(eap, ife->ife_ea);
	return TRUE;
    }
    return FALSE;
}


/* OSN_IPDEFAULT - Find a default IP interface entry; take the first one
 *	that's up and isn't a loopback.
 */
struct ifent *
osn_ipdefault(void)
{
    register int i = 0;
    register struct ifent *ife = iftab;

    for (; i < iftab_nifs; ++i, ++ife)
	if ((ife->ife_flags & IFF_UP) && !(ife->ife_flags & IFF_LOOPBACK))
	    return ife;
    return NULL;
}

/* OSN_IFLOOKUP - Find interface entry in our table; NULL if none.
 */
struct ifent *
osn_iflookup(char *ifnam)
{
    register int i = 0;
    register struct ifent *ife = iftab;

    for (; i < iftab_nifs; ++i, ++ife)
	if (strcmp(ifnam, ife->ife_name) == 0)
	    return ife;
    return NULL;
}

/* OSN_IFEALOOKUP - Find ethernet address, barf if neither in our table nor
 *	available via OS.
 */
int
osn_ifealookup(char *ifnam,		/* Interface name */
	       unsigned char *eap)	/* Where to write ether address */
{
#if NETIF_HAS_ETHLINK
    register struct ifent *ife;

    if ((ife = osn_iflookup(ifnam))
	&& (ife->ife_plink || ife->ife_gotea)) {
	    ea_set(eap, ife->ife_ea);
	    return TRUE;
    }
    return FALSE;
#else
    return osn_ifeaget(-1, ifnam, eap, (unsigned char *)NULL);
#endif
}

/* OSN_ARP_STUFF - stuff emulated-host ARP entry into kernel.
**	Note it isn't necessary to specify an interface!
**	Also, the code assumes that if an ARP entry already exists in the
**	kernel for the given IP address, it will be reset to this new
**	setting rather than (eg) failing.
*/
int
osn_arp_stuff(unsigned char *ipa, unsigned char *eap, int pubf)
{
    char ipbuf[OSN_IPSTRSIZ];
    char eabuf[OSN_EASTRSIZ];

#if NETIF_HAS_ARPIOCTL
    struct arpreq arq;
    int sock;

    memset((char *)&arq, 0, sizeof(arq));	/* Clear & set up ARP req */
    arq.arp_pa.sa_family = AF_INET;		/* Protocol addr is IP type */
    memcpy(					/* Copy IP addr */
	(char *) &((struct sockaddr_in *)&arq.arp_pa)->sin_addr,
	ipa, sizeof(struct in_addr)); 
    arq.arp_ha.sa_family = AF_UNSPEC;		/* Hardware addr is Ether */
    ea_set(arq.arp_ha.sa_data, eap);		/* Copy Ether addr */

    /* Set ARP flags.  Always make permanent so needn't keep checking. */
    arq.arp_flags = ATF_PERM;			/* Make permanent */
    if (pubf)
	arq.arp_flags |= ATF_PUBL;		/* Publish it for us too! */

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {	/* Get random socket */
	syserr(errno, "Cannot set ARP entry for %s %s - socket()",
			ip_adrsprint(ipbuf, ipa),
			eth_adrsprint(eabuf, eap));
	return FALSE;
    }
    (void) ioctl(sock, SIOCDARP, (char *)&arq);		/* Clear old info */
    if (ioctl(sock, SIOCSARP, (char *)&arq) < 0) {	/* Set new */
	syserr(errno, "Cannot set ARP entry for %s %s - SIOCSARP",
			ip_adrsprint(ipbuf, ipa),
			eth_adrsprint(eabuf, eap));
	close(sock);
	return FALSE;
    }
    close(sock);
    return TRUE;
#elif CENV_SYS_XBSD
    /* The new BSD systems completely did away with the ARP ioctls
       and instead substituted a far more complicated PF_ROUTE socket hack.
       Rather than attempt to duplicate the arp(8) utility code here,
       let's try simply invoking it!
       		arp -S <ipaddr> <ethaddr> pub
       Note that NetBSD doesn't support -S yet, only -s.  -S is like -s
       but deletes any existing entry first, avoiding the need for -d
       which is needed on NetBSD to avoid complaints from arp(8).
    */
    FILE *f;
    int err;
    char arpbuff[200];
    char resbuff[200];

    sprintf(arpbuff,
# if CENV_SYS_NETBSD
	    "/usr/sbin/arp -d %s; /usr/sbin/arp -s %s %s %s",
	    ip_adrsprint(ipbuf, ipa),
# else
	    "/usr/sbin/arp -S %s %s %s",
# endif
	    ip_adrsprint(ipbuf, ipa),
	    eth_adrsprint(eabuf, eap),
	    (pubf ? "pub" : ""));
    if (DP_DBGFLG)
	dbprintln("invoking \"%s\"", arpbuff);
    if ((f = popen(arpbuff, "r")) == NULL) {
	syserr(errno, "cannot popen: %s", arpbuff);
	error("Cannot set ARP entry for %s %s",
	      ip_adrsprint(ipbuf, ipa),
	      eth_adrsprint(eabuf, eap));
	return FALSE;
    }
    /* Read resulting output to avoid possibility it might hang otherwise */
    resbuff[0] = '\0';
    (void) fgets(resbuff, sizeof(resbuff)-1, f);
    err = pclose(f);		/* Hope this doesn't wait4() too long */
    if (err) {
	dbprintln("arp exit error: status %d", err);
	dbprintln("arp command was: %s", arpbuff);
    }
    if (DP_DBGFLG)
	dbprintln("arp result \"%s\"", resbuff);
    return TRUE;
#else
    error("Cannot set ARP entry for %s %s - no implementation",
	      ip_adrsprint(ipbuf, ipa),
	      eth_adrsprint(eabuf, eap));
    return FALSE;
#endif
}

/* OSN_IFIPGET - get IP address for a given interface name.
 *	This is separate from osn_ifiplook() in case iftab_init
 *	screws up for some ports.
*/
int
osn_ifipget(int s,		/* Socket for (AF_INET, SOCK_DGRAM, 0) */
	    char *ifnam,	/* Interface name */
	    unsigned char *ipa)	/* Where to write IP address */
{
    int ownsock = FALSE;
    char ipstr[OSN_IPSTRSIZ];
    struct ifreq ifr;

    if (s == -1) {
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	    syserr(errno, "Can't get IP addr for \"%s\": socket()", ifnam);
	    return FALSE;
	}
	ownsock = TRUE;
    }

    /* Find IP address for this interface */
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (ioctl(s, SIOCGIFADDR, (caddr_t *)&ifr) < 0) {
	syserr(errno, "Can't get IP addr for \"%s\": SIOCGIFADDR", ifnam);
	if (ownsock)
	    close(s);
	return FALSE;
    }
    memcpy(ipa,
	   (char *)&(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr),
	   IP_ADRSIZ);

    if (ownsock)
	close(s);
    if (DP_DBGFLG)
	dbprintln("IP address for \"%s\" = %s",
		  ifnam, ip_adrsprint(ipstr, ipa));
    return TRUE;
}

/* OSN_IFNMGET - get IP netmask for a given interface name.
 *	Only needed for IMP, not NI20.
*/
int
osn_ifnmget(int s,		/* Socket for (AF_INET, SOCK_DGRAM, 0) */
	    char *ifnam,	/* Interface name */
	    unsigned char *ipa)	/* Where to write IP netmask */
{
    int ownsock = FALSE;
    char ipstr[OSN_IPSTRSIZ];
    struct ifreq ifr;

    if (s == -1) {
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	    syserr(errno, "Can't get IP netmask for \"%s\": socket()", ifnam);
	    return FALSE;
	}
	ownsock = TRUE;
    }

    /* Find IP address for this interface */
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (ioctl(s, SIOCGIFNETMASK, (caddr_t *)&ifr) < 0) {
	syserr(errno, "Can't get IP netmask for \"%s\": SIOCGIFNETMASK",ifnam);
	if (ownsock)
	    close(s);
	return FALSE;
    }
    memcpy(ipa,
	   (char *)&(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr),
	   IP_ADRSIZ);

    if (ownsock)
	close(s);
    if (DP_DBGFLG)
	dbprintln("IP netmask for \"%s\" = %s",
		  ifnam, ip_adrsprint(ipstr, ipa));
    return TRUE;
}


/* OSN_IFEAGET - get physical ethernet address for a given interface name.
 *
 * This is fairly tricky as the OSD mechanism for this tends to
 * be *very* poorly documented.
 *
 * Apparently only DEC OSF/1 and Linux can find this directly.
 * However, other systems seem to be divided into either of two
 * camps:
 *	4.3BSD: Hack - get the IP address of the interface, then use
 *		use SIOCGARP to look up the hardware address from the
 *		kernel's ARP table.  Clever and gross.
 *	4.4BSD: New regime, no ARP.  But can get it directly as an
 *		AF_LINK family address in the ifconf table.
*/
int
osn_ifeaget(int s,		/* Socket for (AF_INET, SOCK_DGRAM, 0) */
	    char *ifnam,	/* Interface name */
	    unsigned char *eap,	/* Current ether address */
	    unsigned char *def)	/* Default (hardware) ether address */
{
    char eastr[OSN_EASTRSIZ];

#if CENV_SYS_DECOSF		/* Direct approach */
  {
    int ownsock = FALSE;
    struct ifdevea ifdev;

    if (s == -1) {
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	    syserr(errno, "Can't get EN addr for \"%s\": socket()", ifnam);
	    return FALSE;
	}
	ownsock = TRUE;
    }
    strncpy(ifdev.ifr_name, ifnam, sizeof(ifdev.ifr_name));
    if (ioctl(s, SIOCRPHYSADDR, &ifdev) < 0) {
	syserr(errno, "Can't get EN addr for \"%s\": SIOCRPHYSADDR", ifnam);
	if (ownsock) close(s);
	return FALSE;
    }
    if (ownsock)
	close(s);
    ea_set(eap, ifdev.current_pa);
    if (def)
	ea_set(def, ifdev.default_pa);
  }

#elif KLH10_NET_LNX
  {
    int ownsock = FALSE;
    struct ifreq ifr;

    if (s == -1) {
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	    syserr(errno, "Can't get EN addr for \"%s\": socket()", ifnam);
	    return FALSE;
	}
	ownsock = TRUE;
    }
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (ioctl(s, SIOCGIFHWADDR, &ifr) < 0 ) {
	syserr(errno, "SIOCGIFHWADDR of %s failed", ifnam);
	if (ownsock) close(s);
	return FALSE;
    }

    if (ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER) {
	error("%s is not an ethernet - ARPHRD type %d",
	       ifnam, ifr.ifr_hwaddr.sa_family);
	if (ownsock) close(s);
	return FALSE;
    }
    if (ownsock)
	close(s);
    ea_set(eap, &ifr.ifr_addr.sa_data[0]);	/* Return the ether address */
    if (def)
	memset(def, 0, ETHER_ADRSIZ);
  }

#elif NETIF_HAS_ARPIOCTL
  {
    /* Much more general hack */
    unsigned char ipchr[IP_ADRSIZ];
    char ipstr[OSN_IPSTRSIZ];

    /* Get IP address for this interface, as an argument for ARP lookup */
    if (!osn_ifipget(s, ifnam, ipchr)) {
	error("Can't get EN addr for \"%s\": osn_ifipget failed", ifnam);
	return FALSE;
    }

    /* Have IP address, now do ARP lookup hackery */
    if (!osn_arp_look((struct in_addr *)ipchr, eap)) {
	syserr(errno,"Can't find EN addr for \"%s\" %s using ARP",
	       ifnam, ip_adrsprint(ipstr, ipchr));
	return FALSE;
    }
    if (def)
	memset(def, 0, ETHER_ADRSIZ);
  }

#elif NETIF_HAS_ETHLINK
  /* Should never happen unless osn_ifeaget() is called directly
     without going through osn_ifealookup().  If so, attempt
     to do right thing anyway.
  */
  {
    register struct ifent *ife;

    if (iftab_initf == 0) {
	/* Try to initialize iftab, ignore errors */
	(void) osn_iftab_init(IFTAB_ETHS);
    }
    if (!(ife = osn_iflookup(ifnam))
     || !(ife->ife_gotea)) {
	error("No EN addr in iftab for \"%s\"", ifnam);
	return FALSE;
    }
    ea_set(eap, ife->ife_ea);
    if (def)
	memset(def, 0, ETHER_ADRSIZ);
  }

#else
# error "Unimplemented OS routine osn_ifeaget()"
#endif
    dbprintln("EN addr for \"%s\" = %s",
	      ifnam, eth_adrsprint(eastr, eap));
    return TRUE;
}

/* OSN_PFEAGET - get physical ethernet address for an open packetfilter FD.
 *
 * Also not well documented, but generally easier to perform.
*/
int
osn_pfeaget(int pfs,		/* Packetfilter socket or FD */
	    char *ifnam,		/* Interface name (sometimes needed) */
	    unsigned char *eap)	/* Where to write ether address */
{

#if KLH10_NET_NIT
    /* SunOS/Solaris: The EA apparently can't be found until after the PF FD
    ** is open; an attempt to simply do a socket(AF_UNSPEC, SOCK_DGRAM, 0)
    ** will just fail with a protocol-not-supported error.  And using an
    ** AF_INET socket just gets us the IP address, sigh.
    ** Perhaps using AF_DECnet would work, as it does for OSF/1?
    */
    struct ifreq ifr;
    struct strioctl si;		/* Must use kludgy indirect cuz stream */

    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    si.ic_timout = INFTIM;
    si.ic_cmd = SIOCGIFADDR;
    si.ic_len = sizeof ifr;
    si.ic_dp = (char *)&ifr;
    if (ioctl(pfs, I_STR, (char *)&si) < 0) {
	syserr(errno, "ether SIOCGIFADDR failed for \"%s\"", ifnam);
	ea_clr(eap);
	return FALSE;
    }
    ea_set(eap, &ifr.ifr_addr.sa_data[0]);	/* Return the ether address */

#elif KLH10_NET_PFLT	/* Really DECOSF */
    struct endevp endp;

    if (ioctl(pfs, EIOCDEVP, (caddr_t *)&endp) < 0) {
	syserr(errno, "EIOCDEVP failed");
	ea_clr(eap);
	return FALSE;
    }
    if (endp.end_dev_type != ENDT_10MB
	|| endp.end_addr_len != ETHER_ADRSIZ) {
	syserr(errno, "EIOCDEVP returned non-Ethernet info!");
	ea_clr(eap);
	return FALSE;
    }
    ea_set(eap, endp.end_addr);

#elif KLH10_NET_BPF && !CENV_SYS_NETBSD && !CENV_SYS_FREEBSD
    /* NetBSD no longer seems to support this */
    struct ifreq ifr;

    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (ioctl(pfs, SIOCGIFADDR, (char *) &ifr) < 0) {
	syserr(errno, "SIOCGIFADDR for EA failed (%d, \"%s\")",
					pfs, ifnam);
	ea_clr(eap);
	return FALSE;
    }
    ea_set(eap, &ifr.ifr_addr.sa_data[0]);	/* Return the ether address */

#elif KLH10_NET_DLPI
  {
    static int dleaget(int fd, unsigned char *eap);

    /* Use handy auxiliary to hide hideous DLPI details */
    if (!dleaget(pfs, eap))
	return FALSE;
  }
#else
    if (!osn_ifeaget(pfs, ifnam, eap, (unsigned char *)NULL))
	return FALSE;
#endif

    if (DP_DBGFLG) {
	char eastr[OSN_EASTRSIZ];

	dbprintln("EN addr for \"%s\" = %s",
		ifnam, eth_adrsprint(eastr, eap));
    }
    return TRUE;
}

#if CENV_SYS_XBSD
static int osn_pareth(char *cp, unsigned char *adr);
#endif

/* OSN_ARP_LOOK - Attempt looking up ARP information from OS
 *	This may be needed by DPIMP to find the ether addr for an IP address,
 *	or by DPNI20 when trying to determine its own ether address.
 *	In any case, it's a very OS-dependent technique worth encapsulating.
 *
 * NOTE: Failure to find an address should not cause an error printout,
 *	unless there is some actual system glitch.
 */
int
osn_arp_look(struct in_addr *ipa,	/* Look up this IP address */
	     unsigned char *eap)	/* Return EA here */
{
#if NETIF_HAS_ARPIOCTL
    char ipstr[OSN_IPSTRSIZ];
    struct arpreq arq;
    int sock;

    /* Query kernel directly. */
    memset((char *)&arq, 0, sizeof(arq));	/* Clear & set up ARP req */
    arq.arp_pa.sa_family = AF_INET;		/* Protocol addr is IP type */
    arq.arp_ha.sa_family = AF_UNSPEC;		/* Hardware addr is Ether */
    ((struct sockaddr_in *)&arq.arp_pa)->sin_addr = *ipa;

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {	/* Get random socket */
	syserr(errno, "osn_arp_look socket()");
	return FALSE;
    }
    if (ioctl(sock, SIOCGARP, (char *)&arq) != 0) {	/* Get ARP info */
	close(sock);
	/* If SIOCGARP fails, assume it just means the entry wasn't found,
	   rather than implying some error.
	 */
	if (DP_DBGFLG)
	    dbprintln("No ARP info for %s",
		      ip_adrsprint(ipstr, (unsigned char *)ipa));
	return FALSE;
    }
    close(sock);

    /* Won, check flags */
    if (!(arq.arp_flags & ATF_COM)) {
	if (DP_DBGFLG)
	    dbprintln("ARP entry incomplete for %s",
		      ip_adrsprint(ipstr, (unsigned char *)ipa));
	return FALSE;
    }
    ea_set((char *)eap, arq.arp_ha.sa_data);	/* Copy ether addr */
    return TRUE;

#elif CENV_SYS_XBSD
    /* The new BSD stuff did away with the ARP ioctls and substituted an
    ** extremely complicated routing IPC mechanism.  For the time being
    ** I'll just use a horrible hack.
    */
    unsigned char *cp = (unsigned char *)ipa;
    char *arppath = "/usr/sbin/arp";
    char arpbuff[128];
    FILE *f;
    char fhost[100];
    char fip[32];
    char fat[8];
    char fhex[32];
    struct eth_addr etha;
    int res;

    /* Use -n to avoid symbolic lookup hangs */
    sprintf(arpbuff, "%s -n %u.%u.%u.%u", arppath,
		cp[0], cp[1], cp[2], cp[3]);
    if (DP_DBGFLG)
	dbprintln("invoking \"%s\"", arpbuff);
    if ((f = popen(arpbuff, "r")) == NULL) {
	syserr(errno, "cannot popen: %s", arpbuff);
	return FALSE;
    }
    arpbuff[0] = '\0';
    (void) fgets(arpbuff, sizeof(arpbuff)-1, f);
    res = pclose(f);		/* Hope this doesn't wait4() too long */
    if (res)
	dbprintln("arp exit error: status %d", res);
    if (DP_DBGFLG)
	dbprintln("arp result \"%s\"", arpbuff);

    /* Parse the result.  There are three possible formats:
    **	<hostid> (<d.d.d.d>) at <etherhex> <options?>	4+ words
    **	<hostid> (<d.d.d.d>) at (incomplete)	4 words only?
    **	<hostid> (<d.d.d.d>) -- no entry	5 words only
    */
    res = sscanf(arpbuff, "%99s %31s %7s %31s",  fhost, fip, fat, fhex);
    if (res == 4 && (strcmp(fat, "at")==0)
		 && osn_pareth(fhex, (unsigned char *)&etha)) {
	ea_set(eap, &etha);
	return TRUE;
    }

    /* Failed, see if failure is understood or not */
    if (res == 4
      && ( ((strcmp(fat, "at")==0) && strcmp(fhex, "(incomplete)")==0)
	|| ((strcmp(fat, "--")==0) && strcmp(fhex, "no")==0))) {

	/* Failed in a way we understand, so don't complain */
	return FALSE;
    }
    error("osn_arp_look result unparseable: \"%s\"", arpbuff);
    return FALSE;

#else
    error("osn_arp_look not implemented");
    return FALSE;
#endif
}

#if CENV_SYS_XBSD	/* Auxiliary to support code in osn_arp_look() */
static int
osn_pareth(char *cp, unsigned char *adr)
{
    unsigned int b1, b2, b3, b4, b5, b6;
    int cnt;

    cnt = sscanf(cp, "%x:%x:%x:%x:%x:%x", &b1, &b2, &b3, &b4, &b5, &b6);
    if (cnt != 6) {
	/* Later try as single large address #? */
	return FALSE;
    }
    if (b1 > 255 || b2 > 255 || b3 > 255 || b4 > 255 || b5 > 255 || b6 > 255)
	return FALSE;
    *adr++ = b1;
    *adr++ = b2;
    *adr++ = b3;
    *adr++ = b4;
    *adr++ = b5;
    *adr   = b6;
    return TRUE;
}
#endif /* CENV_SYS_XBSD */

#if !OSN_USE_IPONLY	/* This stuff not needed for DPIMP */

/* OSN_IFEASET - Set physical ethernet address for a given interface name.
 *
 * Like osn_ifeaget, also tricky with obscure non-standard mechanisms.
 *
*/
int
osn_ifeaset(int s,		/* Socket for (AF_INET, SOCK_DGRAM, 0) */
	    char *ifnam,	/* Interface name */
	    unsigned char *newpa)	/* New ether address */
{
#if CENV_SYS_DECOSF || KLH10_NET_LNX \
		    || (CENV_SYS_FREEBSD && defined(SIOCSIFLLADDR))

    /* Common preamble code */
    int ownsock = FALSE;
    struct ifreq ifr;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (s == -1) {
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	    syserr(errno, "Failed osn_ifeaset socket()");
	    return FALSE;
	}
	ownsock = TRUE;
    }

# if CENV_SYS_DECOSF		/* Direct approach */
    /* NOTE!!! DECOSF Doc bug!
    ** Contrary to appearances in <sys/ioctl.h where SIOCSPHYSADDR is
    ** said to take a "struct devea" arg, it actually uses an ifreq!
    ** SIOCRPHYSADDR does use devea... barf!
    */
    ifr.ifr_addr.sa_family = AF_DECnet;		/* Odd choice, but works */
    ea_set(ifr.ifr_addr.sa_data, newpa);
    if (ioctl(s, SIOCSPHYSADDR, &ifr) < 0) {
	syserr(errno, "\"%s\" SIOCSPHYSADDR failed", ifnam);
	if (ownsock) close(s);
	return FALSE;
    }

# elif KLH10_NET_LNX

    /* Address family must match what device thinks it is, so find that
       out first... sigh.
    */
    if (ioctl(s, SIOCGIFHWADDR, &ifr) < 0) {
	syserr(errno, "\"%s\" SIOCGIFHWADDR failed", ifnam);
	if (ownsock) close(s);
	return FALSE;
    }
    ea_set(ifr.ifr_addr.sa_data, newpa);	/* Now set new addr */
    if (ioctl(s, SIOCSIFHWADDR, &ifr) < 0) {
	syserr(errno, "\"%s\" SIOCSIFHWADDR failed", ifnam);
	if (ownsock) close(s);
	return FALSE;
    }

# elif CENV_SYS_FREEBSD && defined(SIOCSIFLLADDR)
    /* This works for 4.2 and up; as of 3.3 no known way to set ether addr. */

    ifr.ifr_addr.sa_len = 6;
    ifr.ifr_addr.sa_family = AF_LINK;		/* Must be this */
    ea_set(ifr.ifr_addr.sa_data, newpa);
    if (ioctl(s, SIOCSIFLLADDR, &ifr) < 0) {
	syserr(errno, "\"%s\" SIOCSIFLLADDR failed", ifnam);
	if (ownsock) close(s);
	return FALSE;
    }
# else
#  error "Unimplemented OS routine osn_ifeaset()"
# endif
    /* Common postamble code */
    if (ownsock)
	close(s);
    return TRUE;

#else
# ifdef __GNUC__
#  if CENV_SYS_NETBSD	/* As of 1.6 NetBSD STILL has no way to do this */ 
#   warning "NetBSD still sucks - Unimplemented OS routine osn_ifeaset()"
#  else
#   warning "Unimplemented OS routine osn_ifeaset()"
#  endif
# endif
    error("\"%s\" could not set ether addr - osn_ifeaset() unimplemented",
	  ifnam);
    return FALSE;
#endif
}



/* OSN_IFMCSET - Set ethernet multicast address (add or delete),
 *	or change promiscuous mode.
 *	Hack for now: if "pa" argument is NULL, we are dealing with
 *	promiscuous mode; "delf" is TRUE to turn off, else it's turned on.
 */
int
osn_ifmcset(int s,
	    char *ifnam,
	    int delf,
	    unsigned char *pa)
{
#if CENV_SYS_DECOSF || KLH10_NET_LNX || CENV_SYS_FREEBSD

    /* Common preamble code */
    int ownsock = FALSE;
    struct ifreq ifr;

    if (s == -1) {
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	    syserr(errno, "Failed osn_ifmcset socket()");
	    return FALSE;
	}
	ownsock = TRUE;
    }
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));

# if CENV_SYS_DECOSF
    ifr.ifr_addr.sa_family = AF_DECnet;	/* Known to work; AF_UNSPEC may not */
# elif CENV_SYS_LINUX
    ifr.ifr_addr.sa_family = AF_UNSPEC;	/* MUST be this for Linux! */
# elif CENV_SYS_FREEBSD
    ifr.ifr_addr.sa_family = AF_LINK;	/* MUST be this for FreeBSD! */
# else
#  error "Unimplemented OS routine osn_ifmcset()"
# endif

    if (pa) {
	/* Doing multicast stuff */
	ea_set(ifr.ifr_addr.sa_data, pa);

	if (ioctl(s, (delf ? SIOCDELMULTI : SIOCADDMULTI), &ifr) < 0) {
	    syserr(errno, "\"%s\" %s failed", ifnam,
		   (delf ? "SIOCDELMULTI" : "SIOCADDMULTI"));
	bad:
	    if (ownsock) close(s);
	    return FALSE;
	}
    } else {
	/* Doing promiscuous stuff */
	int flags;

	if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0) {
	    syserr(errno, "SIOCGIFFLAGS failed for interface \"%s\"", ifnam);
	    goto bad;

	}
	if (delf)
	    ifr.ifr_flags &= ~IFF_PROMISC;	/* Turn off */
	else
	    ifr.ifr_flags |= IFF_PROMISC;	/* Turn on */

	if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0) {
	    syserr(errno, "SIOCSIFFLAGS failed for interface \"%s\"", ifnam);
	    goto bad;
	}
    }
    if (ownsock)
	close(s);
    return TRUE;

#else
# ifdef __GNUC__
#  warning "Unimplemented OS routine osn_ifmcset()"
# endif
    error("\"%s\" could not %s multicast addr - osn_ifmcset() unimplemented",
	  ifnam, (delf ? "delete" : "add"));
    return FALSE;
#endif
}

#endif /* !OSN_USE_IPONLY */

/* OSN_PFINIT - Get and initialize file descriptor for packetfilter.
 *	Actual degree to which the PF is initialized is still
 *	very OSD.
 *	FD is always opened for both read/write.
 */

#if KLH10_NET_PFLT || KLH10_NET_BPF

/* Adapted from DEC's pfopen.c - doing it ourselves here because pfopen(3)
 * did not always exist, and this way we can report errors better.
 */

static int
pfopen(void)
{
#if KLH10_NET_PFLT
# define PFDEVPREF "/dev/pf/pfilt"
#elif KLH10_NET_BPF
# define PFDEVPREF "/dev/bpf"
#endif
    char pfname[sizeof(PFDEVPREF)+10];
    int fd;
    int i = 0;

    /* Find first free packetfilter device */
    do {
	(void) sprintf(pfname, "%s%d", PFDEVPREF, i++);
	fd = open(pfname, O_RDWR, 0);
    } while (fd < 0 && errno == EBUSY);	/* If device busy, keep looking */

    if (fd < 0) {
	/* Note possible error meanings:
	   ENOENT - no such filename
	   ENXIO  - not configured in kernel
	*/
	esfatal(1, "Couldn't find or open packetfilter device, last tried %s",
		pfname);
    }
    return fd;		/* Success! */
}

#endif /* KLH10_NET_PFLT || KLH10_NET_BPF */

#if KLH10_NET_PFLT

int
osn_pfinit(struct osnpf *osnpf, void *pfarg)
{
    int fd; 
    int arg;
    unsigned short bits;

    /* Open PF fd.  We'll be doing both R and W. */
    fd = pfopen();		/* Open, will abort if fails  */
    if (osnpf->osnpf_ifnam && osnpf->osnpf_ifnam[0]) {	/* Allow default ifc */
	/* Bind to specified HW interface */
	if (ioctl(fd, EIOCSETIF, osnpf->osnpf_ifnam) < 0)
	    esfatal(1, "Couldn't open packetfilter for \"%s\" - EIOCSETIF",
		    osnpf->osnpf_ifnam);
    }

    /* Set up various mode & flag stuff */
    
    /* Only the super-user can allow promiscuous or copy-all to be set. */
#if 0
    /* Allow promiscuous mode if someone wants to use it.
    ** For shared interface, never true.
    */
    arg = 1;
    if (ioctl(fd, EIOCALLOWPROMISC, &arg))
	esfatal(1, "EIOCALLOWPROMISC failed");	/* Maybe not SU? */
#endif
    /* Allow ENCOPYALL bit to be set.
    ** Always use this for shared interface so the multicast-bit stuff
    ** we grab can also be seen by kernel and other sniffers.
    */
    arg = 1;
    if (ioctl(fd, EIOCALLOWCOPYALL, &arg))
	esfatal(1, "EIOCALLOWCOPYALL failed");	/* Maybe not SU? */


    /* Ensure following bits clear:
    **	no timestamps, no batching, no held signal
    */
    bits = (ENTSTAMP | ENBATCH | ENHOLDSIG);
    if (ioctl(fd, EIOCMBIC, &bits))
	esfatal(1, "EIOCMBIC failed");

    /* Set:
    **	??? Put interface into promisc mode (if allowed by SU)
    **		KLH: No longer -- shouldn't be necessary.
    **	Pass packet along to other filters
    **	Copy packets to/from native kernel ptcls - to allow talking with 
    **		native host platform!
    */
    bits = (/* ENPROMISC | */ ENNONEXCL | ENCOPYALL);
    if (ioctl(fd, EIOCMBIS, &bits))
	esfatal(1, "ioctl: EIOCMBIS");

    /* Set up packet filter for it - only needed if sharing interface */
    if (!osnpf->osnpf_dedic) {
	if (ioctl(fd, EIOCSETF, pfbuild(pfarg, &(osnpf->osnpf_ip.ia_addr))) < 0)
	    esfatal(1, "EIOCSETF failed");
    }

    /* Now can get our interface's ethernet address.
    **	In general, this has to wait until after the packetfilter is opened,
    **	since until then we don't have a handle on the specific interface
    **	that will be used.
    */
    {
	struct endevp endp;

	if (ioctl(fd, EIOCDEVP, (caddr_t *)&endp) < 0)
	    esfatal(1, "EIOCDEVP failed");
	if (endp.end_dev_type != ENDT_10MB
	  || endp.end_addr_len != 6)
	    esfatal(1, "EIOCDEVP returned non-Ethernet info!");
	ea_set(&(osnpf->osnpf_ea), endp.end_addr);
    }

    /* Miscellaneous stuff */

    /* Hack: use timeout mechanism to see if it helps avoid wedging system
    ** when using OSF/1 V3.0.
    */
    if (osnpf->osnpf_rdtmo)
    {
	struct timeval tv;
    
	tv.tv_sec = osnpf->osnpf_rdtmo;
	tv.tv_usec = 0;

	if (ioctl(fd, EIOCSRTIMEOUT, &tv) < 0)
	    esfatal(1, "EIOCSRTIMEOUT failed");
    }

    /* If backlog param was provided, try to set system's idea of how many
    ** input packets can be kept on kernel queue while waiting for us to
    ** read them into user space.
    */
    if (osnpf->osnpf_backlog) {
	if (ioctl(fd, EIOCSETW, &(osnpf->osnpf_backlog)) < 0)
	    esfatal(1, "EIOCSETW failed");
    }

    /* Ready to roll! */
    return fd;
}
#endif /* KLH10_NET_PFLT */

#if KLH10_NET_BPF

int
osn_pfinit(register struct osnpf *osnpf, void *arg)
{
    int fd;
    struct bpf_version bv;
    struct ifreq ifr;
    u_int u;
    int i;
    char *ifnam = osnpf->osnpf_ifnam;

    /* No "default interface" concept here */
    if (!ifnam || !ifnam[0])
	esfatal(1, "Packetfilter interface must be specified");

    /* Open an unused BPF device for R/W */
    fd = pfopen();		/* Will abort if fail */

    /* Check the filter language version number */
    if (ioctl(fd, BIOCVERSION, (char *) &bv) < 0)
	esfatal(1, "kernel BPF interpreter out of date");
    else if (bv.bv_major != BPF_MAJOR_VERSION ||
	     bv.bv_minor < BPF_MINOR_VERSION)
	efatal(1, "requires BPF language %d.%d or higher; kernel is %d.%d",
	       BPF_MAJOR_VERSION, BPF_MINOR_VERSION, bv.bv_major, bv.bv_minor);

    /* Set immediate mode so that packets are processed as they arrive,
       rather than waiting until timeout or buffer full.

       WARNING: NetBSD does not implement this correctly!  The code
       in src/sys/net/bpf.c:bpfread() treats the immediate flag in a way
       that causes it to return EWOULDBLOCK if no input is available.  But
       this flag must still be set in order for bpfpoll() to detect input
       as soon as it arrives!
       See dpni20 and dpimp read loops for workaround.
     */
    i = 1;
    if (ioctl(fd, BIOCIMMEDIATE, (char *) &i) < 0)
	esfatal(1, "BIOCIMMEDIATE failed");

    /* Set the read() buffer size.
       Must be set before interface is attached!
    */
    u = OSN_BPF_MTU;
    if (ioctl(fd, BIOCSBLEN, (char *) &u) < 0)
	esfatal(1, "BIOCSBLEN failed");

    /* Set up packet filter for it - only needed if sharing interface?
       Not sure whether it's needed for a dedicated interface; will need
       to experiment.
     */
    if (!osnpf->osnpf_dedic) {
	struct OSN_PFSTRUCT *pf;

	/* Set the kernel packet filter */
	pf = pfbuild(arg, &(osnpf->osnpf_ip.ia_addr));
	if (ioctl(fd, BIOCSETF, (char *)pf) < 0)
	    esfatal(1, "BIOCSETF failed");
    }


    /* Set read timeout.
       Safety check in order to avoid infinite hangs if something
       wedges up.  The periodic re-check overhead is insignificant.
     */
    /* Set read timeout.
       Safety hack derived from need to use timeout to avoid wedging system
       when using OSF/1 V3.0.  Probably useful for other systems too.
    */
    if (osnpf->osnpf_rdtmo)
    {
	struct timeval tv;
    
	tv.tv_sec = osnpf->osnpf_rdtmo;
	tv.tv_usec = 0;

	if (ioctl(fd, BIOCSRTIMEOUT, (char *) &tv) < 0)
	    esfatal(1, "BIOCSRTIMEOUT failed");
    }

    /* Attach/bind to desired interface device
     */
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (ioctl(fd, BIOCSETIF, (char *) &ifr) < 0)
	esfatal(1, "BIOCSETIF failed for interface \"%s\"", ifnam);

    /* This code only works with Ethernet, so check for that.
       Note cannot check until after interface is attached.
     */
    if (ioctl(fd, BIOCGDLT, (char *) &u) < 0)
	esfatal(1, "BIOCGDLT failed for interface \"%s\"", ifnam);
    if (u != DLT_EN10MB)
	efatal(1, "%s is not an ethernet", ifnam);


    /* Now get our interface's ethernet address.
    **	In general, this has to wait until after the packetfilter is opened,
    **	since until then we don't have a handle on the specific interface
    **	that will be used.
    */
    (void) osn_pfeaget(fd, ifnam, (unsigned char *)&(osnpf->osnpf_ea));

    /* Ready to roll! */
    return fd;
}

#endif /* KLH10_NET_BPF */

#if KLH10_NET_TUN
/*
  In order to use the TUN interface we have to do the equivalent of
	(1) "ifconfig tun0 <localaddr> <destaddr> up"
		as well as
	(2) "arp <destaddr> <localetheraddr> perm pub"
		and finally
	(3) "sysctl -w net.inet.ip.forwarding=1"
		if you want to connect in from other machines besides
		the one the emulator is running on.  This last step must
		still be done externally as it affects overall system
		security.

For (1) the code must flush any existing address, add the new address, and
ensure the interface's IFF_UP flag is set.  Assuming we'll always be running
on *BSD because that's the only system supporting TUN at the moment, the
necessary calls are
		SIOCDIFADDR - Delete existing address
		SIOCAIFADDR - Add existing address (and broadcast and mask)
		SIOCSIFFLAGS - To set IFF_UP

  Some comments about how SIOCAIFADDR and SIOCDIFADDR work, based on
  observation of the FreeBSD code:

  The miniscule doc is in netintro(4).  Use an AF_INET, DGRAM socket.
  Both take:

DELETE takes a struct ifreq (can use an ifaliasreq like the doc claims,
but the SIOCDIFADDR ioctl only defines it as taking an ifreq's worth of
data!).  If the first addr matches an existing one, that one is
deleted.  Otherwise, the first AF_INET address is deleted; INADDR_ANY
from in.h (ie all zeros) is probably the right thing for this case.


ADD takes a new struct so as to set everything at once:

    struct ifaliasreq {
             char    ifra_name[IFNAMSIZ];   // if name, e.g. "en0"
             struct  sockaddr        ifra_addr;
             struct  sockaddr        ifra_broadaddr;
             struct  sockaddr        ifra_mask;
     };

For ADD, the ifra_addr field is the new address to add.
However, the ifra_broadaddr field is actually used in two different ways.
    If the interface has IFF_BROADCAST set, then ifra_broadaddr is the
	broadcast address, as advertised.
    But if it has IFF_POINTOPOINT set, it is interpreted as "ifra_dstaddr";
	the destination (or remote) address.  It is an error if this address
	is INADDR_ANY.
The ifra_mask field is ignored (left as-is or zeroed if new) unless
    ifra_mask.sin_len is non-zero.

 */

int
osn_pfinit(register struct osnpf *osnpf, void *arg)
{
    int allowextern = TRUE;	/* For now, always try for external access */
    int fd;
    char tunname[sizeof "/dev/tun000"];
    char *ifnam = tunname + sizeof("/dev/")-1;
    int i = -1;
    char ipb1[OSN_IPSTRSIZ];
    char ipb2[OSN_IPSTRSIZ];
    struct ifent *ife = NULL;	/* Native host's default IP interface if one */
    struct in_addr iplocal;	/* TUN ifc address at hardware OS end */
    struct in_addr ipremote;	/* Address at remote (emulated host) end */
    static unsigned char ipremset[4] = { 192, 168, 0, 44};

    /* Remote address is always that of emulated machine */
    ipremote = osnpf->osnpf_ip.ia_addr;

    /* Local address is that of hardware machine if we want to permit
       external access.  If not, it doesn't matter (and may not even
       exist, if there is no hardware interface)
    */
    if (allowextern) {
	if (osn_iftab_init(IFTAB_IPS) && (ife = osn_ipdefault())) {
	    iplocal = ife->ife_ipia;
	} else {
	    error("Cannot find default IP interface for host");
	    allowextern = FALSE;
	}
    }
    if (!allowextern) {
	/* Make up bogus IP address for internal use */
	memcpy((char *)&iplocal, ipremset, 4);
    }

    if (DP_DBGFLG)
	dbprint("Opening TUN device");

    do {
	sprintf(tunname, "/dev/tun%d", ++i);
    } while ((fd = open(tunname, O_RDWR)) < 0 && errno == EBUSY);

    if (fd < 0)
	esfatal(1, "Couldn't open tunnel device %s", tunname);

    if (DP_DBGFLG)
	dbprintln("Opened %s, configuring for local %s, remote %s",
	    ifnam,
	    ip_adrsprint(ipb1, (unsigned char *)&iplocal),
	    ip_adrsprint(ipb2, (unsigned char *)&ipremote));


    /* Activate TUN device.
       First address is "local" -- doesn't matter if all we care about is
       talking to the native/hardware host, and it can be set to any of the IP
       address ranges reserved for LAN-only (non-Internet) use, such as
       10.0.0.44.
       However, if planning to allow other machines to access the virtual
       host, probably best to use an address suitable for the same LAN
       subnet as the hardware host.
       Unclear yet whether it works to use the host's own address; it at
       least allows the configuration to happen.

       Second address is "remote" -- the one the emulated host is using.
       It should probably match the same network as the local address,
       especially if planning to connect from other machines.
    */
#if 0	/* Hacky method */
  {
    char cmdbuff[128];
    int res;
    sprintf(cmdbuff, "ifconfig %s %s %s up",
	    ifnam,
	    ip_adrsprint(ipb1, (unsigned char *)&iplocal),
	    ip_adrsprint(ipb2, (unsigned char *)&ipremote));

    if ((res = system(cmdbuff)) != 0) {
	esfatal(1, "osn_pfinit: ifconfig failed to initialize tunnel device?");
    }
  }
#else
  {
    /* Internal method */
    int s;
    struct ifaliasreq ifra;
    struct ifreq ifr;

    if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	esfatal(1, "pf_init: tun socket() failed");
    }

    /* Delete first (only) IP address for this device, if any.
       Ignore errors.
     */
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifnam, sizeof(ifr.ifr_name));
    if (ioctl(s, SIOCDIFADDR, &ifr) < 0) {
	if (DP_DBGFLG)
	    syserr(errno, "osn_pfinit tun SIOCDIFADDR failed");
    }

    memset(&ifra, 0, sizeof(ifra));
    strncpy(ifra.ifra_name, ifnam, sizeof(ifra.ifra_name));
    ((struct sockaddr_in *)(&ifra.ifra_addr))->sin_len = sizeof(struct sockaddr_in);
    ((struct sockaddr_in *)(&ifra.ifra_addr))->sin_family = AF_INET;
    ((struct sockaddr_in *)(&ifra.ifra_addr))->sin_addr   = iplocal;
    ((struct sockaddr_in *)(&ifra.ifra_broadaddr))->sin_len = sizeof(struct sockaddr_in);
    ((struct sockaddr_in *)(&ifra.ifra_broadaddr))->sin_family = AF_INET;
    ((struct sockaddr_in *)(&ifra.ifra_broadaddr))->sin_addr   = ipremote;
    if (ioctl(s, SIOCAIFADDR, &ifra) < 0) {
	esfatal(1, "osn_pfinit tun SIOCAIFADDR failed");
    }

    /* Finally, turn on IFF_UP just in case the above didn't do it.
       Note interface name is still there from the SIOCDIFADDR.
     */
    if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0) {
	esfatal(1, "osn_pfinit tun SIOCGIFFLAGS failed");
    }
    if (!(ifr.ifr_flags & IFF_UP)) {
	ifr.ifr_flags |= IFF_UP;
	if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0) {
	    esfatal(1, "osn_pfinit tun SIOCSIFFLAGS failed");
	}
	if (DP_DBGFLG)
	    dbprint("osn_pfinit tun did SIOCSIFFLAGS");
    }
  }
#endif

    /* Now optionally determine ethernet address.
       This amounts to what if anything we should put in the native
       host's ARP tables.
       - If we only intend to use the net between the virtual host and
       its hardware host, then no ARP hackery is needed.
       - However, if the intent is to allow traffic between the virtual
       host and other machines on the LAN or Internet, then an ARP
       entry is required.  It must advertise the virtual host's IP
       address, using one of the hardware host's ethernet addresses
       so any packets on the LAN for the virtual host will at least
       wind up arriving at the hardware host it's running on.
     */

    /* Simple method is to get the ifconf table and scan it to find the
       first interface with both an IP and ethernet address given.
       Non-4.4BSD systems may not provide the latter datum, but if
       we're using TUN then we almost certainly also have the good stuff
       in ifconf.
    */
    if (allowextern && ife) {
	/* Need to determine ether addr of our default interface, then
	   publish an ARP entry mapping the virtual host to the same
	   ether addr.
	 */
	(void) osn_arp_stuff((unsigned char *)&ipremote, ife->ife_ea, TRUE);

	/* Return that as our ether address */
	ea_set((char *)&osnpf->osnpf_ea, ife->ife_ea);
    } else {
	/* Assume no useful ether addr */
	ea_clr((char *)&osnpf->osnpf_ea);
    }

    if (DP_DBGFLG)
	dbprintln("osn_pfinit tun completed");

    return fd;
}

#endif /* KLH10_NET_TUN */

#if KLH10_NET_LNX

/*
  The Linux PF_PACKET interface is described to some extent
  by the packet(7) man page.

  Linux provides no kernel packet filtering mechanism other than
  possibly a check on the ethernet protocol type, but this is useless
  for us since we'll always want to check for more than just one type;
  e.g. IP and ARP, plus possibly 802.3 or DECNET packets.

  From the man page for packet(7):
       By  default all packets of the specified protocol type are
       passed to a packet socket. To only get packets from a spe-
       cific  interface  use  bind(2)  specifying an address in a
       struct sockaddr_ll to bind the packet socket to an  inter-
       face.  Only  the  sll_protocol and the sll_ifindex address
       fields are used for purposes of binding.
 */
int
osn_pfinit(struct osnpf *osnpf, void *arg)
{
    int fd;
    char *ifcname = osnpf->osnpf_ifnam;
    struct ifreq ifr;

    /* Open a socket of the desired type.
     */
    struct sockaddr_ll sll;
    int ifx;

    /* Get raw packets with ethernet headers
     */
    fd = socket(PF_PACKET, SOCK_RAW,
#if 0 /*OSN_USE_IPONLY*/		/* If DPIMP or otherwise IP only */
		htons(ETH_P_IP)		/* for IP only */
#else
		htons(ETH_P_ALL)	/* for everything */
#endif
		);
    if (fd < 0)
	esfatal(1, "Couldn't open packet socket");

    /* Need ifc index in order to do binding, so get it. */
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifcname, sizeof(ifr.ifr_name));
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0 )
	esfatal(1, "SIOCGIFINDEX of %s failed", ifcname);
    ifx = ifr.ifr_ifindex;

    /* Bind to proper device/interface using ifc index */
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex = ifx;
    if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)))
	esfatal(1, "bind to %s failed", ifcname);

    /* This code only works with Ethernet, so check for that */
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifcname, sizeof(ifr.ifr_name));
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0 )
	esfatal(1, "SIOCGIFHWADDR of %s failed", ifcname);

    if (ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER)
	efatal(1, "%s is not an ethernet - ARPHRD type %d",
	       ifcname, ifr.ifr_hwaddr.sa_family);

    /* Finally, attempt to determine current ethernet MAC address.
       Assume above call returned it in sa_data.
     */
    ea_set(&osnpf->osnpf_ea, &ifr.ifr_addr.sa_data[0]);

    return fd;
}

#endif /* KLH10_NET_LNX */

#if KLH10_NET_NIT

/* NIT packetfilter initialization */

int
osn_pfinit(struct osnpf *osnpf, void *arg)
{
    int fd;
    struct strioctl si;
    struct ifreq ifr;

    /* Open NIT stream.  We'll be doing both R and W. */
    if ((fd = open("/dev/nit", O_RDWR)) < 0)
	esfatal(1, "Couldn't open NIT");

    /* Ensure that each read gives us one packet */
    if (ioctl(fd, I_SRDOPT, (char *)RMSGD) < 0)
	esfatal(1, "I_SRDOPT failed");

    /* Set up kernel filtering */
    if (ioctl(fd, I_PUSH, "pf") < 0)
	esfatal("I_PUSH pf failed");

    /* Set up packet filter for it */
    if (!osnpf->osnpf_dedic) {
	struct OSN_PFSTRUCT *pf;
	pf = pfbuild(arg, &osnpf->osnpf_ip.ia_addr);
	si.ic_timout = INFTIM;		/* Should this be osnpf_rdtmo? */
	si.ic_cmd = NIOCSETF;		/* Set packet filter */
	si.ic_len = sizeof(*pf);	/* XXX Unfortunate dependency */
	si.ic_dp = (char *)pf;
	if (ioctl(fd, I_STR, (char *)&si) < 0)
	    esfatal(1, "NIOCSETF failed");
    }

    /* Finally, bind to proper device and flush anything accumulated */
    strncpy(ifr.ifr_name, osnpf->osnpf_ifnam, sizeof(ifr.ifr_name));
    si.ic_cmd = NIOCBIND;
    si.ic_len = sizeof(ifr);
    si.ic_dp = (char *)&ifr;
    if (ioctl(fd, I_STR, (char *)&si) < 0)
	esfatal(1, "NIOCBIND failed");

    if (ioctl(fd, I_FLUSH, (char *)FLUSHR) < 0)
	esfatal(1, "I_FLUSH failed");

    /* Get our ethernet address.
    **	This can't be done until after the NIT is open and bound.
    */
    (void) osn_pfeaget(fd, osnpf->osnpf_ifnam, &(osnpf->osnpf_ea));

    /* Ready to roll! */
    return fd;
}
#endif /* KLH10_NET_NIT */

#if KLH10_NET_DLPI

/* DLPI packetfilter initialization */

#define	MAXSIZE   (32 * 1024)	/* Whuffo? */
#define	MAXADDR	  1024
#define	MAXCTLBUF (sizeof (dl_unitdata_req_t) + sizeof (long) + MAXADDR)
#define	MAXDLBUF  8192		/* Maybe just MAXCTLBUF */

struct  ledladdr {
	struct  ether_addr  dl_phys;
	unsigned short dl_sap;
};

struct dlpictx {
	int  dc_fd;
	struct strbuf dc_ctl;
	char dc_ebuf[200];		/* Error output */
	long dc_buf[MAXDLBUF];
};

/* Fake SAP must be above 1500 to avoid 802.3 interpretation, but
** might be dangerous if it were 2048 which is IP (try that later, tho)
*/
#define FAKESAP 2049

#if OSN_USE_IPONLY
static int dlfastpathon(struct dlpictx *dc, int sap);
#endif

static int strioctl(int fd, int cmd, int len, char *dp);
static int dl_sendreq(struct dlpictx *dc, char *ptr, int len, char *what);
static int strgetmsg(struct dlpictx *dc, char *caller, struct strbuf *datap,
		     int *flagsp);
static int dlattachreq(struct dlpictx *dc, long ppa);
static int dlokack(struct dlpictx *dc);
static int dlbindreq(struct dlpictx *dc, u_long sap, u_long max_conind,
		     unsigned short service_mode, unsigned short conn_mgmt);
static int dlbindack(struct dlpictx *dc);
static int dlpromisconreq(struct dlpictx *dc, u_long level);
static int dladdrreq(struct dlpictx *dc, int type);
static int dladdrack(struct dlpictx *dc, unsigned char *addr);
static int dlsetaddrreq(struct dlpictx *dc, unsigned char *addr);
#if 0
static int dlpromiscoff(struct dlpictx *dc);
static int dlinforeq(struct dlpictx *dc);
static int dlinfoack(struct dlpictx *dc);
static int dlenabmulti(struct dlpictx *dc, char	*addrp, int len);
static int dldisabmulti(struct dlpictx *dc, char *addrp, int len);
static int dldetachreq(struct dlpictx *dc);
static int dlunbindreq(struct dlpictx *dc);
#endif


int osn_pfinit(struct osnpf *osnpf, void *arg)
{
    int fd;
    int ppa;
    char *cp;
    unsigned char curea[ETHER_ADRSIZ];
    char devpath[IFNAMSIZ+40];
    char eastr[OSN_EASTRSIZ];
    unsigned char *ucp;
    struct dlpictx dc;

    /* Figure out right name of device to use.
    ** Munch device spec, eg "hmeNN" becomes /dev/hme, unit NN.
    ** Code formerly allowed full pathname as device spec, but no longer.
    */
    if (osnpf->osnpf_ifnam[0] == 0)
	efatal(1, "Ethernet DLPI interface must be specified");
    strcpy(devpath, "/dev/");
    strcat(devpath, osnpf->osnpf_ifnam); /* Buffer always big enough */
    ppa = 0;				/* Assume unit #0 */
    cp = &devpath[strlen(devpath)-1];	/* Point to end of string */
    if (isdigit(*cp)) {			/* Go to start of unit digits */
	while (isdigit(*--cp));		/* (code safe cuz of /dev/ prefix) */
	ppa = atoi(++cp);
	*cp = '\0';			/* Chop off unit digits */
    }

    /* Open device.  We'll be doing both R and W. */
    if ((fd = open(devpath, O_RDWR)) < 0) {
	esfatal(1, "Couldn't open DLPI packetfilter for ifc \"%s\" (%s)",
			devpath, osnpf->osnpf_ifnam);
    }
    memset((void *)&dc, 0, sizeof(dc));
    dc.dc_fd = fd;

    /* Attach to specific unit */
    if (dlattachreq(&dc, (long)ppa)
     || dlokack(&dc))
	efatal(1, dc.dc_ebuf);

    /* Bind */
#if OSN_USE_IPONLY	/* Note using IP SAP */
    if (dlbindreq(&dc, 0x800, 0, DL_CLDLS, 0)
#else			/* Note using fake SAP to avoid Solaris lossage */
    if (dlbindreq(&dc, FAKESAP, 0, DL_CLDLS, 0)
#endif
     || dlbindack(&dc))
	efatal(1, dc.dc_ebuf);

#if OSN_USE_IPONLY	/* Apparently only needed for this */
    /* Do stuff for "fastpath" which may be needed to allow header to
       be included in data buffer rather than separate control struct.
    */
    if (dlfastpathon(&dc, 0) < 0)
	efatal(1, dc.dc_ebuf);
#elif 0 /* !OSN_USE_IPONLY */	/* Apparently not needed */
    if (dlfastpathon(&dc, FAKESAP) < 0)
	efatal(1, dc.dc_ebuf);
#endif

    /* Set up various mode & flag stuff */

    /* Do input side of DLPI stream */

#if CENV_SYS_SOLARIS
    /* Turn on raw receive path, so that ethernet header is included in data.
    ** This is a special Solaris frob.
    */
    if (strioctl(fd, DLIOCRAW, 0, NULL) < 0) {
	esfatal(1, "DLIOCRAW ioctl");
    }
#endif

#if !OSN_USE_IPONLY
    /* Don't need this hackery if only want IP packets */
    /* Enable receiving all packets, all SAPs (ethernet header type values) */

# if CENV_SYS_SOLARIS
    /* Enable promiscuous mode.  On Solaris, this is required in order
    ** for ALLSAP to work!!!  Gross bletcherous misfeatured bug!!!
    */
    if ((dlpromisconreq(&dc, DL_PROMISC_PHYS) < 0)
	|| dlokack(&dc) < 0)
	    efatal(1, dc.dc_ebuf);
# endif
    /* Evidently must explicitly ask for promiscuous SAPs */
    if (dlpromisconreq(&dc, DL_PROMISC_SAP) < 0
	|| dlokack(&dc) < 0)
	    efatal(1, dc.dc_ebuf);
    /* And multicast too!?  To quote tcpdump,
    ** "you would have thought promiscuous would be sufficient"
    */
    if (dlpromisconreq(&dc, DL_PROMISC_MULTI) < 0
	|| dlokack(&dc) < 0)
	    efatal(1, dc.dc_ebuf);
#endif /* !OSN_USE_IPONLY */

    /* Find the physical ethernet address of the interface we got.
    ** Do it now, because it may be needed in order to set up the
    ** correct packet filter (sigh).
    */
    if (dladdrreq(&dc, DL_CURR_PHYS_ADDR) < 0
	|| dladdrack(&dc, (unsigned char *)curea) < 0)
	    efatal(1, dc.dc_ebuf);

    /* HACK HACK -- see if ethernet addr already given, and if so,
    ** try to set it if different.
    */
    if (!ea_isclr(&osnpf->osnpf_ea)
      && (ea_cmp(&osnpf->osnpf_ea, curea) != 0)) {
	char old[OSN_EASTRSIZ];
	char new[OSN_EASTRSIZ];

	/* Attempt to set our EN addr */
	eth_adrsprint(old, (unsigned char *)curea);
	eth_adrsprint(new, (unsigned char *)&osnpf->osnpf_ea);

	if (dlsetaddrreq(&dc, (unsigned char *)&osnpf->osnpf_ea) < 0
	    || dlokack(&dc) < 0)
		efatal(1, dc.dc_ebuf);

	/* Double-check by fetching new addr again and using it */
	if (dladdrreq(&dc, DL_CURR_PHYS_ADDR) < 0
	    || dladdrack(&dc, (unsigned char *)curea) < 0)
		efatal(1, dc.dc_ebuf);

	if (ea_cmp(&osnpf->osnpf_ea, curea) == 0) {
	    dbprintln("\"%s\" E/N addr changed: Old=%s New=%s",
		      osnpf->osnpf_ifnam, old, new);
	} else {
	    dbprintln("\"%s\" E/N addr change failed, Old=%s New=%s",
		      osnpf->osnpf_ifnam, old, new);
	}
    }
    ea_set(&osnpf->osnpf_ea, curea);

#if 0
    /* Ensure that each read gives us one packet.
    ** Shouldn't matter since we use getmsg(), but code left here in case.
    */
    if (ioctl(fd, I_SRDOPT, (char *)RMSGD) < 0)
	esfatal(1, "I_SRDOPT failed");
#endif

    /* Set up packet filter for it - should only be needed if
    ** sharing interface, but on Solaris we may be always in promiscuous
    ** mode!
    */
#if !CENV_SYS_SOLARIS
    if (!osnpf->osnpf_dedic)
#endif
    {
	struct OSN_PFSTRUCT *pf;

	if (ioctl(fd, I_PUSH, "pfmod") < 0)
	    esfatal(1, "PUSH of pfmod failed");

#if !OSN_USE_IPONLY
	if (osnpf->osnpf_dedic)
	    /* Filter on our ether addr */
	    pf = pfeabuild(arg, (unsigned char *)&osnpf->osnpf_ea);
	else
#endif
	    /* Filter on our IP addr */
	    pf = pfbuild(arg, &osnpf->osnpf_ip.ia_addr);

	if (strioctl(dc.dc_fd, PFIOCSETF, sizeof(*pf), (char *)pf) < 0)
	    esfatal(1, "PFIOCSETF failed");
    }

    /* Needed?  Flush read side to ensure clear of anything accumulated */
    if (ioctl(fd, I_FLUSH, (char *)FLUSHR) < 0)
	esfatal(1, "I_FLUSH failed");

    /* Ready to roll! */
    return fd;
}

/* Handy auxiliary to pick up EA address given the PF FD, in case
   it's needed again after osn_pfinit is done.
 */
static int
dleaget(int fd, unsigned char *eap)
{
    struct dlpictx dc;

    dc.dc_fd = fd;
    if ((dladdrreq(&dc, DL_CURR_PHYS_ADDR) < 0)
     || (dladdrack(&dc, (unsigned char *)eap) < 0)) {
	error(dc.dc_ebuf);
	ea_clr(eap);
	return FALSE;
    }
    return TRUE;
}

#if OSN_USE_IPONLY	/* Apparently only needed for this */
static int
dlfastpathon(struct dlpictx *dc, int sap)
{
    dl_unitdata_req_t *req;
    struct  ledladdr {
	    struct  ether_addr  dl_phys;
	    unsigned short dl_sap;
    } *dladdrp;
    int	n;

    /* Construct DL_UNITDATA_REQ primitive. */
    req = (dl_unitdata_req_t *) dc->dc_buf;
    req->dl_primitive = DL_UNITDATA_REQ;
    req->dl_dest_addr_length = sizeof (short) + ETHER_ADRSIZ;
    req->dl_dest_addr_offset = sizeof (dl_unitdata_req_t);
    req->dl_priority.dl_min = 0;
    req->dl_priority.dl_max = 0;

    /* Set up addr - instead of a specific dest address, just clear it out */
    dladdrp = (struct ledladdr *)
			(((char *)req) + req->dl_dest_addr_offset);
    dladdrp->dl_sap = sap;
    memset(&dladdrp->dl_phys, 0, ETHER_ADRSIZ);

    if ((n = strioctl(dc->dc_fd, DL_IOC_HDR_INFO,
		      sizeof(*req)+sizeof(*dladdrp),
		      (char *)req)) < 0) {
	esfatal(1, "DL_IOC_HDR_INFO ioctl failed");
    }
    return n;
}
#endif /* OSN_USE_IPONLY */


/* Derived variously from TCPDUMP and Sun's ugly DLTX sample program
 */

static int
strioctl(int fd, int cmd, int len, char *dp)
{
    struct strioctl str;
    int rc;

    str.ic_cmd = cmd;
    str.ic_timout = INFTIM;
    str.ic_len = len;
    str.ic_dp = dp;
    rc = ioctl(fd, I_STR, &str);

    return (rc < 0) ? rc : str.ic_len;
}

static int
dl_sendreq(struct dlpictx *dc, char *ptr, int len, char *what)
{
	struct	strbuf	ctl;
	int	flags = 0;

	ctl.maxlen = 0;
	ctl.len = len;
	ctl.buf = ptr;

	if (putmsg(dc->dc_fd, &ctl, (struct strbuf *) NULL, flags) < 0) {
		sprintf(dc->dc_ebuf, "%s putmsg failed: %s",
		    what, dp_strerror(errno));
		return (-1);
	}
	return (0);
}

#if OSN_USE_IPONLY
# define DL_MAXWAIT 0
#else
# define DL_MAXWAIT 15	/* secs to wait for ACK msg */
#endif

#if DL_MAXWAIT

#include <signal.h>

static void
sigalrm(s)
{
}
#endif

static int
strgetmsg(struct dlpictx *dc,
	  char *caller,
	  struct strbuf *datap,
	  int *flagsp)
{
    int	rc;
    int flags = 0;

    dc->dc_ctl.maxlen = MAXDLBUF;
    dc->dc_ctl.len = 0;
    dc->dc_ctl.buf = (char *)dc->dc_buf;
    if (flagsp)
	*flagsp = 0;
    else
        flagsp = &flags;

#if DL_MAXWAIT
    signal(SIGALRM, sigalrm);
    if (alarm(DL_MAXWAIT) < 0) {
	esfatal(1, "%s: alarm set", caller);
    }
#endif

    /* Get expected message */
    if ((rc = getmsg(dc->dc_fd, &dc->dc_ctl, datap, flagsp)) < 0) {
	sprintf(dc->dc_ebuf, "%s:  getmsg - %s",
		caller, dp_strerror(errno));
	return -1;
    }

#if DL_MAXWAIT
    if (alarm(0) < 0) {
	esfatal(1, "%s: alarm clear", caller);
    }
#endif

    /* Got it - now do paranoia check */
    if (rc & (MORECTL | MOREDATA)) {
	sprintf(dc->dc_ebuf, "%s: getmsg returned%s%s",
		caller,
		((rc & MORECTL)  ? " MORECTL"  : ""),
		((rc & MOREDATA) ? " MOREDATA" : ""));
	return -1;
    }

    /* Yet more paranoia - control info should exist */
    /* Take this out if all callers check length */
    if (dc->dc_ctl.len < sizeof (long)) {
	sprintf(dc->dc_ebuf, "%s: getmsg ctl.len too short: %d",
		dc->dc_ctl.len);
	return -1;
    }
    return 0;
}
#undef DL_MAXWAIT

static int
dlattachreq(struct dlpictx *dc, long ppa)
{
    dl_attach_req_t req;

    req.dl_primitive = DL_ATTACH_REQ;
    req.dl_ppa = ppa;
    return dl_sendreq(dc, (char *)&req, sizeof(req), "dlattach");
}

static int
dlokack(struct dlpictx *dc)
{
    union DL_primitives *dlp;

    if (strgetmsg(dc, "dlokack", NULL, NULL) < 0)
	return -1;

    dlp = (union DL_primitives *) dc->dc_ctl.buf;

    if (dlp->dl_primitive != DL_OK_ACK) {
	sprintf(dc->dc_ebuf, "dlokack unexpected primitive %0lX",
	    (long)dlp->dl_primitive);
	return -1;
#if 0
	    /* Possibly insert general error handler (DLTX) */
	if (dlp->dl_primitive == DL_ERROR_ACK)
	    dlerror(dlp->error_ack.dl_errno);
	else
	    printdlprim(dlp);
#endif
    }

    if (dc->dc_ctl.len != sizeof(dl_ok_ack_t)) {
	sprintf(dc->dc_ebuf, "dlokack incorrect size %ld",
	    (long)dc->dc_ctl.len);
	return -1;
    }
    return 0;
}

static int
dlbindreq(struct dlpictx *dc,
	  u_long sap,
	  u_long max_conind,
	  unsigned short service_mode,
	  unsigned short conn_mgmt)
{
    dl_bind_req_t	req;

    req.dl_primitive = DL_BIND_REQ;
    req.dl_sap = sap;
    req.dl_max_conind = max_conind;
    req.dl_service_mode = service_mode;
    req.dl_conn_mgmt = conn_mgmt;
    req.dl_xidtest_flg = 0;

    return dl_sendreq(dc, (char *)&req, sizeof(req), "dlbind");
}

static int
dlbindack(struct dlpictx *dc)
{
    union DL_primitives *dlp;

    if (strgetmsg(dc, "dlbindack", NULL, NULL) < 0)
	return -1;

    dlp = (union DL_primitives *) dc->dc_ctl.buf;

    if (dlp->dl_primitive != DL_BIND_ACK) {
	sprintf(dc->dc_ebuf, "dlbindack unexpected response %0lX",
		(long)dlp->dl_primitive);
	return -1;
    }

    if (dc->dc_ctl.len < sizeof (dl_bind_ack_t)) {
	sprintf(dc->dc_ebuf, "dlbindack: short response: %d",
		dc->dc_ctl.len);
	return -1;
    }
#if 0	/* Don't understand this */
    if (flags != RS_HIPRI) {
	sprintf(dc->dc_ebuf, "dlbindack:  DL_OK_ACK was not M_PCPROTO");
	return -1;
    }
#endif
    return 0;
}


static int
dlpromisconreq(struct dlpictx *dc, u_long level)
{
    dl_promiscon_req_t req;

    req.dl_primitive = DL_PROMISCON_REQ;
    req.dl_level = level;
    return dl_sendreq(dc, (char *)&req, sizeof(req), "dlpromiscon");
}

/*
 * type arg: DL_FACT_PHYS_ADDR for "factory" address,
 *	     DL_CURR_PHYS_ADDR for actual current address.
 */
static int
dladdrreq(struct dlpictx *dc, int type)
{
    dl_phys_addr_req_t req;

    req.dl_primitive = DL_PHYS_ADDR_REQ;
    req.dl_addr_type = type;		/* DL_{FACT,CURR}_PHYS_ADDR */
    return dl_sendreq(dc, (char *)&req, sizeof(req), "dladdr");
}

static int
dladdrack(struct dlpictx *dc, unsigned char *addr)
{
    union DL_primitives	*dlp;
    unsigned char *ucp;
    int len;

    if (strgetmsg(dc, "dladdrack", NULL, NULL) < 0)
	return -1;

    dlp = (union DL_primitives *) dc->dc_ctl.buf;

    if (dlp->dl_primitive != DL_PHYS_ADDR_ACK) {
	sprintf(dc->dc_ebuf, "dladdrack unexpected response %0lX",
			    (long)dlp->dl_primitive);
	return -1;
    }
    if (dc->dc_ctl.len < sizeof (dl_phys_addr_ack_t)) {
	sprintf(dc->dc_ebuf, "dladdrack: short response: %d",
		dc->dc_ctl.len);
	return -1;
    }

    /* Hardwired paranoia check, assume ethernet for now */
    len = dlp->physaddr_ack.dl_addr_length;
    if (len != ETHER_ADRSIZ) {
	sprintf(dc->dc_ebuf, "dladdrack: non-Ether addr len %d", len);
	return (-1);
    }
    ucp = ((unsigned char *)dlp) + dlp->physaddr_ack.dl_addr_offset;
    memcpy(addr, ucp, (size_t)len);

    return 0;
}

static int
dlsetaddrreq(struct dlpictx *dc, unsigned char *addr)
{
    /* Request is bigger than struct def; overlay it in buffer */
    dl_set_phys_addr_req_t *req = (dl_set_phys_addr_req_t *)dc->dc_buf;

    req->dl_primitive = DL_SET_PHYS_ADDR_REQ;
    req->dl_addr_length = ETHER_ADRSIZ;
    req->dl_addr_offset = sizeof(dl_set_phys_addr_req_t);
    memcpy(&dc->dc_buf[req->dl_addr_offset], addr, ETHER_ADRSIZ);

    return dl_sendreq(dc, (char *)req, sizeof(*req) + ETHER_ADRSIZ,
			"dlsetaddr");
}

#endif /* KLH10_NET_DLPI */

#if 0 /* KLH10_NET_DLPI */

/* DLPI functions not currently used, but kept around in case
 */

/* Turn off promiscuous mode
 */
static int
dlpromiscoff(struct dlpictx *dc)
{
    dl_promiscoff_req_t	req;

    req.dl_primitive = DL_PROMISCOFF_REQ;
    return dl_sendreq(dc, (char *)&req, sizeof(req), "dlpromiscoff");
}


static int
dlinforeq(struct dlpictx *dc)
{
    dl_info_req_t req;

    req.dl_primitive = DL_INFO_REQ;
    return (dl_sendreq(dc, (char *)&req, sizeof(req), "info"));
}

static int
dlinfoack(struct dlpictx *dc)
{
    union DL_primitives *dlp;

    if (strgetmsg(dc, "dlinfoack", NULL, NULL) < 0)
	return -1;

    dlp = (union DL_primitives *) dc->dc_ctl.buf;
    if (dlp->dl_primitive != DL_INFO_ACK) {
	sprintf(dc->dc_ebuf, "dlinfoack unexpected primitive %ld",
			(long)dlp->dl_primitive);
	return -1;
    }

    /* Extra stuff like the broadcast address can be returned */
    if (dc->dc_ctl.len < DL_INFO_ACK_SIZE) {
	sprintf(dc->dc_ebuf, "dlinfoack: incorrect size %ld",
		(long)dc->dc_ctl.len);
	return -1;
    }
    return 0;
}

static int
dlenabmulti(struct dlpictx *dc,
	   char	*addrp,
	   int	len)
{
    dl_enabmulti_req_t	*req;

    req = (dl_enabmulti_req_t *) dc->dc_buf;
    req->dl_primitive = DL_ENABMULTI_REQ;
    req->dl_addr_length = len;
    req->dl_addr_offset = sizeof (dl_enabmulti_req_t);
    memcpy((dc->dc_buf + sizeof(dl_enabmulti_req_t)), addrp, len);

    return (dl_sendreq(dc, (char *)req, sizeof(*req)+len, "dlenabmulti"));
}

static int
dldisabmulti(struct dlpictx *dc,
	   char	*addrp,
	   int	len)
{
    dl_disabmulti_req_t	*req;

    req = (dl_disabmulti_req_t *) dc->dc_buf;
    req->dl_primitive = DL_DISABMULTI_REQ;
    req->dl_addr_length = len;
    req->dl_addr_offset = sizeof (dl_disabmulti_req_t);
    memcpy((dc->dc_buf + sizeof(dl_disabmulti_req_t)), addrp, len);

    return (dl_sendreq(dc, (char *)req, sizeof(*req)+len, "dldisabmulti"));
}

static int
dldetachreq(struct dlpictx *dc)
{
    dl_detach_req_t req;

    req.dl_primitive = DL_DETACH_REQ;
    return (dl_sendreq(dc, (char *)&req, sizeof(req), "dldetach"));
}

static int
dlunbindreq(struct dlpictx *dc)
{
    dl_unbind_req_t req;

    req.dl_primitive = DL_UNBIND_REQ;
    return (dl_sendreq(dc, (char *)&req, sizeof(req), "dlunbind"));
}

#endif /* KLH10_NET_DLPI */

/* Auxiliary OSDNET.C stuff stops here */

#if 0	/* Limit of included code so far */

/* Both need to dump packets out
 */
void
dumppkt(unsigned char *ucp, int cnt)
{
    int i;

    while (cnt > 0) {
	for (i = 8; --i >= 0 && cnt > 0;) {
	    if (--cnt >= 0)
		fprintf(stderr, "  %02x", *ucp++);
	    if (--cnt >= 0)
		fprintf(stderr, "%02x", *ucp++);
	}
	fprintf(stderr, "\r\n");
    }
}

/* Merge DPNI20's arp_myreply() with DPIMP's arp_req() ??
   But one sends out a reply, the other a request...
 */

/* There was an ether_write() for NIT unused in dpni20.c
  whereas dpimp.c still uses it for everything.
*/

#endif /* if 0 - still-excluded code */
