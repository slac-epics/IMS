#include<stdio.h>
#include<stdlib.h>
#include<sys/types.h>
#include<sys/wait.h>
#include<sys/stat.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include<net/if.h>
#include<netinet/in.h>
#include<signal.h>
#include<string.h>
#include<fcntl.h>
#include<errno.h>
#include<ctype.h>

int now = 1;
struct varlist {
    char *name;
    char *val;
    struct varlist *next;
} *vl = NULL;

struct varlist *findvar(char *name)
{
    struct varlist *l;
    char lcn[3];
    lcn[0] = tolower(name[0]);
    if (name[1]) {
	lcn[1] = tolower(name[1]);
	lcn[2] = 0;
    } else
	lcn[1] = 0;
    
    for (l = vl; l != NULL; l = l->next) {
	if (!strcmp(l->name, lcn))
	    return l;
    }
    l = (struct varlist *)calloc(1, sizeof(struct varlist));
    l->name = strdup(lcn);
    l->val = strdup("");
    l->next = vl;
    vl = l;
    return vl;
}

void setvar(char *name, char *val)
{
    struct varlist *l = findvar(name);
    free(l->val);
    l->val = strdup(val);
    printf("Set %s to |%s|\n", name, val);
}

void setvarint(char *name, int val)
{
    char buf[256];
    sprintf(buf, "%d", val);
    setvar(name, buf);
}

int getvarint(char *name)
{
    return atoi(findvar(name)->val);
}

char *getvar(char *name)
{
    if (!strcmp(name, "P"))
	return getvarint("EE") ? getvar("C2") : getvar("C1");
    return findvar(name)->val;
}

char *dopr(char *cmd)
{
#define BUFSIZE 1024
    static char buf[BUFSIZE+1];
    char *s = cmd, *t = buf;
    int n = 0;
    while (*s && n < BUFSIZE) {
	if (*s == '\"') {
	    /* Literal string! */
	    s++;
	    while (*s && *s != '\"' && n < BUFSIZE) {
		*t++ = *s++;
		n++;
	    }
	    if (*s) /* Skip the final quote */
		s++;
	    if (*s == ',') /* Skip the comma, if one. */
		s++;
	} else {
	    /* Variable name */
	    char name[3], *v;
	    name[0] = *s++;
	    if (*s && *s != ',') {
		name[1] = *s++;
		name[2] = 0;
		while (*s && *s != ',')
		    s++;
	    } else {
		name[1] = 0;
	    }
	    v = getvar(name);
	    while (*v && n < BUFSIZE) {
		*t++ = *v++;
		n++;
	    } 
	    if (*s == ',')
		s++;
	}
    }
    *t = 0;
    return buf;
}

void childsig(int sig)
{
    int status;
    while (waitpid(-1, &status, WNOHANG) > 0);
}

void alarmsig(int sig)
{
    now++;
    alarm(1);
}

#define MOVE_DURATION   10
#define SAVE_DURATION   100
#define STATUS_DURATION 12
#define STEP_PER_ELC    25

void serve()
{
    char buf[1024], *s, *t, *u;
    int n;
    struct sigaction temp;
    int status_offset = 0;
    int movetime = 0, updatetime = 0,
	curC1, incC1, finalC1,
	curC2, incC2, finalC2;

    /* Setup some standard variables. */
    setvar("MV", "0");       /* Moving */
    setvar("EE", "1");       /* Encoder enable */
    setvar("C1", "-340980"); /* Motor steps */
    setvar("C2", "-13598");  /* Encoder counts */
    setvar("SV", "0");       /* Save flag: 0=no, 1=Want2Save, 9=request */
    setvar("NS", "0");       /* Number of saves. */
    setvar("PU", "1");       /* Power up */
    setvar("BY", "1");       /* Executing */
    setvar("NE", "0");       /* Numeric enable */
    setvar("TE", "0");       /* Trip enable */
    setvar("ST", "0");       /* Stall */
    setvar("ER", "0");       /* Error code */
    setvar("I1", "0");
    setvar("I2", "0");
    setvar("I4", "0");
    setvar("SM", "1");       /* Stall detection mode */
    setvar("PN", "MDI3CRL23C7-EQ");
    setvar("SN", "097150354");
    setvar("VR", "3.014");
    setvar("S1", "2, 1, 0");
    setvar("S2", "3, 1, 0");
    setvar("S3", "16, 1, 1");
    setvar("S4", "0, 0, 0");
    setvar("S9", "17, 0, 0");
    setvar("MS", "256");
    setvar("VE", "4");
    setvar("ME", "0");       /* Monitor encoder: status position is C2 if 1, C1 if 0 */
    setvar("EL", "512");     /* encoder lines per revolution.  C2 = 4*EL! */
    setvar("ES", "1");       /* Escape mode: Sending escape stops motion and program. */
    /*
      Behaviors to simulate:
      - Periodically set SV to 1 and send "Want2Save".  Actual 450s, maybe 100s?
      - If SV is set to 9, set SV to 0, increment NS and send "Saved ",P.
      - Periodically send status: "BOSx,P=yEOS" where
      y is the position (C1 or C2 depending on ME|EE).
      x is the status:
      bit-16   is NE
      bit-15   is PU
      bit-14   is TE
      bit-13   is ST
      bit-12-6 is ER
      bit-5    is I4
      bit-4    is I2
      bit-3    is I1
      bit-2    is SM
      bit-1    is EE
      bit-0    is MV.
      Maybe send this every 10s if not moving, every 1s if moving?
      - If ESC is received, set BY to 0 and stop motion.
      - EX 1 actually sets BY = 1.
      - MA is move absolute and MR is move relative.  EE controls if this is
        in encoder or motor counts.  The steps per encoder count is 25.
	Let's just say all motions are 10s, reporting every second, moving
	1/10th of the distance each second.
      - P is the position, either C1 or C2 depending on EE.
    */

    /* Setup our timer. */
    sigaction(SIGALRM, NULL, &temp);
    temp.sa_handler = alarmsig;
    sigemptyset(&temp.sa_mask);
    temp.sa_flags &= ~SA_RESTART;
    sigaction(SIGALRM, &temp, NULL);
    alarm(1);
    for (;;) {
	errno = 0;
	n = read(0, buf, sizeof(buf)-1);
	buf[n] = 0;
	/* OK, before we get to the result of the read... anything to do? */
	if (now % SAVE_DURATION == 0 && !getvarint("SV")) {
	    setvar("SV", "1");
	    printf("> Want2Save\r\n");
	    fprintf(stderr, "Want2Save\r\n");
	}
	if (now % STATUS_DURATION == status_offset || movetime || updatetime) {
	    int pos;
	    int ee = getvarint("EE");
	    int status = ((getvarint("NE") << 16) |
			  (getvarint("PU") << 15) |
			  ((getvarint("TE")/4) << 14) | /* TE is either 4 or 0 */
			  (getvarint("ST") << 13) |
			  (getvarint("ER") << 6) |
			  (getvarint("I4") << 5) |
			  (getvarint("I2") << 4) |
			  (getvarint("I1") << 3) |
			  (getvarint("SM") << 2) |
			  (ee << 1));
	    if (movetime) {
		movetime--;
		curC1 = movetime ? (curC1 + incC1) : finalC1;
		curC2 = movetime ? (curC2 + incC2) : finalC2;
		setvarint("C1", curC1);
		setvarint("C2", curC2);
		if (!movetime) {
		    status_offset = now % STATUS_DURATION;
		    setvar("MV", "0");
		} else
		    status |= 1;
		pos = ee ? curC2 : curC1;
	    } else {
		pos = ee ? getvarint("C2") : getvarint("C1");
	    }
	    updatetime = 0;
	    printf("> BOS%d,P=%dEOS\r\n", status, pos);
	    fprintf(stderr, "BOS%d,P=%dEOS\r\n", status, pos);
	}
	if (n <= 0) {
	    if (errno != EINTR)
		return;
	} else {
	    /* Read, not gets.  We might have several commands! */
	    for (s = buf, t = index(s, '\r'); s && *s; s = t, t = index(s, '\r')) {
		if (t) {
		    *t++ = 0;
		    t++;  /* Skip the "\n" */
		}
		{
		    char *q = s;
		    printf("< ");
		    while (*q) {
			if (*q == '\r')
			    printf("\\r");
			else if (*q == '\n')
			    printf("\\n");
			else
			    putchar(*q);
			q++;
		    }
		    putchar('\n');
		}
		if (!strncmp(s, "PR ", 3)) {
		    char *q = dopr(s+3);
		    printf("> %s\n", q);
		    fprintf(stderr, "%s\r\n", q);
		} else if (s[0] == '\e') {
		    setvar("BY", "0");
		    movetime = 0; /* Abort any move! */
		    setvar("MV", "0");
		} else {
		    u = index(s, ' ');
		    if (!u) {
			printf("Can't parse input?!?\n");
			continue;
		    }
		    *u++ = 0;
		    /* Special cases! */
		    if (!strncmp(s, "MA", 2) || !strncmp(s, "MR", 2)) { /* Start a move! */
			if (getvarint("PU") == 1 || getvarint("BY") == 0)
			    continue;
			curC1 = getvarint("C1");
			curC2 = getvarint("C2");
			if (getvarint("EE")) {
			    finalC2 = atoi(u) + ((u[1] == 'R') ? getvarint("C2") : 0);
			    finalC1 = getvarint("C1") + (finalC2 - curC2) * STEP_PER_ELC;
			} else {
			    finalC1 = atoi(u) + ((u[1] == 'R') ? getvarint("C1") : 0);
			    finalC2 = getvarint("C2") + (finalC1 - curC1) / STEP_PER_ELC;
			}
			incC1 = (finalC1 - curC1) / MOVE_DURATION;
			incC2 = (finalC2 - curC2) / MOVE_DURATION;
			movetime = MOVE_DURATION;
			setvar("MV", "1");
			continue;
		    }
		    if (!strncmp(s, "EX", 2)) { /* Start execution! */
			setvar("BY", "1");
			continue;
		    }
		    if (!strncmp(s, "US", 2) && !strncmp(u, "0", 2)) { /* Update now! */
			updatetime = 1;
			setvar(s, u);
			continue;
		    }
		    if (!strncmp(s, "SV", 2)) { /* Save! */
			/* Assume 9? */
			if (!strncmp(u, "9", 2)) {
			    setvar("SV", "0");
			    setvarint("NS", getvarint("NS")+1);
			    printf("> Saved %s\r\n",
				   getvarint("EE") ? getvar("C2") : getvar("C1"));
			    fprintf(stderr, "Saved %s\r\n",
				    getvarint("EE") ? getvar("C2") : getvar("C1"));
			} else if (!strncmp(u, "1", 2)) {
			    setvar("SV", "1");
			    printf("> Want2Save\r\n");
			    fprintf(stderr, "Want2Save\r\n");
			} else {
			    setvar(s, u);
			}
			continue;
		    }
		    setvar(s, u);
		}
	    }
	}
    }
}

int main(int argc, char **argv)
{
    struct sockaddr_in serv_addr, cli_addr;
    int newfd, fd, portno, len, pid;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("ERROR opening socket");
        exit(1);
    }
   
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = atoi(argv[1]);
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
   
    if (bind(fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR on binding");
        exit(1);
    }

    signal(SIGCHLD, childsig);
      
    listen(fd, 5);
    for (;;) {
        len = sizeof(cli_addr);
        newfd = accept(fd, (struct sockaddr *)&cli_addr, &len);
        if (newfd < 0) {
            perror("ERROR on accept");
            exit(1);
        }

        if ((pid = fork()) == 0) {
            dup2(newfd, 0);
            dup2(newfd, 2);
	    serve();
	    printf("Child terminating.\n");
            exit(0);
        } else {
	    printf("New child %d\n", pid);
            close(newfd);
        }
    }
}

