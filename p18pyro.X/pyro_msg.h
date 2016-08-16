#ifndef PYRO_MSG_H_INCLUDED
#define PYRO_MSG_H_INCLUDED

#include "pyro_defs.h"

/* spinner defines */
#define MAX_SHAPES  6
const rom int8_t spin[MAX_SHAPES][20] = {
    "||//--", // classic LCD version with no \ character
    "||//--\\\\", // classic
    "OOOOOO--__-", // eye blink
    "vv<<^^>>", // point spinner
    "..**x#x#XX||--", // warp portal
    "..ooOOoo" // ball bouncer
};

#endif /* PYRO_MSG_H_INCLUDED */

