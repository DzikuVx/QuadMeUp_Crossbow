#include "variables.h"

int8_t txSendSequence[16] = {
    QSP_FRAME_PING,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA
};

int8_t rxSendSequence[16] = {
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1
};