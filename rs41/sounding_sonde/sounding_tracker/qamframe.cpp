#include "qamframe.h"

QAMFrame::QAMFrame(QObject *parent, uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols) : QObject(parent)
{
    uint8_t result_stat;
    // New frame
    amframe = new AMFrame(max_msg_size,max_chunk_size,max_rs_symbols,&result_stat);
}

QAMFrame::~QAMFrame(void) {
    delete amframe;
}
