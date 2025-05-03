#ifndef QAMFRAME_H
#define QAMFRAME_H

#include <QObject>
#include <stdint.h>
#include "amframe.h"

class QAMFrame : public QObject
{
    Q_OBJECT
public:
    explicit QAMFrame(QObject *parent = nullptr, uint16_t max_msg_size = 0, uint16_t max_chunk_size = 0, uint16_t max_rs_symbols = 0);
    ~QAMFrame(void);
signals:

public slots:

private:
    AMFrame *amframe;
};

#endif // QAMFRAME_H
