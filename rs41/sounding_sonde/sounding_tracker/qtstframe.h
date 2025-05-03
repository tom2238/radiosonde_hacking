#ifndef QTSTFRAME_H
#define QTSTFRAME_H

#include <QObject>
#include <QWidget>

class qtstframe : public QObject
{
    Q_OBJECT
public:
    explicit qtstframe(QObject *parent = nullptr);

signals:

public slots:
};

#endif // QTSTFRAME_H
